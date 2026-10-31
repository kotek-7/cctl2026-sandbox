#!/usr/bin/env python3
"""DualSense → PC → CCTL。設定、入力、操縦ループの順に読む。"""

import argparse
import math
import signal
import struct
import sys
import time
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

import evdev
import serial
from evdev import ecodes as E

PERIOD = 0.02
# IDはCCTLの既存プロトコル。速度上限は軸設定から計算する。
PARAM_IDS = {
    "c620_esc_id": 21,
    "c620_slot2_esc_id": 41,
    "el05_host_id": 25,
    "el05_limit_cur": 10,
    "el05_limit_spd": 9,
    "el05_loc_kp": 8,
    "el05_motor_id": 24,
    "el05_period_ms": 28,
    "feedback_timeout_ms": 31,
    "m3508_max_current_ma": 7,
    "m3508_max_rpm": 3,
    "m3508_max_temperature_c": 32,
    "m3508_period_ms": 26,
    "m3508_pos_kd": 2,
    "m3508_pos_ki": 1,
    "m3508_pos_kp": 0,
    "m3508_slot2_max_current_ma": 40,
    "m3508_slot2_max_rpm": 36,
    "m3508_slot2_max_temperature_c": 42,
    "m3508_slot2_pos_kd": 35,
    "m3508_slot2_pos_ki": 34,
    "m3508_slot2_pos_kp": 33,
    "m3508_slot2_vel_kd": 39,
    "m3508_slot2_vel_ki": 38,
    "m3508_slot2_vel_kp": 37,
    "m3508_vel_kd": 6,
    "m3508_vel_ki": 5,
    "m3508_vel_kp": 4,
    "telemetry_period_ms": 29,
    "watchdog_ms": 30,
}
STOP_PWM = "CAN 2 768 0100000000000000"
STOP_STS = "CAN 2 800 0103000000000000"
AXES = (E.ABS_X, E.ABS_Y, E.ABS_RX, E.ABS_RY, E.ABS_Z, E.ABS_RZ)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def load_config(path):
    with open(path, "rb") as file:
        config = tomllib.load(file)
    machine = config["machine"]
    axes, pwm, rotation = machine["axes"], machine["pwm_servos"], machine["rotation"]
    require(len(axes) == 3 and len(pwm) == 4, "軸はr/θ/z、PWMは畳みと把持3本です")
    require(0 < machine["slow_speed_percent"] <= 100, "低速率が不正です")
    for i, axis in enumerate(axes):
        require(
            axis["name"] == ("r", "theta", "z")[i] and axis["slot"] == i,
            "軸の順番が不正です",
        )
        require(
            axis["input_axis"] in range(7) and abs(axis["input_sign"]) == 1,
            "入力割当が不正です",
        )
        require(
            axis["native_per_unit"] != 0
            and axis["speed_per_second"] > 0
            and axis["jog_ramp_seconds"] > 0,
            "換算/速度/加速時間が不正です",
        )
        require(
            axis["minimum"] < axis["maximum"]
            and axis["minimum"] <= axis["origin_position"] <= axis["maximum"],
            "原点/可動範囲が不正です",
        )
        if "limit" in axis:
            limit = axis["limit"]
            require(
                limit["input"] in range(3) and abs(limit["direction"]) == 1,
                "接点設定が不正です",
            )
    require(all("limit" in axes[i] for i in (0, 2)), "r/zの接点が必要です")
    for i, servo in enumerate(pwm):
        require(
            servo["channel"] == i
            and 100 <= servo["minimum_us"] < servo["maximum_us"] <= 20000,
            "PWM設定が不正です",
        )
        require(
            servo["minimum_us"] <= servo["initial_us"] <= servo["maximum_us"],
            "PWM初期値が範囲外です",
        )
        require(
            servo["speed_us_per_second"] > 0
            and servo["acceleration_us_per_second2"] >= 0
            and abs(servo["input_sign"]) == 1,
            "PWM速度/方向が不正です",
        )
        if i:
            for name in ("grip_open", "grip_closed", "grip_handoff_open"):
                require(
                    len(config[name]) == 3
                    and servo["minimum_us"]
                    <= config[name][i - 1]
                    <= servo["maximum_us"],
                    "把持幅が範囲外です",
                )
    require(
        0 < rotation["id"] < 254 and abs(rotation["input_sign"]) == 1,
        "STSのID/方向が不正です",
    )
    require(
        rotation["counts_per_deg"] != 0 and abs(rotation["zero0_count"]) <= 28672,
        "STS換算が不正です",
    )
    require(
        0 <= rotation["speed_position_per_second"] <= 1000
        and 0 <= rotation["acceleration"] <= 255,
        "STS速度/加速が不正です",
    )
    require(set(machine["parameters"]) <= PARAM_IDS.keys(), "未知のCCTLパラメータです")
    for board in ("parameters", "svmd_parameters", "serial_svmd_parameters"):
        require(machine[board]["watchdog_ms"] == 250, "watchdog_msは250にしてください")
    require(
        set(machine["svmd_parameters"]) == {"watchdog_ms"}
        and set(machine["serial_svmd_parameters"]) == {"watchdog_ms"},
        "周辺基板の設定はwatchdog_msのみ対応します",
    )
    # TOMLはnan/infも受理する。計算前にすべて拒否する。
    pending = [config]
    while pending:
        value = pending.pop()
        if isinstance(value, dict):
            pending.extend(value.values())
        elif isinstance(value, list):
            pending.extend(value)
        elif isinstance(value, float):
            require(math.isfinite(value), "設定に非有限値があります")
    return config


# 通信データを保持するだけ。指令クラスや通信サービスは設けない。
@dataclass
class Feedback:
    buffer: bytes = b""
    device: dict = field(default_factory=dict)
    state: dict = field(default_factory=dict)
    state_time: float = -math.inf
    pwm: dict = field(default_factory=dict)
    rotation: tuple | None = None
    parameters: dict = field(default_factory=dict)


def send(port, *lines):
    data = ("\n".join(lines) + "\n").encode("ascii")
    require(port.write(data) == len(data), "USB書込みが完了しませんでした")


def can(board, opcode, channel=0, flags=0, data=b"\0\0\0\0"):
    return f"CAN 2 {board} " + (bytes((1, opcode, channel, flags)) + data).hex().upper()


def receive(port, feedback):
    feedback.buffer += port.read(min(port.in_waiting, 4096))
    require(len(feedback.buffer) <= 16384, "USB応答が長すぎます")
    while b"\n" in feedback.buffer:
        raw, feedback.buffer = feedback.buffer.split(b"\n", 1)
        line = raw.decode("ascii").strip()
        require(not line.startswith("ERR "), f"基板エラー: {line}")
        words = line.split()
        if not words:
            continue
        if words[0] in ("DEVICE", "STATE", "CAN_RX"):
            values = dict(word.split("=", 1) for word in words[1:])
        if words[0] == "DEVICE":
            feedback.device = values
        elif words[0] == "STATE":
            state = values
            for key in ("t", "en", "sw", "stale", "can", "hold"):
                state[key] = int(state[key])
            state["position"] = [float(state[f"a{i}"].split("/")[1]) for i in range(3)]
            require(
                all(math.isfinite(v) for v in state["position"]), "実測値が不正です"
            )
            if feedback.state:
                old = feedback.state["t"]
                require(
                    state["t"] >= old or old > 0xFFFFFFFF - 1000, "CCTLが再起動しました"
                )
            feedback.state, feedback.state_time = state, time.monotonic()
        elif words[0] == "PARAM":
            feedback.parameters[(0, int(words[1]))] = float(words[2])
        elif words[0] == "CAN_RX" and values["bus"] == "2":
            board, data = int(values["id"]), bytes.fromhex(values["data"])
            require(len(data) == 8 and data[0] == 1, "CAN応答の形式が不正です")
            if board == 769:
                require(data[1] == 0 and data[3] < 4, "PWM基板の応答異常")
                feedback.pwm[data[3]] = (
                    int.from_bytes(data[5:7]),
                    bool(data[4] & (1 << data[3])),
                    time.monotonic(),
                )
            elif board == 802:
                require(data[5] == 0 and data[6] & 1, "STSの実測位置が無効です")
                raw = int.from_bytes(data[2:4])
                position = (raw & 0x7FFF) * (-1 if raw & 0x8000 else 1)
                feedback.rotation = (data[1], position, bool(data[4]), time.monotonic())
            elif board in (772, 804):
                feedback.parameters[(board, data[1])] = struct.unpack(">f", data[4:8])[
                    0
                ]
            elif board == 805:
                raise RuntimeError(f"STS通信異常: {data.hex()}")


def wait_for(port, feedback, predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while not predicate():
        require(time.monotonic() < deadline, "基板の応答待ちがタイムアウトしました")
        receive(port, feedback)
        time.sleep(0.01)


def configure(port, feedback, config):
    machine = config["machine"]
    send(port, "HELLO 1", "STOP", STOP_PWM, STOP_STS, "ENABLE 7 0", "SAFE")
    wait_for(
        port,
        feedback,
        lambda: (
            feedback.device
            and feedback.state.get("mode") == "SAFE"
            and feedback.state.get("en") == 0
        ),
        3.0,
    )
    device = feedback.device
    require(
        device["board"] == "cctl"
        and device["protocol"] == "1"
        and device["slots"] == "3"
        and device["jog"] == "1"
        and device["motors"] == "el05,m3508,m3508",
        "CCTLの機種/版が違います",
    )
    params = dict(machine["parameters"])
    for i, axis in enumerate(machine["axes"]):
        params[("el05_limit_spd", "m3508_max_rpm", "m3508_slot2_max_rpm")[i]] = (
            axis["speed_per_second"]
            * abs(axis["native_per_unit"])
            / (1 if i == 0 else 6)
        )
    plan = [(0, PARAM_IDS[name], value) for name, value in params.items()]
    servos = machine["pwm_servos"]
    plan += [
        (768, 0, min(s["minimum_us"] for s in servos)),
        (768, 1, max(s["maximum_us"] for s in servos)),
        (768, 2, 250),
        (800, 3, 250),
    ]
    for board, number, value in plan:
        value = float(f"{value:.5f}") if board == 0 else value
        key = (board + 4 if board else 0, number)
        feedback.parameters.pop(key, None)
        send(
            port,
            f"PARAM {number} {value:.5f}"
            if board == 0
            else can(
                board, 4 if board == 768 else 9, number, data=struct.pack(">f", value)
            ),
        )
        wait_for(port, feedback, lambda key=key: key in feedback.parameters)
        require(
            math.isclose(feedback.parameters[key], value, rel_tol=1e-5, abs_tol=1e-4),
            f"設定の照合不一致: {key}",
        )
    for i, servo in enumerate(servos):
        send(port, can(768, 1, i, data=struct.pack(">HH", servo["initial_us"], 0)))
    print("設定照合済み。原点を合わせ、Create長押し→全入力を離す→Options。", flush=True)


def open_pad():
    pads = []
    for path in evdev.list_devices():
        device = evdev.InputDevice(path)
        axes = device.capabilities(absinfo=False).get(E.EV_ABS, [])
        if (
            device.info.vendor == 0x054C
            and device.info.product in (0x0CE6, 0x0DF2)
            and set(AXES).issubset(axes)
        ):
            pads.append(device)
        else:
            device.close()
    require(
        len(pads) == 1,
        "DualSenseを1台接続してください（入力デバイスのアクセス権も確認）",
    )
    return pads[0]


def read_pad(device):
    try:
        for event in device.read():
            require(
                not (event.type == E.EV_SYN and event.code == E.SYN_DROPPED),
                "パッド入力が欠落しました",
            )
    except BlockingIOError:
        pass
    axes = []
    for code in AXES:
        info = device.absinfo(code)
        require(
            info.max > info.min and info.min <= info.value <= info.max,
            "パッド軸の範囲が不正です",
        )
        value = (info.value - info.min) / (info.max - info.min)
        axes.append(value if code in (E.ABS_Z, E.ABS_RZ) else value * 2 - 1)
    axes[1] *= -1  # gilrsと同じ、上を正にする。
    return (
        axes,
        set(device.active_keys()),
        (device.absinfo(E.ABS_HAT0X).value, device.absinfo(E.ABS_HAT0Y).value),
    )


def reached(axis, contacts):
    limit = axis["limit"]
    return bool(contacts & (1 << limit["input"])) != limit["normally_closed"]


def limited_speed(axis, position, speed, contacts):
    low, high = axis["minimum"], axis["maximum"]
    if "limit" in axis:
        direction = axis["limit"]["direction"]
        if reached(axis, contacts) and speed * direction > 0:
            return 0.0
        if direction > 0:
            high = min(high, axis["origin_position"])
        else:
            low = max(low, axis["origin_position"])
    distance = max(0, high - position if speed > 0 else position - low)
    acceleration = axis["speed_per_second"] / axis["jog_ramp_seconds"]
    delay_speed = acceleration * 0.2
    cap = max(0, math.sqrt(delay_speed**2 + 2 * acceleration * distance) - delay_speed)
    return math.copysign(min(abs(speed), cap), speed)


def rotation_target(rotation, field_angle, theta):
    count = round(
        rotation["zero0_count"] + rotation["counts_per_deg"] * (field_angle - theta)
    )
    require(-28672 <= count <= 28672, "EE回転目標が範囲外です")
    encoded = abs(count) | (0x8000 if count < 0 else 0)
    return can(
        800,
        4,
        rotation["id"],
        rotation["acceleration"],
        struct.pack(">HH", encoded, round(rotation["speed_position_per_second"])),
    )


def pwm_step(position, target, velocity, speed, acceleration, dt):
    remaining = target - position
    desired = math.copysign(
        min(speed, math.sqrt(2 * acceleration * abs(remaining)))
        if acceleration
        else speed,
        remaining,
    )
    velocity = (
        velocity + max(-acceleration * dt, min(acceleration * dt, desired - velocity))
        if acceleration
        else desired
    )
    movement = velocity * dt
    if (
        abs(remaining) < 0.01
        or movement * remaining >= 0
        and abs(movement) >= abs(remaining)
    ):
        return target, 0.0
    return position + movement, velocity


def drive(port, pad, config, feedback):
    machine = config["machine"]
    axes, servos, rotation = machine["axes"], machine["pwm_servos"], machine["rotation"]
    mode, origins = "stopped", None
    previous_keys, previous_hat = set(), (0, 0)
    neutral_seen, home_since = False, None
    speeds, targets = [0.0] * 3, [0.0] * 3
    pulses = [s["initial_us"] for s in servos]
    goals, pwm_speeds = list(pulses), [0.0] * 4
    field_angle = 0.0
    last_tick, last_poll, run_sent = time.monotonic(), -math.inf, -math.inf
    while True:
        now = time.monotonic()
        require(now - last_tick < 0.2, "制御ループが遅延しました")
        dt, last_tick = min(now - last_tick, 0.05), now
        receive(port, feedback)
        state = feedback.state
        require(now - feedback.state_time < 0.2, "CCTL応答が途絶えました")
        require(
            state["err"] == "00,00,00"
            and state["stale"] == 0
            and state["can"] & 3 == 3,
            "モータ/CAN異常です",
        )
        sticks, keys, hat = read_pad(pad)  # 切断/読取りエラーはfinallyの全出力解除へ。
        pressed = keys - previous_keys
        previous_keys, old_hat = keys, previous_hat
        previous_hat = hat
        neutral = all(abs(v) < 0.1 for v in sticks) and hat == (0, 0)
        if neutral and not keys:
            neutral_seen = True
        send(port, "HEARTBEAT")
        if now - last_poll >= 0.1:
            for channel in range(4):
                send(port, can(768, 3 if mode == "running" else 2, channel))
            send(port, can(800, 7, rotation["id"]))
            last_poll = now

        # PSは保持停止。再開には全入力を離してからOptionsが必要。
        if E.BTN_MODE in keys:
            if E.BTN_MODE in pressed and mode in ("starting", "running"):
                send(port, "HOLD 7", STOP_PWM, STOP_STS)
                mode, neutral_seen, home_since = "held", False, None
                speeds = [0.0] * 3
                print("保持停止（EEは出力解除）", flush=True)
            time.sleep(max(0, PERIOD - (time.monotonic() - now)))
            continue
        if mode == "stopped" and E.BTN_SELECT in keys and neutral and neutral_seen:
            home_since = now if home_since is None else home_since
            if now - home_since >= 1.0:
                if all(reached(axes[i], state["sw"]) for i in (0, 2)):
                    origins = [
                        state["position"][i]
                        - a["origin_position"] * a["native_per_unit"]
                        for i, a in enumerate(axes)
                    ]
                    print("原点設定済み", flush=True)
                else:
                    print("r前端・z下端の両接点へ合わせてください", flush=True)
                neutral_seen, home_since = False, None
        else:
            home_since = None

        # 始動前に位置と応答を確認し、RUNの反映を待つ。
        ee_fresh = (
            len(feedback.pwm) == 4
            and all(now - p[2] < 0.6 for p in feedback.pwm.values())
            and feedback.rotation is not None
            and now - feedback.rotation[3] < 0.6
            and feedback.rotation[0] == rotation["id"]
        )
        if E.BTN_START in pressed and mode in ("stopped", "held"):
            if origins is None or not neutral_seen or not neutral or not ee_fresh:
                print("原点設定・中立復帰・EE応答を確認してください", flush=True)
            else:
                pulses = [float(feedback.pwm[i][0]) for i in range(4)]
                require(
                    all(
                        s["minimum_us"] <= p <= s["maximum_us"]
                        for s, p in zip(servos, pulses)
                    ),
                    "PWM保持値が範囲外です",
                )
                goals, pwm_speeds = list(pulses), [0.0] * 4
                theta = (state["position"][1] - origins[1]) / axes[1]["native_per_unit"]
                field_angle = (
                    theta
                    + (feedback.rotation[1] - rotation["zero0_count"])
                    / rotation["counts_per_deg"]
                )
                targets, speeds = list(state["position"]), [0.0] * 3
                send(port, "ENABLE 7 1", "RUN")
                mode, run_sent = "starting", now
        if mode == "starting":
            require(now - run_sent < 0.25, "RUNの応答がありません")
            if (
                state["mode"] == "RUN"
                and state["en"] == 7
                and feedback.state_time > run_sent
            ):
                theta = (state["position"][1] - origins[1]) / axes[1]["native_per_unit"]
                send(
                    port,
                    rotation_target(rotation, field_angle, theta),
                    can(800, 6, rotation["id"], 1),
                    can(800, 2),
                )
                for i, pulse in enumerate(pulses):
                    send(
                        port,
                        can(768, 1, i, data=struct.pack(">HH", round(pulse), 0)),
                        can(768, 2, i, 1),
                    )
                mode = "running"
                print("手動運転", flush=True)
        if mode == "running":
            require(
                state["mode"] == "RUN" and state["en"] == 7 and ee_fresh,
                "運転状態/EE応答を失いました",
            )
            if now - run_sent > 0.25:
                require(
                    all(p[1] for p in feedback.pwm.values()) and feedback.rotation[2],
                    "EE出力が無効になりました",
                )
            scale = machine["slow_speed_percent"] / 100 if E.BTN_TL in keys else 1.0
            inputs = sticks + [sticks[5] - sticks[4]]
            positions = [
                (p - origin) / a["native_per_unit"]
                for p, origin, a in zip(state["position"], origins, axes)
            ]
            for i, axis in enumerate(axes):
                raw = inputs[axis["input_axis"]]
                desired = (
                    (raw if abs(raw) >= 0.1 else 0)
                    * axis["input_sign"]
                    * axis["speed_per_second"]
                    * scale
                )
                desired = limited_speed(axis, positions[i], desired, state["sw"])
                delta = axis["speed_per_second"] / axis["jog_ramp_seconds"] * dt
                speed = limited_speed(
                    axis,
                    positions[i],
                    speeds[i] + max(-delta, min(delta, desired - speeds[i])),
                    state["sw"],
                )
                if speed:
                    send(port, f"JOG {i} {speed * axis['native_per_unit']:.5f}")
                else:
                    if speeds[i]:
                        targets[i] = state["position"][i]
                    send(port, f"TARGET {i} {targets[i]:.5f}")
                speeds[i] = speed
            # 畳みは押している間、把持は押した瞬間に指定幅へ動かす。
            if hat[1]:
                goals[0] = servos[0][
                    "maximum_us"
                    if -hat[1] * servos[0]["input_sign"] > 0
                    else "minimum_us"
                ]
            else:
                goals[0], pwm_speeds[0] = pulses[0], 0.0
            if hat[0] and hat[0] != old_hat[0]:
                goals[1:] = config["grip_open" if hat[0] < 0 else "grip_closed"]
            elif E.BTN_EAST in pressed:
                goals[1:] = config["grip_handoff_open"]
            for i, servo in enumerate(servos):
                pulses[i], pwm_speeds[i] = pwm_step(
                    pulses[i],
                    goals[i],
                    pwm_speeds[i],
                    servo["speed_us_per_second"] * scale,
                    servo["acceleration_us_per_second2"] * scale,
                    dt,
                )
                send(port, can(768, 1, i, data=struct.pack(">HH", round(pulses[i]), 0)))
            trim = math.copysign(max(0, abs(sticks[2]) - 0.1) / 0.9, sticks[2])
            rate = (
                min(
                    15,
                    rotation["speed_position_per_second"]
                    / abs(rotation["counts_per_deg"]),
                )
                if rotation["speed_position_per_second"]
                else 15
            )
            field_angle += trim * rotation["input_sign"] * rate * scale * dt
            send(
                port, rotation_target(rotation, field_angle, positions[1]), can(800, 5)
            )
        time.sleep(max(0, PERIOD - (time.monotonic() - now)))


def stop_signal(_signal, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "device", help="CCTLの /dev/serial/by-id/... または /dev/ttyACM..."
    )
    args = parser.parse_args()
    config = load_config(Path(__file__).with_name("machine.toml"))
    signal.signal(signal.SIGTERM, stop_signal)
    pad = open_pad()
    try:
        with serial.Serial(
            args.device, 115200, timeout=0, write_timeout=0.1, exclusive=True
        ) as port:
            try:
                feedback = Feedback()
                configure(port, feedback, config)
                drive(port, pad, config, feedback)
            finally:
                # 一つの書込みが失敗しても、残りの基板への解除を試す。
                failures = []
                for command in ("STOP", STOP_PWM, STOP_STS):
                    try:
                        send(port, command)
                    except (OSError, serial.SerialException, RuntimeError) as error:
                        failures.append(str(error))
                require(not failures, "出力解除の送信失敗: " + "; ".join(failures))
    finally:
        pad.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("終了。出力解除の受信確認は実機側で行ってください。")
    except (OSError, RuntimeError, ValueError, KeyError) as error:
        print(f"停止: {error}", file=sys.stderr)
        sys.exit(1)
