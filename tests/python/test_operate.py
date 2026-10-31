"""実機へアクセスせず、公開するoperate.py自体を実行する。"""

import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import operate as app  # noqa: E402


class Clock:
    def __init__(self):
        self.now = 1.0

    def sleep(self, seconds):
        self.now += max(seconds, 0.001)


class Board:
    """必要なUSB/CAN応答だけを返す。実機の物理挙動は再現しない。"""

    def __init__(self, clock):
        self.clock = clock
        self.rx = bytearray()
        self.commands = []
        self.mode, self.enabled, self.held = "SAFE", 0, 0
        self.position = [0.0] * 3
        self.pulses, self.pwm_mask = [1500] * 4, 0
        self.rotation, self.sts_enabled, self.sts_run = 1024, False, False
        self.contacts, self.silent, self.error, self.reject_run = 7, False, 0, False

    def reply(self, line):
        if not self.silent:
            self.rx.extend((line + "\n").encode())

    def can_reply(self, board, data):
        self.reply(f"CAN_RX bus=2 id={board} data={bytes(data).hex()}")

    @property
    def in_waiting(self):
        positions = " ".join(f"a{i}={p}/{p}" for i, p in enumerate(self.position))
        self.reply(
            f"STATE t={int(self.clock.now * 1000)} mode={self.mode} en={self.enabled} {positions} err={self.error:02X},00,00 sw={self.contacts} stale=0 can=3 hold={self.held}"
        )
        return len(self.rx)

    def read(self, count):
        result = bytes(self.rx[:count])
        del self.rx[:count]
        return result

    def write(self, data):
        for line in data.decode().splitlines():
            self.commands.append(line)
            words = line.split()
            if words[:1] == ["HELLO"]:
                self.reply(
                    "DEVICE protocol=1 board=cctl slots=3 jog=1 motors=el05,m3508,m3508"
                )
            elif words[0] in ("STOP", "SAFE"):
                self.mode, self.held = words[0], 0
            elif words[0] == "ENABLE":
                self.enabled = int(words[1]) if words[2] == "1" else 0
            elif words[0] == "RUN" and not self.reject_run:
                self.mode, self.held = "RUN", 0
            elif words[0] == "HOLD":
                self.mode, self.held = "SAFE", int(words[1])
            elif words[0] == "PARAM":
                if self.mode != "SAFE":
                    self.reply("ERR code=BUSY")
                else:
                    self.reply(line)
            elif words[0] == "CAN":
                board, frame = int(words[2]), bytes.fromhex(words[3])
                _, op, channel, flags = frame[:4]
                if (board, op) in ((768, 4), (800, 9)):
                    self.can_reply(board + 4, bytes((1, channel, 0, 0)) + frame[4:])
                elif board == 768:
                    if op == 0:
                        self.pwm_mask = 0
                    elif op == 1:
                        self.pulses[channel] = int.from_bytes(frame[4:6])
                    elif op == 2:
                        self.pwm_mask = (
                            self.pwm_mask | (1 << channel)
                            if flags
                            else self.pwm_mask & ~(1 << channel)
                        )
                    self.can_reply(
                        769,
                        bytes((1, 0, op, channel, self.pwm_mask))
                        + self.pulses[channel].to_bytes(2)
                        + b"\0",
                    )
                elif board == 800:
                    if op == 2:
                        self.sts_run = True
                    elif op == 3:
                        self.sts_run = self.sts_enabled = False
                    elif op == 4:
                        raw = int.from_bytes(frame[4:6])
                        self.rotation = (raw & 0x7FFF) * (-1 if raw & 0x8000 else 1)
                    elif op == 6:
                        self.sts_enabled = bool(flags)
                    elif op == 7:
                        raw = abs(self.rotation) | (0x8000 if self.rotation < 0 else 0)
                        self.can_reply(
                            802,
                            bytes((1, channel))
                            + raw.to_bytes(2)
                            + bytes((self.sts_enabled and self.sts_run, 0, 1, 0)),
                        )
        return len(data)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass


def snapshot(keys=(), sticks=None, hat=(0, 0)):
    return (sticks or [0.0] * 6, set(keys), hat)


def home_and_start():
    return (
        [snapshot()]
        + [snapshot([app.E.BTN_SELECT])] * 53
        + [snapshot(), snapshot([app.E.BTN_START])]
        + [snapshot()] * 20
    )


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.config = app.load_config(ROOT / "machine.toml")
        self.clock = Clock()
        self.board = Board(self.clock)
        self.feedback = app.Feedback()
        for target, value in (
            ("monotonic", lambda: self.clock.now),
            ("sleep", self.clock.sleep),
        ):
            patcher = patch.object(app.time, target, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        app.configure(self.board, self.feedback, self.config)

    def run_inputs(self, inputs, action=None):
        inputs = iter(inputs)

        def read(_):
            try:
                value = next(inputs)
            except StopIteration:
                raise KeyboardInterrupt
            if action:
                action(self.board)
            return value

        with patch.object(app, "read_pad", read):
            app.drive(self.board, object(), self.config, self.feedback)

    def test_setup_is_disabled_and_parameters_are_checked(self):
        self.assertEqual(self.board.enabled, 0)
        self.assertEqual(self.board.pwm_mask, 0)
        self.assertNotIn("RUN", self.board.commands)
        self.assertEqual(self.board.pulses, [500] * 4)
        self.assertEqual(self.feedback.parameters[(0, 9)], 24.0)

    def test_start_requires_home_and_released_inputs(self):
        for inputs in (
            [snapshot([app.E.BTN_START])] * 5,
            [snapshot(), snapshot([app.E.BTN_START])],
        ):
            with self.assertRaises(KeyboardInterrupt):
                self.run_inputs(inputs)
        self.assertNotIn("RUN", self.board.commands)

    def test_run_enables_servos_after_targets_and_holds_with_ps(self):
        with self.assertRaises(KeyboardInterrupt):
            self.run_inputs(
                home_and_start() + [snapshot([app.E.BTN_MODE])] + [snapshot()] * 5
            )
        self.assertIn("RUN", self.board.commands)
        self.assertIn("HOLD 7", self.board.commands)
        self.assertEqual(self.board.pwm_mask, 0)
        self.assertFalse(self.board.sts_enabled)
        for i in range(4):
            enable = app.can(768, 2, i, 1)
            index = self.board.commands.index(enable)
            self.assertTrue(
                self.board.commands[index - 1].startswith(f"CAN 2 768 0101{i:02X}")
            )

    def test_cannot_resume_hold_without_releasing_buttons(self):
        inputs = (
            home_and_start()
            + [snapshot([app.E.BTN_MODE]), snapshot([app.E.BTN_START])] * 3
        )
        with self.assertRaises(KeyboardInterrupt):
            self.run_inputs(inputs)
        self.assertEqual(self.board.commands.count("RUN"), 1)

    def test_missing_contacts_prevent_origin(self):
        self.board.contacts = 1
        with self.assertRaises(KeyboardInterrupt):
            self.run_inputs(home_and_start())
        self.assertNotIn("RUN", self.board.commands)

    def test_run_without_ack_times_out(self):
        self.board.reject_run = True
        with self.assertRaisesRegex(RuntimeError, "RUN"):
            self.run_inputs(home_and_start())

    def test_motor_fault_and_silence_stop_running(self):
        for fault in ("error", "silent"):
            self.clock.now = 1.0
            self.board = Board(self.clock)
            self.feedback = app.Feedback()
            app.configure(self.board, self.feedback, self.config)

            def fail_after_run(board, fault=fault):
                if board.mode == "RUN":
                    setattr(board, fault, True)

            with self.assertRaises(RuntimeError):
                self.run_inputs(home_and_start() + [snapshot()] * 20, fail_after_run)

    def test_axis_ramps_and_native_signs(self):
        move = snapshot(sticks=[0.0, -1.0, 0.0, 0.0, 0.0, 1.0])
        with self.assertRaises(KeyboardInterrupt):
            self.run_inputs(home_and_start() + [move] * 4)
        r = [float(s.split()[2]) for s in self.board.commands if s.startswith("JOG 0 ")]
        z = [float(s.split()[2]) for s in self.board.commands if s.startswith("JOG 2 ")]
        self.assertAlmostEqual(r[0], 2.4, places=4)
        self.assertAlmostEqual(z[0], -1920.32, places=3)
        self.assertGreater(r[-1], r[0])

    def test_ps_cancels_pending_run(self):
        self.board.reject_run = True
        with self.assertRaises(KeyboardInterrupt):
            self.run_inputs(
                home_and_start()[:56] + [snapshot([app.E.BTN_MODE])] + [snapshot()] * 20
            )
        self.assertIn("HOLD 7", self.board.commands)
        self.assertEqual(self.board.pwm_mask, 0)
        self.assertFalse(self.board.sts_enabled)

    def test_all_exit_paths_send_three_release_commands(self):
        for failure in (
            KeyboardInterrupt(),
            OSError("pad disconnected"),
            RuntimeError("loop stalled"),
        ):
            fake_pad = unittest.mock.Mock()
            with (
                patch.object(app, "open_pad", return_value=fake_pad),
                patch.object(app.serial, "Serial", return_value=self.board),
                patch.object(app, "drive", side_effect=failure),
                patch.object(sys, "argv", ["operate.py", "/test-port"]),
            ):
                with self.assertRaises(type(failure)):
                    app.main()
            self.assertEqual(
                self.board.commands[-3:], ["STOP", app.STOP_PWM, app.STOP_STS]
            )
            fake_pad.close.assert_called_once()


class MappingAndMotionTests(unittest.TestCase):
    def test_evdev_axes_and_pressed_keys(self):
        class Pad:
            def read(self):
                raise BlockingIOError

            def absinfo(self, code):
                value = {
                    app.E.ABS_X: 255,
                    app.E.ABS_Y: 0,
                    app.E.ABS_Z: 0,
                    app.E.ABS_RZ: 255,
                }.get(code, 128)
                if code in (app.E.ABS_HAT0X, app.E.ABS_HAT0Y):
                    return evdev.AbsInfo(0, -1, 1, 0, 0, 0)
                return evdev.AbsInfo(value, 0, 255, 0, 0, 0)

            def active_keys(self):
                return [app.E.BTN_START]

        import evdev

        axes, keys, hat = app.read_pad(Pad())
        self.assertEqual((axes[0], axes[1], axes[4], axes[5]), (1, 1, 0, 1))
        self.assertEqual(keys, {app.E.BTN_START})
        self.assertEqual(hat, (0, 0))

    def test_limits_braking_and_rotation(self):
        machine = app.load_config(ROOT / "machine.toml")["machine"]
        axis = machine["axes"][0]
        self.assertEqual(app.limited_speed(axis, 570, 600, 0), 0)
        self.assertLess(app.limited_speed(axis, 570, -600, 0), 0)
        self.assertEqual(app.limited_speed(axis, 300, 600, 1), 0)
        for distance in (0.01, 1, 10, 100):
            speed = app.limited_speed(axis, 570 - distance, 600, 0)
            self.assertLessEqual(speed * 0.2 + speed**2 / (2 * 3000), distance + 1e-6)
        with self.assertRaises(RuntimeError):
            app.rotation_target(machine["rotation"], 10000, 0)
        self.assertIn("0400", app.rotation_target(machine["rotation"], 10, 10))

    def test_pwm_approaches_target_without_overshooting(self):
        position, speed = 500, 0
        for _ in range(150):
            position, speed = app.pwm_step(position, 1444, speed, 1000, 3000, 0.02)
            self.assertLessEqual(position, 1444)
        self.assertEqual(position, 1444)


class SerialIntegrationTests(unittest.TestCase):
    def test_actual_pyserial_setup_over_a_virtual_terminal(self):
        import os
        import select
        import threading
        import time

        master, slave = os.openpty()
        clock = Clock()
        board = Board(clock)
        stop = threading.Event()
        failures = []

        def emulate():
            pending = b""
            try:
                while not stop.is_set():
                    clock.now = time.monotonic()
                    if select.select([master], [], [], 0.01)[0]:
                        pending += os.read(master, 4096)
                        while b"\n" in pending:
                            line, pending = pending.split(b"\n", 1)
                            board.write(line + b"\n")
                    os.write(master, board.read(board.in_waiting))
            except OSError as error:
                if not stop.is_set():
                    failures.append(error)

        try:
            with app.serial.Serial(
                os.ttyname(slave), 115200, timeout=0, write_timeout=0.1, exclusive=True
            ) as port:
                worker = threading.Thread(target=emulate)
                worker.start()
                try:
                    feedback = app.Feedback()
                    app.configure(
                        port, feedback, app.load_config(ROOT / "machine.toml")
                    )
                    self.assertEqual(feedback.device["board"], "cctl")
                    self.assertEqual(board.enabled, 0)
                    self.assertNotIn("RUN", board.commands)
                finally:
                    stop.set()
                    worker.join(2)
                    self.assertFalse(worker.is_alive())
            self.assertFalse(failures)
        finally:
            os.close(master)
            os.close(slave)


if __name__ == "__main__":
    unittest.main()
