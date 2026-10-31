//! DualSense → PC → CCTL。初期設定を済ませ、単一ループで手動操縦する。
mod board;
mod robot;
use anyhow::{Context, Result, ensure};
use board::{Feedback, STOP_PWM, STOP_STS, can, pwm_target, rotation_target, send};
use gilrs::{Axis, Button, Gilrs};
use robot::{Config, limited_speed, pwm_step};
use serialport::SerialPort;
use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};
const PERIOD: Duration = Duration::from_millis(20);

#[derive(Clone, Default)]
struct Pad {
    axes: [f32; 7],
    hat: [i8; 2],
    create: bool,
    start: bool,
    stop: bool,
    slow: bool,
    handoff: bool,
    released: bool,
}
impl Pad {
    fn neutral(&self) -> bool {
        self.axes.iter().all(|v| v.abs() < 0.1) && self.hat == [0, 0]
    }
}
fn read_pad(gamepad: &gilrs::Gamepad) -> Pad {
    let trigger = |button| gamepad.button_data(button).map_or(0.0, |d| d.value());
    let left = trigger(Button::LeftTrigger2);
    let right = trigger(Button::RightTrigger2);
    let down = |button| gamepad.is_pressed(button);
    Pad {
        axes: [
            gamepad.value(Axis::LeftStickX),
            gamepad.value(Axis::LeftStickY),
            gamepad.value(Axis::RightStickX),
            gamepad.value(Axis::RightStickY),
            left,
            right,
            right - left,
        ],
        hat: [
            i8::from(down(Button::DPadRight)) - i8::from(down(Button::DPadLeft)),
            i8::from(down(Button::DPadDown)) - i8::from(down(Button::DPadUp)),
        ],
        create: down(Button::Select),
        start: down(Button::Start),
        stop: down(Button::Mode),
        slow: down(Button::LeftTrigger),
        handoff: down(Button::East),
        released: [
            Button::South,
            Button::East,
            Button::West,
            Button::North,
            Button::Select,
            Button::Start,
            Button::Mode,
            Button::LeftThumb,
            Button::RightThumb,
            Button::LeftTrigger,
            Button::RightTrigger,
            Button::DPadUp,
            Button::DPadDown,
            Button::DPadLeft,
            Button::DPadRight,
        ]
        .iter()
        .all(|b| !down(*b)),
    }
}
#[derive(Clone, Copy)]
enum Mode {
    Stopped,
    Held(Instant),
    Starting(Instant),
    Running(Instant),
}

fn drive(
    port: &mut dyn SerialPort,
    config: &Config,
    feedback: &mut Feedback,
    stop: &AtomicBool,
    mut input: impl FnMut() -> Result<Pad>,
) -> Result<()> {
    let machine = &config.machine;
    let (axes, servos, rotation) = (&machine.axes, &machine.pwm_servos, &machine.rotation);
    let mut mode = Mode::Stopped;
    let mut origins: Option<[f32; 3]> = None;
    let mut previous = Pad::default();
    let (mut neutral_seen, mut home_since) = (false, None);
    let (mut speeds, mut targets) = ([0.0; 3], [0.0; 3]);
    let mut pulses = servos.clone().map(|s| f32::from(s.initial_us));
    let (mut goals, mut pwm_speeds) = (pulses, [0.0; 4]);
    let mut field_angle = 0.0;
    let (mut last_tick, mut last_poll) = (Instant::now(), Instant::now() - Duration::from_secs(1));
    while !stop.load(Ordering::Relaxed) {
        let now = Instant::now();
        ensure!(
            now.duration_since(last_tick) < Duration::from_millis(200),
            "制御ループが遅延しました"
        );
        let dt = now.duration_since(last_tick).as_secs_f32().min(0.05);
        last_tick = now;
        board::receive(port, feedback)?;
        let (state, seen) = feedback.state.as_ref().context("CCTL応答がありません")?;
        ensure!(
            now.saturating_duration_since(*seen) < Duration::from_millis(200),
            "CCTL応答が途絶えました"
        );
        ensure!(
            state.errors == [0; 3] && state.stale == 0 && state.buses & 3 == 3,
            "モータ/CAN異常です"
        );
        if let Mode::Held(sent) = mode
            && now.duration_since(sent) > Duration::from_millis(250)
        {
            ensure!(
                state.mode == "SAFE" && state.held == 7 && state.enabled == 7,
                "保持状態を失いました"
            );
        }
        let pad = input()?; // 切断・読取りエラーはsession末尾の全出力解除へ。
        ensure!(
            now.elapsed() < Duration::from_millis(200),
            "入力処理が遅延しました"
        );
        ensure!(
            pad.axes.iter().all(|v| v.is_finite() && v.abs() <= 1.01),
            "パッド入力が不正です"
        );
        let old = std::mem::replace(&mut previous, pad.clone());
        if pad.neutral() && pad.released {
            neutral_seen = true;
        }
        send(port, "HEARTBEAT")?;
        if now.duration_since(last_poll) >= Duration::from_millis(100) {
            for channel in 0..4 {
                send(
                    port,
                    &can(
                        768,
                        if matches!(mode, Mode::Running(_)) {
                            3
                        } else {
                            2
                        },
                        channel,
                        0,
                        [0; 4],
                    ),
                )?;
            }
            send(port, &can(800, 7, rotation.id, 0, [0; 4]))?;
            last_poll = now;
        }
        // PSは3軸を保持停止しEEを解除。再開前に全入力を離す。
        if pad.stop {
            if !old.stop && matches!(mode, Mode::Starting(_) | Mode::Running(_)) {
                for line in ["HOLD 7", STOP_PWM, STOP_STS] {
                    send(port, line)?;
                }
                mode = Mode::Held(now);
                neutral_seen = false;
                home_since = None;
                speeds = [0.0; 3];
                println!("保持停止（EEは出力解除）");
            }
            std::thread::sleep(PERIOD.saturating_sub(now.elapsed()));
            continue;
        }
        if matches!(mode, Mode::Stopped) && pad.create && pad.neutral() && neutral_seen {
            let since = *home_since.get_or_insert(now);
            if now.duration_since(since) >= Duration::from_secs(1) {
                if [0, 2]
                    .iter()
                    .all(|i| axes[*i].limit.unwrap().reached(state.contacts))
                {
                    origins = Some(std::array::from_fn(|i| {
                        state.position[i] - axes[i].origin_position * axes[i].native_per_unit
                    }));
                    println!("原点設定済み");
                } else {
                    println!("r前端・z下端の両接点へ合わせてください");
                }
                neutral_seen = false;
                home_since = None;
            }
        } else {
            home_since = None;
        }

        // 開始前の照合とRUN待ち。待っている間も入力と応答を読み続ける。
        let ee_fresh = feedback.pwm.iter().all(|p| {
            p.is_some_and(|p| now.saturating_duration_since(p.seen) < Duration::from_millis(600))
        }) && feedback.rotation.is_some_and(|s| {
            s.id == rotation.id
                && now.saturating_duration_since(s.seen) < Duration::from_millis(600)
        });
        if pad.start && !old.start && matches!(mode, Mode::Stopped | Mode::Held(_)) {
            if let Some(origin) = origins.filter(|_| neutral_seen && pad.neutral() && ee_fresh) {
                pulses = std::array::from_fn(|i| feedback.pwm[i].unwrap().pulse.into());
                ensure!(
                    servos
                        .iter()
                        .zip(pulses)
                        .all(|(s, p)| (f32::from(s.minimum_us)..=f32::from(s.maximum_us))
                            .contains(&p)),
                    "PWM保持値が範囲外です"
                );
                goals = pulses;
                pwm_speeds = [0.0; 4];
                let theta = (state.position[1] - origin[1]) / axes[1].native_per_unit;
                field_angle = theta
                    + (feedback.rotation.unwrap().position as f32 - rotation.zero0_count)
                        / rotation.counts_per_deg;
                targets = state.position;
                speeds = [0.0; 3];
                send(port, "ENABLE 7 1")?;
                send(port, "RUN")?;
                mode = Mode::Starting(now);
            } else {
                println!("原点設定・中立復帰・EE応答を確認してください");
            }
        }
        if let Mode::Starting(sent) = mode {
            ensure!(
                now.duration_since(sent) < Duration::from_millis(250),
                "RUNの応答がありません"
            );
            if state.mode == "RUN" && state.enabled == 7 && *seen > sent {
                let theta = (state.position[1] - origins.unwrap()[1]) / axes[1].native_per_unit;
                send(port, &rotation_target(rotation, field_angle, theta)?)?;
                send(port, &can(800, 6, rotation.id, 1, [0; 4]))?;
                send(port, &can(800, 2, 0, 0, [0; 4]))?;
                for (i, pulse) in pulses.iter().enumerate() {
                    send(port, &pwm_target(i, *pulse))?;
                    send(port, &can(768, 2, i as u8, 1, [0; 4]))?;
                }
                mode = Mode::Running(sent);
                println!("手動運転");
            }
        }
        if let Mode::Running(sent) = mode {
            ensure!(
                state.mode == "RUN" && state.enabled == 7 && ee_fresh,
                "運転状態/EE応答を失いました"
            );
            if now.duration_since(sent) > Duration::from_millis(250) {
                ensure!(
                    feedback.pwm.iter().all(|p| p.unwrap().enabled)
                        && feedback.rotation.unwrap().enabled,
                    "EE出力が無効になりました"
                );
            }
            let scale = if pad.slow {
                machine.slow_speed_percent / 100.0
            } else {
                1.0
            };
            let origins = origins.unwrap();
            let positions: [f32; 3] =
                std::array::from_fn(|i| (state.position[i] - origins[i]) / axes[i].native_per_unit);
            for (i, axis) in axes.iter().enumerate() {
                let raw = pad.axes[axis.input_axis];
                let desired = if raw.abs() >= 0.1 {
                    raw * axis.input_sign * axis.speed_per_second * scale
                } else {
                    0.0
                };
                let desired = limited_speed(axis, positions[i], desired, state.contacts);
                let delta = axis.speed_per_second / axis.jog_ramp_seconds * dt;
                let speed = limited_speed(
                    axis,
                    positions[i],
                    speeds[i] + (desired - speeds[i]).clamp(-delta, delta),
                    state.contacts,
                );
                if speed != 0.0 {
                    send(
                        port,
                        &format!("JOG {i} {:.5}", speed * axis.native_per_unit),
                    )?;
                } else {
                    if speeds[i] != 0.0 {
                        targets[i] = state.position[i];
                    }
                    send(port, &format!("TARGET {i} {:.5}", targets[i]))?;
                }
                speeds[i] = speed;
            }
            // 畳みは押している間。把持は押した瞬間に指定幅へ動かす。
            if pad.hat[1] != 0 {
                goals[0] = if -f32::from(pad.hat[1]) * servos[0].input_sign > 0.0 {
                    servos[0].maximum_us
                } else {
                    servos[0].minimum_us
                }
                .into();
            } else {
                goals[0] = pulses[0];
                pwm_speeds[0] = 0.0;
            }
            if pad.hat[0] != 0 && pad.hat[0] != old.hat[0] {
                goals[1..].copy_from_slice(if pad.hat[0] < 0 {
                    &config.grip_open
                } else {
                    &config.grip_closed
                });
            } else if pad.handoff && !old.handoff {
                goals[1..].copy_from_slice(&config.grip_handoff_open);
            }
            for (i, servo) in servos.iter().enumerate() {
                (pulses[i], pwm_speeds[i]) = pwm_step(
                    pulses[i],
                    goals[i],
                    pwm_speeds[i],
                    servo.speed_us_per_second * scale,
                    servo.acceleration_us_per_second2 * scale,
                    dt,
                );
                send(port, &pwm_target(i, pulses[i]))?;
            }
            let trim = pad.axes[2].signum() * (pad.axes[2].abs() - 0.1).max(0.0) / 0.9;
            let rate = if rotation.speed_position_per_second > 0.0 {
                15.0_f32.min(rotation.speed_position_per_second / rotation.counts_per_deg.abs())
            } else {
                15.0
            };
            field_angle += trim * rotation.input_sign * rate * scale * dt;
            send(port, &rotation_target(rotation, field_angle, positions[1])?)?;
            send(port, &can(800, 5, 0, 0, [0; 4]))?;
        }
        std::thread::sleep(PERIOD.saturating_sub(now.elapsed()));
    }
    Ok(())
}

/// 正常終了・入力切断・基板エラーのいずれでも、全基板の出力解除を試す。
fn session(
    port: &mut dyn SerialPort,
    config: &Config,
    stop: &AtomicBool,
    input: impl FnMut() -> Result<Pad>,
) -> Result<()> {
    let result = (|| {
        let mut feedback = Feedback::default();
        board::configure(port, &mut feedback, config, stop)?;
        drive(port, config, &mut feedback, stop, input)
    })();
    let mut failure = None;
    for command in ["STOP", STOP_PWM, STOP_STS] {
        if let Err(error) = send(port, command) {
            eprintln!("出力解除の送信失敗: {error}");
            failure = Some(error);
        }
    }
    if let Some(error) = failure {
        return Err(error);
    }
    if stop.load(Ordering::Relaxed) {
        Ok(())
    } else {
        result
    }
}

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args == ["--help"] || args == ["-h"] {
        println!(
            "使い方: cargo run --release --locked -- /dev/serial/by-id/対象CCTL\nCreate長押し=原点確定 / Options=運転 / PS=保持 / Ctrl+C=全出力解除"
        );
        return Ok(());
    }
    ensure!(
        args.len() == 1,
        "CCTLの接続先を1つ指定してください。--helpで使い方を表示"
    );
    let config = Config::parse(&std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/machine.toml"
    ))?)?;
    let stop = Arc::new(AtomicBool::new(false));
    let signal = stop.clone();
    ctrlc::set_handler(move || signal.store(true, Ordering::Relaxed))?;
    let mut pads = Gilrs::new().map_err(|e| anyhow::anyhow!("パッド初期化: {e}"))?;
    let ids: Vec<_> = pads
        .gamepads()
        .filter(|(_, g)| {
            g.is_connected()
                && g.vendor_id() == Some(0x054c)
                && matches!(g.product_id(), Some(0x0ce6 | 0x0df2))
        })
        .map(|(id, _)| id)
        .collect();
    ensure!(ids.len() == 1, "DualSenseを1台接続してください");
    let id = ids[0];
    let mut port = serialport::new(&args[0], 115200)
        .timeout(Duration::from_millis(5))
        .open()?;
    session(port.as_mut(), &config, &stop, || {
        while pads.next_event().is_some() {}
        let pad = pads.gamepad(id);
        ensure!(pad.is_connected(), "DualSenseが切断されました");
        Ok(read_pad(&pad))
    })
}

#[cfg(test)]
#[path = "../tests/rust/control.rs"]
mod tests;
