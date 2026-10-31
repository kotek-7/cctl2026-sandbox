//! CCTLの行通信と周辺基板のCAN形式。ライブラリのSerialPortを直接使う。
use crate::robot::{Config, PARAMETER_NAMES, Rotation, rotation_count};
use anyhow::{Context, Result, bail, ensure};
use serialport::SerialPort;
use std::{
    collections::BTreeMap,
    io::ErrorKind,
    sync::atomic::{AtomicBool, Ordering},
    time::{Duration, Instant},
};

pub const STOP_PWM: &str = "CAN 2 768 0100000000000000";
pub const STOP_STS: &str = "CAN 2 800 0103000000000000";
#[derive(Clone)]
pub struct State {
    pub uptime: u32,
    pub mode: String,
    pub enabled: u8,
    pub contacts: u8,
    pub stale: u8,
    pub buses: u8,
    pub errors: [u8; 3],
    pub held: u8,
    pub position: [f32; 3],
}
#[derive(Clone, Copy)]
pub struct PwmState {
    pub pulse: u16,
    pub enabled: bool,
    pub seen: Instant,
}
#[derive(Clone, Copy)]
pub struct RotationState {
    pub id: u8,
    pub position: i32,
    pub enabled: bool,
    pub seen: Instant,
}
/// 受信値だけを保持する。接続や制御器を隠すラッパではない。
#[derive(Default)]
pub struct Feedback {
    buffer: Vec<u8>,
    pub device: bool,
    pub state: Option<(State, Instant)>,
    pub pwm: [Option<PwmState>; 4],
    pub rotation: Option<RotationState>,
    parameters: BTreeMap<(u16, u8), f32>,
}

pub fn send(port: &mut dyn SerialPort, line: &str) -> Result<()> {
    writeln!(port, "{line}")?;
    Ok(())
}
pub fn can(board: u16, opcode: u8, channel: u8, flags: u8, data: [u8; 4]) -> String {
    let bytes = [
        1, opcode, channel, flags, data[0], data[1], data[2], data[3],
    ];
    format!(
        "CAN 2 {board} {}",
        bytes.iter().map(|v| format!("{v:02X}")).collect::<String>()
    )
}
pub fn pwm_target(channel: usize, pulse: f32) -> String {
    let [hi, lo] = (pulse.round() as u16).to_be_bytes();
    can(768, 1, channel as u8, 0, [hi, lo, 0, 0])
}
pub fn rotation_target(rotation: &Rotation, field: f32, theta: f32) -> Result<String> {
    let count = rotation_count(rotation, field, theta)?;
    let [hi, lo] = (count.unsigned_abs() | if count < 0 { 0x8000 } else { 0 }).to_be_bytes();
    let [speed_hi, speed_lo] = (rotation.speed_position_per_second.round() as u16).to_be_bytes();
    Ok(can(
        800,
        4,
        rotation.id,
        rotation.acceleration,
        [hi, lo, speed_hi, speed_lo],
    ))
}

pub fn receive(port: &mut dyn SerialPort, feedback: &mut Feedback) -> Result<()> {
    let mut bytes = [0; 4096];
    match port.read(&mut bytes) {
        Ok(count) => feedback.buffer.extend_from_slice(&bytes[..count]),
        Err(e)
            if matches!(
                e.kind(),
                ErrorKind::TimedOut | ErrorKind::WouldBlock | ErrorKind::Interrupted
            ) => {}
        Err(e) => return Err(e.into()),
    }
    ensure!(feedback.buffer.len() <= 16384, "USB応答が長すぎます");
    while let Some(end) = feedback.buffer.iter().position(|v| *v == b'\n') {
        let line: Vec<_> = feedback.buffer.drain(..=end).collect();
        parse_line(std::str::from_utf8(&line)?.trim(), feedback, Instant::now())?;
    }
    Ok(())
}

fn parse_line(line: &str, feedback: &mut Feedback, now: Instant) -> Result<()> {
    ensure!(!line.starts_with("ERR "), "基板エラー: {line}");
    let mut words = line.split_whitespace();
    let kind = words.next().unwrap_or("");
    if kind == "PARAM" {
        let id = words.next().context("PARAM id")?.parse()?;
        let value = words.next().context("PARAM value")?.parse()?;
        feedback.parameters.insert((0, id), value);
        return Ok(());
    }
    if !matches!(kind, "DEVICE" | "STATE" | "CAN_RX") {
        return Ok(());
    }
    let fields: BTreeMap<_, _> = words.filter_map(|s| s.split_once('=')).collect();
    let get = |key| {
        fields
            .get(key)
            .copied()
            .with_context(|| format!("応答に{key}がありません"))
    };
    if kind == "DEVICE" {
        ensure!(
            get("board")? == "cctl"
                && get("protocol")? == "1"
                && get("slots")? == "3"
                && get("jog")? == "1"
                && get("motors")? == "el05,m3508,m3508",
            "CCTLの機種/版が違います"
        );
        feedback.device = true;
    } else if kind == "STATE" {
        let mut state = State {
            uptime: get("t")?.parse()?,
            mode: get("mode")?.into(),
            enabled: get("en")?.parse()?,
            contacts: get("sw")?.parse()?,
            stale: get("stale")?.parse()?,
            buses: get("can")?.parse()?,
            held: get("hold")?.parse()?,
            errors: [0; 3],
            position: [0.0; 3],
        };
        let errors: Vec<_> = get("err")?.split(',').collect();
        ensure!(errors.len() == 3, "モータ異常応答が不正です");
        for i in 0..3 {
            state.errors[i] = u8::from_str_radix(errors[i], 16)?;
            state.position[i] = get(["a0", "a1", "a2"][i])?
                .split_once('/')
                .context("位置応答が不正です")?
                .1
                .parse()?;
            ensure!(state.position[i].is_finite(), "実測値が不正です");
        }
        ensure!(
            state.errors == [0; 3] && state.stale == 0 && state.buses & 3 == 3,
            "モータ/CAN異常です"
        );
        if let Some((old, _)) = &feedback.state {
            ensure!(
                state.uptime >= old.uptime || old.uptime > u32::MAX - 1000,
                "CCTLが再起動しました"
            );
        }
        feedback.state = Some((state, now));
    } else if get("bus")? == "2" {
        let id: u16 = get("id")?.parse()?;
        let text = get("data")?;
        ensure!(
            text.len() == 16 && text.is_ascii(),
            "CAN応答の形式が不正です"
        );
        let mut data = [0u8; 8];
        for (i, byte) in data.iter_mut().enumerate() {
            *byte = u8::from_str_radix(&text[i * 2..i * 2 + 2], 16)?;
        }
        ensure!(data[0] == 1, "CAN応答の版が違います");
        match id {
            769 => {
                ensure!(data[1] == 0 && data[3] < 4, "PWM基板の応答異常");
                feedback.pwm[data[3] as usize] = Some(PwmState {
                    pulse: u16::from_be_bytes([data[5], data[6]]),
                    enabled: data[4] & (1 << data[3]) != 0,
                    seen: now,
                });
            }
            802 => {
                ensure!(data[5] == 0 && data[6] & 1 != 0, "STS実測位置が無効です");
                let raw = u16::from_be_bytes([data[2], data[3]]);
                feedback.rotation = Some(RotationState {
                    id: data[1],
                    position: i32::from(raw & 0x7fff) * if raw & 0x8000 != 0 { -1 } else { 1 },
                    enabled: data[4] != 0,
                    seen: now,
                });
            }
            772 | 804 => {
                feedback.parameters.insert(
                    (id, data[1]),
                    f32::from_be_bytes([data[4], data[5], data[6], data[7]]),
                );
            }
            805 => bail!("STS通信異常: {text}"),
            _ => {}
        }
    }
    Ok(())
}

fn wait_for(
    port: &mut dyn SerialPort,
    feedback: &mut Feedback,
    stop: &AtomicBool,
    timeout: Duration,
    ready: impl Fn(&Feedback) -> bool,
) -> Result<()> {
    let deadline = Instant::now() + timeout;
    while !ready(feedback) {
        ensure!(!stop.load(Ordering::Relaxed), "終了要求");
        ensure!(
            Instant::now() < deadline,
            "基板の応答待ちがタイムアウトしました"
        );
        receive(port, feedback)?;
        std::thread::sleep(Duration::from_millis(5));
    }
    Ok(())
}

/// 出力無効のまま1件ずつ書き、応答を照合する。操縦ループには持ち込まない。
pub fn configure(
    port: &mut dyn SerialPort,
    feedback: &mut Feedback,
    config: &Config,
    stop: &AtomicBool,
) -> Result<()> {
    for line in ["HELLO 1", "STOP", STOP_PWM, STOP_STS, "ENABLE 7 0", "SAFE"] {
        send(port, line)?;
    }
    wait_for(port, feedback, stop, Duration::from_secs(3), |f| {
        f.device
            && f.state
                .as_ref()
                .is_some_and(|(s, _)| s.mode == "SAFE" && s.enabled == 0)
    })?;
    let machine = &config.machine;
    let mut params = machine.parameters.clone();
    for (i, axis) in machine.axes.iter().enumerate() {
        params.insert(
            ["el05_limit_spd", "m3508_max_rpm", "m3508_slot2_max_rpm"][i].into(),
            axis.speed_per_second * axis.native_per_unit.abs() / if i == 0 { 1.0 } else { 6.0 },
        );
    }
    let mut plan: Vec<(u16, u8, f32)> = params
        .iter()
        .map(|(name, value)| {
            (
                0,
                PARAMETER_NAMES
                    .iter()
                    .position(|n| n == name)
                    .expect("validated parameter") as u8,
                *value,
            )
        })
        .collect();
    plan.extend([
        (
            768,
            0,
            machine
                .pwm_servos
                .iter()
                .map(|s| s.minimum_us)
                .min()
                .unwrap() as f32,
        ),
        (
            768,
            1,
            machine
                .pwm_servos
                .iter()
                .map(|s| s.maximum_us)
                .max()
                .unwrap() as f32,
        ),
        (768, 2, 250.0),
        (800, 3, 250.0),
    ]);
    for (board, id, value) in plan {
        let value = if board == 0 {
            format!("{value:.5}").parse::<f32>()?
        } else {
            value
        };
        let key = (if board == 0 { 0 } else { board + 4 }, id);
        feedback.parameters.remove(&key);
        let line = if board == 0 {
            format!("PARAM {id} {value:.5}")
        } else {
            can(
                board,
                if board == 768 { 4 } else { 9 },
                id,
                0,
                value.to_be_bytes(),
            )
        };
        send(port, &line)?;
        wait_for(port, feedback, stop, Duration::from_secs(1), |f| {
            f.parameters.contains_key(&key)
        })?;
        let actual = feedback.parameters[&key];
        ensure!(
            actual.is_finite() && (actual - value).abs() <= 0.0001_f32.max(value.abs() * 0.00001),
            "設定の照合不一致: {key:?}"
        );
    }
    for (i, servo) in machine.pwm_servos.iter().enumerate() {
        send(port, &pwm_target(i, servo.initial_us.into()))?;
    }
    println!("設定照合済み。Create長押し→全入力を離す→Options。");
    Ok(())
}
