//! この機体だけの設定。配列順は r/θ/z と 畳み/把持1/2/3。
use anyhow::{Result, ensure};
use serde::Deserialize;
pub type ParameterMap = std::collections::BTreeMap<String, f32>;

pub const PARAMETER_NAMES: [&str; 43] = [
    "m3508_pos_kp",
    "m3508_pos_ki",
    "m3508_pos_kd",
    "m3508_max_rpm",
    "m3508_vel_kp",
    "m3508_vel_ki",
    "m3508_vel_kd",
    "m3508_max_current_ma",
    "el05_loc_kp",
    "el05_limit_spd",
    "el05_limit_cur",
    "dm_p_max",
    "dm_v_max",
    "dm_t_max",
    "dm_pos_vel_limit",
    "reserved_15",
    "reserved_16",
    "reserved_17",
    "reserved_18",
    "reserved_19",
    "reserved_20",
    "c620_esc_id",
    "dm_can_id",
    "dm_mst_id",
    "el05_motor_id",
    "el05_host_id",
    "m3508_period_ms",
    "dm_period_ms",
    "el05_period_ms",
    "telemetry_period_ms",
    "watchdog_ms",
    "feedback_timeout_ms",
    "m3508_max_temperature_c",
    "m3508_slot2_pos_kp",
    "m3508_slot2_pos_ki",
    "m3508_slot2_pos_kd",
    "m3508_slot2_max_rpm",
    "m3508_slot2_vel_kp",
    "m3508_slot2_vel_ki",
    "m3508_slot2_vel_kd",
    "m3508_slot2_max_current_ma",
    "c620_slot2_esc_id",
    "m3508_slot2_max_temperature_c",
];

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub machine: MachineConfig,
    pub grip_open: [f32; 3],
    pub grip_closed: [f32; 3],
    pub grip_handoff_open: [f32; 3],
}
#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MachineConfig {
    pub slow_speed_percent: f32,
    pub axes: [Axis; 3],
    pub pwm_servos: [Pwm; 4],
    pub rotation: Rotation,
    pub parameters: ParameterMap,
    pub svmd_parameters: ParameterMap,
    pub serial_svmd_parameters: ParameterMap,
}
#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Axis {
    pub name: String,
    pub slot: u8,
    pub input_axis: usize,
    pub input_sign: f32,
    pub speed_per_second: f32,
    pub jog_ramp_seconds: f32,
    pub native_per_unit: f32,
    pub minimum: f32,
    pub maximum: f32,
    pub origin_position: f32,
    pub limit: Option<Limit>,
}
#[derive(Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Limit {
    pub input: u8,
    pub direction: f32,
    pub normally_closed: bool,
}
impl Limit {
    pub fn reached(&self, contacts: u8) -> bool {
        (contacts & (1 << self.input) != 0) != self.normally_closed
    }
}
#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Pwm {
    pub channel: u8,
    pub input_sign: f32,
    pub speed_us_per_second: f32,
    pub acceleration_us_per_second2: f32,
    pub minimum_us: u16,
    pub maximum_us: u16,
    pub initial_us: u16,
}
#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Rotation {
    pub id: u8,
    pub input_sign: f32,
    pub speed_position_per_second: f32,
    pub acceleration: u8,
    pub zero0_count: f32,
    pub counts_per_deg: f32,
}
impl Config {
    pub fn parse(text: &str) -> Result<Self> {
        let c: Self = toml::from_str(text)?;
        let m = &c.machine;
        ensure!(
            m.slow_speed_percent.is_finite() && (1.0..=100.0).contains(&m.slow_speed_percent),
            "低速率は1〜100%です"
        );
        for (i, a) in m.axes.iter().enumerate() {
            ensure!(
                a.name == ["r", "theta", "z"][i] && a.slot == i as u8 && a.input_axis <= 6,
                "軸の順番/slot/入力番号が不正です"
            );
            ensure!(
                [
                    a.input_sign,
                    a.speed_per_second,
                    a.jog_ramp_seconds,
                    a.native_per_unit,
                    a.minimum,
                    a.maximum,
                    a.origin_position
                ]
                .iter()
                .all(|v| v.is_finite()),
                "軸設定に非有限値があります"
            );
            ensure!(
                a.input_sign.abs() == 1.0
                    && a.native_per_unit != 0.0
                    && a.speed_per_second > 0.0
                    && a.jog_ramp_seconds > 0.0,
                "軸の方向/換算/速度/加速時間が不正です"
            );
            ensure!(
                a.minimum < a.maximum && (a.minimum..=a.maximum).contains(&a.origin_position),
                "軸の範囲/原点が不正です"
            );
            if let Some(l) = a.limit {
                ensure!(
                    l.input < 3 && l.direction.abs() == 1.0,
                    "接点の設定が不正です"
                );
            }
        }
        ensure!(
            m.axes[0].limit.is_some() && m.axes[2].limit.is_some(),
            "r/zの原点接点が必要です"
        );
        for (i, p) in m.pwm_servos.iter().enumerate() {
            ensure!(
                p.channel == i as u8
                    && p.minimum_us >= 100
                    && p.maximum_us <= 20000
                    && p.minimum_us < p.maximum_us
                    && (p.minimum_us..=p.maximum_us).contains(&p.initial_us),
                "PWMの割当/範囲/初期値が不正です"
            );
            ensure!(
                p.input_sign.abs() == 1.0
                    && p.speed_us_per_second.is_finite()
                    && p.speed_us_per_second > 0.0
                    && p.acceleration_us_per_second2.is_finite()
                    && p.acceleration_us_per_second2 >= 0.0,
                "PWM速度/方向が不正です"
            );
            if i > 0 {
                for pose in [c.grip_open, c.grip_closed, c.grip_handoff_open] {
                    ensure!(
                        pose[i - 1].is_finite()
                            && (f32::from(p.minimum_us)..=f32::from(p.maximum_us))
                                .contains(&pose[i - 1]),
                        "把持設定が範囲外です"
                    );
                }
            }
        }
        let s = &m.rotation;
        ensure!(
            s.id > 0
                && s.id < 254
                && s.input_sign.abs() == 1.0
                && s.speed_position_per_second.is_finite()
                && (0.0..=1000.0).contains(&s.speed_position_per_second)
                && s.zero0_count.is_finite()
                && s.zero0_count.abs() <= 28672.0
                && s.counts_per_deg.is_finite()
                && s.counts_per_deg != 0.0,
            "EE回転設定が不正です"
        );
        for (values, names) in [
            (&m.parameters, PARAMETER_NAMES.as_slice()),
            (
                &m.svmd_parameters,
                &["min_pulse_us", "max_pulse_us", "watchdog_ms"],
            ),
            (
                &m.serial_svmd_parameters,
                &[
                    "servo_baud",
                    "servo_timeout_ms",
                    "wait_for_write_status",
                    "watchdog_ms",
                ],
            ),
        ] {
            for (key, value) in values {
                ensure!(
                    names.contains(&key.as_str()) && value.is_finite(),
                    "未知/非有限の基板設定: {key}"
                );
            }
            ensure!(
                values.get("watchdog_ms") == Some(&250.0),
                "各基板のwatchdog_msは250にしてください"
            );
        }
        ensure!(
            m.svmd_parameters.len() == 1 && m.serial_svmd_parameters.len() == 1,
            "周辺基板はwatchdog_msのみ設定できます"
        );
        Ok(c)
    }
}

/// 0.2秒の通信・指令先行量を含め、端点までに制動できる速度へ制限する。
pub fn limited_speed(axis: &Axis, position: f32, speed: f32, contacts: u8) -> f32 {
    let (mut low, mut high) = (axis.minimum, axis.maximum);
    if let Some(limit) = axis.limit {
        if limit.reached(contacts) && speed * limit.direction > 0.0 {
            return 0.0;
        }
        if limit.direction > 0.0 {
            high = high.min(axis.origin_position);
        } else {
            low = low.max(axis.origin_position);
        }
    }
    let distance = if speed > 0.0 {
        high - position
    } else {
        position - low
    };
    let acceleration = axis.speed_per_second / axis.jog_ramp_seconds;
    let delay_speed = acceleration * 0.2;
    let cap = ((delay_speed.powi(2) + 2.0 * acceleration * distance.max(0.0)).sqrt() - delay_speed)
        .max(0.0);
    speed.signum() * speed.abs().min(cap)
}

pub fn rotation_count(rotation: &Rotation, field: f32, theta: f32) -> Result<i16> {
    let count = (rotation.zero0_count + rotation.counts_per_deg * (field - theta)).round();
    ensure!(
        count.is_finite() && (-28672.0..=28672.0).contains(&count),
        "EE回転目標が範囲外です"
    );
    Ok(count as i16)
}

pub fn pwm_step(
    position: f32,
    target: f32,
    velocity: f32,
    speed: f32,
    acceleration: f32,
    dt: f32,
) -> (f32, f32) {
    let remaining = target - position;
    let desired = remaining.signum()
        * if acceleration > 0.0 {
            speed.min((2.0 * acceleration * remaining.abs()).sqrt())
        } else {
            speed
        };
    let velocity = if acceleration > 0.0 {
        velocity + (desired - velocity).clamp(-acceleration * dt, acceleration * dt)
    } else {
        desired
    };
    let movement = velocity * dt;
    if remaining.abs() < 0.01 || movement * remaining >= 0.0 && movement.abs() >= remaining.abs() {
        (target, 0.0)
    } else {
        (position + movement, velocity)
    }
}
