//! 原点・単位換算・3軸の加減速。機体座標の速度を最後にモータ単位へ変換する。
use crate::{
    config::{MachineConfig, Rotation},
    input::ControllerState,
    protocol::{
        serial_svmd,
        telemetry::{RunMode, Telemetry},
    },
};
use anyhow::{Result, ensure};
use std::time::Instant;

pub struct MachineController {
    profile: MachineConfig,
    origins: [Option<f32>; 3],
    targets: [f32; 3],
    moving: [bool; 3],
    velocity: [f32; 3],
    tick: Option<Instant>,
}
impl MachineController {
    pub fn new(profile: MachineConfig) -> Self {
        Self {
            profile,
            origins: [None; 3],
            targets: [0.0; 3],
            moving: [false; 3],
            velocity: [0.0; 3],
            tick: None,
        }
    }
    pub fn origins_ready(&self) -> bool {
        self.origins.iter().all(Option::is_some)
    }
    pub fn invalidate_origins(&mut self) {
        self.origins = [None; 3];
        self.reset_jog();
    }
    pub fn reset_jog(&mut self) {
        self.velocity = [0.0; 3];
        self.tick = None;
        self.moving = [false; 3];
    }
    pub fn observe(&mut self, t: &Telemetry) {
        for i in 0..3 {
            if t.stale_slots & (1 << i) != 0 {
                self.origins[i] = None;
            }
            if t.mode != RunMode::Run || t.enabled_slots & (1 << i) == 0 {
                self.targets[i] = t.slots[i].measured;
                self.moving[i] = false;
            }
        }
    }
    pub fn capture_origin(&mut self, i: usize, t: Option<&Telemetry>) -> bool {
        let Some(t) = t else {
            return false;
        };
        let a = &self.profile.axes[i];
        let measured = t.slots[i].measured;
        if !measured.is_finite() || t.stale_slots & (1 << i) != 0 {
            return false;
        }
        self.origins[i] = Some(measured - a.origin_position * a.native_per_unit);
        self.targets[i] = measured;
        true
    }
    pub fn axis_position(&self, name: &str, t: Option<&Telemetry>) -> Option<f32> {
        let i = self.profile.axes.iter().position(|a| a.name == name)?;
        let t = t?;
        if t.stale_slots & (1 << i) != 0 {
            return None;
        }
        Some((t.slots[i].measured - self.origins[i]?) / self.profile.axes[i].native_per_unit)
    }
    pub fn hold_at_measured(&mut self, t: Option<&Telemetry>) {
        self.reset_jog();
        if let Some(t) = t {
            self.targets = t.slots.map(|s| s.measured);
        }
    }
    fn limited_velocity(&self, i: usize, velocity: f32, t: &Telemetry) -> f32 {
        let a = &self.profile.axes[i];
        let Some(origin) = self.origins[i] else {
            return 0.0;
        };
        if t.stale_slots & (1 << i) != 0 {
            return 0.0;
        }
        let mut lo = a.minimum;
        let mut hi = a.maximum;
        if let Some(l) = a.limit {
            if t.contacts
                .is_none_or(|contacts| l.reached(contacts) && velocity * l.direction > 0.0)
            {
                return 0.0;
            }
            if l.direction > 0.0 {
                hi = hi.min(a.origin_position);
            } else {
                lo = lo.max(a.origin_position);
            }
        }
        let position = (t.slots[i].measured - origin) / a.native_per_unit;
        let distance = if velocity > 0.0 {
            hi - position
        } else {
            position - lo
        };
        let acceleration = a.speed_per_second / a.jog_ramp_seconds;
        // 既存機体と同じ0.2秒の通信・指令先行量を含めて制動する。
        let delay_velocity = acceleration * 0.2;
        let cap = ((delay_velocity * delay_velocity + 2.0 * acceleration * distance.max(0.0))
            .sqrt()
            - delay_velocity)
            .max(0.0);
        velocity.signum() * velocity.abs().min(cap)
    }
    pub fn ramped_jog_lines(
        &mut self,
        input: &ControllerState,
        t: &Telemetry,
        slow: bool,
        now: Instant,
    ) -> Vec<String> {
        let dt = self.tick.replace(now).map_or(0.0, |old| {
            now.saturating_duration_since(old).as_secs_f32().min(0.05)
        });
        let mut lines = Vec::with_capacity(3);
        for i in 0..3 {
            let a = &self.profile.axes[i];
            let raw = input.machine_axis(a.input_axis).unwrap_or(0.0);
            let raw = if raw.is_finite() && raw.abs() >= 0.1 {
                raw.clamp(-1.0, 1.0)
            } else {
                0.0
            };
            let scale = if slow {
                self.profile.slow_speed_percent / 100.0
            } else {
                1.0
            };
            let desired =
                self.limited_velocity(i, raw * a.input_sign * a.speed_per_second * scale, t);
            let delta = a.speed_per_second / a.jog_ramp_seconds * dt;
            let ramped = self.velocity[i] + (desired - self.velocity[i]).clamp(-delta, delta);
            let velocity = self.limited_velocity(i, ramped, t);
            self.velocity[i] = velocity;
            if velocity != 0.0 {
                self.moving[i] = true;
                lines.push(format!("JOG {i} {:.5}", velocity * a.native_per_unit));
            } else {
                if self.moving[i] {
                    self.targets[i] = t.slots[i].measured;
                    self.moving[i] = false;
                }
                lines.push(format!("TARGET {i} {:.5}", self.targets[i]));
            }
        }
        lines
    }
}
impl Rotation {
    pub fn rotation_trim_velocity(&self, input: &ControllerState) -> f32 {
        let value = input.axes[2];
        if value.abs() <= 0.1 {
            return 0.0;
        }
        let speed = if self.speed_position_per_second > 0.0 {
            15.0_f32.min(self.speed_position_per_second / self.counts_per_deg.abs())
        } else {
            15.0
        };
        value.signum() * (value.abs().min(1.0) - 0.1) / 0.9 * self.input_sign * speed
    }
    pub fn rotation_count(&self, field: f32, theta: f32) -> Result<i16> {
        let count = (self.zero0_count + self.counts_per_deg * (field - theta)).round();
        ensure!(
            count.is_finite() && (-28672.0..=28672.0).contains(&count),
            "EE回転目標がSTS指令範囲外です"
        );
        Ok(count as i16)
    }
    pub fn rotation_command(&self, count: i16, initial: bool) -> Vec<String> {
        let mut lines = vec![
            serial_svmd::Command::Target {
                id: self.id,
                position: count,
                speed: self.speed_position_per_second.round() as u16,
                acceleration: self.acceleration,
            }
            .to_cctl_line(),
        ];
        if initial {
            lines.extend([
                serial_svmd::Command::Enable {
                    id: self.id,
                    enabled: true,
                }
                .to_cctl_line(),
                serial_svmd::Command::Run.to_cctl_line(),
            ]);
        }
        lines
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{config::Config, protocol::telemetry::parse_telemetry};
    fn fixture() -> (MachineController, Telemetry, Instant) {
        let c = Config::parse(include_str!("../../machine.toml")).unwrap();
        let mut m = MachineController::new(c.machine);
        let t = parse_telemetry(
            "STATE t=100 mode=RUN en=7 a0=0/0 a1=0/0 a2=0/0 err=00,00,00 sw=0 stale=0 can=3 hold=0",
        )
        .unwrap();
        for i in 0..3 {
            assert!(m.capture_origin(i, Some(&t)));
        }
        m.hold_at_measured(Some(&t));
        (m, t, Instant::now())
    }
    #[test]
    fn origins_use_signed_scales_and_configured_coordinates() {
        let (m, mut t, _) = fixture();
        assert_eq!(m.axis_position("r", Some(&t)), Some(570.0));
        t.slots[0].measured = 4.0;
        t.slots[1].measured = 132.02206 * 10.0;
        t.slots[2].measured = -96.016 * 10.0;
        assert!((m.axis_position("r", Some(&t)).unwrap() - 470.0).abs() < 0.001);
        assert!((m.axis_position("theta", Some(&t)).unwrap() - 10.0).abs() < 0.001);
        assert!((m.axis_position("z", Some(&t)).unwrap() - 10.0).abs() < 0.001);
    }
    #[test]
    fn endpoint_and_contact_block_only_outward_motion() {
        let (m, mut t, _) = fixture();
        assert_eq!(m.limited_velocity(0, 600.0, &t), 0.0);
        assert!(m.limited_velocity(0, -600.0, &t) < 0.0);
        assert_eq!(m.limited_velocity(2, -200.0, &t), 0.0);
        assert!(m.limited_velocity(2, 200.0, &t) > 0.0);
        t.slots[0].measured = 4.0;
        t.contacts = Some(1);
        assert_eq!(m.limited_velocity(0, 100.0, &t), 0.0);
        assert!(m.limited_velocity(0, -100.0, &t) < 0.0);
        t.contacts = None;
        assert_eq!(m.limited_velocity(0, -100.0, &t), 0.0);
    }
    #[test]
    fn acceleration_and_braking_stay_within_limits() {
        let (mut m, mut t, now) = fixture();
        let mut input = ControllerState::default();
        input.axes[1] = -1.0;
        m.ramped_jog_lines(&input, &t, false, now);
        let lines = m.ramped_jog_lines(
            &input,
            &t,
            false,
            now + std::time::Duration::from_millis(20),
        );
        assert_eq!(lines[0], "JOG 0 2.40000"); // 60 mm/s × -0.04, sign inverted by the stick.
        let acceleration = 600.0 / 0.2;
        for distance in [0.0_f32, 0.01, 1.0, 10.0, 100.0] {
            t.slots[0].measured = distance * 0.04;
            let speed = m.limited_velocity(0, 600.0, &t);
            assert!(speed * 0.2 + speed * speed / (2.0 * acceleration) <= distance + 0.001);
        }
    }
    #[test]
    fn stopped_target_does_not_follow_external_displacement() {
        let (mut m, mut t, now) = fixture();
        t.slots[2].measured = -100.0;
        m.observe(&t);
        let lines = m.ramped_jog_lines(&ControllerState::default(), &t, false, now);
        assert_eq!(lines[2], "TARGET 2 0.00000");
        t.stale_slots = 4;
        m.observe(&t);
        assert!(!m.origins_ready());
    }
    #[test]
    fn rotation_compensates_theta_and_rejects_overflow() {
        let r = Config::parse(include_str!("../../machine.toml"))
            .unwrap()
            .machine
            .rotation;
        assert_eq!(r.rotation_count(10.0, 10.0).unwrap(), 1024);
        assert!(r.rotation_count(0.0, 10.0).unwrap() < 1024);
        assert!(r.rotation_count(10000.0, 0.0).is_err());
    }
}
