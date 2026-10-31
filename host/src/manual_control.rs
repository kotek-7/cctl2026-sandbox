//! Single-owner control state. No threads, shared state, sockets or request dispatch.
use crate::{
    application::settings::Settings,
    input::ControllerState,
    machine::{MachineController, MachineProfile, ee},
    protocol::{
        device::parse_device_info,
        serial_svmd, svmd,
        telemetry::{RunMode, Telemetry, parse_telemetry},
    },
};
use anyhow::{Context, Result, bail, ensure};
use serde::Deserialize;
use std::time::{Duration, Instant};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub machine: MachineProfile,
    pub grip_open: [f32; 3],
    pub grip_closed: [f32; 3],
    pub grip_handoff_open: [f32; 3],
}
impl Config {
    pub fn parse(text: &str) -> Result<Self> {
        let c: Self = toml::from_str(text)?;
        c.machine.validate()?;
        ensure!(c.machine.axes.len() == 3, "r/theta/zの3軸が必要です");
        for (i, name) in ["r", "theta", "z"].iter().enumerate() {
            ensure!(
                c.machine.axes[i].name == *name && c.machine.axes[i].slot == i as u8,
                "軸順はr/theta/z、slotは0/1/2です"
            );
        }
        ensure!(
            c.machine.bonus.is_none() && c.machine.dc_motors.is_empty(),
            "ボーナス/DC機構は最小版の対象外です"
        );
        ensure!(
            c.machine.pwm_servos.len() == 4,
            "PWMは畳み1本・把持3本が必要です"
        );
        for (channel, name) in ["ee_fold", "ee_grip_1", "ee_grip_2", "ee_grip_3"]
            .iter()
            .enumerate()
        {
            let s = &c.machine.pwm_servos[channel];
            ensure!(
                s.name == *name && s.channel == channel as u8 && s.enabled,
                "PWMの順番/割当/有効状態を確認してください"
            );
            if channel > 0 {
                for pose in [c.grip_open, c.grip_closed, c.grip_handoff_open] {
                    ensure!(
                        pose[channel - 1].is_finite()
                            && (f32::from(s.minimum_us)..=f32::from(s.maximum_us))
                                .contains(&pose[channel - 1]),
                        "把持設定が範囲外です"
                    );
                }
            }
        }
        let servos = &c
            .machine
            .serial_svmd
            .as_ref()
            .context("EE回転設定が必要です")?
            .servos;
        ensure!(
            servos.len() == 1 && servos[0].name == "ee_rotation" && servos[0].enabled,
            "STSはEE回転1本のみ対応します"
        );
        ensure!(
            c.machine.axes[0].limit.is_some() && c.machine.axes[2].limit.is_some(),
            "r/zの原点接点が必要です"
        );
        Ok(c)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mode {
    Setup,
    Stopped,
    AwaitRun,
    Running,
    Hold,
}

pub struct Control {
    pub config: Config,
    pub mode: Mode,
    pub emergency: bool,
    pub message: String,
    pub telemetry: Option<Telemetry>,
    model: MachineController,
    settings: Settings,
    device: bool,
    seen: Option<Instant>,
    started: Instant,
    run_sent: Instant,
    last_poll: Instant,
    last_step: Instant,
    previous: [u8; 17],
    neutral_seen: bool,
    home_since: Option<Instant>,
    home_released: bool,
    pwm: [Option<(svmd::State, Instant)>; 4],
    rotation: Option<(serial_svmd::ServoState, Instant)>,
    pwm_current: [f32; 4],
    pwm_goal: [f32; 4],
    pwm_velocity: [f32; 4],
    field: f32,
    pub out: Vec<String>,
}

impl Control {
    pub fn new(config: Config, now: Instant) -> Self {
        Self {
            model: MachineController::new(config.machine.clone()),
            settings: Settings::new(&config.machine),
            config,
            mode: Mode::Setup,
            emergency: false,
            message: "接続・設定照合待ち".into(),
            telemetry: None,
            device: false,
            seen: None,
            started: now,
            run_sent: now,
            last_poll: now - Duration::from_secs(1),
            last_step: now,
            previous: [0; 17],
            neutral_seen: false,
            home_since: None,
            home_released: false,
            pwm: [None; 4],
            rotation: None,
            pwm_current: [0.; 4],
            pwm_goal: [0.; 4],
            pwm_velocity: [0.; 4],
            field: 0.,
            out: vec![
                "HELLO 1".into(),
                "STOP".into(),
                svmd::Command::Stop.to_cctl_line(),
                serial_svmd::Command::Stop.to_cctl_line(),
                "ENABLE 7 0".into(),
                "SAFE".into(),
            ],
        }
    }
    pub fn ready(&self) -> bool {
        self.mode != Mode::Setup && self.settings.ready()
    }
    pub fn origins_ready(&self) -> bool {
        self.model
            .origin_states(self.telemetry.as_ref())
            .iter()
            .all(|o| o.captured)
    }
    fn rotation_axis(&self) -> ee::Axis {
        ee::axes(&self.config.machine)
            .into_iter()
            .find(|a| a.name == "ee_rotation")
            .expect("validated rotation axis")
    }
    fn health(&self, now: Instant) -> Result<()> {
        ensure!(
            self.seen
                .is_some_and(|t| now.saturating_duration_since(t) <= Duration::from_millis(200)),
            "CCTL応答が途絶えました"
        );
        let t = self.telemetry.as_ref().context("CCTL位置応答待ち")?;
        ensure!(
            t.error_bits == [0; 3] && t.stale_slots == 0 && t.buses & 3 == 3,
            "モータ/CAN異常です"
        );
        ensure!(
            t.contacts.is_some() && t.held_slots.is_some(),
            "接点/保持対応のFWが必要です"
        );
        Ok(())
    }
    fn ee_health(&self, now: Instant) -> Result<()> {
        for state in &self.pwm {
            let (state, seen) = state.context("PWM基板の応答待ち")?;
            ensure!(
                state.result == 0
                    && now.saturating_duration_since(seen) < Duration::from_millis(600),
                "PWM基板の応答異常です"
            );
        }
        let (state, seen) = self.rotation.context("STS実測位置の応答待ち")?;
        ensure!(
            state.error == 0
                && state.absolute_position
                && now.saturating_duration_since(seen) < Duration::from_millis(600),
            "STS位置が無効/応答途絶です"
        );
        Ok(())
    }
    pub fn receive(&mut self, line: &str, now: Instant) -> Result<()> {
        if let Some(d) = parse_device_info(line) {
            ensure!(
                d.board == "cctl"
                    && d.protocol == 1
                    && d.slots == 3
                    && d.jog
                    && d.motor_layout == "el05,m3508,m3508",
                "CCTLの種類/版が一致しません"
            );
            self.device = true;
        }
        if line.starts_with("ERR ") {
            bail!("CCTL拒否: {line}");
        }
        if let Some(t) = parse_telemetry(line) {
            if let Some(old) = &self.telemetry {
                ensure!(
                    t.uptime_ms >= old.uptime_ms || old.uptime_ms > u32::MAX - 1000,
                    "CCTLが再起動しました"
                );
            }
            self.model.observe(&t);
            self.telemetry = Some(t);
            self.seen = Some(now);
        }
        self.settings.receive(line)?;
        if let Some(s) = svmd::parse_state(line) {
            self.pwm[s.channel as usize] = Some((s, now));
        }
        if let Some(s) = serial_svmd::parse_state(line) {
            if s.id == self.config.machine.serial_svmd.as_ref().unwrap().servos[0].id {
                self.rotation = Some((s, now));
            }
        }
        if let Some((_, detail)) = serial_svmd::parse_diagnostic(line) {
            bail!("STS: {detail}");
        }
        Ok(())
    }
    /// STOP is torque release. HOLD is the separate, explicit normal stop.
    pub fn release(&mut self, emergency: bool) {
        self.out.clear(); // Cancel any motion accumulated earlier in this cycle.
        self.out.extend([
            "STOP".into(),
            svmd::Command::Stop.to_cctl_line(),
            serial_svmd::Command::Stop.to_cctl_line(),
        ]);
        self.mode = if self.settings.ready() {
            Mode::Stopped
        } else {
            Mode::Setup
        };
        self.emergency |= emergency;
        self.model.reset_jog();
        self.model.invalidate_origins();
        self.neutral_seen = false;
        self.home_since = None;
        self.home_released = false;
        self.message = if emergency {
            "緊停・全出力解除"
        } else {
            "全出力解除・原点再確認"
        }
        .into();
    }
    pub fn hold(&mut self, now: Instant) -> Result<()> {
        ensure!(
            !self.emergency && self.ready(),
            "緊停中/設定待ちは保持できません"
        );
        self.health(now)?;
        self.out.clear();
        self.out.extend([
            "HOLD 7".into(),
            svmd::Command::Stop.to_cctl_line(),
            serial_svmd::Command::Stop.to_cctl_line(),
        ]);
        self.mode = Mode::Hold;
        self.model.reset_jog();
        self.neutral_seen = false;
        self.home_since = None;
        self.message = "停止・3軸保持（EEは出力解除）".into();
        Ok(())
    }
    pub fn reset_emergency(&mut self) {
        self.emergency = false;
        if self.mode == Mode::Setup {
            self.out.extend(["ENABLE 7 0".into(), "SAFE".into()]);
        }
        self.message = "緊停解除・出力停止。原点を再設定してください".into();
    }

    fn capture_origins(&mut self) -> Result<()> {
        ensure!(
            self.mode == Mode::Stopped,
            "出力解除中に原点を合わせてください"
        );
        let t = self.telemetry.as_ref().context("応答待ち")?;
        for i in [0, 2] {
            ensure!(
                self.config.machine.axes[i]
                    .limit
                    .unwrap()
                    .reached(t.contacts.context("接点応答待ち")?),
                "r前端とz下端の両接点へ手動で合わせてください"
            );
        }
        for i in 0..3 {
            ensure!(
                self.model.capture_origin(i, self.telemetry.as_ref()),
                "原点採用に失敗しました"
            );
        }
        self.message = "原点設定済み。中立に戻してOptionsで運転".into();
        Ok(())
    }
    fn start(&mut self, now: Instant, input: &ControllerState) -> Result<()> {
        ensure!(
            self.ready() && !self.emergency && matches!(self.mode, Mode::Stopped | Mode::Hold),
            "設定待ち/緊停/運転中です"
        );
        self.health(now)?;
        self.ee_health(now)?;
        ensure!(self.origins_ready(), "原点を設定してください");
        ensure!(
            self.neutral_seen
                && input.axes.iter().all(|v| v.is_finite() && v.abs() < 0.1)
                && input.buttons[11..15].iter().all(|v| *v == 0),
            "スティックと十字キーを中立にしてください"
        );
        for i in 0..4 {
            let pulse = f32::from(self.pwm[i].unwrap().0.pulse_us);
            let s = &self.config.machine.pwm_servos[i];
            ensure!(
                (f32::from(s.minimum_us)..=f32::from(s.maximum_us)).contains(&pulse),
                "PWM保持値が可動範囲外です。元の調整ツールで確認してください"
            );
            self.pwm_current[i] = pulse;
            self.pwm_goal[i] = pulse;
            self.pwm_velocity[i] = 0.;
        }
        let axis = self.rotation_axis();
        let theta = self
            .model
            .axis_position("theta", self.telemetry.as_ref())
            .context("θ原点がありません")?;
        self.field = theta
            + (self.rotation.unwrap().0.position as f32 - axis.zero0_count) / axis.counts_per_deg;
        self.model.hold_at_measured(self.telemetry.as_ref());
        self.out.extend(["ENABLE 7 1".into(), "RUN".into()]);
        self.mode = Mode::AwaitRun;
        self.run_sent = now;
        self.message = "RUN応答待ち".into();
        Ok(())
    }
    pub fn step(&mut self, now: Instant, input: Option<&ControllerState>) -> Result<()> {
        ensure!(
            !matches!(self.mode, Mode::Running | Mode::AwaitRun | Mode::Hold)
                || now.saturating_duration_since(self.last_step) < Duration::from_millis(200),
            "制御ループが遅延しました"
        );
        let dt = now
            .saturating_duration_since(self.last_step)
            .as_secs_f32()
            .min(0.05);
        self.last_step = now;
        if self.seen.is_none() || !self.device {
            ensure!(
                now.duration_since(self.started) < Duration::from_secs(3),
                "CCTLが応答しません"
            );
        } else {
            self.health(now)?;
        }
        if self.device && self.mode == Mode::Setup && !self.emergency {
            ensure!(
                now.duration_since(self.started) < Duration::from_secs(10),
                "設定準備が完了しません"
            );
            if self
                .telemetry
                .as_ref()
                .is_none_or(|t| t.mode != RunMode::Safe || t.enabled_slots != 0)
            {
                return Ok(());
            }
            if let Some(cmd) = self.settings.poll_command()? {
                self.out.push(cmd);
            }
            if self.settings.ready() {
                // PWM has no position feedback. Set the explicit startup posture while disabled.
                for servo in &self.config.machine.pwm_servos {
                    self.out.push(
                        svmd::Command::Set {
                            channel: servo.channel,
                            pulse_us: servo.initial_us,
                        }
                        .to_cctl_line(),
                    );
                }
                self.mode = Mode::Stopped;
                self.message = "設定照合済み・原点未設定".into();
            }
        }
        if now.duration_since(self.last_poll) >= Duration::from_millis(100) {
            self.last_poll = now;
            if !self.device {
                self.out.push("HELLO 1".into());
            }
            for channel in 0..4 {
                self.out.push(if self.mode == Mode::Running {
                    // Heartbeat selects the channel whose retained pulse is returned.
                    format!("CAN 2 768 0103{channel:02X}0000000000")
                } else {
                    svmd::Command::Enable {
                        channel,
                        enabled: false,
                    }
                    .to_cctl_line()
                });
            }
            let id = self.config.machine.serial_svmd.as_ref().unwrap().servos[0].id;
            self.out
                .push(serial_svmd::Command::Read { id }.to_cctl_line());
        }
        if self.ready() {
            self.out.push("HEARTBEAT".into());
        }
        let Some(input) = input else {
            self.previous = [0; 17];
            self.neutral_seen = false;
            self.home_since = None;
            self.home_released = false;
            if matches!(self.mode, Mode::Running | Mode::AwaitRun | Mode::Hold) {
                self.release(false);
            }
            return Ok(());
        };
        ensure!(
            input.axes.iter().all(|v| v.is_finite() && v.abs() <= 1.01),
            "パッド入力が不正です"
        );
        let old = self.previous;
        self.previous = input.buttons;
        if input.buttons[5] != 0 {
            if old[5] == 0 && self.ready() {
                self.hold(now)?;
            }
            return Ok(());
        }
        if self.emergency {
            return Ok(());
        }
        let neutral = input.axes.iter().all(|v| v.abs() < 0.1)
            && input.buttons[11..15].iter().all(|v| *v == 0);
        if neutral && input.buttons.iter().all(|v| *v == 0) {
            self.neutral_seen = true;
            self.home_released = true;
        }
        if self.mode == Mode::Stopped
            && self.ready()
            && input.buttons[4] != 0
            && neutral
            && self.home_released
        {
            let since = *self.home_since.get_or_insert(now);
            if now.duration_since(since) >= Duration::from_secs(1) {
                self.home_since = None;
                self.home_released = false;
                if let Err(e) = self.capture_origins() {
                    self.message = e.to_string();
                }
            }
        } else {
            self.home_since = None;
        }
        if input.buttons[6] != 0 && old[6] == 0 {
            if let Err(e) = self.start(now, input) {
                self.message = e.to_string();
            }
        }
        if self.mode == Mode::AwaitRun {
            ensure!(
                now.duration_since(self.run_sent) < Duration::from_millis(250),
                "RUN応答がありません"
            );
            let t = self.telemetry.as_ref().unwrap();
            if t.mode == RunMode::Run && t.enabled_slots == 7 && self.seen.unwrap() >= self.run_sent
            {
                self.mode = Mode::Running;
                self.message = "手動運転".into();
                let axis = self.rotation_axis();
                let theta = self
                    .model
                    .axis_position("theta", self.telemetry.as_ref())
                    .unwrap();
                self.out
                    .extend(axis.rotation_command(axis.rotation_count(self.field, theta)?, true));
                for channel in 0..4 {
                    self.out.push(
                        svmd::Command::Set {
                            channel,
                            pulse_us: self.pwm_current[channel as usize].round() as u16,
                        }
                        .to_cctl_line(),
                    );
                    self.out.push(
                        svmd::Command::Enable {
                            channel,
                            enabled: true,
                        }
                        .to_cctl_line(),
                    );
                }
            }
        }
        if self.mode != Mode::Running {
            return Ok(());
        }
        self.ee_health(now)?;
        if now.duration_since(self.run_sent) > Duration::from_millis(250) {
            for (channel, state) in self.pwm.iter().enumerate() {
                ensure!(
                    state.unwrap().0.enabled_mask & (1 << channel) != 0,
                    "PWM出力が無効になりました"
                );
            }
            ensure!(
                self.rotation.unwrap().0.enabled,
                "STS出力が無効になりました"
            );
        }
        ensure!(self.origins_ready(), "原点を失いました");
        let t = self.telemetry.as_ref().unwrap();
        ensure!(
            t.mode == RunMode::Run && t.enabled_slots == 7,
            "機体が運転状態を失いました"
        );
        let slow = input.buttons[9] != 0;
        self.out
            .extend(self.model.ramped_jog_lines(input, t, slow, now));
        let scale = if slow {
            self.config.machine.slow_speed_percent / 100.
        } else {
            1.
        };
        if input.buttons[11] != input.buttons[12] {
            let s = &self.config.machine.pwm_servos[0];
            let positive = (input.buttons[11] != 0) == (s.input_sign > 0.);
            self.pwm_goal[0] = f32::from(if positive { s.maximum_us } else { s.minimum_us });
        } else {
            self.pwm_goal[0] = self.pwm_current[0];
            self.pwm_velocity[0] = 0.;
        }
        let grip = if input.buttons[13] != 0 && old[13] == 0 {
            Some(self.config.grip_open)
        } else if input.buttons[14] != 0 && old[14] == 0 {
            Some(self.config.grip_closed)
        } else if input.buttons[1] != 0 && old[1] == 0 {
            Some(self.config.grip_handoff_open)
        } else {
            None
        };
        if let Some(grip) = grip {
            self.pwm_goal[1..].copy_from_slice(&grip);
        }
        for i in 0..4 {
            let s = &self.config.machine.pwm_servos[i];
            let (next, velocity) = pwm_step(
                self.pwm_current[i],
                self.pwm_goal[i],
                self.pwm_velocity[i],
                s.speed_us_per_second * scale,
                s.acceleration_us_per_second2 * scale,
                dt,
            );
            self.pwm_current[i] = next;
            self.pwm_velocity[i] = velocity;
            self.out.push(
                svmd::Command::Set {
                    channel: i as u8,
                    pulse_us: next.round() as u16,
                }
                .to_cctl_line(),
            );
        }
        let axis = self.rotation_axis();
        self.field += axis.rotation_trim_velocity(input) * scale * dt;
        let theta = self
            .model
            .axis_position("theta", self.telemetry.as_ref())
            .unwrap();
        self.out
            .extend(axis.rotation_command(axis.rotation_count(self.field, theta)?, false));
        // SerialSVMD heartbeat; RUN is only sent when starting.
        self.out.push("CAN 2 800 0105000000000000".into());
        Ok(())
    }
}

fn pwm_step(
    position: f32,
    target: f32,
    velocity: f32,
    max_speed: f32,
    acceleration: f32,
    dt: f32,
) -> (f32, f32) {
    let remaining = target - position;
    if remaining.abs() < 0.01 {
        return (target, 0.);
    }
    let desired = remaining.signum()
        * max_speed.min(if acceleration > 0. {
            (2. * acceleration * remaining.abs()).sqrt()
        } else {
            max_speed
        });
    let velocity = if acceleration > 0. {
        velocity + (desired - velocity).clamp(-acceleration * dt, acceleration * dt)
    } else {
        desired
    };
    let movement = velocity * dt;
    if movement.signum() == remaining.signum() && movement.abs() >= remaining.abs() {
        (target, 0.)
    } else {
        (position + movement, velocity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::simulator::Simulator;

    struct Rig {
        control: Control,
        sim: Simulator,
        now: Instant,
        input: ControllerState,
    }
    impl Rig {
        fn new() -> Self {
            let now = Instant::now();
            let config = Config::parse(include_str!("../../machine.toml")).unwrap();
            let mut rig = Self {
                control: Control::new(config, now),
                sim: Simulator::new(),
                now,
                input: ControllerState::default(),
            };
            for _ in 0..200 {
                rig.cycle().unwrap();
                if rig.control.ready() {
                    return rig;
                }
            }
            panic!("configuration did not finish");
        }
        fn cycle(&mut self) -> Result<Vec<String>> {
            for line in self.control.out.drain(..) {
                self.sim.write(&line)?;
            }
            self.now += Duration::from_millis(20);
            for line in self.sim.read() {
                self.control.receive(&line, self.now)?;
            }
            self.control.step(self.now, Some(&self.input))?;
            Ok(self.control.out.clone())
        }
        fn home(&mut self) {
            self.input.buttons[4] = 1;
            for _ in 0..52 {
                self.cycle().unwrap();
            }
            assert!(self.control.origins_ready(), "{}", self.control.message);
            self.input.buttons[4] = 0;
            self.cycle().unwrap();
        }
        fn run(&mut self) -> Vec<String> {
            self.home();
            self.input.buttons[6] = 1;
            self.cycle().unwrap();
            assert_eq!(
                self.control.mode,
                Mode::AwaitRun,
                "{}",
                self.control.message
            );
            self.input.buttons[6] = 0;
            let sent = self.cycle().unwrap();
            assert_eq!(self.control.mode, Mode::Running);
            sent
        }
    }
    #[test]
    fn setup_does_not_enable_outputs_and_requires_origins() {
        let mut r = Rig::new();
        assert_eq!(r.control.mode, Mode::Stopped);
        assert!(!r.control.origins_ready());
        r.input.buttons[6] = 1;
        let sent = r.cycle().unwrap();
        assert!(!sent.iter().any(|s| s == "RUN"));
        assert_eq!(r.control.mode, Mode::Stopped);
    }
    #[test]
    fn run_enables_pwm_after_setting_retained_pulses() {
        let mut r = Rig::new();
        let sent = r.run();
        for channel in 0..4 {
            let enable = svmd::Command::Enable {
                channel,
                enabled: true,
            }
            .to_cctl_line();
            let index = sent.iter().position(|s| s == &enable).unwrap();
            assert!(sent[index - 1].starts_with(&format!("CAN 2 768 0101{channel:02X}")));
        }
        assert!(sent.contains(&serial_svmd::Command::Run.to_cctl_line()));
    }
    #[test]
    fn disconnect_and_emergency_cancel_queued_motion_and_origins() {
        let mut r = Rig::new();
        r.run();
        r.control
            .step(r.now + Duration::from_millis(20), None)
            .unwrap();
        assert_eq!(r.control.out[0], "STOP");
        assert_eq!(r.control.out.len(), 3);
        assert!(!r.control.origins_ready());
        r.control.release(true);
        assert!(r.control.hold(r.now).is_err());
        r.control.reset_emergency();
        assert_eq!(r.control.mode, Mode::Stopped);
    }
    #[test]
    fn non_neutral_start_and_stale_feedback_are_rejected() {
        let mut r = Rig::new();
        r.home();
        r.input.axes[0] = 0.5;
        r.input.buttons[6] = 1;
        r.cycle().unwrap();
        assert_eq!(r.control.mode, Mode::Stopped);
        r.sim.fault("stale_slot0").unwrap();
        assert!(r.cycle().is_err());
    }
    #[test]
    fn hold_resumes_only_after_buttons_are_released() {
        let mut r = Rig::new();
        r.run();
        r.input.buttons[5] = 1;
        r.cycle().unwrap();
        assert_eq!(r.control.mode, Mode::Hold);
        r.input.buttons[5] = 0;
        r.input.buttons[6] = 1;
        r.cycle().unwrap();
        assert_eq!(r.control.mode, Mode::Hold);
        r.input.buttons[6] = 0;
        r.cycle().unwrap();
        r.input.buttons[6] = 1;
        r.cycle().unwrap();
        r.input.buttons[6] = 0;
        r.cycle().unwrap();
        assert_eq!(r.control.mode, Mode::Running);
    }
    #[test]
    fn stalled_loop_and_motor_fault_stop_control() {
        let mut r = Rig::new();
        r.run();
        assert!(
            r.control
                .step(r.now + Duration::from_millis(300), Some(&r.input))
                .is_err()
        );
        let mut r = Rig::new();
        r.run();
        r.sim.fault("error_slot0").unwrap();
        assert!(r.cycle().is_err());
    }
    #[test]
    fn sustained_run_checks_peripheral_enable_and_feedback_timeout() {
        let mut r = Rig::new();
        r.run();
        for _ in 0..50 {
            r.cycle().unwrap();
        }
        r.sim
            .write(
                &svmd::Command::Enable {
                    channel: 2,
                    enabled: false,
                }
                .to_cctl_line(),
            )
            .unwrap();
        assert!(r.cycle().unwrap_err().to_string().contains("PWM"));
        let mut r = Rig::new();
        r.run();
        r.sim.fault("silence_rx").unwrap();
        r.control.out.clear();
        assert!(
            r.control
                .step(r.now + Duration::from_millis(201), Some(&r.input))
                .is_err()
        );
    }
    #[test]
    fn origin_requires_both_contacts_and_run_requires_acknowledgement() {
        let mut r = Rig::new();
        r.control.telemetry.as_mut().unwrap().contacts = Some(1);
        assert!(r.control.capture_origins().is_err());
        r.home();
        r.input.buttons[6] = 1;
        r.cycle().unwrap();
        r.control.out.clear(); // The RUN command never reaches the board.
        for _ in 0..12 {
            r.cycle().unwrap();
        }
        assert!(r.cycle().is_err());
    }
    #[test]
    fn invalid_pose_is_rejected_before_opening_hardware() {
        let text = include_str!("../../machine.toml").replace(
            "grip_open = [1500.0, 1500.0, 1500.0]",
            "grip_open = [9999.0, 1500.0, 1500.0]",
        );
        assert!(Config::parse(&text).is_err());
    }
}
