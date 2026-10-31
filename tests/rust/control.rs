use super::*;
use std::{
    io::{ErrorKind, Read, Write},
    sync::Mutex,
    thread,
};
mod simulator;

struct Board {
    simulator: simulator::Simulator,
    commands: Vec<String>,
    reject_run: bool,
    contacts: u8,
    fault: Option<&'static str>,
}
struct Rig {
    port: serialport::TTYPort,
    board: Arc<Mutex<Board>>,
    exit: Arc<AtomicBool>,
    thread: Option<thread::JoinHandle<()>>,
}
impl Rig {
    fn new() -> Self {
        let (mut peer, mut port) = serialport::TTYPort::pair().unwrap();
        peer.set_timeout(Duration::from_millis(2)).unwrap();
        port.set_timeout(Duration::from_millis(5)).unwrap();
        let board = Arc::new(Mutex::new(Board {
            simulator: simulator::Simulator::new(),
            commands: vec![],
            reject_run: false,
            contacts: 7,
            fault: None,
        }));
        let exit = Arc::new(AtomicBool::new(false));
        let shared = board.clone();
        let done = exit.clone();
        let worker = thread::spawn(move || {
            let mut buffer = String::new();
            while !done.load(Ordering::Relaxed) {
                let mut bytes = [0; 4096];
                match peer.read(&mut bytes) {
                    Ok(n) => buffer.push_str(std::str::from_utf8(&bytes[..n]).unwrap()),
                    Err(e) if matches!(e.kind(), ErrorKind::TimedOut | ErrorKind::WouldBlock) => {}
                    Err(_) => break,
                }
                let mut board = shared.lock().unwrap();
                while let Some(end) = buffer.find('\n') {
                    let line: String = buffer.drain(..=end).collect();
                    let line = line.trim();
                    board.commands.push(line.to_owned());
                    if line != "RUN" || !board.reject_run {
                        board.simulator.write(line).unwrap();
                    }
                }
                if let Some(fault) = board.fault.take() {
                    board.simulator.fault(fault).unwrap();
                }
                for line in board.simulator.read() {
                    let line = line.replace("sw=7", &format!("sw={}", board.contacts));
                    if writeln!(peer, "{line}").is_err() {
                        return;
                    }
                }
                drop(board);
                thread::sleep(Duration::from_millis(5));
            }
        });
        Self {
            port,
            board,
            exit,
            thread: Some(worker),
        }
    }
    fn script(&mut self, inputs: Vec<Pad>) -> String {
        let config = config();
        let mut inputs = inputs.into_iter();
        let result = session(&mut self.port, &config, &AtomicBool::new(false), || {
            inputs.next().context("試験終了")
        });
        // Stop commands travel through the actual TTYPort before inspecting the peer.
        let deadline = Instant::now() + Duration::from_secs(1);
        while Instant::now() < deadline {
            if self.board.lock().unwrap().commands.ends_with(&[
                "STOP".into(),
                STOP_PWM.into(),
                STOP_STS.into(),
            ]) {
                break;
            }
            thread::sleep(Duration::from_millis(5));
        }
        format!("{:#}", result.unwrap_err())
    }
    fn commands(&self) -> Vec<String> {
        self.board.lock().unwrap().commands.clone()
    }
}
impl Drop for Rig {
    fn drop(&mut self) {
        self.exit.store(true, Ordering::Relaxed);
        self.thread.take().unwrap().join().unwrap();
    }
}
fn config() -> Config {
    Config::parse(include_str!("../../machine.toml")).unwrap()
}
fn neutral() -> Pad {
    Pad {
        released: true,
        ..Pad::default()
    }
}
fn start_script() -> Vec<Pad> {
    let mut inputs = vec![neutral()];
    inputs.extend(vec![
        Pad {
            create: true,
            ..Pad::default()
        };
        60
    ]);
    inputs.push(neutral());
    inputs.push(Pad {
        start: true,
        ..Pad::default()
    });
    inputs.extend(vec![neutral(); 20]);
    inputs
}

#[test]
fn startup_does_not_enable_outputs_and_start_needs_an_origin() {
    let mut rig = Rig::new();
    let result = rig.script(vec![
        neutral(),
        Pad {
            start: true,
            ..Pad::default()
        },
        neutral(),
    ]);
    assert!(result.contains("試験終了"));
    assert!(!rig.commands().iter().any(|s| s == "RUN"));
    assert!(
        rig.commands()
            .iter()
            .any(|s| s.starts_with("PARAM 9 24.00000"))
    );
    assert!(
        rig.commands()
            .ends_with(&["STOP".into(), STOP_PWM.into(), STOP_STS.into()])
    );
}
#[test]
fn run_sets_targets_before_enabling_and_ps_holds() {
    let mut rig = Rig::new();
    let mut inputs = start_script();
    inputs.push(Pad {
        stop: true,
        ..Pad::default()
    });
    inputs.extend(vec![neutral(); 20]);
    assert!(rig.script(inputs).contains("試験終了"));
    let commands = rig.commands();
    assert!(commands.contains(&"RUN".into()));
    assert!(commands.contains(&"HOLD 7".into()));
    for channel in 0..4 {
        let index = commands
            .iter()
            .position(|s| s == &can(768, 2, channel, 1, [0; 4]))
            .unwrap();
        assert!(commands[index - 1].starts_with(&format!("CAN 2 768 0101{channel:02X}")));
    }
}
#[test]
fn ps_cancels_a_pending_run() {
    let mut rig = Rig::new();
    rig.board.lock().unwrap().reject_run = true;
    let mut inputs = start_script();
    inputs.truncate(63);
    inputs.push(Pad {
        stop: true,
        ..Pad::default()
    });
    inputs.extend(vec![neutral(); 20]);
    assert!(rig.script(inputs).contains("試験終了"));
    assert!(rig.commands().contains(&"HOLD 7".into()));
    assert!(!rig.commands().contains(&can(800, 2, 0, 0, [0; 4])));
}
#[test]
fn missing_run_ack_times_out_and_releases_outputs() {
    let mut rig = Rig::new();
    rig.board.lock().unwrap().reject_run = true;
    assert!(rig.script(start_script()).contains("RUNの応答"));
    assert!(
        rig.commands()
            .ends_with(&["STOP".into(), STOP_PWM.into(), STOP_STS.into()])
    );
}
#[test]
fn origin_requires_both_contacts() {
    let mut rig = Rig::new();
    rig.board.lock().unwrap().contacts = 1;
    assert!(rig.script(start_script()).contains("試験終了"));
    assert!(!rig.commands().contains(&"RUN".into()));
}
#[test]
fn held_buttons_cannot_resume() {
    let mut rig = Rig::new();
    let mut inputs = start_script();
    for _ in 0..3 {
        inputs.push(Pad {
            stop: true,
            ..Pad::default()
        });
        inputs.push(Pad {
            start: true,
            ..Pad::default()
        });
    }
    assert!(rig.script(inputs).contains("試験終了"));
    assert_eq!(rig.commands().iter().filter(|s| *s == "RUN").count(), 1);
}
#[test]
fn motor_fault_ends_the_session_and_releases_outputs() {
    let mut rig = Rig::new();
    // Inject after setup, as soon as the drive loop starts reading input.
    let shared = rig.board.clone();
    let mut inputs = start_script().into_iter();
    let mut count = 0;
    let result = session(&mut rig.port, &config(), &AtomicBool::new(false), || {
        count += 1;
        if count == 65 {
            shared.lock().unwrap().fault = Some("error_slot0");
        }
        inputs.next().context("試験終了")
    });
    let error = result.unwrap_err().to_string();
    assert!(error.contains("モータ/CAN"), "{error}");
}
#[test]
fn limits_and_native_signs_match_the_machine() {
    let config = config();
    let r = &config.machine.axes[0];
    let z = &config.machine.axes[2];
    assert_eq!(limited_speed(r, 570.0, 600.0, 0), 0.0);
    assert!(limited_speed(r, 570.0, -600.0, 0) < 0.0);
    assert_eq!(limited_speed(z, 0.0, -200.0, 0), 0.0);
    assert_eq!(limited_speed(r, 300.0, 600.0, 1), 0.0);
    assert!(((-60.0 * r.native_per_unit) - 2.4).abs() < 0.0001);
    assert!((20.0 * z.native_per_unit + 1920.32).abs() < 0.001);
    for distance in [0.01, 1.0, 10.0, 100.0] {
        let speed = limited_speed(r, 570.0 - distance, 600.0, 0);
        assert!(speed * 0.2 + speed * speed / 6000.0 <= distance + 0.001);
    }
}
#[test]
fn rotation_compensates_theta_and_rejects_overflow() {
    let config = config();
    let rotation = &config.machine.rotation;
    assert_eq!(robot::rotation_count(rotation, 10.0, 10.0).unwrap(), 1024);
    assert!(robot::rotation_count(rotation, 0.0, 10.0).unwrap() < 1024);
    assert!(robot::rotation_count(rotation, 10000.0, 0.0).is_err());
}
#[test]
fn pwm_reaches_endpoint_without_overshooting() {
    let (mut position, mut speed) = (500.0, 0.0);
    for _ in 0..150 {
        (position, speed) = pwm_step(position, 1444.0, speed, 1000.0, 3000.0, 0.02);
        assert!(position <= 1444.0);
    }
    assert_eq!(position, 1444.0);
}
#[test]
fn bad_configuration_is_rejected_before_hardware_is_opened() {
    let text = include_str!("../../machine.toml").replace(
        "grip_open = [1500.0, 1500.0, 1500.0]",
        "grip_open = [9999.0, 1500.0, 1500.0]",
    );
    assert!(Config::parse(&text).is_err());
}

#[test]
fn silent_feedback_times_out() {
    let mut rig = Rig::new();
    let shared = rig.board.clone();
    let mut count = 0;
    let mut script = start_script();
    script.extend(vec![neutral(); 30]);
    let mut script = script.into_iter();
    let error = session(&mut rig.port, &config(), &AtomicBool::new(false), || {
        count += 1;
        if count == 70 {
            shared.lock().unwrap().fault = Some("silence_rx");
        }
        script.next().context("試験終了")
    })
    .unwrap_err()
    .to_string();
    assert!(error.contains("CCTL応答が途絶"), "{error}");
}
#[test]
fn input_stall_exits_before_more_motion_is_sent() {
    let mut rig = Rig::new();
    let mut count = 0;
    let mut script = start_script().into_iter();
    let error = session(&mut rig.port, &config(), &AtomicBool::new(false), || {
        count += 1;
        if count == 70 {
            thread::sleep(Duration::from_millis(220));
        }
        script.next().context("試験終了")
    })
    .unwrap_err()
    .to_string();
    assert!(error.contains("入力処理が遅延"), "{error}");
}
#[test]
fn exit_flag_takes_the_normal_shutdown_path() {
    let mut rig = Rig::new();
    let stop = AtomicBool::new(false);
    let result = session(&mut rig.port, &config(), &stop, || {
        stop.store(true, Ordering::Relaxed);
        Ok(neutral())
    });
    assert!(result.is_ok());
}
