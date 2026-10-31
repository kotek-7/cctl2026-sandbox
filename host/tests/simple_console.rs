//! Run the actual minimal executable with its simulator. No hardware is opened.
use std::{
    io::{BufRead, BufReader, Write},
    process::{Child, ChildStdin, Command, Stdio},
    sync::mpsc::{self, Receiver},
    thread,
    time::{Duration, Instant},
};
struct Console {
    child: Child,
    input: Option<ChildStdin>,
    output: Receiver<String>,
}
impl Console {
    fn new() -> Self {
        let mut child = Command::new(env!("CARGO_BIN_EXE_simple"))
            .args(["--simulate", "--no-gamepad"])
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .unwrap();
        let input = child.stdin.take();
        let stdout = child.stdout.take().unwrap();
        let (tx, output) = mpsc::channel();
        thread::spawn(move || {
            for line in BufReader::new(stdout).lines() {
                if tx.send(line.unwrap()).is_err() {
                    break;
                }
            }
        });
        Self {
            child,
            input,
            output,
        }
    }
    fn send(&mut self, line: &str) {
        writeln!(self.input.as_mut().unwrap(), "{line}").unwrap();
    }
    fn wait_line(&self, wanted: &str) -> String {
        let end = Instant::now() + Duration::from_secs(15);
        let mut seen = Vec::new();
        while Instant::now() < end {
            if let Ok(line) = self.output.recv_timeout(Duration::from_millis(100)) {
                if line.contains(wanted) {
                    return line;
                }
                seen.push(line);
            }
        }
        panic!("Missing {wanted:?}; seen {seen:?}");
    }
    fn wait_exit(&mut self) {
        let end = Instant::now() + Duration::from_secs(5);
        while Instant::now() < end {
            if let Some(status) = self.child.try_wait().unwrap() {
                assert!(status.success());
                return;
            }
            thread::sleep(Duration::from_millis(20));
        }
        panic!("Console did not terminate");
    }
}
impl Drop for Console {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

#[test]
fn terminal_startup_rejects_motion_commands_and_handles_stop_reset_quit() {
    let mut c = Console::new();
    let ready = c.wait_line("configured=true");
    assert!(ready.contains("running=false outputs=false"));
    c.send("run");
    c.wait_line("未対応");
    c.send("estop");
    c.wait_line("OK");
    let stopped = c.wait_line("emergency=true");
    assert!(stopped.contains("running=false outputs=false"));
    c.send("reset");
    c.wait_line("OK");
    let reset = c.wait_line("emergency=false");
    assert!(reset.contains("running=false outputs=false"));
    c.send("stop");
    c.wait_line("OK");
    c.send("quit");
    c.wait_line("終了処理完了");
    c.wait_exit();
}
#[test]
fn eof_and_sigterm_take_the_normal_shutdown_path() {
    for signal in [false, true] {
        let mut c = Console::new();
        c.wait_line("configured=true");
        if signal {
            assert_eq!(unsafe { libc::kill(c.child.id() as i32, libc::SIGTERM) }, 0);
        } else {
            c.input.take();
        }
        c.wait_line("終了処理完了");
        c.wait_exit();
    }
}
#[test]
fn real_mode_requires_an_explicit_device() {
    assert!(
        !Command::new(env!("CARGO_BIN_EXE_simple"))
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .unwrap()
            .success()
    );
}
