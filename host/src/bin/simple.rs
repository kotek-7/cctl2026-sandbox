//! One loop: read gilrs + serialport, update control, write commands.
use anyhow::{Context, Result, ensure};
use clap::Parser;
use host::{
    input::controller,
    manual_control::{Config, Control, Mode},
    transport::simulator::Simulator,
};
use std::{
    io::{self, Read, Write},
    path::PathBuf,
    sync::atomic::{AtomicBool, Ordering},
    time::{Duration, Instant},
};
static EXIT: AtomicBool = AtomicBool::new(false);
extern "C" fn stop_signal(_: libc::c_int) {
    EXIT.store(true, Ordering::Relaxed);
}
#[derive(Parser)]
#[command(about = "DualSense + PC + CCTL 手動操縦")]
struct Args {
    #[arg(
        long,
        required_unless_present = "simulate",
        conflicts_with = "simulate"
    )]
    device: Option<String>,
    #[arg(long)]
    simulate: bool,
    #[arg(long, requires = "simulate")]
    no_gamepad: bool,
    #[arg(long)]
    config: Option<PathBuf>,
}
// poll() avoids a reader thread and leaves terminal flags untouched.
fn terminal_lines(buffer: &mut String) -> Result<(Vec<String>, bool)> {
    let mut fd = libc::pollfd {
        fd: 0,
        events: libc::POLLIN,
        revents: 0,
    };
    let result = unsafe { libc::poll(&mut fd, 1, 0) };
    if result < 0 {
        let e = io::Error::last_os_error();
        if e.kind() == io::ErrorKind::Interrupted {
            return Ok((vec![], false));
        }
        return Err(e.into());
    }
    if result == 0 {
        return Ok((vec![], false));
    }
    let mut bytes = [0u8; 1024];
    let count = unsafe { libc::read(0, bytes.as_mut_ptr().cast(), bytes.len()) };
    if count < 0 {
        return Err(io::Error::last_os_error().into());
    }
    buffer.push_str(std::str::from_utf8(&bytes[..count as usize])?);
    ensure!(buffer.len() <= 4096, "端末入力が長すぎます");
    let mut lines = vec![];
    while let Some(end) = buffer.find('\n') {
        lines.push(buffer.drain(..=end).collect());
    }
    Ok((lines, count == 0))
}
fn status(c: &Control) -> String {
    format!(
        "state={:?} configured={} running={} outputs={} emergency={} origins={} reason={}",
        c.mode,
        c.ready(),
        c.mode == Mode::Running,
        matches!(c.mode, Mode::Running | Mode::AwaitRun | Mode::Hold),
        c.emergency,
        c.origins_ready(),
        c.message
    )
}
fn main() -> Result<()> {
    let args = Args::parse();
    unsafe {
        let mut action: libc::sigaction = std::mem::zeroed();
        action.sa_sigaction = stop_signal as *const () as usize;
        libc::sigemptyset(&mut action.sa_mask);
        for signal in [libc::SIGINT, libc::SIGTERM] {
            ensure!(
                libc::sigaction(signal, &action, std::ptr::null_mut()) == 0,
                "シグナル登録失敗"
            );
        }
    }
    let path = args
        .config
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../machine.toml"));
    let config = Config::parse(
        &std::fs::read_to_string(&path).with_context(|| format!("設定: {}", path.display()))?,
    )?;
    let mut pad = if args.no_gamepad {
        None
    } else {
        Some(gilrs::Gilrs::new().map_err(|e| anyhow::anyhow!("パッド初期化: {e}"))?)
    };
    let mut serial = if let Some(device) = args.device {
        Some(
            serialport::new(device, 115200)
                .timeout(Duration::from_millis(5))
                .open()?,
        )
    } else {
        None
    };
    let mut simulator = args.simulate.then(Simulator::new);
    let mut control = Control::new(config, Instant::now());
    let mut rx = String::new();
    let mut stdin = String::new();
    let mut previous = String::new();
    println!("Create長押し=手動原点確定 / Options=運転 / PS=3軸保持・EE解除");
    println!("原点: 出力解除中にr前端・z下端・θ正面へ手動で合わせてください。");
    println!("コマンド: status / stop / estop / reset / quit。終了時は全出力解除。");
    let result = (|| -> Result<()> {
        loop {
            let tick = Instant::now();
            if EXIT.load(Ordering::Relaxed) {
                break;
            }
            if let Some(port) = serial.as_mut() {
                let mut bytes = [0u8; 4096];
                match port.read(&mut bytes) {
                    Ok(n) => rx.push_str(std::str::from_utf8(&bytes[..n])?),
                    Err(e)
                        if matches!(
                            e.kind(),
                            io::ErrorKind::TimedOut
                                | io::ErrorKind::WouldBlock
                                | io::ErrorKind::Interrupted
                        ) => {}
                    Err(e) => return Err(e.into()),
                }
                ensure!(rx.len() <= 16384, "シリアル応答が長すぎます");
                while let Some(end) = rx.find('\n') {
                    let line: String = rx.drain(..=end).collect();
                    control.receive(line.trim(), tick)?;
                }
            } else if let Some(sim) = simulator.as_mut() {
                for line in sim.read() {
                    control.receive(&line, tick)?;
                }
            }
            let input = if let Some(gilrs) = pad.as_mut() {
                while gilrs.next_event().is_some() {}
                let candidates: Vec<_> = gilrs
                    .gamepads()
                    .filter(|(_, p)| {
                        p.is_connected()
                            && p.vendor_id() == Some(0x054c)
                            && matches!(p.product_id(), Some(0x0ce6 | 0x0df2))
                    })
                    .collect();
                ensure!(candidates.len() <= 1, "DualSenseは1台だけ接続してください");
                candidates.first().map(|(_, p)| controller::read(p))
            } else {
                None
            };
            control.step(tick, input.as_ref())?;
            let (lines, eof) = terminal_lines(&mut stdin)?;
            if eof {
                break;
            }
            let mut quit = false;
            for line in lines {
                match line.trim() {
                    "quit" => {
                        quit = true;
                        break;
                    }
                    "status" => println!("{}", status(&control)),
                    "stop" => {
                        control.hold(tick)?;
                        println!("OK stop");
                    }
                    "estop" => {
                        control.release(true);
                        println!("OK estop");
                    }
                    "reset" => {
                        control.reset_emergency();
                        println!("OK reset");
                    }
                    "" => {}
                    _ => println!("未対応: status/stop/estop/reset/quit"),
                }
            }
            if quit {
                break;
            }
            for line in control.out.drain(..) {
                if let Some(port) = serial.as_mut() {
                    writeln!(port, "{line}")?;
                } else if let Some(sim) = simulator.as_mut() {
                    sim.write(&line)?;
                }
            }
            let current = status(&control);
            if current != previous {
                println!("{current}");
                previous = current;
            }
            std::thread::sleep(Duration::from_millis(20).saturating_sub(tick.elapsed()));
        }
        Ok(())
    })();
    control.release(result.is_err());
    // Attempt every shutdown command even if an earlier write fails.
    let mut shutdown_error = None;
    for line in control.out.drain(..) {
        let sent = if let Some(port) = serial.as_mut() {
            writeln!(port, "{line}").map_err(anyhow::Error::from)
        } else {
            simulator.as_mut().unwrap().write(&line)
        };
        if let Err(e) = sent {
            shutdown_error = Some(e);
        }
    }
    if let Some(e) = shutdown_error {
        eprintln!("出力解除の送信失敗: {e}");
        return Err(e);
    }
    println!("終了処理完了。出力解除を送信しました（実機の受信保証ではありません）。");
    result
}
