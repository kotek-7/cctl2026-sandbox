mod pad;
use gilrs::Gilrs;
use std::{
    error::Error,
    io::Write,
    time::{Duration, Instant},
};

const SEND_INTERVAL: Duration = Duration::from_millis(20);

// DualSenseとシリアルポートを開き、入力を20msごとにCCTLへ送る。
fn main() -> Result<(), Box<dyn Error>> {
    let device = std::env::args()
        .nth(1)
        .ok_or("シリアルポートを指定してください")?;
    if device == "--help" {
        println!("cargo run --release -- /dev/serial/by-id/対象CCTL");
        return Ok(());
    }
    let mut pads = Gilrs::new()?;
    let pad_id = pads
        .gamepads()
        .find(|(_, pad)| {
            pad.vendor_id() == Some(0x054c) && matches!(pad.product_id(), Some(0x0ce6 | 0x0df2))
        })
        .map(|(id, _)| id)
        .ok_or("DualSenseを接続してください")?;
    let mut port = serialport::new(device, 115200)
        .timeout(Duration::from_millis(20))
        .open()?;
    println!("接続後は常時操作 / L1: 低速 / Ctrl+C: 終了");
    loop {
        let started = Instant::now();
        // gilrs内部の入力状態を最新にしてから、全軸をまとめて読む。
        while pads.next_event().is_some() {}
        let gamepad = pads.gamepad(pad_id);
        if !gamepad.is_connected() {
            return Err("DualSenseが切断されました".into());
        }
        let [r, theta, z, rotate, fold, grip] = pad::read(&gamepad);
        writeln!(port, "{r} {theta} {z} {rotate} {fold} {grip}")?;
        std::thread::sleep(SEND_INTERVAL.saturating_sub(started.elapsed()));
    }
}
