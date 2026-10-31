//! DualSenseの値を20msごとにCCTLへ送る。
mod pad;
use gilrs::Gilrs;
use std::{
    error::Error,
    io::Write,
    time::{Duration, Instant},
};

fn main() -> Result<(), Box<dyn Error>> {
    let device = std::env::args()
        .nth(1)
        .ok_or("シリアルポートを指定してください")?;
    if device == "--help" {
        println!("cargo run --release -- /dev/serial/by-id/対象CCTL");
        return Ok(());
    }
    let mut pads = Gilrs::new()?;
    let id = pads
        .gamepads()
        .find(|(_, p)| {
            p.vendor_id() == Some(0x054c) && matches!(p.product_id(), Some(0x0ce6 | 0x0df2))
        })
        .map(|(id, _)| id)
        .ok_or("DualSenseを接続してください")?;
    let mut port = serialport::new(device, 115200)
        .timeout(Duration::from_millis(20))
        .open()?;
    println!("Create: 原点登録して操作開始 / Ctrl+C: 終了");
    loop {
        let started = Instant::now();
        while pads.next_event().is_some() {}
        let gamepad = pads.gamepad(id);
        if !gamepad.is_connected() {
            return Err("DualSenseが切断されました".into());
        }
        let [r, theta, z, rotate, fold, grip, home] = pad::read(&gamepad);
        writeln!(port, "{r} {theta} {z} {rotate} {fold} {grip} {home}")?;
        std::thread::sleep(Duration::from_millis(20).saturating_sub(started.elapsed()));
    }
}
