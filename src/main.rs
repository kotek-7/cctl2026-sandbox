mod pad;
use gilrs::Gilrs;
use std::{
    error::Error,
    io::Write,
    time::{Duration, Instant},
};

const SEND_INTERVAL: Duration = Duration::from_millis(20);

// CCTLの状態を端末に表示する。同じ状態の繰返しは省く。
fn read_status(
    port: &mut dyn serialport::SerialPort,
    line: &mut String,
    previous: &mut String,
) -> Result<(), Box<dyn Error>> {
    let available = port.bytes_to_read()? as usize;
    if available == 0 {
        return Ok(());
    }
    let mut bytes = [0; 256];
    let capacity = available.min(bytes.len());
    let count = port.read(&mut bytes[..capacity])?;
    for &byte in &bytes[..count] {
        if byte == b'\n' {
            if line != previous {
                println!("CCTL: {line}");
                previous.clone_from(line);
            }
            line.clear();
        } else {
            line.push(char::from(byte));
        }
    }
    Ok(())
}

// パッド入力をCCTLへ送り、機体の状態を端末に表示する。
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
    let mut status_line = String::new();
    let mut previous_status = String::new();
    loop {
        let started = Instant::now();
        while pads.next_event().is_some() {}
        let gamepad = pads.gamepad(pad_id);
        if !gamepad.is_connected() {
            return Err("DualSenseが切断されました".into());
        }
        let [r, theta, z, rotate, fold, grip] = pad::read(&gamepad);
        writeln!(port, "{r} {theta} {z} {rotate} {fold} {grip}")?;
        read_status(port.as_mut(), &mut status_line, &mut previous_status)?;
        std::thread::sleep(SEND_INTERVAL.saturating_sub(started.elapsed()));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serialport::SerialPort;

    // 状態受信テストで、PTYへの転送完了を待つ。
    fn wait_for_bytes(port: &serialport::TTYPort, count: u32) {
        let deadline = Instant::now() + Duration::from_secs(1);
        while port.bytes_to_read().unwrap() < count {
            assert!(Instant::now() < deadline, "PTYにデータが届かない");
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    #[test]
    fn status_lines_can_arrive_in_fragments_or_together() {
        let (mut pc, mut cctl) = serialport::TTYPort::pair().unwrap();
        let mut line = String::new();
        let mut previous = String::new();
        cctl.write_all(b"PC=OK R_").unwrap();
        cctl.flush().unwrap();
        wait_for_bytes(&pc, 8);
        read_status(&mut pc, &mut line, &mut previous).unwrap();
        assert_eq!(line, "PC=OK R_");
        assert!(previous.is_empty());
        cctl.write_all(b"FB=NO\nPC=OK R_FB=NO\nPC=OK R_FB=OK\n")
            .unwrap();
        cctl.flush().unwrap();
        wait_for_bytes(&pc, b"FB=NO\nPC=OK R_FB=NO\nPC=OK R_FB=OK\n".len() as u32);
        read_status(&mut pc, &mut line, &mut previous).unwrap();
        assert!(line.is_empty());
        assert_eq!(previous, "PC=OK R_FB=OK");
    }
}
