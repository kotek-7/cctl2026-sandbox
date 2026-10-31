use anyhow::{Result, bail, ensure};
use std::time::Instant;
pub struct Simulator {
    started: Instant,
    mode: &'static str,
    enabled: u8,
    held: u8,
    positions: [f32; 3],
    pulses: [u16; 4],
    pwm_enabled: u8,
    rotation: i16,
    rotation_enabled: bool,
    rotation_running: bool,
    replies: Vec<String>,
    stale: u8,
    error: u8,
    silent: bool,
}
impl Default for Simulator {
    fn default() -> Self {
        Self::new()
    }
}
impl Simulator {
    pub fn new() -> Self {
        Self {
            started: Instant::now(),
            mode: "SAFE",
            enabled: 0,
            held: 0,
            positions: [0.0; 3],
            pulses: [1500; 4],
            pwm_enabled: 0,
            rotation: 1024,
            rotation_enabled: false,
            rotation_running: false,
            replies: vec![],
            stale: 0,
            error: 0,
            silent: false,
        }
    }
    fn can_reply(&mut self, id: u16, data: [u8; 8]) {
        self.replies.push(format!(
            "CAN_RX bus=2 id={id} data={}",
            data.iter().map(|v| format!("{v:02X}")).collect::<String>()
        ));
    }
    pub fn write(&mut self, line: &str) -> Result<()> {
        let words: Vec<_> = line.split_whitespace().collect();
        match words.as_slice() {
            ["HELLO","1"]=>self.replies.push("DEVICE protocol=1 board=cctl slots=3 can=2 watchdog_ms=250 params=default jog=1 motors=el05,m3508,m3508".into()),
            ["SAFE"]|["STOP"]=>{self.mode=if words[0]=="SAFE" {"SAFE"}else{"STOP"};self.held=0;},
            ["ENABLE",mask,value]=>{let mask:u8=mask.parse()?;if *value=="1" {self.enabled|=mask;}else{self.enabled&=!mask;}},
            ["RUN"]=>{self.mode="RUN";self.held=0;},
            ["HOLD",mask]=>{self.held=mask.parse()?;self.enabled=self.held;self.mode="SAFE";},
            ["HEARTBEAT"]=>{},
            ["TARGET",slot,value]=>{if self.mode=="RUN" {self.positions[slot.parse::<usize>()?]=value.parse()?;}},
            ["JOG",slot,value]=>{if self.mode=="RUN" {self.positions[slot.parse::<usize>()?]+=value.parse::<f32>()?*0.02;}},
            ["PARAM",id,value]=>{ensure!(self.mode=="SAFE","PARAM requires SAFE");self.replies.push(format!("PARAM {id} {value}"));},
            ["CAN","2",id,payload]=>{
                ensure!(payload.len()==16 && payload.is_ascii(),"bad CAN payload");
                let id:u16=id.parse()?;
                let mut data=[0u8;8];for (i,byte) in data.iter_mut().enumerate() {*byte=u8::from_str_radix(&payload[i*2..i*2+2],16)?;}
                if (id==768 && data[1]==4)||(id==800 && data[1]==9) {
                    self.can_reply(id+4,[1,data[2],0,0,data[4],data[5],data[6],data[7]]);
                } else if id==768 {
                    let channel=usize::from(data[2]);ensure!(channel<4,"bad PWM channel");
                    match data[1] {
                        0=>self.pwm_enabled=0,
                        1=>self.pulses[channel]=u16::from_be_bytes([data[4],data[5]]),
                        2=>if data[3]==1 {self.pwm_enabled|=1<<channel;}else{self.pwm_enabled&=!(1<<channel);},
                        3=>{},_=>bail!("unknown PWM opcode"),
                    }
                    let pulse=self.pulses[channel].to_be_bytes();
                    self.can_reply(769,[1,0,data[1],data[2],self.pwm_enabled,pulse[0],pulse[1],0]);
                } else if id==800 {
                    match data[1] {
                        2=>self.rotation_running=true,
                        3=>{self.rotation_running=false;self.rotation_enabled=false;},
                        4=>{let raw=u16::from_be_bytes([data[4],data[5]]);self.rotation=(raw&0x7fff) as i16*if raw&0x8000==0 {1}else{-1};},
                        5=>{},
                        6=>self.rotation_enabled=data[3]==1,
                        7=>{let raw=self.rotation.unsigned_abs()|if self.rotation<0 {0x8000}else{0};let raw=raw.to_be_bytes();self.can_reply(802,[1,data[2],raw[0],raw[1],u8::from(self.rotation_enabled&&self.rotation_running),0,1,0]);},
                        _=>bail!("unknown STS opcode"),
                    }
                } else {bail!("unknown CAN device");}
            },
            _=>bail!("unknown simulator command: {line}"),
        }
        Ok(())
    }
    pub fn read(&mut self) -> Vec<String> {
        if self.silent {
            self.replies.clear();
            return vec![];
        }
        let mut lines = std::mem::take(&mut self.replies);
        let [a, b, c] = self.positions;
        lines.push(format!("STATE t={} mode={} en={} a0={a}/{a} a1={b}/{b} a2={c}/{c} err={:02X},00,00 sw=7 stale={} can=3 hold={}",self.started.elapsed().as_millis(),self.mode,self.enabled,self.error,self.stale,self.held));
        self.stale = 0;
        self.error = 0;
        lines
    }
    #[cfg(test)]
    pub fn fault(&mut self, name: &str) -> Result<()> {
        match name {
            "stale_slot0" => self.stale = 1,
            "error_slot0" => self.error = 128,
            "silence_rx" => self.silent = true,
            _ => bail!("unknown fault"),
        }
        Ok(())
    }
}
