use crate::{
    config::{MachineConfig, PARAMETER_NAMES, ParameterMap},
    protocol::{
        parameters::{ParameterBoard, ParameterKey, ParameterValue},
        serial_svmd, svmd,
    },
};
use anyhow::{Result, bail};
use std::{
    collections::VecDeque,
    time::{Duration, Instant},
};

pub struct Settings {
    pending: VecDeque<ParameterValue>,
    sent: Option<Instant>,
}
impl Settings {
    pub fn new(profile: &MachineConfig) -> Self {
        let pending: VecDeque<_> = parameter_plan(profile).into();
        Self {
            pending,
            sent: None,
        }
    }
    pub fn ready(&self) -> bool {
        self.pending.is_empty()
    }
    pub fn poll_command(&mut self) -> Result<Option<String>> {
        if self
            .sent
            .is_some_and(|t| t.elapsed() > Duration::from_secs(1))
        {
            bail!("設定の反映応答がありません。接続とFWを確認して再適用してください");
        }
        if self.sent.is_some() {
            return Ok(None);
        }
        if let Some(parameter) = self.pending.front() {
            self.sent = Some(Instant::now());
            return Ok(Some(parameter.command()));
        }
        Ok(None)
    }
    pub fn receive(&mut self, line: &str) -> Result<()> {
        let Some(ParameterValue { key, value }) = ParameterValue::parse_reply(line) else {
            return Ok(());
        };
        let Some(expected) = self.pending.front() else {
            return Ok(());
        };
        if self.sent.is_none() || key != expected.key {
            return Ok(());
        }
        if !value.is_finite()
            || (value - expected.value).abs() > 0.0001_f32.max(expected.value.abs() * 0.00001)
        {
            let names: &[&str] = match key.board {
                ParameterBoard::Cctl => &PARAMETER_NAMES,
                ParameterBoard::Svmd => &svmd::PARAMETER_NAMES,
                ParameterBoard::SerialSvmd => &serial_svmd::PARAMETER_NAMES,
            };
            let name = names.get(usize::from(key.id)).copied().unwrap_or("unknown");
            bail!(
                "設定の応答値がPCと不一致です: {:?} / {} (ID {})、送信値 {}、応答値 {}",
                key.board,
                name,
                key.id,
                expected.value,
                value
            );
        }
        self.pending.pop_front();
        self.sent = None;
        Ok(())
    }
}
/// 検証済みプロファイルを送信順の設定値へ変換する。
/// 基板とIDを保持し、指令文字列の逆解析で応答先を推測しない。
pub fn parameter_plan(profile: &MachineConfig) -> Vec<ParameterValue> {
    let cctl_parameters = profile.cctl_parameters();
    let svmd_parameters = profile.effective_svmd_parameters();
    let boards: [(ParameterBoard, &ParameterMap, &[&str]); 3] = [
        (ParameterBoard::Cctl, &cctl_parameters, &PARAMETER_NAMES),
        (
            ParameterBoard::Svmd,
            &svmd_parameters,
            &svmd::PARAMETER_NAMES,
        ),
        (
            ParameterBoard::SerialSvmd,
            &profile.serial_svmd_parameters,
            &serial_svmd::PARAMETER_NAMES,
        ),
    ];
    boards
        .into_iter()
        .flat_map(|(board, values, names)| {
            values.iter().map(move |(name, value)| {
                let id = names
                    .iter()
                    .position(|entry| entry == name)
                    .expect("MachineConfig must be validated before synchronization")
                    as u8;
                // ASCII基板の小数5桁という送信精度を、照合値にも適用する。
                let value = if board == ParameterBoard::Cctl {
                    format!("{value:.5}").parse().expect("formatted float")
                } else {
                    *value
                };
                ParameterValue {
                    key: ParameterKey { board, id },
                    value,
                }
            })
        })
        .collect()
}
