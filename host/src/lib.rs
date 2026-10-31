//! 機体操作・通信・保守ツールの共通実装。
pub mod application;
pub mod diagnostics;
pub mod input;
#[cfg(feature = "test-harness")]
pub mod interface;
pub mod machine;
pub mod protocol;
pub mod transport;

pub mod manual_control;
