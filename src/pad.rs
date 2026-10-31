use gilrs::{Axis, Button, Gamepad};

// ボタンとスティックを機体の操作に割り当てる。戻り値はCCTLへ送る6項目の指令。
pub fn read(pad: &Gamepad) -> [i32; 6] {
    let pressed = |button| pad.is_pressed(button);
    let trigger = |button| pad.button_data(button).map_or(0.0, |data| data.value());
    // gilrsのLeftTriggerはL1。アナログのL2/R2はLeftTrigger2/RightTrigger2。
    let scale = if pressed(Button::LeftTrigger) {
        0.3
    } else {
        1.0
    };
    let scaled_input = |value: f32| {
        if value.abs() < 0.1 {
            0
        } else {
            (value * scale * 1000.0) as i32
        }
    };
    let r = scaled_input(pad.value(Axis::LeftStickY));
    let theta = scaled_input(-pad.value(Axis::LeftStickX));
    let z = scaled_input(trigger(Button::RightTrigger2) - trigger(Button::LeftTrigger2));
    let rotate = scaled_input(-pad.value(Axis::RightStickX));
    let fold = i32::from(pressed(Button::DPadUp)) - i32::from(pressed(Button::DPadDown));
    let grip = if pressed(Button::DPadLeft) {
        1
    } else if pressed(Button::DPadRight) {
        2
    } else {
        0
    };
    [r, theta, z, rotate, fold, grip]
}
