use gilrs::{Axis, Button, Gamepad};

// r / θ / z / 回転 / 畳み / 把持 / ボタン。符号は機体の正方向に合わせる。
pub fn read(pad: &Gamepad) -> [i32; 7] {
    let pressed = |b| pad.is_pressed(b);
    let trigger = |b| pad.button_data(b).map_or(0.0, |v| v.value());
    let scale = if pressed(Button::LeftTrigger) {
        0.3
    } else {
        1.0
    };
    let axis = |value: f32| {
        if value.abs() < 0.1 {
            0
        } else {
            (value * scale * 1000.0) as i32
        }
    };
    [
        axis(pad.value(Axis::LeftStickY)),
        axis(-pad.value(Axis::LeftStickX)),
        axis(trigger(Button::RightTrigger2) - trigger(Button::LeftTrigger2)),
        axis(-pad.value(Axis::RightStickX)),
        i32::from(pressed(Button::DPadUp)) - i32::from(pressed(Button::DPadDown)),
        if pressed(Button::DPadLeft) {
            1
        } else if pressed(Button::DPadRight) {
            2
        } else if pressed(Button::East) {
            3
        } else {
            0
        },
        i32::from(pressed(Button::Select))
            | (i32::from(pressed(Button::Start)) << 1)
            | (i32::from(pressed(Button::Mode)) << 2),
    ]
}
