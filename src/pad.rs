use gilrs::{Axis, Button, Gamepad};

// r / θ / z / 回転 / 畳み / 把持。符号は機体の正方向に合わせる。
pub fn read(pad: &Gamepad) -> [i32; 6] {
    let pressed = |button| pad.is_pressed(button);
    let trigger = |button| pad.button_data(button).map_or(0.0, |data| data.value());
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
    [
        scaled_input(pad.value(Axis::LeftStickY)),
        scaled_input(-pad.value(Axis::LeftStickX)),
        scaled_input(trigger(Button::RightTrigger2) - trigger(Button::LeftTrigger2)),
        scaled_input(-pad.value(Axis::RightStickX)),
        i32::from(pressed(Button::DPadUp)) - i32::from(pressed(Button::DPadDown)),
        if pressed(Button::DPadLeft) {
            1
        } else if pressed(Button::DPadRight) {
            2
        } else {
            0
        },
    ]
}
