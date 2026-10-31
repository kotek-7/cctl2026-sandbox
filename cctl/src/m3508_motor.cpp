#include "m3508_motor.hpp"
#include "config.hpp"
#include "main.h"

M3508Motor::M3508Motor(uint8_t esc_id, float max_rpm)
    : esc_id_(esc_id), position_pid_(config::position_kp, 0, 0, max_rpm),
      velocity_pid_(config::velocity_kp, config::velocity_ki, config::velocity_kd,
                    config::max_current_ma) {}
void M3508Motor::onFeedback(const uint8_t data[8]) {
    feedback_ = domain::c620::decodeFeedback(data);
    angle_.update(feedback_.raw_angle);
}
void M3508Motor::setEnabled(bool enabled) {
    if (enabled_ == enabled)
        return;
    enabled_ = enabled;
    // 再開後の計算に、停止前の積分値や停止中の経過時間を使わない。
    position_pid_.reset();
    velocity_pid_.reset();
    timer_.reset();
}
int16_t M3508Motor::commandRaw() {
    if (!enabled_ || !angle_.hasReference())
        return 0;
    const float dt = timer_.update(HAL_GetTick());
    // 二段の制御: モータ角度の誤差 → 回転数目標 → 電流[mA] → C620指令値。
    const float rpm = position_pid_.update(target_, angle_.degrees(), dt);
    const float current = velocity_pid_.update(rpm, feedback_.rpm, dt);
    return domain::c620::currentToRaw(static_cast<int32_t>(current));
}
