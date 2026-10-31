#include "m3508_motor.hpp"
#include "config.hpp"
#include "main.h"

// θ・zの速度制御に使うゲインと電流上限を設定する。
M3508Motor::M3508Motor(uint8_t esc_id)
    : esc_id_(esc_id), velocity_pid_(config::velocity_kp, config::velocity_ki, config::velocity_kd,
                                     config::max_current_ma) {}
// 速度制御と接点原点に使う回転数・角度を、C620の返信から受け取る。
void M3508Motor::onFeedback(const uint8_t data[8]) {
    feedback_ = domain::c620::decodeFeedback(data);
    angle_.update(feedback_.raw_angle);
}
// θ・zの制御を開始・停止する。再開時に停止前の積分値や停止中の時間を持ち込まない。
void M3508Motor::setEnabled(bool enabled) {
    if (enabled_ == enabled)
        return;
    enabled_ = enabled;
    // 再開後の計算に、停止前の積分値や停止中の経過時間を使わない。
    velocity_pid_.reset();
    timer_.reset();
}
// 実測回転数を指令回転数に近づける電流を求める。出力OFF中や返信未受信なら0。
int16_t M3508Motor::commandRaw() {
    if (!enabled_ || !angle_.hasReference())
        return 0;
    const float dt = timer_.update(HAL_GetTick());
    const float current = velocity_pid_.update(target_rpm_, feedback_.rpm, dt);
    return domain::c620::currentToRaw(static_cast<int32_t>(current));
}
