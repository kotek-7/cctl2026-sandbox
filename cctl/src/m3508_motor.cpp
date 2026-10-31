#include "m3508_motor.hpp"
#include "config.hpp"
#include "main.h"

// モータIDと最高回転数を受け取り、角度・回転数を制御するPIDを設定する。
M3508Motor::M3508Motor(uint8_t esc_id, float max_rpm)
    : esc_id_(esc_id), position_pid_(config::position_kp, 0, 0, max_rpm),
      velocity_pid_(config::velocity_kp, config::velocity_ki, config::velocity_kd,
                    config::max_current_ma) {}
// 8バイトの返信を読み、角度・回転数・電流・温度を更新する。
void M3508Motor::onFeedback(const uint8_t data[8]) {
    feedback_ = domain::c620::decodeFeedback(data);
    angle_.update(feedback_.raw_angle);
}
// 出力の有効・無効を切り替え、切替時にPIDの積分値と経過時間をリセットする。
void M3508Motor::setEnabled(bool enabled) {
    if (enabled_ == enabled)
        return;
    enabled_ = enabled;
    // 再開後の計算に、停止前の積分値や停止中の経過時間を使わない。
    position_pid_.reset();
    velocity_pid_.reset();
    timer_.reset();
}
// 目標角度に近づける電流を計算し、C620へ送る整数値を返す。停止中や角度未受信なら0。
int16_t M3508Motor::commandRaw() {
    if (!enabled_ || !angle_.hasReference())
        return 0;
    const float dt = timer_.update(HAL_GetTick());
    // まず角度のずれから必要な回転数を求め、次に回転数のずれから電流を求める。
    const float rpm = position_pid_.update(target_, angle_.degrees(), dt);
    const float current = velocity_pid_.update(rpm, feedback_.rpm, dt);
    return domain::c620::currentToRaw(static_cast<int32_t>(current));
}
