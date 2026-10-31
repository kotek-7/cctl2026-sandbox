#include "m3508_motor.hpp"
#include "config.hpp"
#include <algorithm>

// 返信が届いた時点で速度と角度を更新する。未受信の間の速度は初期値0を使う。
void M3508Motor::onFeedback(const uint8_t data[8]) {
    feedback_ = domain::c620::decodeFeedback(data);
    angle_.update(feedback_.raw_angle);
}

// 速度誤差に比例した電流を指令する。初回PC入力前だけ出力を0にする。
int16_t M3508Motor::commandRaw() {
    if (!enabled_)
        return 0;
    const float current = std::clamp(config::velocity_kp * (target_rpm_ - feedback_.rpm),
                                     -config::max_current_ma, config::max_current_ma);
    return domain::c620::currentToRaw(static_cast<int32_t>(current));
}
