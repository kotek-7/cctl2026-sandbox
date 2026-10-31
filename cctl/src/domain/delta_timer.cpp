#include "domain/delta_timer.hpp"

namespace domain {

// PIDが積分・微分に使う経過秒を求める。基準時刻のない初回は0を返す。
float DeltaTimer::update(uint32_t now_ms) {
    if (!has_previous_) {
        previous_ms_ = now_ms;
        has_previous_ = true;
        return 0.0f;
    }

    const uint32_t elapsed_ms = now_ms - previous_ms_;
    previous_ms_ = now_ms;
    return static_cast<float>(elapsed_ms) / 1000.0f;
}

// モータの停止時間を、再開後のPID計算に含めないよう時刻の基準を捨てる。
void DeltaTimer::reset() {
    has_previous_ = false;
    previous_ms_ = 0;
}

} // namespace domain
