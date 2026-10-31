#include "domain/delta_timer.hpp"

namespace domain {

// 起動後のミリ秒を受け取り、前回の呼出しからの経過秒を返す。初回は0を返す。
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

// 前回時刻を消し、次の呼出しで経過秒が0になるようにする。
void DeltaTimer::reset() {
    has_previous_ = false;
    previous_ms_ = 0;
}

} // namespace domain
