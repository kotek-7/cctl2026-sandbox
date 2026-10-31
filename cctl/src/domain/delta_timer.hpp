#pragma once

#include <cstdint>

namespace domain {

// ミリ秒のtickから経過秒を求める。初回は0を返す。
// 符号なしの差分を使い、32bitのtickが0に戻る場合も扱う。
class DeltaTimer {
  public:
    float update(uint32_t now_ms);
    void reset();

  private:
    bool has_previous_ = false;
    uint32_t previous_ms_ = 0;
};

} // namespace domain
