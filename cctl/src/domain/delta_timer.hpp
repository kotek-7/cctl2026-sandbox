#pragma once

#include <cstdint>

namespace domain {

// 前回の呼出しから何秒たったかを返す。入力は起動後のミリ秒、初回の結果は0。
// uint32_tの引き算なので、ミリ秒カウンタが上限を超えて0に戻っても差を求められる。
class DeltaTimer {
  public:
    float update(uint32_t now_ms);
    void reset();

  private:
    bool has_previous_ = false;
    uint32_t previous_ms_ = 0;
};

} // namespace domain
