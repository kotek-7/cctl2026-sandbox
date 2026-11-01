#pragma once
#include <cstdint>

namespace domain::c620 {
constexpr int32_t COUNTS_PER_REV = 8192;

// zの原点計算に使う角度と、θ・zのP制御に使う回転数。
struct Feedback {
    uint16_t raw_angle;
    int16_t rpm;
};
Feedback decodeFeedback(const uint8_t data[8]);

// zの最初の角度を0とし、1回転ごとの折返しを補正して回転量を数える。
class MultiTurnCounter {
  public:
    void update(uint16_t raw_angle);
    float degrees() const;

  private:
    bool has_reference_ = false;
    uint16_t last_raw_angle_ = 0;
    int32_t counts_ = 0;
};

int16_t currentToRaw(int32_t milli_amp);
void writeCommandSlot(uint8_t frame[8], uint8_t esc_id, int16_t raw);
} // namespace domain::c620
