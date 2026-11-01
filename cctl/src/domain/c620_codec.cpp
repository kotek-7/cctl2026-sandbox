#include "domain/c620_codec.hpp"

namespace domain::c620 {
namespace {
constexpr int32_t HALF_REV = COUNTS_PER_REV / 2;
// C620 電流スケール: ±20000mA ↔ ±16384
constexpr int32_t CURRENT_SCALE_NUM = 16384;
constexpr int32_t CURRENT_SCALE_DEN = 20000;
} // namespace

// C620の返信から、zの位置計算に使う角度とθ・zの速度制御に使うrpmを読む。
Feedback decodeFeedback(const uint8_t data[8]) {
    Feedback fb = {};
    fb.raw_angle = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    fb.rpm = static_cast<int16_t>((data[2] << 8) | data[3]);
    return fb;
}

// zの原点登録と端点減速に使う回転量を、エンコーダの折返しを補正して積算する。
void MultiTurnCounter::update(uint16_t raw_angle) {
    if (!has_reference_) {
        last_raw_angle_ = raw_angle;
        counts_ = 0;
        has_reference_ = true;
        return;
    }

    // 1 サンプルあたりの移動量が半回転未満である前提で、巻き戻りを補正する。
    int32_t delta = static_cast<int32_t>(raw_angle) - static_cast<int32_t>(last_raw_angle_);
    if (delta > HALF_REV) {
        delta -= COUNTS_PER_REV;
    } else if (delta < -HALF_REV) {
        delta += COUNTS_PER_REV;
    }

    counts_ += delta;
    last_raw_angle_ = raw_angle;
}

// 機体の位置に換算するモータ角度。基準からの回転量を度単位で返す。
float MultiTurnCounter::degrees() const {
    return static_cast<float>(counts_) * 360.0f / static_cast<float>(COUNTS_PER_REV);
}

// P制御で求めた電流[mA]を、C620へ送る整数値に換算する。
int16_t currentToRaw(int32_t milli_amp) {
    return static_cast<int16_t>(milli_amp * CURRENT_SCALE_NUM / CURRENT_SCALE_DEN);
}

// θ・zの電流指令を同じCANフレームに載せるため、指定モータの分だけ書き込む。
void writeCommandSlot(uint8_t frame[8], uint8_t esc_id, int16_t raw) {
    const uint8_t slot = static_cast<uint8_t>((esc_id - 1) * 2);
    frame[slot] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    frame[slot + 1] = static_cast<uint8_t>(raw & 0xFF);
}

} // namespace domain::c620
