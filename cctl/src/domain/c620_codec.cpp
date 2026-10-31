#include "domain/c620_codec.hpp"

namespace domain::c620 {
namespace {
constexpr int32_t HALF_REV = COUNTS_PER_REV / 2;
// C620 電流スケール: ±20000mA ↔ ±16384
constexpr int32_t CURRENT_SCALE_NUM = 16384;
constexpr int32_t CURRENT_SCALE_DEN = 20000;
} // namespace

// C620の通信形式を、θ・zの制御で扱う実測値に変える。
Feedback decodeFeedback(const uint8_t data[8]) {
    Feedback fb = {};
    fb.raw_angle = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    fb.rpm = static_cast<int16_t>((data[2] << 8) | data[3]);
    fb.current_raw = static_cast<int16_t>((data[4] << 8) | data[5]);
    fb.temperature = data[6];
    return fb;
}

// 1回転ごとに戻るエンコーダ値を、接点原点と端点減速に使う連続した回転量として追跡する。
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

// モータの回転量を数え直す。機体の接点原点とは別に、次の受信角度を基準にする。
void MultiTurnCounter::reset() {
    has_reference_ = false;
    last_raw_angle_ = 0;
    counts_ = 0;
}

// 機体の位置に換算するモータ角度。基準からの回転量を度単位で返す。
float MultiTurnCounter::degrees() const {
    return static_cast<float>(counts_) * 360.0f / static_cast<float>(COUNTS_PER_REV);
}

// PIDで決めた電流を、C620の電流指令に合わせる。入力はmA。
int16_t currentToRaw(int32_t milli_amp) {
    return static_cast<int16_t>(milli_amp * CURRENT_SCALE_NUM / CURRENT_SCALE_DEN);
}

// C620の通信値をmAで読めるようにする。電流変換の往復テストにも使う。
int32_t rawToCurrent(int16_t raw) {
    return static_cast<int32_t>(raw) * CURRENT_SCALE_DEN / CURRENT_SCALE_NUM;
}

// 4台単位で共有する電流指令の宛先を決める。モータIDが1〜8以外なら0を返す。
uint16_t groupCommandId(uint8_t esc_id) {
    if (esc_id < MIN_ESC_ID || esc_id > MAX_ESC_ID) {
        return 0;
    }
    return esc_id <= 4 ? COMMAND_ID_1_TO_4 : COMMAND_ID_5_TO_8;
}

// θ・zの電流指令を同じCANフレームに載せるため、指定モータの分だけ書き込む。
void writeCommandSlot(uint8_t frame[8], uint8_t esc_id, int16_t raw) {
    const uint8_t slot = static_cast<uint8_t>(((esc_id - 1) % MOTORS_PER_FRAME) * 2);
    frame[slot] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    frame[slot + 1] = static_cast<uint8_t>(raw & 0xFF);
}

// 次の送信に前回の電流指令を残さないため、4台分を未指令の状態に戻す。
void CommandFrame::clear() {
    for (uint8_t &byte : data_) {
        byte = 0;
    }
}

// 同じ宛先で送れるモータの電流指令をまとめる。別グループのIDが混ざったらfalseを返す。
bool CommandFrame::set(uint8_t esc_id, int16_t raw) {
    if (groupCommandId(esc_id) != command_id_) {
        return false;
    }
    writeCommandSlot(data_, esc_id, raw);
    return true;
}

} // namespace domain::c620
