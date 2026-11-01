#include "domain/el05_codec.hpp"

#include <cstring>

namespace domain::el05 {

// EL05の指令用CAN IDを組み立てる。
uint32_t buildCanId(uint8_t comm_type, uint16_t data_area_2, uint8_t target_id) {
    return ((static_cast<uint32_t>(comm_type) & 0x1F) << 24) |
           ((static_cast<uint32_t>(data_area_2) & 0xFFFF) << 8) | static_cast<uint32_t>(target_id);
}

// EL05の通常返信を識別するための通信種別を返す。
uint8_t commType(uint32_t ext_id) {
    return static_cast<uint8_t>((ext_id >> 24) & 0x1F);
}
// EL05の返信元IDを含む領域を取り出す。
uint16_t dataArea2(uint32_t ext_id) {
    return static_cast<uint16_t>((ext_id >> 8) & 0xFFFF);
}

// rの通常返信から角度と異常ビットを読み取る。
Feedback decodeFeedback(uint32_t ext_id, const uint8_t data[8]) {
    return {static_cast<uint16_t>((data[0] << 8) | data[1]),
            static_cast<uint8_t>((ext_id >> 16) & 0x3F)};
}

// 角度の折返しを補正し、rの移動量[rad]を求める。
// 返信間の移動が半周期未満であることを前提にする。
float positionDelta(uint16_t previous, uint16_t current) {
    int32_t delta = static_cast<int32_t>(current) - previous;
    if (delta > 32767)
        delta -= 65536;
    if (delta < -32768)
        delta += 65536;
    return delta * (POSITION_MAX - POSITION_MIN) / 65535.0f;
}

// EL05の速度・加速度・電流上限を書き込むデータを作る。
void encodeParamFloat(uint16_t param, float value, uint8_t out[8]) {
    for (int i = 0; i < 8; ++i) {
        out[i] = 0;
    }
    out[0] = static_cast<uint8_t>(param & 0xFF);
    out[1] = static_cast<uint8_t>((param >> 8) & 0xFF);
    std::memcpy(&out[4], &value, sizeof(float));
}

// EL05の制御モードを書き込むデータを作る。
void encodeParamU8(uint16_t param, uint8_t value, uint8_t out[8]) {
    for (int i = 0; i < 8; ++i) {
        out[i] = 0;
    }
    out[0] = static_cast<uint8_t>(param & 0xFF);
    out[1] = static_cast<uint8_t>((param >> 8) & 0xFF);
    out[4] = value;
}

} // namespace domain::el05
