#include "domain/el05_codec.hpp"

#include <cstring>

namespace domain::el05 {

// EL05が指令の種類と宛先を判別するためのCAN IDを作る。
uint32_t buildCanId(uint8_t comm_type, uint16_t data_area_2, uint8_t target_id) {
    return ((static_cast<uint32_t>(comm_type) & 0x1F) << 24) |
           ((static_cast<uint32_t>(data_area_2) & 0xFFFF) << 8) | static_cast<uint32_t>(target_id);
}

// 通常返信を受け取ったかを確認するため、通信種別を返す。
uint8_t commType(uint32_t ext_id) {
    return static_cast<uint8_t>((ext_id >> 24) & 0x1F);
}
// EL05の返信元や状態が載るID内の領域を返す。内容は通信種別によって異なる。
uint16_t dataArea2(uint32_t ext_id) {
    return static_cast<uint16_t>((ext_id >> 8) & 0xFFFF);
}

// rの原点計算と異常処理で使う値を通常返信から取り出す。
Feedback decodeFeedback(uint32_t ext_id, const uint8_t data[8]) {
    return {static_cast<uint16_t>((data[0] << 8) | data[1]),
            static_cast<uint8_t>((ext_id >> 16) & 0x3F)};
}

// 通常返信の角度の折返しを補正し、返信間の移動量をradで求める。
// 返信間の移動が半周期未満であることを前提にする。
float positionDelta(uint16_t previous, uint16_t current) {
    int32_t delta = static_cast<int32_t>(current) - previous;
    if (delta > 32767)
        delta -= 65536;
    if (delta < -32768)
        delta += 65536;
    return delta * (POSITION_MAX - POSITION_MIN) / 65535.0f;
}

// 速度指令や電流上限などの実数設定を、EL05が受け取る書込み形式にする。
void encodeParamFloat(uint16_t param, float value, uint8_t out[8]) {
    for (int i = 0; i < 8; ++i) {
        out[i] = 0;
    }
    out[0] = static_cast<uint8_t>(param & 0xFF);
    out[1] = static_cast<uint8_t>((param >> 8) & 0xFF);
    std::memcpy(&out[4], &value, sizeof(float));
}

// 制御モードなどの整数設定を、EL05が受け取る書込み形式にする。
void encodeParamU8(uint16_t param, uint8_t value, uint8_t out[8]) {
    for (int i = 0; i < 8; ++i) {
        out[i] = 0;
    }
    out[0] = static_cast<uint8_t>(param & 0xFF);
    out[1] = static_cast<uint8_t>((param >> 8) & 0xFF);
    out[4] = value;
}

} // namespace domain::el05
