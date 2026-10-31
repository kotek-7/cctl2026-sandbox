#include "domain/el05_codec.hpp"

#include <cstring>

namespace domain::el05 {

// EL05が指令の種類と宛先を判別するためのCAN IDを作る。
uint32_t buildCanId(uint8_t comm_type, uint16_t data_area_2, uint8_t target_id) {
    return ((static_cast<uint32_t>(comm_type) & 0x1F) << 24) |
           ((static_cast<uint32_t>(data_area_2) & 0xFFFF) << 8) | static_cast<uint32_t>(target_id);
}

// EL05の返信を、通常の位置返信かパラメータ読出しの返信かに振り分けるための通信種別を返す。
uint8_t commType(uint32_t ext_id) {
    return static_cast<uint8_t>((ext_id >> 24) & 0x1F);
}
// パラメータ返信がこのCCTL宛てかを確認するため、宛先IDを返す。
uint8_t targetId(uint32_t ext_id) {
    return static_cast<uint8_t>(ext_id & 0xFF);
}
// EL05の返信元や状態が載るID内の領域を返す。内容は通信種別によって異なる。
uint16_t dataArea2(uint32_t ext_id) {
    return static_cast<uint16_t>((ext_id >> 8) & 0xFFFF);
}

// EL05の整数表現を、制御で使う角度・速度・トルクの実数に戻す。
float uint16ToFloat(uint16_t raw, float min, float max) {
    return static_cast<float>(raw) * (max - min) / 65535.0f + min;
}

// EL05の数値変換をテストするため、実数を通信上の整数表現に戻す。範囲外は0か65535に収める。
uint16_t floatToUint16(float value, float min, float max) {
    const float span = max - min;
    if (span <= 0.0f) {
        return 0;
    }
    const float ratio = (value - min) / span;
    if (!(ratio > 0.0f)) {
        return 0; // NaN もここに落ちる
    }
    if (ratio >= 1.0f) {
        return 65535;
    }
    return static_cast<uint16_t>(ratio * 65535.0f + 0.5f);
}

// EL05の通常返信をモータ情報に変える。角度の折返し補正はEl05Motorが行う。
Feedback decodeFeedback(uint32_t ext_id, const uint8_t data[8]) {
    Feedback feedback;
    // フィードバックの data_area_2 には下位 8bit にモータID、上位に状態が載る。
    feedback.motor_id = static_cast<uint8_t>((ext_id >> 8) & 0xFF);
    feedback.state = static_cast<uint8_t>((ext_id >> 22) & 0x03);
    feedback.fault_bits = static_cast<uint8_t>((ext_id >> 16) & 0x3F);

    const uint16_t pos_raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    const uint16_t vel_raw = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    const uint16_t tau_raw = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    const uint16_t tmp_raw = (static_cast<uint16_t>(data[6]) << 8) | data[7];

    feedback.position_rad = uint16ToFloat(pos_raw, POSITION_MIN, POSITION_MAX);
    feedback.velocity_rad_s = uint16ToFloat(vel_raw, VELOCITY_MIN, VELOCITY_MAX);
    feedback.torque_nm = uint16ToFloat(tau_raw, TORQUE_MIN, TORQUE_MAX);
    feedback.temperature_c = static_cast<float>(tmp_raw) / 10.0f;
    return feedback;
}

// 位置目標や制御上限などの実数設定を、EL05が受け取る書込み形式にする。
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

// MECH_POSなどの値を問い合わせるため、EL05の読出し要求を作る。
void encodeParamRequest(uint16_t param, uint8_t out[8]) {
    for (int i = 0; i < 8; ++i) {
        out[i] = 0;
    }
    out[0] = static_cast<uint8_t>(param & 0xFF);
    out[1] = static_cast<uint8_t>((param >> 8) & 0xFF);
}

// EL05の設定保存命令に必要なデータを作る。現在の起動処理からは保存を指令しない。
void encodeSave(uint8_t out[8]) {
    for (uint8_t i = 0; i < 8; ++i) {
        out[i] = static_cast<uint8_t>(i + 1);
    }
}

// 返ってきた値がMECH_POSなどのどのパラメータかを識別する。
uint16_t paramReplyIndex(const uint8_t data[8]) {
    return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

// MECH_POSの返信を、rの位置確認に使うラジアン値として取り出す。
float paramReplyFloat(const uint8_t data[8]) {
    float value = 0.0f;
    std::memcpy(&value, &data[4], sizeof(float));
    return value;
}

} // namespace domain::el05
