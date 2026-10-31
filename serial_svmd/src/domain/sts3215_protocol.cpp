#include "domain/sts3215_protocol.hpp"

#include <cstring>

namespace domain::sts3215 {

// サーボが通信データの破損を検出できるよう、STS方式のチェックサムを求める。
uint8_t checksum(const uint8_t *data, std::size_t length) {
    uint16_t sum = 0;
    for (std::size_t i = 0; i < length; ++i) {
        sum = static_cast<uint16_t>(sum + data[i]);
    }
    return static_cast<uint8_t>(~sum);
}

// 回転サーボへの指令を、シリアル送信できるSTSパケットにする。作れなければ長さ0を返す。
std::size_t buildInstruction(uint8_t *packet, std::size_t capacity, uint8_t id, uint8_t instruction,
                             const uint8_t *parameters, uint8_t parameter_count) {
    const std::size_t length = static_cast<std::size_t>(parameter_count) + PACKET_OVERHEAD;

    if (packet == nullptr || capacity < length || (parameters == nullptr && parameter_count != 0) ||
        parameter_count > MAX_TX_PARAMETERS) {
        return 0;
    }

    packet[0] = HEADER;
    packet[1] = HEADER;
    packet[2] = id;
    packet[3] = static_cast<uint8_t>(parameter_count + 2);
    packet[4] = instruction;
    if (parameter_count != 0) {
        std::memcpy(&packet[5], parameters, parameter_count);
    }
    // チェックサムの対象は ID から最後のパラメータまで。
    packet[length - 1] = checksum(&packet[2], length - 3);

    return length;
}

// STSの返信に破損がないかを検査する。現在のFWは返信を読まず、通信形式のテストで使う。
bool verifyStatusChecksum(uint8_t response_id, uint8_t body_length, const uint8_t *body) {
    if (body == nullptr || body_length < 2 || body_length > (MAX_RX_PARAMETERS + 2)) {
        return false;
    }

    uint8_t data[MAX_RX_PARAMETERS + 3];
    data[0] = response_id;
    data[1] = body_length;
    std::memcpy(&data[2], body, body_length - 1);

    return checksum(data, static_cast<std::size_t>(body_length) + 1) == body[body_length - 1];
}

// 位置・時間・速度の数値を、STSが要求する下位バイト先の並びにそろえる。
void encodeUint16(uint16_t value, uint8_t *data) {
    data[0] = static_cast<uint8_t>(value & 0xFF);
    data[1] = static_cast<uint8_t>(value >> 8);
}

// STSのバイト列を数値として読む。送信内容のテストで位置・時間・速度の確認に使う。
uint16_t decodeUint16(const uint8_t *data) {
    return static_cast<uint16_t>(data[0]) | static_cast<uint16_t>(data[1] << 8);
}

// 回転サーボの動き方と目標位置を、まとめて書き込める7バイトの指令にする。
void encodeTarget(const Target &target, uint8_t *data) {
    data[0] = target.acceleration;
    encodeUint16(target.position, &data[1]);
    encodeUint16(target.time, &data[3]);
    encodeUint16(target.speed, &data[5]);
}

} // namespace domain::sts3215
