#include "sts3215.hpp"
#include "domain/sts3215_protocol.hpp"
#include "main.h"
#include <cstring>
extern "C" UART_HandleTypeDef huart1;
namespace {
// ID 1の指定アドレスへデータを書き込む。応答を返さないSYNC_WRITE命令を使う。
void write(uint8_t address, const uint8_t *data, uint8_t length) {
    uint8_t parameters[10] = {address, length, 1};
    std::memcpy(parameters + 3, data, length);
    uint8_t packet[16];
    const auto size = domain::sts3215::buildInstruction(packet, sizeof(packet), 0xfe, 0x83,
                                                        parameters, length + 3);
    HAL_UART_Transmit(&huart1, packet, size, 5);
}
} // namespace
// シリアル通信を1Mbpsに設定し、ID 1のサーボへ出力停止を指令する。
void servoBegin() {
    huart1.Init.BaudRate = 1000000;
    huart1.Init.OverSampling = UART_OVERSAMPLING_8;
    if (HAL_UART_Init(&huart1) != HAL_OK)
        Error_Handler();
    servoEnable(false);
}
// 符号付きの目標位置をサーボ用の数値形式に変換し、ID 1へ送る。
void servoTarget(int16_t position) {
    // STSの負数は2の補数ではなく符号+絶対値。
    const uint16_t encoded = position < 0 ? (0x8000 | -position) : position;
    const domain::sts3215::Target target{1, 100, encoded, 0, 0};
    uint8_t data[7];
    domain::sts3215::encodeTarget(target, data);
    write(41, data, sizeof(data));
}
// ID 1のサーボへ、出力の有効・無効を指令する。
void servoEnable(bool enabled) {
    const uint8_t value = enabled;
    write(40, &value, 1);
}
