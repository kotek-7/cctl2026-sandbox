#include "app.h"
#include "main.h"
#include "sts3215.hpp"
#include <cstdint>
extern "C" CAN_HandleTypeDef hcan;
namespace {
constexpr uint16_t COMMAND_ID = 0x320;
uint32_t last_command = 0;
} // namespace
extern "C" void setup() {
    servoBegin();
    CAN_FilterTypeDef filter{};
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = COMMAND_ID << 5;
    filter.FilterMaskIdHigh = 0x7ff << 5;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK || HAL_CAN_Start(&hcan) != HAL_OK)
        Error_Handler();
}
extern "C" void loop() {
    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)) {
        CAN_RxHeaderTypeDef header{};
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
            break;
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.StdId != COMMAND_ID ||
            header.DLC != 3 || data[0] > 1)
            continue;
        // CAN位置は上位byteが先の2の補数。負数を明示的に復号する。
        const int raw = (data[1] << 8) | data[2];
        const int position = raw >= 32768 ? raw - 65536 : raw;
        if (position < -28672 || position > 28672)
            continue;
        last_command = HAL_GetTick();
        // 目標を書いてからトルクを入れる。無効指令は毎回OFFを送る。
        if (data[0])
            servoTarget(static_cast<int16_t>(position));
        servoEnable(data[0]);
    }
    // 正常な指令だけが受信期限を延長する。途絶時はトルクを解除する。
    if (HAL_GetTick() - last_command > 250) {
        servoEnable(false);
        HAL_Delay(1);
    }
}
