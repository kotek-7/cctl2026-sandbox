#include "can_bus.hpp"

// 受信フィルタを設定してCAN通信を開始する。失敗したらfalseを返す。
bool CanBus::begin() {
    return HAL_FDCAN_ConfigGlobalFilter(handle_, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                        FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) == HAL_OK &&
           HAL_FDCAN_Start(handle_) == HAL_OK;
}
// 送信領域の空きを最大2ms待ち、指令を登録する。時間切れや登録失敗ならfalseを返す。
bool CanBus::send(uint32_t id, uint32_t type, const uint8_t *data, uint8_t length) {
    FDCAN_TxHeaderTypeDef header{};
    header.Identifier = id;
    header.IdType = type;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = length; // STM32G4 HALのClassic CAN DLCはバイト数。
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    const auto start = HAL_GetTick();
    while (!HAL_FDCAN_GetTxFifoFreeLevel(handle_))
        if (HAL_GetTick() - start >= 2)
            return false;
    return HAL_FDCAN_AddMessageToTxFifoQ(handle_, &header, data) == HAL_OK;
}
// 標準IDでCAN指令を送信待ちに登録する。登録できなければfalseを返す。
bool CanBus::sendStd(uint16_t id, const uint8_t *data, uint8_t length) {
    return send(id, FDCAN_STANDARD_ID, data, length);
}
// 拡張IDでCAN指令を送信待ちに登録する。登録できなければfalseを返す。
bool CanBus::sendExt(uint32_t id, const uint8_t *data, uint8_t length) {
    return send(id, FDCAN_EXTENDED_ID, data, length);
}
// CANを1件読み取ってframeに格納する。未受信または読取り失敗ならfalseを返す。
bool CanBus::receive(domain::CanFrame &frame) {
    if (!HAL_FDCAN_GetRxFifoFillLevel(handle_, FDCAN_RX_FIFO0))
        return false;
    FDCAN_RxHeaderTypeDef header{};
    if (HAL_FDCAN_GetRxMessage(handle_, FDCAN_RX_FIFO0, &header, frame.data) != HAL_OK)
        return false;
    frame.id = header.Identifier;
    frame.extended = header.IdType == FDCAN_EXTENDED_ID;
    frame.length = header.DataLength;
    return true;
}
