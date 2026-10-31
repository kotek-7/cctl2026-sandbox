#pragma once
#include "domain/can_frame.hpp"
#include "main.h"

class CanBus {
  public:
    // この通信口をモータ用またはサーボ基板用のCANに結び付ける。通信開始はbeginが行う。
    explicit CanBus(FDCAN_HandleTypeDef *handle) : handle_(handle) {}
    bool begin();
    bool sendStd(uint16_t id, const uint8_t *data, uint8_t length);
    bool sendExt(uint32_t id, const uint8_t *data, uint8_t length);
    bool receive(domain::CanFrame &frame);

  private:
    bool send(uint32_t id, uint32_t type, const uint8_t *data, uint8_t length);
    FDCAN_HandleTypeDef *handle_;
};
