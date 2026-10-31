#pragma once
#include "domain/can_frame.hpp"
#include "main.h"

class CanBus {
  public:
    explicit CanBus(FDCAN_HandleTypeDef *handle) : handle_(handle) {}
    bool begin();
    bool sendStd(uint16_t id, const uint8_t *data, uint8_t length);
    bool sendExt(uint32_t id, const uint8_t *data, uint8_t length);
    bool receive(domain::CanFrame &frame);

  private:
    bool send(uint32_t id, uint32_t type, const uint8_t *data, uint8_t length);
    FDCAN_HandleTypeDef *handle_;
};
