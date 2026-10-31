#pragma once

#include <cstdint>

namespace domain {

struct CanFrame {
    uint32_t id = 0;
    bool extended = false;
    uint8_t length = 0;
    uint8_t data[8] = {};
};

} // namespace domain
