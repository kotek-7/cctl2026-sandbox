#include "app.h"
#include "config.hpp"
#include "ee.hpp"
#include "main.h"
#include "motors.hpp"
#include <algorithm>
#include <cstdlib>

namespace {
// USB割込みはコピーだけ。行の解析と機体操作はloopで行う。
volatile char received_line[80];
volatile bool line_ready = false;
int input[7]{};
uint32_t last_input = 0;
bool homed = false;
bool previous_home = false;

bool parse(char *text, int values[7]) {
    for (unsigned i = 0; i < 7; ++i) {
        char *end;
        const long value = std::strtol(text, &end, 10);
        if (end == text || value < -1000 || value > 1000)
            return false;
        if (i < 6 && *end != ' ')
            return false;
        values[i] = static_cast<int>(value);
        text = end;
    }
    while (*text == ' ' || *text == '\r')
        ++text;
    return *text == '\0' && values[4] >= -1 && values[4] <= 1 && values[5] >= 0 && values[5] <= 3 &&
           values[6] >= 0 && values[6] <= 1;
}

void readInput() {
    char line[80];
    __disable_irq();
    const bool ready = line_ready;
    if (ready) {
        for (unsigned i = 0; i < sizeof(line); ++i)
            line[i] = received_line[i];
        line_ready = false;
    }
    __enable_irq();
    if (!ready)
        return;
    int values[7];
    if (!parse(line, values))
        return;
    std::copy(values, values + 7, input);
    last_input = HAL_GetTick();
    const bool home = input[6];
    if (home && !previous_home && !homed && motorsReady()) {
        motorsOrigin();
        homed = true;
    }
    previous_home = home;
}
} // namespace

extern "C" void cctl_usbcdc_receive(const uint8_t *data, uint32_t length) {
    static char line[80];
    static unsigned used = 0;
    static bool overflow = false;
    for (uint32_t i = 0; i < length; ++i) {
        if (data[i] == '\n') {
            if (!overflow) {
                line[used] = '\0';
                for (unsigned n = 0; n <= used; ++n)
                    received_line[n] = line[n];
                line_ready = true;
            }
            used = 0;
            overflow = false;
        } else if (used < sizeof(line) - 1)
            line[used++] = data[i];
        else
            overflow = true;
    }
}
extern "C" void setup() {
    motorsBegin();
    eeBegin();
}
extern "C" void loop() {
    motorsReceive();
    readInput();
    const uint32_t now = HAL_GetTick();
    if (now - last_input > config::timeout_ms || !motorsReady())
        homed = false;
    motorsEnable(homed);
    static uint32_t motion_at = 0;
    if (now - motion_at >= 20) {
        const float dt = std::min((now - motion_at) / 1000.0f, 0.02f);
        motion_at = now;
        if (homed)
            motorsMove(input, dt, HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET,
                       HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET);
        eeUpdate(homed, input[3], input[4], input[5], motorPosition(1), dt);
    }
    motorsUpdate(now);
}
