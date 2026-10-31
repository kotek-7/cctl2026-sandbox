#include "app.h"
#include "ee.hpp"
#include "input.hpp"
#include "main.h"
#include "motors.hpp"
#include <algorithm>

namespace {
// USB割込みはコピーだけ。行の解析と機体操作はloopで行う。
volatile char received_line[80];
volatile bool line_ready = false;
int input[6]{};
uint32_t last_input = 0;
bool input_received = false;

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
    int values[6];
    if (!parseInput(line, values))
        return;
    std::copy(values, values + 6, input);
    last_input = HAL_GetTick();
    input_received = true;
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
    const bool r_contact = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET;
    const bool z_contact = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET;
    motorsReference(r_contact, z_contact);
    const bool enabled = outputsEnabled(input_received, last_input, now, motorsReady());
    motorsEnable(enabled);
    static uint32_t motion_at = 0;
    if (now - motion_at >= 20) {
        const float dt = std::min((now - motion_at) / 1000.0f, 0.02f);
        motion_at = now;
        if (enabled)
            motorsMove(input, dt, r_contact, z_contact);
        eeUpdate(enabled, input[3], input[4], input[5], dt);
    }
    motorsUpdate(now);
}
