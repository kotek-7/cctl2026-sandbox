#include "app.h"
#include "ee.hpp"
#include "input.hpp"
#include "main.h"
#include "motors.hpp"
#include "status.hpp"
#include <algorithm>

namespace {
volatile char received_line[80];
volatile bool line_ready = false;
int input[6]{};
uint32_t last_input = 0;
bool input_received = false;

// PCから届いた最新の有効な指令を操作入力にする。
void readInput() {
    char line[80];
    // 受信割込みが書き換えるデータをコピーする間は割込みを止める。
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

// USB受信を行単位にまとめ、制御ループへ渡す。
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
// LCDとモータ・サーボ基板への通信を初期化する。
extern "C" void setup() {
    statusBegin();
    motorsBegin();
    eeBegin();
}
// 入力の受信、原点登録、軸・EE・表示の周期更新を行う。
extern "C" void loop() {
    motorsReceive();
    readInput();
    const uint32_t now = HAL_GetTick();
    const bool r_contact = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET;
    const bool z_contact = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET;
    // スイッチはLowでON。
    const auto status = motorsStatus(now);
    motorsReference(r_contact, z_contact, status);
    const bool fresh = inputFresh(input_received, last_input, now);
    static uint32_t motion_at = 0, current_at = 0, status_at = 0;
    if (now - motion_at >= 20) {
        constexpr float dt = 0.02f;
        motion_at = now;
        expireInput(fresh, input);
        motorsStep(input_received, input, dt, r_contact, z_contact);
        eeUpdate(input_received, input[3], input[4], input[5], dt);
    }
    if (now - current_at >= 1) {
        current_at = now;
        motorsCurrent(input_received);
    }
    if (now - status_at >= 250) {
        status_at = now;
        statusUpdate(input_received, fresh, status);
    }
}
