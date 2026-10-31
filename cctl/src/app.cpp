#include "app.h"
#include "ee.hpp"
#include "input.hpp"
#include "main.h"
#include "motors.hpp"
#include <algorithm>

namespace {
volatile char received_line[80];
volatile bool line_ready = false;
int input[6]{};
uint32_t last_input = 0;
bool input_received = false;

// 受信した最新の1行を検査し、正常なら操作入力と受信時刻を更新する。
void readInput() {
    char line[80];
    // USB割込みとの競合を防ぐ。割込み禁止区間はコピー中だけ。
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

// USB受信は行の途中で分割される。改行まで組み立て、最新の完全な1行を渡す。
// 長すぎる行は、末尾を別の指令と誤認しないよう改行まで捨てる。
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
// モータとサーボ基板につながるCAN通信を開始する。
extern "C" void setup() {
    motorsBegin();
    eeBegin();
}
// 入力とモータ応答から出力の可否を判定し、軸とサーボへの指令を更新する。
extern "C" void loop() {
    motorsReceive();
    readInput();
    const uint32_t now = HAL_GetTick();
    const bool r_contact = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET;
    const bool z_contact = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET;
    // 接点はLowでON。PC未接続でも原点を登録する。
    motorsReference(r_contact, z_contact);
    const bool enabled = outputsEnabled(input_received, last_input, now, motorsReady());
    motorsEnable(enabled);
    static uint32_t motion_at = 0;
    if (now - motion_at >= 20) {
        // 遅延分を一度に動かさず、目標の更新量を20ms分に抑える。
        const float dt = std::min((now - motion_at) / 1000.0f, 0.02f);
        motion_at = now;
        if (enabled)
            motorsMove(input, dt, r_contact, z_contact);
        eeUpdate(enabled, input[3], input[4], input[5], dt);
    }
    motorsUpdate(now);
}
