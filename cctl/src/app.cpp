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

// USB受信データを操縦に使う入力として受け入れる。不正な行では通信の有効期限も延ばさない。
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

// USB割込みから制御ループへ指令を渡す。分割して届くデータを、改行までそろえて引き渡す。
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
// CCTLの起動処理。軸モータとEEを、制御ループから指令できる状態にする。
extern "C" void setup() {
    statusBegin();
    motorsBegin();
    eeBegin();
}
// CCTLの制御ループ。原点登録、出力の許可、軸とEEの周期更新をここでまとめる。
extern "C" void loop() {
    motorsReceive();
    readInput();
    const uint32_t now = HAL_GetTick();
    const bool r_contact = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET;
    const bool z_contact = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET;
    // 接点はLowでON。PC未接続でも原点を登録する。
    const auto status = motorsStatus(now);
    motorsReference(r_contact, z_contact, status);
    const bool fresh = inputFresh(input_received, last_input, now);
    static uint32_t motion_at = 0, current_at = 0, status_at = 0;
    if (now - motion_at >= 20) {
        // 軸の加減速とEEの姿勢を20ms分ずつ更新する。
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
