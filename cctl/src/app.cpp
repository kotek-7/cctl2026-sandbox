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
    // コピー中にUSB割込みが行を書き換えないよう、受け渡し部分だけ割込みを止める。
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
    // 全項目が正常な行だけを採用する。不正な行では通信の有効期限も延長しない。
    int values[6];
    if (!parseInput(line, values))
        return;
    std::copy(values, values + 6, input);
    last_input = HAL_GetTick();
    input_received = true;
}
} // namespace

// USB受信は行の途中で分割される。改行まで組み立て、最新の完全な1行を渡す。
// 長すぎる行は改行まで捨て、途中の文字列を別の指令として解釈しない。
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
    // 接点はLowでON。原点はPC入力の有無にかかわらず、軸の位置が有効なら登録する。
    motorsReference(r_contact, z_contact);
    // 出力条件は毎ループ評価する。条件回復時も登録済みの原点はそのまま使う。
    const bool enabled = outputsEnabled(input_received, last_input, now, motorsReady());
    motorsEnable(enabled);
    static uint32_t motion_at = 0;
    if (now - motion_at >= 20) {
        // ループが遅れても、1回の更新で20ms分を超える移動目標を積まない。
        const float dt = std::min((now - motion_at) / 1000.0f, 0.02f);
        motion_at = now;
        if (enabled)
            motorsMove(input, dt, r_contact, z_contact);
        eeUpdate(enabled, input[3], input[4], input[5], dt);
    }
    motorsUpdate(now);
}
