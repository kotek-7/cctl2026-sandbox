#include "status.hpp"
#include "main.h"
#include "usbd_cdc_if.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

extern "C" I2C_HandleTypeDef hi2c1;
extern "C" USBD_HandleTypeDef hUsbDeviceFS;
namespace {
constexpr uint16_t lcd_address = 0x3e << 1;

// AQM1602の表示位置や表示モードを設定する。
bool lcdCommand(uint8_t value) {
    uint8_t data[2]{0, value};
    return HAL_I2C_Master_Transmit(&hi2c1, lcd_address, data, sizeof(data), 2) == HAL_OK;
}

// LCDの表示を、変化した行だけ書き換える。
bool lcdLine(unsigned row, const char *text) {
    static uint8_t displayed[2][16]{};
    uint8_t data[17];
    data[0] = 0x40;
    std::fill(data + 1, data + 17, ' ');
    std::memcpy(data + 1, text, std::min(std::strlen(text), size_t{16}));
    if (std::memcmp(displayed[row], data + 1, 16) == 0)
        return true;
    if (!lcdCommand(row == 0 ? 0x80 : 0xc0) ||
        HAL_I2C_Master_Transmit(&hi2c1, lcd_address, data, sizeof(data), 3) != HAL_OK)
        return false;
    std::memcpy(displayed[row], data + 1, 16);
    return true;
}
} // namespace

// 状態表示用のLCDを初期化する。
void statusBegin() {
    HAL_Delay(40);
    for (uint8_t command : {0x38, 0x39, 0x14, 0x7f, 0x5e, 0x6c})
        lcdCommand(command);
    HAL_Delay(200);
    lcdCommand(0x38);
    lcdCommand(0x0c);
    lcdCommand(0x01);
    HAL_Delay(2);
    lcdCommand(0x06);
}

// 通信状態とrの異常コードをLCDとPCへ送る。
void statusUpdate(bool input_received, bool input_fresh, const MotorStatus &motors) {
    const char *pc = !input_received ? "WAIT" : input_fresh ? "OK" : "LOST";
    const char *r = motors.feedback_ok[0] ? "OK" : "NO";
    const char *theta = motors.feedback_ok[1] ? "OK" : "NO";
    const char *z = motors.feedback_ok[2] ? "OK" : "NO";
    char line[17];
    std::snprintf(line, sizeof(line), "PC:%-4s RERR:%02X", pc, unsigned(motors.r_fault));
    const bool first_line = lcdLine(0, line);
    std::snprintf(line, sizeof(line), "R:%s T:%s Z:%s", r, theta, z);
    const bool lcd_ok = first_line && lcdLine(1, line);

    char message[128];
    const int length = std::snprintf(
        message, sizeof(message), "PC=%s R_FB=%s THETA_FB=%s Z_FB=%s R_FAULT=0x%02X LCD=%s\n", pc,
        r, theta, z, unsigned(motors.r_fault), lcd_ok ? "OK" : "ERROR");
    // USB送信完了までバッファを保持する。
    // CDCへのアクセス中に切断割込みが入るのを防ぐ。
    __disable_irq();
    const auto *cdc = static_cast<USBD_CDC_HandleTypeDef *>(hUsbDeviceFS.pClassData);
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || !cdc || cdc->TxState != 0) {
        __enable_irq();
        return;
    }
    static char sending[128];
    std::memcpy(sending, message, length);
    CDC_Transmit_FS(reinterpret_cast<uint8_t *>(sending), static_cast<uint16_t>(length));
    __enable_irq();
}
