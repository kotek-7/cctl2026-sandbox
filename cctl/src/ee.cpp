#include "ee.hpp"
#include "can_bus.hpp"
#include "config.hpp"
#include <algorithm>
#include <cmath>
extern "C" FDCAN_HandleTypeDef hfdcan2;
namespace {
CanBus bus(&hfdcan2);
// pulseはサーボへ送るパルス幅（µs）。angleは開始姿勢からの回転角（度）。
// 出力を切ってもこの値を残し、再開時に同じ姿勢を指令する。
float fold_pulse = 500;
float grip_pulse = 500;
float rotation_angle = 0;
} // namespace
// EEを動かす2枚のサーボ基板との通信を準備する。失敗したらError_Handlerを呼ぶ。
void eeBegin() {
    if (!bus.begin())
        Error_Handler();
}
// EEの回転・畳み・把持を受け持つ。姿勢目標を保ちながら、出力の可否に応じたCAN指令を送る。
void eeUpdate(bool enabled, int rotate, int fold, int grip, float dt) {
    if (enabled) {
        fold_pulse = std::clamp(fold_pulse + fold * config::fold_speed * dt, config::fold_min,
                                config::fold_max);
        if (grip == 1)
            grip_pulse = config::grip_open;
        if (grip == 2)
            grip_pulse = config::grip_closed;
        rotation_angle += rotate * config::rotation_speed * dt / 1000.0f;
        rotation_angle =
            std::clamp(rotation_angle, (-28672 - config::rotation_zero) / config::counts_per_degree,
                       (28672 - config::rotation_zero) / config::counts_per_degree);
    }
    // PWMは畳み1本、把持3本の順。各値は上位バイトから送り、0で出力を切る。
    uint8_t pwm[8]{};
    if (enabled) {
        for (unsigned i = 0; i < 4; ++i) {
            const uint16_t pulse = static_cast<uint16_t>(i == 0 ? fold_pulse : grip_pulse);
            pwm[2 * i] = pulse >> 8;
            pwm[2 * i + 1] = pulse;
        }
    }
    bus.sendStd(config::pwm_can, pwm, 8);
    // 回転指令は出力ON/OFFと16bitの位置。負数は2の補数で送る。
    // サーボが使う「符号+絶対値」への変換はSerialSVMDが行う。
    const int16_t position =
        std::lround(config::rotation_zero + rotation_angle * config::counts_per_degree);
    const uint16_t bits = static_cast<uint16_t>(position);
    const uint8_t sts[3] = {static_cast<uint8_t>(enabled), static_cast<uint8_t>(bits >> 8),
                            static_cast<uint8_t>(bits)};
    bus.sendStd(config::rotation_can, sts, 3);
}
