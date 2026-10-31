#include "ee.hpp"
#include "can_bus.hpp"
#include "config.hpp"
#include <algorithm>
#include <cmath>
extern "C" FDCAN_HandleTypeDef hfdcan2;
namespace {
CanBus bus(&hfdcan2);
// PWM幅はµs、回転角は初期カウントからの相対角度（deg）。停止中も目標を保持する。
float fold_pulse = 500, grip_pulse = 500, rotation_angle = 0;
} // namespace
void eeBegin() {
    if (!bus.begin())
        Error_Handler();
}
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
    // CAN 0x300: 畳み1本・把持3本のPWM幅を16bit上位byteから送る。0は出力解除。
    uint8_t pwm[8]{};
    for (unsigned i = 0; i < 4; ++i) {
        const uint16_t pulse =
            enabled ? static_cast<uint16_t>(i == 0 ? fold_pulse : grip_pulse) : 0;
        pwm[2 * i] = pulse >> 8;
        pwm[2 * i + 1] = pulse;
    }
    bus.sendStd(config::pwm_can, pwm, 8);
    // CAN 0x320: 有効フラグ + 符号付き16bit位置。STS固有の符号形式への変換は受信側。
    const int16_t position =
        std::lround(config::rotation_zero + rotation_angle * config::counts_per_degree);
    const uint16_t bits = static_cast<uint16_t>(position);
    const uint8_t sts[3] = {static_cast<uint8_t>(enabled), static_cast<uint8_t>(bits >> 8),
                            static_cast<uint8_t>(bits)};
    bus.sendStd(config::rotation_can, sts, 3);
}
