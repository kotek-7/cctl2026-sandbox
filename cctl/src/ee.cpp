#include "ee.hpp"
#include "can_bus.hpp"
#include "config.hpp"
#include <algorithm>
#include <cmath>
extern "C" FDCAN_HandleTypeDef hfdcan2;
namespace {
CanBus bus(&hfdcan2);
float fold_pulse = 500, grip_pulse = 500, field_angle = 0;
bool was_running = false;
} // namespace
void eeBegin() {
    if (!bus.begin())
        Error_Handler();
}
void eeUpdate(bool running, int rotate, int fold, int grip, float theta, float dt) {
    // 再開時の回転目標は前回値を保持し、θ補償だけを新しい姿勢に合わせる。
    static float rotation_angle = 0;
    if (running && !was_running)
        field_angle = theta + rotation_angle;
    was_running = running;
    if (running) {
        fold_pulse = std::clamp(fold_pulse + fold * config::fold_speed * dt, config::fold_min,
                                config::fold_max);
        if (grip == 1)
            grip_pulse = config::grip_open;
        if (grip == 2)
            grip_pulse = config::grip_closed;
        if (grip == 3)
            grip_pulse = config::grip_handoff;
        field_angle += rotate * config::rotation_speed * dt / 1000.0f;
        rotation_angle = std::clamp(field_angle - theta,
                                    (-28672 - config::rotation_zero) / config::counts_per_degree,
                                    (28672 - config::rotation_zero) / config::counts_per_degree);
        field_angle = theta + rotation_angle;
    }
    uint8_t pwm[8]{};
    for (unsigned i = 0; i < 4; ++i) {
        const uint16_t pulse =
            running ? static_cast<uint16_t>(i == 0 ? fold_pulse : grip_pulse) : 0;
        pwm[2 * i] = pulse >> 8;
        pwm[2 * i + 1] = pulse;
    }
    bus.sendStd(config::pwm_can, pwm, 8);
    const int16_t position =
        std::lround(config::rotation_zero + rotation_angle * config::counts_per_degree);
    const uint16_t bits = static_cast<uint16_t>(position);
    const uint8_t sts[3] = {static_cast<uint8_t>(running), static_cast<uint8_t>(bits >> 8),
                            static_cast<uint8_t>(bits)};
    bus.sendStd(config::rotation_can, sts, 3);
}
