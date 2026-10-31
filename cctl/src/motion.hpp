#pragma once
#include "config.hpp"
#include <algorithm>
#include <cmath>

// 1軸の座標と目標。targetはモータの単位、speedは機体の単位/秒。
struct AxisMotion {
    bool referenced = false;
    float offset = 0;
    float target = 0;
    float speed = 0;
};

inline void referenceAxis(AxisMotion &axis, unsigned index, float measured, bool contact) {
    if (index == 1 || axis.referenced || !contact)
        return;
    axis.offset = measured - config::origin[index] * config::native_per_unit[index];
    axis.target = measured;
    axis.speed = 0;
    axis.referenced = true;
}

inline void moveAxis(AxisMotion &axis, unsigned index, float measured, int input, float dt,
                     bool contact) {
    const float native_per_unit = config::native_per_unit[index];
    const float acceleration = config::speed[index] / config::ramp_seconds;
    const float requested_speed = input * config::speed[index] / 1000.0f;
    float speed = axis.speed +
                  std::clamp(requested_speed - axis.speed, -acceleration * dt, acceleration * dt);
    // v² = 2ad から、端点までに停止できる速度を求める。
    if (axis.referenced) {
        const float position = (measured - axis.offset) / native_per_unit;
        const float distance = std::max(0.0f, speed > 0 ? config::maximum[index] - position
                                                        : position - config::minimum[index]);
        const float stopping_speed = std::sqrt(2 * acceleration * distance);
        speed = std::clamp(speed, -stopping_speed, stopping_speed);
    }
    // 接点への押し込みは、原点未登録でも止める。残っている位置目標も解除する。
    const float direction = config::contact_direction[index];
    if (contact &&
        (speed * direction > 0 || (axis.target - measured) / native_per_unit * direction > 0)) {
        speed = 0;
        axis.target = measured;
    }
    if (speed == 0 && axis.speed != 0)
        axis.target = measured;
    if (speed != 0) {
        const float max_target_lead =
            std::abs(config::speed[index] * native_per_unit) * config::target_lead_seconds;
        axis.target = std::clamp(axis.target + speed * native_per_unit * dt,
                                 measured - max_target_lead, measured + max_target_lead);
        if (axis.referenced) {
            const float target = std::clamp((axis.target - axis.offset) / native_per_unit,
                                            config::minimum[index], config::maximum[index]);
            axis.target = axis.offset + target * native_per_unit;
        }
    }
    axis.speed = speed;
}
