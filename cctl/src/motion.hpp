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

inline void referenceAxis(AxisMotion &axis, unsigned i, float measured, bool contact) {
    if (i == 1 || axis.referenced || !contact)
        return;
    axis.offset = measured - config::origin[i] * config::native_per_unit[i];
    axis.target = measured;
    axis.speed = 0;
    axis.referenced = true;
}

inline void moveAxis(AxisMotion &axis, unsigned i, float measured, int input, float dt,
                     bool contact) {
    const float native = config::native_per_unit[i];
    const float acceleration = config::speed[i] / config::ramp_seconds;
    const float desired = input * config::speed[i] / 1000.0f;
    float speed =
        axis.speed + std::clamp(desired - axis.speed, -acceleration * dt, acceleration * dt);
    if (axis.referenced) {
        const float position = (measured - axis.offset) / native;
        const float distance = std::max(0.0f, speed > 0 ? config::maximum[i] - position
                                                        : position - config::minimum[i]);
        const float limit = std::sqrt(2 * acceleration * distance);
        speed = std::clamp(speed, -limit, limit);
    }
    // 接点への押し込みは、原点未登録でも止める。残っている位置目標も解除する。
    const float direction = i == 0 ? 1.0f : i == 2 ? -1.0f : 0.0f;
    if (contact && (speed * direction > 0 || (axis.target - measured) / native * direction > 0)) {
        speed = 0;
        axis.target = measured;
    }
    if (speed == 0 && axis.speed != 0)
        axis.target = measured;
    if (speed != 0) {
        const float lead = std::abs(config::speed[i] * native) * 0.1f;
        axis.target =
            std::clamp(axis.target + speed * native * dt, measured - lead, measured + lead);
        if (axis.referenced) {
            const float target = std::clamp((axis.target - axis.offset) / native,
                                            config::minimum[i], config::maximum[i]);
            axis.target = axis.offset + target * native;
        }
    }
    axis.speed = speed;
}
