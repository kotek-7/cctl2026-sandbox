#pragma once
#include "config.hpp"
#include <algorithm>
#include <cmath>

// speedは機体への速度指令（r・zはmm/s、θは度/s）。
struct AxisMotion {
    bool referenced = false;
    // モータ角度 = offset + 機体の位置 × native_per_unit。
    float offset = 0;
    float speed = 0;
};

// 最初のスイッチ接触でr・zの原点と位置制限を確定する。
// indexは r=0 / θ=1 / z=2。measuredはモータから読んだ角度。
inline void referenceAxis(AxisMotion &axis, unsigned index, float measured, bool contact) {
    if (index == 1 || axis.referenced || !contact)
        return;
    axis.offset = measured - config::origin[index] * config::native_per_unit[index];
    axis.speed = 0;
    axis.referenced = true;
}

// パッド入力から、加減速と移動範囲を考慮した軸の速度指令を求める。
// inputは±1000、dtは秒。
inline void moveAxis(AxisMotion &axis, unsigned index, float measured, int input, float dt,
                     bool contact) {
    const float native_per_unit = config::native_per_unit[index];
    const float acceleration = config::speed[index] / config::ramp_seconds;
    const float requested_speed = input * config::speed[index] / 1000.0f;
    // rの加減速はモータ内部で行う。
    float speed = requested_speed;
    if (index != 0) {
        const float max_speed_change = acceleration * dt;
        speed = axis.speed +
                std::clamp(requested_speed - axis.speed, -max_speed_change, max_speed_change);
    }
    // 端点までの距離から、停止できる速度を求める。
    if (axis.referenced) {
        const float position = (measured - axis.offset) / native_per_unit;
        const float distance_to_endpoint =
            speed > 0 ? config::maximum[index] - position : position - config::minimum[index];
        const float distance = std::max(0.0f, distance_to_endpoint);
        const float stopping_speed = std::sqrt(2 * acceleration * distance);
        speed = std::clamp(speed, -stopping_speed, stopping_speed);
    }
    // 押されているスイッチから離れる方向には動かせる。
    if (contact && speed * config::contact_direction[index] > 0)
        speed = 0;
    axis.speed = speed;
}

// θ・zの速度を制御する電流[mA]を求める。
inline int32_t motorCurrent(float target_rpm, float measured_rpm) {
    return static_cast<int32_t>(std::clamp(config::velocity_kp * (target_rpm - measured_rpm),
                                           -config::max_current_ma, config::max_current_ma));
}
