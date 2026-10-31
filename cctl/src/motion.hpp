#pragma once
#include "config.hpp"
#include <algorithm>
#include <cmath>

// 1軸の座標と目標。targetはモータの単位、speedは機体の単位/秒。
struct AxisMotion {
    bool referenced = false;
    // モータ座標 = offset + 機体座標 × native_per_unit。原点登録時に確定する。
    float offset = 0;
    float target = 0;
    float speed = 0;
};

// indexは r=0 / θ=1 / z=2。measuredはモータ座標。
inline void referenceAxis(AxisMotion &axis, unsigned index, float measured, bool contact) {
    if (index == 1 || axis.referenced || !contact)
        return;
    axis.offset = measured - config::origin[index] * config::native_per_unit[index];
    // 原点登録時に、接触前の移動目標を解除する。
    axis.target = measured;
    axis.speed = 0;
    axis.referenced = true;
}

// inputは±1000、dtは秒。速度は機体単位（mm/s・deg/s）、目標はモータ単位で扱う。
inline void moveAxis(AxisMotion &axis, unsigned index, float measured, int input, float dt,
                     bool contact) {
    const float native_per_unit = config::native_per_unit[index];
    const float acceleration = config::speed[index] / config::ramp_seconds;
    const float requested_speed = input * config::speed[index] / 1000.0f;
    const float max_speed_change = acceleration * dt;
    float speed =
        axis.speed + std::clamp(requested_speed - axis.speed, -max_speed_change, max_speed_change);
    // v² = 2ad から、端点までに停止できる速度を求める。
    if (axis.referenced) {
        const float position = (measured - axis.offset) / native_per_unit;
        const float distance_to_endpoint =
            speed > 0 ? config::maximum[index] - position : position - config::minimum[index];
        const float distance = std::max(0.0f, distance_to_endpoint);
        const float stopping_speed = std::sqrt(2 * acceleration * distance);
        speed = std::clamp(speed, -stopping_speed, stopping_speed);
    }
    // 接点への押し込みは、原点未登録でも止める。残っている位置目標も解除する。
    const float direction = config::contact_direction[index];
    const float target_lead = (axis.target - measured) / native_per_unit;
    const bool moving_into_contact = speed * direction > 0;
    const bool targeting_past_contact = target_lead * direction > 0;
    if (contact && (moving_into_contact || targeting_past_contact)) {
        speed = 0;
        axis.target = measured;
    }
    // 減速が完了した位置を保持し、先行していた目標への移動を止める。
    if (speed == 0 && axis.speed != 0)
        axis.target = measured;
    if (speed != 0) {
        // モータの追従が遅れたときに、目標だけが先へ進むのを防ぐ。
        const float max_target_lead =
            std::abs(config::speed[index] * native_per_unit) * config::target_lead_seconds;
        axis.target = std::clamp(axis.target + speed * native_per_unit * dt,
                                 measured - max_target_lead, measured + max_target_lead);
        if (axis.referenced) {
            // 換算係数が負の軸もあるので、上下限は機体座標に戻して適用する。
            const float limited_position =
                std::clamp((axis.target - axis.offset) / native_per_unit, config::minimum[index],
                           config::maximum[index]);
            axis.target = axis.offset + limited_position * native_per_unit;
        }
    }
    axis.speed = speed;
}
