#pragma once
#include "config.hpp"
#include <algorithm>
#include <cmath>

// speedは機体への速度指令（r・zはmm/s、θは度/s）。
struct AxisMotion {
    bool referenced = false;
    // スイッチに当たった位置から、モータ角度と機体の位置のずれを求める。
    // モータ角度 = offset + 機体の位置 × native_per_unit。
    float offset = 0;
    float speed = 0;
};

// モータの角度と機体の位置を、最初の接点で対応付ける。以後はこの対応でr・zの可動範囲を決める。
// indexは r=0 / θ=1 / z=2。measuredはモータから読んだ角度。
inline void referenceAxis(AxisMotion &axis, unsigned index, float measured, bool contact) {
    if (index == 1 || axis.referenced || !contact)
        return;
    axis.offset = measured - config::origin[index] * config::native_per_unit[index];
    axis.speed = 0;
    axis.referenced = true;
}

// 1軸の手動移動を計算する。操作入力から速度を求め、加減速・接点・可動範囲で動きを制限する。
// inputは±1000、dtは秒。
inline void moveAxis(AxisMotion &axis, unsigned index, float measured, int input, float dt,
                     bool contact) {
    const float native_per_unit = config::native_per_unit[index];
    const float acceleration = config::speed[index] / config::ramp_seconds;
    const float requested_speed = input * config::speed[index] / 1000.0f;
    const float max_speed_change = acceleration * dt;
    float speed =
        axis.speed + std::clamp(requested_speed - axis.speed, -max_speed_change, max_speed_change);
    // 残り距離dと減速度aから、端点で止まれる速度の上限v = √(2ad)を求める。
    if (axis.referenced) {
        const float position = (measured - axis.offset) / native_per_unit;
        const float distance_to_endpoint =
            speed > 0 ? config::maximum[index] - position : position - config::minimum[index];
        const float distance = std::max(0.0f, distance_to_endpoint);
        const float stopping_speed = std::sqrt(2 * acceleration * distance);
        speed = std::clamp(speed, -stopping_speed, stopping_speed);
    }
    // 接点から離れる方向は許可し、押し込む方向の速度だけ0にする。
    if (contact && speed * config::contact_direction[index] > 0)
        speed = 0;
    axis.speed = speed;
}
