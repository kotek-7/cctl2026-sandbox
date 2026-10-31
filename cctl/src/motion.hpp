#pragma once
#include "config.hpp"
#include <algorithm>
#include <cmath>

// targetはモータの目標角度。speedは機体の移動速度（r・zはmm/s、θは度/s）。
struct AxisMotion {
    bool referenced = false;
    // スイッチに当たった位置から、モータ角度と機体の位置のずれを求める。
    // モータ角度 = offset + 機体の位置 × native_per_unit。
    float offset = 0;
    float target = 0;
    float speed = 0;
};

// 最初の接点検出時にr・zの原点を登録する。目標を現在位置に合わせ、速度を0にする。
// indexは r=0 / θ=1 / z=2。measuredはモータから読んだ角度。
inline void referenceAxis(AxisMotion &axis, unsigned index, float measured, bool contact) {
    if (index == 1 || axis.referenced || !contact)
        return;
    axis.offset = measured - config::origin[index] * config::native_per_unit[index];
    axis.target = measured;
    axis.speed = 0;
    axis.referenced = true;
}

// 入力と経過時間から1軸の速度・位置目標を更新し、接点と可動範囲で制限する。
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
    // 原点登録前でも、スイッチを押し込む方向は止める。
    // 速度を0にするだけでは前の目標へ動くため、目標も現在位置に戻す。
    const float direction = config::contact_direction[index];
    const float target_lead = (axis.target - measured) / native_per_unit;
    const bool moving_into_contact = speed * direction > 0;
    const bool targeting_past_contact = target_lead * direction > 0;
    if (contact && (moving_into_contact || targeting_past_contact)) {
        speed = 0;
        axis.target = measured;
    }
    // 速度が0になったら現在位置を目標にする。前の目標を残すと、そこまで動き続ける。
    if (speed == 0 && axis.speed != 0)
        axis.target = measured;
    if (speed != 0) {
        // モータが動けない間も入力は届くため、目標が実測位置から離れすぎないようにする。
        const float max_target_lead =
            std::abs(config::speed[index] * native_per_unit) * config::target_lead_seconds;
        axis.target = std::clamp(axis.target + speed * native_per_unit * dt,
                                 measured - max_target_lead, measured + max_target_lead);
        if (axis.referenced) {
            // r・zはモータの回転方向と機体の移動方向が逆。
            // モータ角度をmmに戻してから、機体の可動範囲に収める。
            const float limited_position =
                std::clamp((axis.target - axis.offset) / native_per_unit, config::minimum[index],
                           config::maximum[index]);
            axis.target = axis.offset + limited_position * native_per_unit;
        }
    }
    axis.speed = speed;
}
