#pragma once
#include <cstdint>

// 配列の順序は r / θ / z。機体座標の単位は mm / deg / mm。
// モータ座標の単位は rad / deg / deg。
namespace config {
constexpr float native_per_unit[3] = {-0.04f, 132.02206f, -96.016f};
constexpr float origin[3] = {570, 0, 0};
// θ（index 1）は原点登録も座標制限も行わない。
constexpr float minimum[3] = {0, 0, 0};
constexpr float maximum[3] = {570, 0, 315};
constexpr float speed[3] = {600, 120, 200};
constexpr float ramp_seconds = 0.2f;
constexpr float target_lead_seconds = 0.1f;
constexpr float contact_direction[3] = {1, 0, -1};
constexpr uint32_t timeout_ms = 250;
constexpr uint32_t feedback_ms = 200;
constexpr uint8_t el05_id = 127, host_id = 253;
constexpr uint8_t theta_id = 1, z_id = 2;
constexpr float position_kp = 10, velocity_kp = 7, velocity_ki = 0.5f, velocity_kd = 0.05f;
constexpr float max_current_ma = 10000;
constexpr float max_rpm[2] = {2640.4412f, 3200.5332f};
constexpr float el05_current = 10, el05_speed = 24, el05_kp = 30;
constexpr uint16_t pwm_can = 0x300, rotation_can = 0x320;
constexpr float fold_min = 400, fold_max = 1444, fold_speed = 1000;
constexpr float grip_open = 1500, grip_closed = 500;
constexpr float rotation_zero = 1024, counts_per_degree = 34.133335f;
constexpr float rotation_speed = 90;
} // namespace config
