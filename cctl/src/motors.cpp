#include "motors.hpp"
#include "can_bus.hpp"
#include "config.hpp"
#include "el05_motor.hpp"
#include "m3508_motor.hpp"
#include <algorithm>
#include <cmath>

extern "C" FDCAN_HandleTypeDef hfdcan1;
namespace {
CanBus bus(&hfdcan1);
El05Motor r(bus, config::el05_id, config::host_id);
M3508Motor theta(config::theta_id, config::max_rpm[0]);
M3508Motor z(config::z_id, config::max_rpm[1]);
float offsets[3]{}, targets[3]{}, speeds[3]{};
uint32_t seen[3]{};
bool received[3]{}, enabled = false;
float measured(unsigned i) {
    return i == 0 ? r.position() : i == 1 ? theta.motorDeg() : z.motorDeg();
}
} // namespace

void motorsBegin() {
    if (!bus.begin())
        Error_Handler();
    r.disable();
    HAL_Delay(50);
    r.setRunMode(El05Motor::RunMode::PositionCsp);
    HAL_Delay(20);
    r.writeParamFloat(domain::el05::param::LIMIT_SPD, config::el05_speed);
    HAL_Delay(20);
    r.writeParamFloat(domain::el05::param::LIMIT_CUR, config::el05_current);
    HAL_Delay(20);
    r.writeParamFloat(domain::el05::param::LOC_KP, config::el05_kp);
    HAL_Delay(20);
    r.requestParam(domain::el05::param::MECH_POS);
}

void motorsReceive() {
    domain::CanFrame frame;
    while (bus.receive(frame)) {
        if (frame.length != 8)
            continue;
        if (frame.extended) {
            const bool feedback = r.onFeedback(frame.id, frame.data);
            const bool position =
                domain::el05::commType(frame.id) == domain::el05::comm::READ_PARAM &&
                domain::el05::targetId(frame.id) == config::host_id &&
                domain::el05::dataArea2(frame.id) == config::el05_id &&
                domain::el05::paramReplyIndex(frame.data) == domain::el05::param::MECH_POS &&
                std::isfinite(domain::el05::paramReplyFloat(frame.data));
            if (feedback || position) {
                seen[0] = HAL_GetTick();
                received[0] = true;
            }
        } else if (frame.id == theta.feedbackId()) {
            theta.onFeedback(frame.data);
            seen[1] = HAL_GetTick();
            received[1] = true;
        } else if (frame.id == z.feedbackId()) {
            z.onFeedback(frame.data);
            seen[2] = HAL_GetTick();
            received[2] = true;
        }
    }
}

bool motorsReady() {
    const auto now = HAL_GetTick();
    for (unsigned i = 0; i < 3; ++i)
        if (!received[i] || now - seen[i] > config::feedback_ms)
            return false;
    return r.positionReady() && r.faultBits() == 0 && theta.temperature() < 80 &&
           z.temperature() < 80;
}

float motorPosition(unsigned i) {
    return (measured(i) - offsets[i]) / config::native_per_unit[i];
}

void motorsHold() {
    for (unsigned i = 0; i < 3; ++i) {
        targets[i] = measured(i);
        speeds[i] = 0;
    }
    theta.setTargetMotorDeg(targets[1]);
    z.setTargetMotorDeg(targets[2]);
    if (enabled)
        r.setLocRef(targets[0]);
}

void motorsOrigin() {
    for (unsigned i = 0; i < 3; ++i)
        offsets[i] = measured(i) - config::origin[i] * config::native_per_unit[i];
    motorsHold();
}

void motorsEnable(bool value) {
    if (enabled == value)
        return;
    motorsHold();
    if (value) {
        r.setLocRef(targets[0]);
        r.enable();
    } else
        r.disable();
    theta.setEnabled(value);
    z.setEnabled(value);
    enabled = value;
}

void motorsMove(const int input[3], float dt, bool r_contact, bool z_contact) {
    for (unsigned i = 0; i < 3; ++i) {
        const float position = motorPosition(i);
        const float acceleration = config::speed[i] / config::ramp_seconds;
        const float desired = input[i] * config::speed[i] / 1000.0f;
        float speed =
            speeds[i] + std::clamp(desired - speeds[i], -acceleration * dt, acceleration * dt);
        const float distance = std::max(0.0f, speed > 0 ? config::maximum[i] - position
                                                        : position - config::minimum[i]);
        const float limit = std::sqrt(2 * acceleration * distance);
        speed = std::clamp(speed, -limit, limit);
        if ((i == 0 && r_contact && speed > 0) || (i == 2 && z_contact && speed < 0))
            speed = 0;
        // 動いていない間の追従誤差を、次の移動へ積み残さない。
        if (speed == 0 && speeds[i] != 0)
            targets[i] = measured(i);
        if (speed != 0) {
            const float native = config::native_per_unit[i];
            const float lead = std::abs(config::speed[i] * native) * 0.1f;
            targets[i] = std::clamp(targets[i] + speed * native * dt, measured(i) - lead,
                                    measured(i) + lead);
            const float unit_target = std::clamp((targets[i] - offsets[i]) / native,
                                                 config::minimum[i], config::maximum[i]);
            targets[i] = offsets[i] + unit_target * native;
        }
        speeds[i] = speed;
    }
    theta.setTargetMotorDeg(targets[1]);
    z.setTargetMotorDeg(targets[2]);
}

void motorsUpdate(uint32_t now) {
    static uint32_t current_at = 0, position_at = 0;
    if (now - current_at >= 1) {
        uint8_t data[8]{};
        domain::c620::writeCommandSlot(data, config::theta_id, theta.commandRaw());
        domain::c620::writeCommandSlot(data, config::z_id, z.commandRaw());
        bus.sendStd(0x200, data, 8);
        current_at = now;
    }
    if (now - position_at >= 20) {
        if (enabled)
            r.setLocRef(targets[0]);
        else
            r.disable(false);
        r.requestParam(domain::el05::param::MECH_POS);
        position_at = now;
    }
}
