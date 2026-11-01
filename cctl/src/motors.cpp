#include "motors.hpp"
#include "can_bus.hpp"
#include "config.hpp"
#include "domain/c620_codec.hpp"
#include "el05_motor.hpp"
#include "motion.hpp"
#include <cmath>

extern "C" FDCAN_HandleTypeDef hfdcan1;
namespace {
CanBus bus(&hfdcan1);
El05Motor r(bus, config::el05_id, config::host_id);
domain::c620::Feedback theta{}, z{};
domain::c620::MultiTurnCounter z_angle;
AxisMotion axes[3];
uint32_t last_feedback[3]{};
bool feedback_received[3]{}, r_enabled = false;
} // namespace

// モータ用CANを開始し、rの速度制御を設定する。
void motorsBegin() {
    if (!bus.begin())
        Error_Handler();
    r.disable();
    HAL_Delay(50);
    r.setVelocityMode();
    HAL_Delay(20);
    r.setSpeed(0);
    r.writeParamFloat(domain::el05::param::LIMIT_CUR, config::el05_current);
    HAL_Delay(20);
    r.writeParamFloat(domain::el05::param::ACC_RAD,
                      std::abs(config::speed[0] * config::native_per_unit[0]) /
                          config::ramp_seconds);
    HAL_Delay(20);
}

// モータの返信から実測値と異常コードを更新する。
void motorsReceive() {
    domain::CanFrame frame;
    while (bus.receive(frame)) {
        if (frame.length != 8)
            continue;
        if (frame.extended) {
            if (r.onFeedback(frame.id, frame.data)) {
                last_feedback[0] = HAL_GetTick();
                feedback_received[0] = true;
            }
        } else if (frame.id == 0x200 + config::theta_id) {
            theta = domain::c620::decodeFeedback(frame.data);
            last_feedback[1] = HAL_GetTick();
            feedback_received[1] = true;
        } else if (frame.id == 0x200 + config::z_id) {
            z = domain::c620::decodeFeedback(frame.data);
            z_angle.update(z.raw_angle);
            last_feedback[2] = HAL_GetTick();
            feedback_received[2] = true;
        }
    }
}

// LCDとPCへ表示するモータの通信状態をまとめる。
MotorStatus motorsStatus(uint32_t now) {
    MotorStatus status{{}, r.faultBits()};
    for (unsigned i = 0; i < 3; ++i)
        status.feedback_ok[i] =
            feedback_received[i] && now - last_feedback[i] <= config::feedback_ms;
    return status;
}

// 位置を受信できている軸の原点を、最初のスイッチ接触で登録する。
void motorsReference(bool r_contact, bool z_contact, const MotorStatus &status) {
    if (status.feedback_ok[0])
        referenceAxis(axes[0], 0, r.position(), r_contact);
    if (status.feedback_ok[2])
        referenceAxis(axes[2], 2, z_angle.degrees(), z_contact);
}

// 3軸の速度指令を更新し、rへ運転・停止・異常解除の指令を送る。
void motorsStep(bool active, const int input[3], float dt, bool r_contact, bool z_contact) {
    const bool run_r = active && r.faultBits() == 0;
    if (run_r && !r_enabled) {
        r.setSpeed(0);
        r.enable();
    }
    r_enabled = run_r;
    const float position[3]{r.position(), 0, z_angle.degrees()};
    const bool contact[3]{r_contact, false, z_contact};
    for (unsigned i = 0; i < 3; ++i) {
        if (active && (i != 0 || run_r))
            moveAxis(axes[i], i, position[i], input[i], dt, contact[i]);
        else
            axes[i].speed = 0;
    }
    if (run_r)
        r.setSpeed(axes[0].speed * config::native_per_unit[0]);
    else
        r.disable(r.faultBits() != 0);
}

// θ・zの速度制御に必要な電流をC620へ送る。
void motorsCurrent(bool active) {
    uint8_t data[8]{};
    if (active) {
        const int32_t theta_current =
            motorCurrent(axes[1].speed * config::native_per_unit[1] / 6, theta.rpm);
        const int32_t z_current =
            motorCurrent(axes[2].speed * config::native_per_unit[2] / 6, z.rpm);
        domain::c620::writeCommandSlot(data, config::theta_id,
                                       domain::c620::currentToRaw(theta_current));
        domain::c620::writeCommandSlot(data, config::z_id, domain::c620::currentToRaw(z_current));
    }
    bus.sendStd(0x200, data, 8);
}
