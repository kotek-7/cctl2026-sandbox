#include "motors.hpp"
#include "can_bus.hpp"
#include "config.hpp"
#include "el05_motor.hpp"
#include "m3508_motor.hpp"
#include "motion.hpp"
#include <cmath>

extern "C" FDCAN_HandleTypeDef hfdcan1;
namespace {
CanBus bus(&hfdcan1);
El05Motor r(bus, config::el05_id, config::host_id);
M3508Motor theta(config::theta_id), z(config::z_id);
AxisMotion axes[3];
uint32_t last_feedback[3]{};
bool feedback_received[3]{}, feedback_stale[3]{}, enabled[3]{};

// 原点登録と端点減速に使うモータ角度。rはrad、θ・zは度。
float measured(unsigned i) {
    switch (i) {
    case 0:
        return r.position();
    case 1:
        return theta.motorDeg();
    default:
        return z.motorDeg();
    }
}
} // namespace

// rを速度モードに設定する。起動中は出力を切り、速度指令も0にする。
void motorsBegin() {
    if (!bus.begin())
        Error_Handler();
    r.disable();
    HAL_Delay(50);
    r.setRunMode(El05Motor::RunMode::Velocity);
    HAL_Delay(20);
    r.setSpeed(0);
    r.writeParamFloat(domain::el05::param::LIMIT_CUR, config::el05_current);
    HAL_Delay(20);
    r.writeParamFloat(domain::el05::param::ACC_RAD,
                      std::abs(config::speed[0] * config::native_per_unit[0]) /
                          config::ramp_seconds);
    HAL_Delay(20);
}

// 制御用の実測値と、r自身が報告した異常を受け取る。
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
        } else if (frame.id == theta.feedbackId()) {
            theta.onFeedback(frame.data);
            last_feedback[1] = HAL_GetTick();
            feedback_received[1] = true;
        } else if (frame.id == z.feedbackId()) {
            z.onFeedback(frame.data);
            last_feedback[2] = HAL_GetTick();
            feedback_received[2] = true;
        }
    }
    // 返信途絶は記録するが、出力を止めない。古い位置での原点登録には使わない。
    for (unsigned i = 0; i < 3; ++i)
        feedback_stale[i] =
            feedback_received[i] && HAL_GetTick() - last_feedback[i] > config::feedback_ms;
}

// 接点でr・zの座標を決める。他軸やPCの接続を待たず、最初の接触だけを使う。
void motorsReference(bool r_contact, bool z_contact) {
    if (feedback_received[0] && !feedback_stale[0])
        referenceAxis(axes[0], 0, measured(0), r_contact);
    if (feedback_received[2] && !feedback_stale[2])
        referenceAxis(axes[2], 2, measured(2), z_contact);
}

// 初回PC入力から出力する。モータ応答は開始条件にせず、rの異常ビットだけを出力停止に使う。
void motorsEnable(bool input_received) {
    for (unsigned i = 0; i < 3; ++i) {
        const bool requested = input_received && (i != 0 || r.faultBits() == 0);
        if (enabled[i] == requested)
            continue;
        axes[i].speed = 0;
        if (i == 0) {
            r.setSpeed(0);
            if (requested)
                r.enable();
            else
                r.disable(r.faultBits() != 0);
        } else if (i == 1) {
            theta.setTargetRpm(0);
            theta.setEnabled(requested);
        } else {
            z.setTargetRpm(0);
            z.setEnabled(requested);
        }
        enabled[i] = requested;
    }
}

// 入力から各軸の速度を決める。角度毎秒からrpmへの換算は1/6。
void motorsMove(const int input[3], float dt, bool r_contact, bool z_contact) {
    const bool contacts[3] = {r_contact, false, z_contact};
    for (unsigned i = 0; i < 3; ++i) {
        if (enabled[i])
            moveAxis(axes[i], i, measured(i), input[i], dt, contacts[i]);
    }
    theta.setTargetRpm(axes[1].speed * config::native_per_unit[1] / 6);
    z.setTargetRpm(axes[2].speed * config::native_per_unit[2] / 6);
}

// θ・zへ電流、rへ角速度を送る。rの角度と異常はこの指令への通常返信で受け取る。
void motorsUpdate(uint32_t now) {
    static uint32_t current_at = 0, velocity_at = 0;
    if (now - current_at >= 1) {
        uint8_t data[8]{};
        domain::c620::writeCommandSlot(data, config::theta_id, theta.commandRaw());
        domain::c620::writeCommandSlot(data, config::z_id, z.commandRaw());
        bus.sendStd(0x200, data, 8);
        current_at = now;
    }
    // 異常ビットがある場合だけリセットも要求する。異常が消えれば通常の条件で再開する。
    if (now - velocity_at >= 20) {
        if (enabled[0])
            r.setSpeed(axes[0].speed * config::native_per_unit[0]);
        else
            r.disable(r.faultBits() != 0);
        velocity_at = now;
    }
}

// 出力の可否とは独立して、CCTLが受け取った通信状態を表示へ渡す。
MotorStatus motorsStatus() {
    return {{feedback_received[0] && !feedback_stale[0], feedback_received[1] && !feedback_stale[1],
             feedback_received[2] && !feedback_stale[2]},
            r.faultBits()};
}
