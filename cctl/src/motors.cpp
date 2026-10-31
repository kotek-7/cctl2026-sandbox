#include "motors.hpp"
#include "can_bus.hpp"
#include "config.hpp"
#include "el05_motor.hpp"
#include "m3508_motor.hpp"
#include "motion.hpp"
#include <cmath>

extern "C" FDCAN_HandleTypeDef hfdcan1;
namespace {
// rはEL05内部で位置制御し、θ・zはCCTLで電流指令を計算する。
CanBus bus(&hfdcan1);
El05Motor r(bus, config::el05_id, config::host_id);
M3508Motor theta(config::theta_id, config::max_rpm[0]);
M3508Motor z(config::z_id, config::max_rpm[1]);
AxisMotion axes[3];
uint32_t last_feedback[3]{};
bool feedback_received[3]{};
bool enabled = false;
// 指定した軸のモータ角度を返す。r=0はラジアン、θ=1とz=2は度。
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

// CAN通信を開始し、rのモータを停止した状態で位置制御モードと上限値を設定する。
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

// CANに届いた返信を読み、各モータの角度・状態と最終受信時刻を更新する。
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
}

// 3軸の返信が200ms以内で、rの位置が有効・異常なし、θ・zが80℃未満ならtrue。
bool motorsReady() {
    const auto now = HAL_GetTick();
    for (unsigned i = 0; i < 3; ++i)
        if (!feedback_received[i] || now - last_feedback[i] > config::feedback_ms)
            return false;
    return r.positionReady() && r.faultBits() == 0 && theta.temperature() < 80 &&
           z.temperature() < 80;
}

// 出力切替時の目標を実測に合わせる。原点は維持する。
static void holdPosition() {
    for (unsigned i = 0; i < 3; ++i) {
        axes[i].target = measured(i);
        axes[i].speed = 0;
    }
    theta.setTargetMotorDeg(axes[1].target);
    z.setTargetMotorDeg(axes[2].target);
    if (enabled)
        r.setLocRef(axes[0].target);
}

// 原点登録はrとzで別々に判定する。片方のモータが未接続でも、もう片方は登録できる。
void motorsReference(bool r_contact, bool z_contact) {
    const auto now = HAL_GetTick();
    if (feedback_received[0] && now - last_feedback[0] <= config::feedback_ms && r.positionReady())
        referenceAxis(axes[0], 0, measured(0), r_contact);
    if (feedback_received[2] && now - last_feedback[2] <= config::feedback_ms)
        referenceAxis(axes[2], 2, measured(2), z_contact);
    // 原点登録で目標を変更したら、次の電流計算からその目標を使う。
    theta.setTargetMotorDeg(axes[1].target);
    z.setTargetMotorDeg(axes[2].target);
}

// 出力の有効・無効を切り替える。切替時は位置目標を現在位置に戻す。
void motorsEnable(bool requested_enabled) {
    if (enabled == requested_enabled)
        return;
    holdPosition();
    if (requested_enabled) {
        r.setLocRef(axes[0].target);
        r.enable();
    } else
        r.disable();
    theta.setEnabled(requested_enabled);
    z.setEnabled(requested_enabled);
    enabled = requested_enabled;
}

// r・θ・zの入力から速度と位置目標を更新し、接点と可動範囲で制限する。
void motorsMove(const int input[3], float dt, bool r_contact, bool z_contact) {
    const bool contacts[3] = {r_contact, false, z_contact};
    for (unsigned i = 0; i < 3; ++i)
        moveAxis(axes[i], i, measured(i), input[i], dt, contacts[i]);
    theta.setTargetMotorDeg(axes[1].target);
    z.setTargetMotorDeg(axes[2].target);
}

// θ・zの電流指令を1msごとに送る。rは20msごとに位置または停止を指令し、位置を読む。
void motorsUpdate(uint32_t now) {
    static uint32_t current_at = 0, position_at = 0;
    if (now - current_at >= 1) {
        uint8_t data[8]{};
        domain::c620::writeCommandSlot(data, config::theta_id, theta.commandRaw());
        domain::c620::writeCommandSlot(data, config::z_id, z.commandRaw());
        bus.sendStd(0x200, data, 8);
        current_at = now;
    }
    // MECH_POSを定期的に読み、角度の折返し回数を補正する。返信は通信断の判定にも使う。
    if (now - position_at >= 20) {
        if (enabled)
            r.setLocRef(axes[0].target);
        else
            r.disable(false);
        r.requestParam(domain::el05::param::MECH_POS);
        position_at = now;
    }
}
