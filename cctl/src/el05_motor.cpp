#include "el05_motor.hpp"
#include <cmath>

namespace {
namespace codec = domain::el05;
} // namespace

// このEL05への指令に送受信先IDを付ける。各操作はこの関数を通してCANへ送る。
bool El05Motor::sendFrame(uint8_t comm_type, const uint8_t data[8]) {
    return bus_.sendExt(codec::buildCanId(comm_type, host_id_, motor_id_), data, 8);
}

// CCTLが操作を許可したときに、rのモータの速度制御を有効にする。
bool El05Motor::enable() {
    const uint8_t data[8] = {};
    return sendFrame(codec::comm::ENABLE, data);
}

// rのモータの出力を止める。clear_faultがtrueなら、モータへ異常リセットも要求する。
bool El05Motor::disable(bool clear_fault) {
    uint8_t data[8] = {};
    data[0] = clear_fault ? 1 : 0;
    return sendFrame(codec::comm::DISABLE, data);
}

// EL05の制御方式を選ぶ。この機体では起動時に速度モードを指定する。
bool El05Motor::setRunMode(RunMode mode) {
    return writeParamU8(codec::param::RUN_MODE, static_cast<uint8_t>(mode));
}

// 機体の速度から換算した角速度（rad/s）を、EL05へ指令する。
bool El05Motor::setSpeed(float radians_per_second) {
    return writeParamFloat(codec::param::SPD_REF, radians_per_second);
}

// EL05の加速度・電流上限や速度指令など、実数で指定する設定を書き込む。
bool El05Motor::writeParamFloat(uint16_t param, float value) {
    uint8_t data[8] = {};
    codec::encodeParamFloat(param, value, data);
    return sendFrame(codec::comm::WRITE_PARAM, data);
}

// EL05の制御モードなど、1バイトで指定する設定を書き込む。
bool El05Motor::writeParamU8(uint16_t param, uint8_t value) {
    uint8_t data[8] = {};
    codec::encodeParamU8(param, value, data);
    return sendFrame(codec::comm::WRITE_PARAM, data);
}

// 通常の位置返信とは別に、EL05の設定値を問い合わせる。rの位置確認ではMECH_POSを読む。
bool El05Motor::requestParam(uint16_t param) {
    uint8_t data[8] = {};
    codec::encodeParamRequest(param, data);
    return sendFrame(codec::comm::READ_PARAM, data);
}

// rの異常情報と機械角を受け取る。対象モータの有効な返信ならtrueを返す。
bool El05Motor::onFeedback(uint32_t ext_id, const uint8_t data[8]) {
    switch (codec::commType(ext_id)) {
    case codec::comm::FEEDBACK:
        if (static_cast<uint8_t>(codec::dataArea2(ext_id)) != motor_id_)
            return false;
        feedback_ = codec::decodeFeedback(ext_id, data);
        return true;
    case codec::comm::READ_PARAM: {
        if (codec::targetId(ext_id) != host_id_ ||
            static_cast<uint8_t>(codec::dataArea2(ext_id)) != motor_id_ ||
            ((ext_id >> 16) & 0xFF) != 0 || codec::paramReplyIndex(data) != codec::param::MECH_POS)
            return false;
        const float value = codec::paramReplyFloat(data);
        if (!std::isfinite(value))
            return false;
        position_rad_ = value;
        position_ready_ = true;
        return true;
    }
    default:
        return false;
    }
}
