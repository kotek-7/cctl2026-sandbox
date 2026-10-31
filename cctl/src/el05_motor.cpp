#include "el05_motor.hpp"

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

// 通常返信の角度を積算し、速度・温度・異常も更新する。最初の角度を0とする。
bool El05Motor::onFeedback(uint32_t ext_id, const uint8_t data[8]) {
    if (codec::commType(ext_id) != codec::comm::FEEDBACK ||
        static_cast<uint8_t>(codec::dataArea2(ext_id)) != motor_id_)
        return false;
    const uint16_t raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    if (position_ready_)
        position_rad_ += codec::positionDelta(last_position_raw_, raw);
    last_position_raw_ = raw;
    position_ready_ = true;
    feedback_ = codec::decodeFeedback(ext_id, data);
    return true;
}
