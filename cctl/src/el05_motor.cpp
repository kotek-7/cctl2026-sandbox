#include "el05_motor.hpp"

namespace {
namespace codec = domain::el05;
} // namespace

// rのEL05へCAN指令を送る。
bool El05Motor::sendFrame(uint8_t comm_type, const uint8_t data[8]) {
    return bus_.sendExt(codec::buildCanId(comm_type, host_id_, motor_id_), data, 8);
}

// rのモータ出力を有効にする。
bool El05Motor::enable() {
    const uint8_t data[8] = {};
    return sendFrame(codec::comm::ENABLE, data);
}

// rの出力を止める。clear_faultで異常リセットも要求する。
bool El05Motor::disable(bool clear_fault) {
    uint8_t data[8] = {};
    data[0] = clear_fault ? 1 : 0;
    return sendFrame(codec::comm::DISABLE, data);
}

// rのEL05を速度制御モードに設定する。
bool El05Motor::setVelocityMode() {
    uint8_t data[8];
    codec::encodeParamU8(codec::param::RUN_MODE, 2, data);
    return sendFrame(codec::comm::WRITE_PARAM, data);
}

// rの角速度[rad/s]を指令する。
bool El05Motor::setSpeed(float radians_per_second) {
    return writeParamFloat(codec::param::SPD_REF, radians_per_second);
}

// EL05の速度指令や制御パラメータを書き込む。
bool El05Motor::writeParamFloat(uint16_t param, float value) {
    uint8_t data[8] = {};
    codec::encodeParamFloat(param, value, data);
    return sendFrame(codec::comm::WRITE_PARAM, data);
}

// rの返信から回転量と異常コードを更新する。
bool El05Motor::onFeedback(uint32_t ext_id, const uint8_t data[8]) {
    if (codec::commType(ext_id) != codec::comm::FEEDBACK ||
        static_cast<uint8_t>(codec::dataArea2(ext_id)) != motor_id_)
        return false;
    feedback_ = codec::decodeFeedback(ext_id, data);
    const uint16_t raw = feedback_.raw_angle;
    if (position_ready_)
        position_rad_ += codec::positionDelta(last_position_raw_, raw);
    last_position_raw_ = raw;
    position_ready_ = true;
    return true;
}
