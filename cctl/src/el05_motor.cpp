#include "el05_motor.hpp"
#include <cmath>

namespace {
namespace codec = domain::el05;
constexpr float POSITION_SPAN = codec::POSITION_MAX - codec::POSITION_MIN;
} // namespace

bool El05Motor::sendFrame(uint8_t comm_type, const uint8_t data[8]) {
    return bus_.sendExt(codec::buildCanId(comm_type, host_id_, motor_id_), data, 8);
}

bool El05Motor::enable() {
    const uint8_t data[8] = {};
    return sendFrame(codec::comm::ENABLE, data);
}

bool El05Motor::disable(bool clear_fault) {
    uint8_t data[8] = {};
    data[0] = clear_fault ? 1 : 0;
    return sendFrame(codec::comm::DISABLE, data);
}

bool El05Motor::setRunMode(RunMode mode) {
    return writeParamU8(codec::param::RUN_MODE, static_cast<uint8_t>(mode));
}

bool El05Motor::setLocRef(float pos_rad) {
    return writeParamFloat(codec::param::LOC_REF, pos_rad);
}

bool El05Motor::writeParamFloat(uint16_t param, float value) {
    uint8_t data[8] = {};
    codec::encodeParamFloat(param, value, data);
    return sendFrame(codec::comm::WRITE_PARAM, data);
}

bool El05Motor::writeParamU8(uint16_t param, uint8_t value) {
    uint8_t data[8] = {};
    codec::encodeParamU8(param, value, data);
    return sendFrame(codec::comm::WRITE_PARAM, data);
}

bool El05Motor::requestParam(uint16_t param) {
    uint8_t data[8] = {};
    codec::encodeParamRequest(param, data);
    return sendFrame(codec::comm::READ_PARAM, data);
}

bool El05Motor::onFeedback(uint32_t ext_id, const uint8_t data[8]) {
    switch (codec::commType(ext_id)) {
    case codec::comm::FEEDBACK: {
        // フィードバックは data_area_2 の下位 8bit に発信元のモータIDが載る。
        if (static_cast<uint8_t>(codec::dataArea2(ext_id) & 0xFF) != motor_id_) {
            return false;
        }
        const codec::Feedback decoded = codec::decodeFeedback(ext_id, data);
        const uint16_t position_raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
        const float wrapped = decoded.position_rad;
        if (!has_wrapped_position_) {
            if (position_ready_) {
                // 周回数を加えて、MECH_POSに最も近い位置に合わせる。
                position_rad_ =
                    wrapped + std::round((position_rad_ - wrapped) / POSITION_SPAN) * POSITION_SPAN;
            } else {
                position_rad_ = wrapped;
            }
            has_wrapped_position_ = true;
        } else {
            int32_t delta_raw = static_cast<int32_t>(position_raw) - last_position_raw_;
            if (delta_raw > 32767)
                delta_raw -= 65536;
            if (delta_raw < -32768)
                delta_raw += 65536;
            position_rad_ += static_cast<float>(delta_raw) * POSITION_SPAN / 65535.0f;
        }
        last_position_raw_ = position_raw;
        feedback_ = decoded;
        feedback_.position_rad = position_rad_;
        return true;
    }
    case codec::comm::READ_PARAM: {
        if (codec::targetId(ext_id) != host_id_ ||
            static_cast<uint8_t>(codec::dataArea2(ext_id)) != motor_id_ ||
            ((ext_id >> 16) & 0xFF) != 0)
            return false;
        const auto index = codec::paramReplyIndex(data);
        const float value = codec::paramReplyFloat(data);
        if (index == codec::param::MECH_POS && std::isfinite(value)) {
            if (has_wrapped_position_) {
                // MECH_POSとは取得時刻が異なるため、位置を置き換えず周回数だけ補正する。
                position_rad_ +=
                    std::round((value - position_rad_) / POSITION_SPAN) * POSITION_SPAN;
            } else {
                position_rad_ = value;
            }
            position_ready_ = true;
            feedback_.position_rad = position_rad_;
        }
        // パラメータ応答は通常の位置フィードバックと区別してfalseを返す。
        return false;
    }
    default:
        return false;
    }
}
