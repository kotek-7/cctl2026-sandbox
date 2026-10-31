#pragma once
#include "can_bus.hpp"
#include "domain/el05_codec.hpp"

// EL05の位置モード。周期位置の折返しを、多回転位置へ連続化する。
class El05Motor {
  public:
    using RunMode = domain::el05::RunMode;
    El05Motor(CanBus &bus, uint8_t motor_id, uint8_t host_id)
        : bus_(bus), motor_id_(motor_id), host_id_(host_id) {}
    bool enable();
    bool disable(bool clear_fault = true);
    bool setRunMode(RunMode mode);
    bool setLocRef(float position);
    bool writeParamFloat(uint16_t param, float value);
    bool requestParam(uint16_t param);
    bool onFeedback(uint32_t id, const uint8_t data[8]);
    float position() const {
        return position_rad_;
    }
    bool positionReady() const {
        return position_ready_;
    }
    uint8_t faultBits() const {
        return feedback_.fault_bits;
    }

  private:
    bool sendFrame(uint8_t type, const uint8_t data[8]);
    bool writeParamU8(uint16_t param, uint8_t value);
    CanBus &bus_;
    uint8_t motor_id_, host_id_;
    domain::el05::Feedback feedback_{};
    float position_rad_ = 0;
    uint16_t last_position_raw_ = 0;
    bool has_wrapped_position_ = false, position_ready_ = false;
};
