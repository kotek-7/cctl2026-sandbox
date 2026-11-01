#pragma once
#include "can_bus.hpp"
#include "domain/el05_codec.hpp"

// rのEL05への指令と返信を扱う。
class El05Motor {
  public:
    // rのモータIDとCCTLの送信元IDを指定する。
    El05Motor(CanBus &bus, uint8_t motor_id, uint8_t host_id)
        : bus_(bus), motor_id_(motor_id), host_id_(host_id) {}
    bool enable();
    bool disable(bool clear_fault = false);
    bool setVelocityMode();
    bool setSpeed(float radians_per_second);
    bool writeParamFloat(uint16_t param, float value);
    bool onFeedback(uint32_t id, const uint8_t data[8]);
    // 原点登録と端点減速に使う、初回返信からの回転量[rad]。
    float position() const {
        return position_rad_;
    }
    // rが報告した異常ビット。
    uint8_t faultBits() const {
        return feedback_.fault_bits;
    }

  private:
    bool sendFrame(uint8_t type, const uint8_t data[8]);
    CanBus &bus_;
    uint8_t motor_id_, host_id_;
    domain::el05::Feedback feedback_{};
    float position_rad_ = 0;
    uint16_t last_position_raw_ = 0;
    bool position_ready_ = false;
};
