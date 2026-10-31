#pragma once
#include "can_bus.hpp"
#include "domain/el05_codec.hpp"

// EL05の位置制御。返信の角度が上限から下限へ戻った分を足し、
// 複数回転しても連続した角度として扱う。
class El05Motor {
  public:
    using RunMode = domain::el05::RunMode;
    // 使用するCANと、モータ・CCTLのIDを保存する。
    El05Motor(CanBus &bus, uint8_t motor_id, uint8_t host_id)
        : bus_(bus), motor_id_(motor_id), host_id_(host_id) {}
    bool enable();
    bool disable(bool clear_fault = true);
    bool setRunMode(RunMode mode);
    bool setLocRef(float position);
    bool writeParamFloat(uint16_t param, float value);
    bool requestParam(uint16_t param);
    bool onFeedback(uint32_t id, const uint8_t data[8]);
    // 折返しを補正したモータ角度をラジアンで返す。
    float position() const {
        return position_rad_;
    }
    // MECH_POSの読出し返信で角度を取得済みならtrueを返す。
    bool positionReady() const {
        return position_ready_;
    }
    // 最後に受信したモータの異常ビットを返す。
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
