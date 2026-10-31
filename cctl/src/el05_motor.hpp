#pragma once
#include "can_bus.hpp"
#include "domain/el05_codec.hpp"

// rの速度制御と、接点原点に使うMECH_POSの読取り。
class El05Motor {
  public:
    using RunMode = domain::el05::RunMode;
    // rのモータとの通信先を決める。実際の起動設定はmotorsBeginが行う。
    El05Motor(CanBus &bus, uint8_t motor_id, uint8_t host_id)
        : bus_(bus), motor_id_(motor_id), host_id_(host_id) {}
    bool enable();
    bool disable(bool clear_fault = false);
    bool setRunMode(RunMode mode);
    bool setSpeed(float radians_per_second);
    bool writeParamFloat(uint16_t param, float value);
    bool requestParam(uint16_t param);
    bool onFeedback(uint32_t id, const uint8_t data[8]);
    // rの原点登録と移動計算に使う実測角度（ラジアン）。
    float position() const {
        return position_rad_;
    }
    // rの角度を原点登録や出力判定に使えるかを示す。MECH_POSの取得後にtrueになる。
    bool positionReady() const {
        return position_ready_;
    }
    // 停止＋リセット要求の対象となる、モータ自身が報告した異常。
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
    bool position_ready_ = false;
};
