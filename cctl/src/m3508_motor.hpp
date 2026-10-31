#pragma once
#include "domain/c620_codec.hpp"
#include "domain/delta_timer.hpp"
#include "domain/pid.hpp"

// C620の電流制御へ、位置PID→速度PIDの順に渡す。
class M3508Motor {
  public:
    M3508Motor(uint8_t esc_id, float max_rpm);
    uint16_t feedbackId() const {
        return 0x200 + esc_id_;
    }
    void onFeedback(const uint8_t data[8]);
    void setTargetMotorDeg(float value) {
        target_ = value;
    }
    void setEnabled(bool enabled);
    float motorDeg() const {
        return angle_.degrees();
    }
    uint8_t temperature() const {
        return feedback_.temperature;
    }
    int16_t commandRaw();

  private:
    uint8_t esc_id_;
    domain::Pid position_pid_, velocity_pid_;
    domain::DeltaTimer timer_;
    domain::c620::MultiTurnCounter angle_;
    domain::c620::Feedback feedback_{};
    bool enabled_ = false;
    float target_ = 0;
};
