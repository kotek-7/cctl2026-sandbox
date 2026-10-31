#pragma once
#include "domain/c620_codec.hpp"
#include "domain/delta_timer.hpp"
#include "domain/pid.hpp"

// C620の電流制御へ、位置PID→速度PIDの順に渡す。
class M3508Motor {
  public:
    M3508Motor(uint8_t esc_id, float max_rpm);
    // CANの返信がθとzのどちらのものかを振り分けるためのID。
    uint16_t feedbackId() const {
        return 0x200 + esc_id_;
    }
    void onFeedback(const uint8_t data[8]);
    // 手動移動で決めた目標を、このモータの位置制御へ渡す。単位は度。
    void setTargetMotorDeg(float value) {
        target_ = value;
    }
    void setEnabled(bool enabled);
    // 軸の移動計算に使う実測角度。最初に受信した位置を基準とする度単位の値。
    float motorDeg() const {
        return angle_.degrees();
    }
    // 出力を許可する前の過熱判定に使う、最後の受信温度（℃）。
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
