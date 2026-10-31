#pragma once
#include "domain/c620_codec.hpp"

// θ・zの速度指令から、C620へ送る電流を求める。
class M3508Motor {
  public:
    // この軸のC620のCAN IDを指定する。
    explicit M3508Motor(uint8_t esc_id) : esc_id_(esc_id) {}
    // CANの返信がθとzのどちらのものかを振り分けるためのID。
    uint16_t feedbackId() const {
        return 0x200 + esc_id_;
    }
    void onFeedback(const uint8_t data[8]);
    // 機体の速度から換算したモータ回転数（rpm）を受け取る。
    void setTargetRpm(float value) {
        target_rpm_ = value;
    }
    // 初回PC入力以降は速度制御を続ける。
    void setEnabled(bool enabled) {
        enabled_ = enabled;
    }
    // 軸の移動計算に使う実測角度。最初に受信した位置を基準とする度単位の値。
    float motorDeg() const {
        return angle_.degrees();
    }
    int16_t commandRaw();

  private:
    uint8_t esc_id_;
    domain::c620::MultiTurnCounter angle_;
    domain::c620::Feedback feedback_{};
    bool enabled_ = false;
    float target_rpm_ = 0;
};
