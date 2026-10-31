#pragma once

namespace domain {

// 経過時間 dt は呼び出し側が渡す。
// 出力は ±out_limit にクランプし、飽和時は積分器を anti-windup する。
class Pid {
  public:
    Pid(float kp, float ki, float kd, float out_limit)
        : kp_(kp), ki_(ki), kd_(kd), out_limit_(out_limit) {}

    void setGains(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void setOutLimit(float out_limit) {
        out_limit_ = out_limit;
    }
    float outLimit() const {
        return out_limit_;
    }

    void reset();

    // 目標値と計測値から出力を計算し、内部状態を更新する。
    // dt_s <= 0 では積分を加算せず、微分項を0とする。蓄積済みの積分項は使う。
    float update(float target, float measured, float dt_s);

  private:
    float kp_;
    float ki_;
    float kd_;
    float out_limit_;

    float integral_ = 0.0f;
    float prev_error_ = 0.0f;
};

} // namespace domain
