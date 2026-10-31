#pragma once

namespace domain {

// 出力を±out_limitに制限する。
// 出力が飽和し、誤差がさらに飽和を強める方向なら、積分値をリセットする。
class Pid {
  public:
    // θ・zの速度制御のゲインと電流上限を設定する。
    Pid(float kp, float ki, float kd, float out_limit)
        : kp_(kp), ki_(ki), kd_(kd), out_limit_(out_limit) {}

    // 制御の応答の強さを調整する。これまでに蓄積した積分値は引き継ぐ。
    void setGains(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    // 制御計算が要求できる回転数または電流の最大値を変える。
    void setOutLimit(float out_limit) {
        out_limit_ = out_limit;
    }
    // 出力制限の確認に使う設定値。正負のどちらの出力にも、この絶対値を上限として使う。
    float outLimit() const {
        return out_limit_;
    }

    void reset();

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
