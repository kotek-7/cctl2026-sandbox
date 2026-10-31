#pragma once

#include <cstddef>
#include <cstdint>

// C620 (M3508 用 ESC) のフレーム符号化・復号と、多回転角の追跡。
namespace domain::c620 {

// C620 の角度分解能。生角度は 0..8191 で 1 回転する。
constexpr int32_t COUNTS_PER_REV = 8192;

// C620 が返すフィードバックフレームの内容。
struct Feedback {
    uint16_t raw_angle; // 0..8191
    int16_t rpm;
    int16_t current_raw; // 電流の生値
    uint8_t temperature; // [degC]
};

Feedback decodeFeedback(const uint8_t data[8]);

// 0..8191 で巻き戻る生角度から、多回転の累積カウントを追う。
// 最初の角度を累積カウント0とする。機体座標の原点はAxisMotionで別に管理する。
class MultiTurnCounter {
  public:
    void update(uint16_t raw_angle);

    // 原点を取り直す。次に受け取った角度が新しい 0 になる。
    void reset();

    bool hasReference() const {
        return has_reference_;
    }
    int32_t counts() const {
        return counts_;
    }
    float degrees() const;

  private:
    bool has_reference_ = false;
    uint16_t last_raw_angle_ = 0;
    int32_t counts_ = 0;
};

// 電流[mA] を C620 の指令生値へ変換する（±20000mA ↔ ±16384）。
int16_t currentToRaw(int32_t milli_amp);

// 指令生値[±16384] を電流[mA] へ戻す。
int32_t rawToCurrent(int16_t raw);

// 電流指令はESC ID 1..4と5..8のグループごとに1フレームへまとめる。
constexpr uint16_t COMMAND_ID_1_TO_4 = 0x200;
constexpr uint16_t COMMAND_ID_5_TO_8 = 0x1FF;
constexpr uint8_t MIN_ESC_ID = 1;
constexpr uint8_t MAX_ESC_ID = 8;
constexpr std::size_t MOTORS_PER_FRAME = 4;

// ESC ID が属する指令フレームの ID。範囲外なら 0 を返す。
uint16_t groupCommandId(uint8_t esc_id);

// 4ch 一括指令フレームのうち、esc_id に対応する 2 バイトへ書き込む。
// ESC ID 1..4 と 5..8 は別フレームだが、フレーム内の位置は同じ並びになる。
void writeCommandSlot(uint8_t frame[8], uint8_t esc_id, int16_t raw);

// 同一グループの最大4台分をまとめて送る。
// 1台ずつ送ると、他の台の指令が未設定スロットの0で上書きされる。
class CommandFrame {
  public:
    explicit CommandFrame(uint16_t command_id) : command_id_(command_id) {}

    uint16_t commandId() const {
        return command_id_;
    }
    const uint8_t *data() const {
        return data_;
    }

    // 送信ごとに全スロットを0に戻す。未設定の台には電流0を指令する。
    void clear();

    // esc_id がこのフレームの担当なら書き込んで true を返す。
    bool set(uint8_t esc_id, int16_t raw);

  private:
    uint16_t command_id_;
    uint8_t data_[8] = {};
};

} // namespace domain::c620
