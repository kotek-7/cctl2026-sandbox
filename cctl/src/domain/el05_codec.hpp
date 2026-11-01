#pragma once
#include <cstdint>

// EL05の速度指令・設定と通常返信に使う通信形式。
namespace domain::el05 {
namespace comm {
constexpr uint8_t FEEDBACK = 2, ENABLE = 3, DISABLE = 4, WRITE_PARAM = 18;
}
namespace param {
constexpr uint16_t RUN_MODE = 0x7005, SPD_REF = 0x700A, LIMIT_CUR = 0x7018, ACC_RAD = 0x7022;
}
constexpr float POSITION_MIN = -12.57f, POSITION_MAX = 12.57f;

// 原点計算用の角度と、停止＋リセット要求に使う異常ビット。
struct Feedback {
    uint16_t raw_angle;
    uint8_t fault_bits;
};
uint32_t buildCanId(uint8_t comm_type, uint16_t host_id, uint8_t motor_id);
uint8_t commType(uint32_t ext_id);
uint16_t dataArea2(uint32_t ext_id);
Feedback decodeFeedback(uint32_t ext_id, const uint8_t data[8]);
float positionDelta(uint16_t previous, uint16_t current);
void encodeParamFloat(uint16_t param, float value, uint8_t out[8]);
void encodeParamU8(uint16_t param, uint8_t value, uint8_t out[8]);
} // namespace domain::el05
