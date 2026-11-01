#pragma once
#include <cstdint>

// LCDとPCへ返す、各軸に200ms以内の返信があるかと、rの異常コード。
struct MotorStatus {
    bool feedback_ok[3];
    uint8_t r_fault;
};
void motorsBegin();
void motorsReceive();
MotorStatus motorsStatus(uint32_t now);
void motorsReference(bool r_contact, bool z_contact, const MotorStatus &status);
void motorsStep(bool active, const int input[3], float dt, bool r_contact, bool z_contact);
void motorsCurrent(bool active);
