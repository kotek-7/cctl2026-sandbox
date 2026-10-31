#pragma once
#include <cstdint>

void motorsBegin();
void motorsReceive();
void motorsReference(bool r_contact, bool z_contact);
void motorsEnable(bool input_received);
void motorsMove(const int input[3], float dt, bool r_contact, bool z_contact);
void motorsUpdate(uint32_t now);

// LCDとPCへ返す、3軸の返信の有無・鮮度とrの異常コード。
struct MotorStatus {
    bool feedback_ok[3];
    uint8_t r_fault;
};
MotorStatus motorsStatus();
