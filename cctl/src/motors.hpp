#pragma once
#include <cstdint>

void motorsBegin();
void motorsReceive();
bool motorsReady();
void motorsOrigin();
void motorsEnable(bool enabled);
void motorsHold();
void motorsMove(const int input[3], float dt, bool r_contact, bool z_contact);
void motorsUpdate(uint32_t now);
float motorPosition(unsigned axis);
