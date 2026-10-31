#pragma once
#include <cstdint>

void motorsBegin();
void motorsReceive();
bool motorsReady();
void motorsReference(bool r_contact, bool z_contact);
void motorsEnable(bool enabled);
void motorsMove(const int input[3], float dt, bool r_contact, bool z_contact);
void motorsUpdate(uint32_t now);
