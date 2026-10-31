#pragma once
#include <cstdint>

// ID=1、1Mbps、位置モードに設定済みのSTS3215を使う。
void servoBegin();
void servoTarget(int16_t position);
void servoEnable(bool enabled);
