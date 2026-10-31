#pragma once
#include "motors.hpp"

void statusBegin();
void statusUpdate(bool input_received, bool input_fresh, const MotorStatus &motors);
