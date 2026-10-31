#include "doctest.h"
#include "m3508_motor.hpp"

TEST_CASE("P制御は初回PC入力からFBを待たず電流を出す") {
    M3508Motor motor(1);
    motor.setTargetRpm(100);
    CHECK(motor.commandRaw() == 0);
    motor.setEnabled(true);
    CHECK(motor.commandRaw() == domain::c620::currentToRaw(700));
    // FBを受け取ると、その速度との差に比例する電流へ変わる。
    const uint8_t reply[8]{0, 0, 0, 50, 0, 0, 0, 0};
    motor.onFeedback(reply);
    CHECK(motor.commandRaw() == domain::c620::currentToRaw(350));
    motor.setTargetRpm(50);
    CHECK(motor.commandRaw() == 0);
    motor.setTargetRpm(0);
    CHECK(motor.commandRaw() == domain::c620::currentToRaw(-350));
}

TEST_CASE("P制御は電流を正負10A以内に収め積分を持たない") {
    M3508Motor motor(2);
    motor.setEnabled(true);
    motor.setTargetRpm(10000);
    CHECK(motor.commandRaw() == domain::c620::currentToRaw(10000));
    motor.setTargetRpm(-10000);
    CHECK(motor.commandRaw() == domain::c620::currentToRaw(-10000));
    motor.setTargetRpm(0);
    CHECK(motor.commandRaw() == 0);
}
