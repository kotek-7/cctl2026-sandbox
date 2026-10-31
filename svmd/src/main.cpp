#include <Arduino.h>
#include <Arduino_CAN.h>
#include <Servo.h>

namespace {
constexpr uint16_t COMMAND_ID = 0x300;
constexpr uint8_t PINS[4] = {3, 6, 10, 9};
Servo servos[4];
uint32_t last_command = 0;
// CCTLとの通信断で、畳み・把持のサーボへ出すPWM信号を止める。
void stop() {
    for (auto &servo : servos)
        if (servo.attached())
            servo.detach();
}
} // namespace
// 畳み・把持を動かす基板の起動処理。CCTLからのPWM指令を受信できる状態にする。
void setup() {
    CAN.begin(CanBitRate::BR_1000k);
}
// CCTLの畳み・把持の目標を4本のPWM信号へ反映する。通信断時の出力停止もこの基板が担う。
void loop() {
    while (CAN.available()) {
        const auto msg = CAN.read();
        if (msg.isExtendedId() || msg.id != COMMAND_ID || msg.data_length != 8)
            continue;
        // 不正なフレームを一部だけ反映しないよう、4本とも検証する。
        // 各幅は上位byteが先のµs値。0は解除、それ以外は400〜1500。
        uint16_t pulse[4];
        bool valid = true;
        for (unsigned i = 0; i < 4; ++i) {
            pulse[i] = (msg.data[2 * i] << 8) | msg.data[2 * i + 1];
            if (pulse[i] != 0 && (pulse[i] < 400 || pulse[i] > 1500))
                valid = false;
        }
        if (!valid)
            continue;
        last_command = millis();
        for (unsigned i = 0; i < 4; ++i) {
            if (!pulse[i]) {
                if (servos[i].attached())
                    servos[i].detach();
            } else {
                if (!servos[i].attached())
                    servos[i].attach(PINS[i], 400, 1500);
                servos[i].writeMicroseconds(pulse[i]);
            }
        }
    }
    // 指令途絶時はPWM信号を切る。サーボの位置保持は行わない。
    if (millis() - last_command > 250)
        stop();
}
