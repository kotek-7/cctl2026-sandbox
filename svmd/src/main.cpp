#include <Arduino.h>
#include <Arduino_CAN.h>
#include <Servo.h>

namespace {
constexpr uint16_t COMMAND_ID = 0x300;
constexpr uint8_t PINS[4] = {3, 6, 10, 9};
Servo servos[4];
uint32_t last_command = 0;
// 接続中の4本のサーボ出力をすべて切る。
void stop() {
    for (auto &servo : servos)
        if (servo.attached())
            servo.detach();
}
} // namespace
// CAN通信を1Mbpsで開始する。
void setup() {
    CAN.begin(CanBitRate::BR_1000k);
}
// CANの4本分のPWM指令を反映し、正常な指令が250msを超えて途絶えたら出力を切る。
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
