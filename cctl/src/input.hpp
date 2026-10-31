#pragma once
#include "config.hpp"
#include <cstdlib>

// r / θ / z / 回転 / 畳み / 把持。改行を除いた完全な1行を読む。
// false時はvaluesが途中まで書かれるため、呼び出し側は採用しない。
inline bool parseInput(const char *text, int values[6]) {
    for (unsigned i = 0; i < 6; ++i) {
        char *end;
        const long value = std::strtol(text, &end, 10);
        if (end == text || value < -1000 || value > 1000)
            return false;
        if (i < 5 && *end != ' ')
            return false;
        values[i] = static_cast<int>(value);
        text = end;
    }
    while (*text == ' ' || *text == '\r')
        ++text;
    return *text == '\0' && values[4] >= -1 && values[4] <= 1 && values[5] >= 0 && values[5] <= 2;
}

// 未受信の初期値を有効入力と区別する。符号なしの時刻差でtickの周回を扱う。
inline bool outputsEnabled(bool input_received, uint32_t last_input, uint32_t now,
                           bool motors_ready) {
    return input_received && now - last_input <= config::timeout_ms && motors_ready;
}
