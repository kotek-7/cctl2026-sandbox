#pragma once
#include "config.hpp"
#include <cstdlib>

// 改行を除いた1行を、r / θ / z / 回転 / 畳み / 把持の順に読む。
// 途中で不正な値を見つけるとfalseを返す。その場合はvalues全体を使わない。
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
    const bool valid_fold = values[4] >= -1 && values[4] <= 1;
    const bool valid_grip = values[5] >= 0 && values[5] <= 2;
    return *text == '\0' && valid_fold && valid_grip;
}

// 起動後のミリ秒カウンタが上限を超えて0に戻る場合も、符号なしの引き算で扱う。
inline bool outputsEnabled(bool input_received, uint32_t last_input, uint32_t now,
                           bool motors_ready) {
    return input_received && now - last_input <= config::timeout_ms && motors_ready;
}
