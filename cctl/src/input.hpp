#pragma once
#include "config.hpp"
#include <algorithm>
#include <cstdlib>

// PCからの1行が操作指令として使えるかを検査する。項目はr・θ・z・回転・畳み・把持の順。
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

// PC入力が操作に使えるかを判定する。初回受信前と250msを超えた通信断ではfalse。
// 起動後のミリ秒カウンタが上限を超えて0に戻る場合も、符号なしの引き算で扱う。
inline bool inputFresh(bool input_received, uint32_t last_input, uint32_t now) {
    return input_received && now - last_input <= config::timeout_ms;
}

// PC入力が途絶えたら全操作を中立にする。出力の許可とEEの姿勢目標は変更しない。
inline void expireInput(bool fresh, int values[6]) {
    if (!fresh)
        std::fill(values, values + 6, 0);
}
