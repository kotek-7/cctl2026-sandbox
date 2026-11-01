#pragma once
#include "config.hpp"
#include <algorithm>
#include <cstdlib>

// PCの指令をr・θ・z・回転・畳み・把持の6入力として読み取る。
// falseの場合、valuesの内容は使わない。
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

// 最後の有効なPC入力が250ms以内かを判定する。
inline bool inputFresh(bool input_received, uint32_t last_input, uint32_t now) {
    return input_received && now - last_input <= config::timeout_ms;
}

// PC入力が途絶えたら全操作を中立に戻す。
inline void expireInput(bool fresh, int values[6]) {
    if (!fresh)
        std::fill(values, values + 6, 0);
}
