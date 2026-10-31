#include "doctest.h"
#include "input.hpp"
#include "motion.hpp"

TEST_CASE("USBは6整数だけを受け付ける") {
    int values[6]{};
    CHECK(parseInput("1000 -1000 500 -20 -1 2", values));
    CHECK(values[0] == 1000);
    CHECK(values[1] == -1000);
    CHECK(values[4] == -1);
    CHECK(values[5] == 2);
    CHECK(parseInput("0 0 0 0 0 0\r", values));
    for (const auto *text :
         {"0 0 0 0 0", "0 0 0 0 0 0 0", "0 0 0 0 0 3", "1001 0 0 0 0 0", "0 -1001 0 0 0 0",
          "0 0 0 0 2 0", "0 0 0 0 0 -1", "0-1 0 0 0 0", "nan 0 0 0 0 0", "0 0 0 0 0 0x"})
        CHECK_FALSE(parseInput(text, values));
}

TEST_CASE("PC入力は初回受信から250ms以内だけ有効") {
    CHECK_FALSE(inputFresh(false, 0, 0));
    CHECK(inputFresh(true, 100, 100));
    CHECK(inputFresh(true, 100, 350));
    CHECK_FALSE(inputFresh(true, 100, 351));
    CHECK(inputFresh(true, 400, 400));
    CHECK(inputFresh(true, 0xfffffff0u, 20));
    CHECK_FALSE(inputFresh(true, 0xfffffff0u, 300));
}

TEST_CASE("原点未登録では全軸が座標範囲外でも両方向へ動ける") {
    for (unsigned i = 0; i < 3; ++i) {
        for (int sign : {-1, 1}) {
            const float measured = sign * 1000 * config::native_per_unit[i];
            AxisMotion axis;
            moveAxis(axis, i, measured, sign * 1000, 0.02f, false);
            CHECK(axis.speed * sign > 0);
            CHECK_FALSE(axis.referenced);
        }
    }
}

TEST_CASE("rとzは最初の接触だけで個別に原点を登録する") {
    AxisMotion axes[3]{{false, 0, 100}, {false, 0, 50}, {false, 0, -100}};
    referenceAxis(axes[0], 0, 11, false);
    CHECK_FALSE(axes[0].referenced);
    referenceAxis(axes[0], 0, 11, true);
    CHECK(axes[0].referenced);
    CHECK((11 - axes[0].offset) / config::native_per_unit[0] == doctest::Approx(570));
    CHECK(axes[0].speed == 0);
    CHECK_FALSE(axes[2].referenced);
    CHECK(axes[2].speed == -100);
    referenceAxis(axes[2], 2, 31, true);
    CHECK(axes[2].referenced);
    CHECK(axes[2].offset == 31);
    CHECK(axes[2].speed == 0);
    CHECK_FALSE(axes[1].referenced);
    CHECK(axes[1].speed == 50);
    const float offset = axes[0].offset;
    axes[0].speed = -20;
    referenceAxis(axes[0], 0, 13, true);
    CHECK(axes[0].offset == offset);
    CHECK(axes[0].speed == -20);
}

TEST_CASE("θは原点登録も角度制限も行わない") {
    AxisMotion axis;
    const float measured = 720 * config::native_per_unit[1];
    referenceAxis(axis, 1, measured, true);
    CHECK_FALSE(axis.referenced);
    moveAxis(axis, 1, measured, 1000, 0.02f, true);
    CHECK(axis.speed > 0);
}

TEST_CASE("登録前後とも接点への押し込みを止め反対方向へ離れられる") {
    for (unsigned i : {0u, 2u}) {
        const int toward = i == 0 ? 1 : -1;
        for (bool referenced : {false, true}) {
            AxisMotion axis;
            if (referenced)
                referenceAxis(axis, i, 12, true);
            axis.speed = toward * config::speed[i];
            moveAxis(axis, i, 12, toward * 1000, 0.02f, true);
            CHECK(axis.speed == 0);
            moveAxis(axis, i, 12, -toward * 1000, 0.02f, true);
            CHECK(axis.speed * toward < 0);
        }
    }
}

TEST_CASE("登録後は端点で外向きの速度を0にし内向きには動ける") {
    for (unsigned i : {0u, 2u}) {
        AxisMotion axis;
        referenceAxis(axis, i, 15, true);
        for (int direction : {-1, 1}) {
            const float edge = direction < 0 ? config::minimum[i] : config::maximum[i];
            // 端点を越えた実測値でも、範囲内へ戻る方向は許可する。
            for (float overshoot : {0.0f, 2.0f}) {
                const float measured =
                    axis.offset + (edge + direction * overshoot) * config::native_per_unit[i];
                axis.speed = direction * config::speed[i];
                moveAxis(axis, i, measured, direction * 1000, 0.02f, false);
                CHECK(axis.speed == 0);
                moveAxis(axis, i, measured, -direction * 1000, 0.02f, false);
                CHECK(axis.speed * direction < 0);
            }
            const float near = axis.offset + (edge - direction * 0.1f) * config::native_per_unit[i];
            axis.speed = direction * config::speed[i];
            moveAxis(axis, i, near, direction * 1000, 0.02f, false);
            CHECK(axis.speed * direction > 0);
            CHECK(std::abs(axis.speed) < config::speed[i]);
        }
    }
}

TEST_CASE("PC入力途絶と復帰で原点を取り直さない") {
    AxisMotion axis;
    referenceAxis(axis, 0, 12, true);
    const float offset = axis.offset;
    CHECK_FALSE(inputFresh(true, 1, 300));
    axis.speed = 0;
    CHECK(inputFresh(true, 301, 301));
    referenceAxis(axis, 0, 13, true);
    CHECK(axis.referenced);
    CHECK(axis.offset == offset);
}

TEST_CASE("0.2秒で最高速度へ達し入力を離すと速度0になる") {
    for (unsigned i = 0; i < 3; ++i) {
        for (int sign : {-1, 1}) {
            AxisMotion axis;
            moveAxis(axis, i, 0, sign * 1000, 0.02f, false);
            CHECK(axis.speed == doctest::Approx(sign * config::speed[i] * 0.1f));
            for (unsigned n = 1; n < 10; ++n)
                moveAxis(axis, i, 0, sign * 1000, 0.02f, false);
            CHECK(axis.speed == doctest::Approx(sign * config::speed[i]));
            for (unsigned n = 0; n < 11; ++n)
                moveAxis(axis, i, 0, 0, 0.02f, false);
            CHECK(axis.speed == 0);
        }
    }
}

TEST_CASE("速度0では実測位置がずれても元の位置へ戻す指令を出さない") {
    for (unsigned i = 0; i < 3; ++i) {
        AxisMotion axis;
        referenceAxis(axis, i, 0, true);
        const float inside = i == 0 ? 500 : 100;
        for (float displacement : {0.0f, 10.0f}) {
            const float measured =
                axis.offset + (inside + displacement) * config::native_per_unit[i];
            moveAxis(axis, i, measured, 0, 0.02f, false);
            CHECK(axis.speed == 0);
        }
    }
}

TEST_CASE("PC入力途絶では6操作を中立にし再受信した入力を使う") {
    int values[6]{1000, -1000, 1000, -1000, 1, 2};
    expireInput(inputFresh(true, 100, 350), values);
    CHECK(values[0] == 1000);
    expireInput(inputFresh(true, 100, 351), values);
    for (int value : values)
        CHECK(value == 0);
    CHECK(parseInput("-1000 0 0 0 0 0", values));
    expireInput(inputFresh(true, 400, 400), values);
    CHECK(values[0] == -1000);
}
