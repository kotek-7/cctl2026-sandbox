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

TEST_CASE("初回入力と正常応答が揃えば原点登録なしで出力できる") {
    CHECK_FALSE(outputsEnabled(false, 0, 0, true));
    CHECK(outputsEnabled(true, 100, 100, true));
    CHECK(outputsEnabled(true, 100, 350, true));
    CHECK_FALSE(outputsEnabled(true, 100, 351, true));
    CHECK_FALSE(outputsEnabled(true, 100, 100, false));
    CHECK(outputsEnabled(true, 400, 400, true));
    CHECK(outputsEnabled(true, 0xfffffff0u, 20, true));
    CHECK_FALSE(outputsEnabled(true, 0xfffffff0u, 300, true));
}

TEST_CASE("原点未登録では全軸が従来の座標範囲外でも動ける") {
    for (unsigned i = 0; i < 3; ++i) {
        for (int sign : {-1, 1}) {
            const float measured = sign * 1000 * config::native_per_unit[i];
            AxisMotion axis{false, 0, measured, 0};
            moveAxis(axis, i, measured, sign * 1000, 0.02f, false);
            CHECK(axis.speed * sign > 0);
            CHECK((axis.target - measured) / config::native_per_unit[i] * sign > 0);
            CHECK_FALSE(axis.referenced);
        }
    }
}

TEST_CASE("rとzはそれぞれ最初の接触だけで原点を登録する") {
    AxisMotion axes[3]{{false, 0, 10, 100}, {false, 0, 20, 50}, {false, 0, 30, -100}};
    referenceAxis(axes[0], 0, 11, false);
    CHECK_FALSE(axes[0].referenced);
    referenceAxis(axes[0], 0, 11, true);
    CHECK(axes[0].referenced);
    CHECK((11 - axes[0].offset) / config::native_per_unit[0] == doctest::Approx(570));
    CHECK(axes[0].target == 11);
    CHECK(axes[0].speed == 0);
    CHECK_FALSE(axes[2].referenced);
    CHECK(axes[2].target == 30);
    CHECK(axes[2].speed == -100);
    referenceAxis(axes[2], 2, 31, true);
    CHECK(axes[2].referenced);
    CHECK(axes[2].offset == 31);
    CHECK(axes[2].target == 31);
    CHECK(axes[2].speed == 0);
    CHECK_FALSE(axes[1].referenced);
    CHECK(axes[1].target == 20);
    CHECK(axes[1].speed == 50);
    const float offset = axes[0].offset;
    axes[0].target = 12;
    axes[0].speed = -20;
    referenceAxis(axes[0], 0, 13, true);
    CHECK(axes[0].offset == offset);
    CHECK(axes[0].target == 12);
    CHECK(axes[0].speed == -20);
}

TEST_CASE("θは原点登録も角度制限も行わない") {
    AxisMotion axis{false, 0, 720 * config::native_per_unit[1], 0};
    referenceAxis(axis, 1, axis.target, true);
    CHECK_FALSE(axis.referenced);
    const float measured = axis.target;
    moveAxis(axis, 1, measured, 1000, 0.02f, true);
    CHECK(axis.target > measured);
}

TEST_CASE("登録前後とも接点方向を止め反対方向へ離れられる") {
    for (unsigned i : {0u, 2u}) {
        const int toward = i == 0 ? 1 : -1;
        for (bool referenced : {false, true}) {
            AxisMotion axis{false, 0, 12, 0};
            if (referenced)
                referenceAxis(axis, i, 12, true);
            // 既存の位置目標が接点を越えて残っている場合も止める。
            axis.target = 12 + toward * config::native_per_unit[i];
            moveAxis(axis, i, 12, toward * 1000, 0.02f, true);
            CHECK(axis.speed == 0);
            CHECK(axis.target == 12);
            moveAxis(axis, i, 12, -toward * 1000, 0.02f, true);
            CHECK(axis.speed * toward < 0);
            CHECK((axis.target - 12) / config::native_per_unit[i] * toward < 0);
        }
    }
}

TEST_CASE("登録後はrとzの上下端で速度と目標を制限する") {
    for (unsigned i : {0u, 2u}) {
        AxisMotion axis;
        referenceAxis(axis, i, 15, true);
        for (int direction : {-1, 1}) {
            const float edge = direction < 0 ? config::minimum[i] : config::maximum[i];
            const float measured = axis.offset + edge * config::native_per_unit[i];
            axis.target = measured;
            axis.speed = 0;
            moveAxis(axis, i, measured, direction * 1000, 0.02f, false);
            CHECK(axis.speed == 0);
            CHECK(axis.target == doctest::Approx(measured));
            // 端点の直前でも次の目標を範囲内に収める。
            const float near =
                axis.offset + (edge - direction * 0.01f) * config::native_per_unit[i];
            axis.target = near;
            axis.speed = direction * config::speed[i];
            moveAxis(axis, i, near, direction * 1000, 0.02f, false);
            const float position = (axis.target - axis.offset) / config::native_per_unit[i];
            CHECK(position >= config::minimum[i] - 0.001f);
            CHECK(position <= config::maximum[i] + 0.001f);
            CHECK(std::abs(axis.speed) < config::speed[i]);
        }
    }
}

TEST_CASE("通信途絶と復帰で軸の原点は変わらない") {
    AxisMotion axis;
    referenceAxis(axis, 0, 12, true);
    const float offset = axis.offset;
    CHECK_FALSE(outputsEnabled(true, 1, 300, true));
    // 出力解除・再開時に行う現在位置保持は座標情報を変更しない。
    axis.target = 13;
    axis.speed = 0;
    CHECK(outputsEnabled(true, 301, 301, true));
    referenceAxis(axis, 0, 13, true);
    CHECK(axis.referenced);
    CHECK(axis.offset == offset);
}

TEST_CASE("未登録軸でも加速量と目標の先行幅を制限する") {
    for (unsigned i = 0; i < 3; ++i) {
        AxisMotion axis{false, 0, 0, 0};
        moveAxis(axis, i, 0, 1000, 0.02f, false);
        CHECK(axis.speed == doctest::Approx(config::speed[i] * 0.1f));
        for (unsigned n = 0; n < 100; ++n)
            moveAxis(axis, i, 0, 1000, 0.02f, false);
        CHECK(axis.speed <= config::speed[i]);
        CHECK(std::abs(axis.target) <=
              std::abs(config::speed[i] * config::native_per_unit[i]) * 0.1f + 0.001f);
    }
}
