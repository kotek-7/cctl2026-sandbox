#include "doctest.h"

#include "domain/pid.hpp"

using domain::Pid;

TEST_CASE("dt が 0 なら P 項だけが出る") {
    Pid pid(2.0f, 100.0f, 100.0f, 1000.0f);

    CHECK(pid.update(1.0f, 0.0f, 0.0f) == doctest::Approx(2.0f));
}

TEST_CASE("積分項は誤差と時間の積を溜める") {
    Pid pid(0.0f, 1.0f, 0.0f, 100.0f);

    CHECK(pid.update(1.0f, 0.0f, 0.5f) == doctest::Approx(0.5f));
    CHECK(pid.update(1.0f, 0.0f, 0.5f) == doctest::Approx(1.0f));
}

TEST_CASE("微分項は誤差の変化率に比例する") {
    Pid pid(0.0f, 0.0f, 2.0f, 100.0f);

    pid.update(0.0f, 0.0f, 0.0f);
    CHECK(pid.update(1.0f, 0.0f, 0.1f) == doctest::Approx(20.0f));
    CHECK(pid.update(1.0f, 0.0f, 0.1f) == doctest::Approx(0.0f));
}

TEST_CASE("出力は上下限にクランプされる") {
    Pid pid(100.0f, 0.0f, 0.0f, 5.0f);

    CHECK(pid.update(1.0f, 0.0f, 0.1f) == doctest::Approx(5.0f));
    CHECK(pid.update(-1.0f, 0.0f, 0.1f) == doctest::Approx(-5.0f));
}

TEST_CASE("飽和して誤差が同方向なら積分を捨てる") {
    // anti-windup。飽和中に積分が伸び続けると復帰時に大きく行き過ぎる。
    Pid pid(0.0f, 10.0f, 0.0f, 1.0f);

    CHECK(pid.update(1.0f, 0.0f, 1.0f) == doctest::Approx(1.0f));

    CHECK(pid.update(0.0f, 0.0f, 1.0f) == doctest::Approx(0.0f));
}

TEST_CASE("飽和していなければ積分は保持される") {
    Pid pid(0.0f, 1.0f, 0.0f, 100.0f);

    pid.update(1.0f, 0.0f, 1.0f);
    CHECK(pid.update(0.0f, 0.0f, 1.0f) == doctest::Approx(1.0f));
}

TEST_CASE("reset で内部状態が消える") {
    Pid pid(0.0f, 1.0f, 0.0f, 100.0f);
    pid.update(1.0f, 0.0f, 1.0f);

    pid.reset();

    CHECK(pid.update(1.0f, 0.0f, 1.0f) == doctest::Approx(1.0f));
}

TEST_CASE("ゲインは後から差し替えられる") {
    Pid pid(1.0f, 0.0f, 0.0f, 100.0f);
    pid.setGains(3.0f, 0.0f, 0.0f);

    CHECK(pid.update(1.0f, 0.0f, 0.0f) == doctest::Approx(3.0f));
}

TEST_CASE("出力制限を実行時に変えられる") {
    domain::Pid pid(100.0f, 0.0f, 0.0f, 10.0f);
    CHECK(pid.update(100.0f, 0.0f, 0.01f) == doctest::Approx(10.0f));
    pid.setOutLimit(50.0f);
    CHECK(pid.outLimit() == doctest::Approx(50.0f));
    CHECK(pid.update(100.0f, 0.0f, 0.01f) == doctest::Approx(50.0f));
}
