#include "doctest.h"

#include "domain/el05_codec.hpp"

#include <cmath>
#include <cstring>
#include <initializer_list>

using namespace domain::el05;

TEST_CASE("拡張IDは通信種別・data_area_2・対象IDを詰める") {
    const uint32_t id = buildCanId(comm::WRITE_PARAM, 0xFD, 0x7F);

    CHECK(commType(id) == comm::WRITE_PARAM);
    CHECK(dataArea2(id) == 0xFD);
    CHECK((id & 0xff) == 0x7F);
}

TEST_CASE("拡張IDは 29bit に収まる") {
    const uint32_t id = buildCanId(0x1F, 0xFFFF, 0xFF);
    CHECK(id <= 0x1FFFFFFFu);
}

TEST_CASE("通信種別の上位ビットは切り捨てる") {
    const uint32_t id = buildCanId(0x3F, 0x00, 0x10);
    CHECK(commType(id) == 0x1F);
    CHECK((id & 0xff) == 0x10);
}

TEST_CASE("フィードバックはビッグエンディアンで復号する") {
    const uint32_t id = buildCanId(comm::FEEDBACK, 0x7F, 0xFD);
    const uint8_t data[8] = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x01, 0x2C};
    const Feedback fb = decodeFeedback(id, data);

    CHECK(fb.raw_angle == 0x8000);
    CHECK(fb.fault_bits == 0);
}

TEST_CASE("フィードバックの位置は端まで復号できる") {
    const uint32_t id = buildCanId(comm::FEEDBACK, 0x7F, 0xFD);
    const uint8_t low[8] = {0x00, 0x00, 0, 0, 0, 0, 0, 0};
    const uint8_t high[8] = {0xFF, 0xFF, 0, 0, 0, 0, 0, 0};

    CHECK(decodeFeedback(id, low).raw_angle == 0);
    CHECK(decodeFeedback(id, high).raw_angle == 65535);
}

TEST_CASE("パラメータ書込はインデックスと値を並べる") {
    uint8_t out[8] = {};
    encodeParamFloat(param::SPD_REF, 1.5f, out);

    CHECK(out[0] == 0x0a);
    CHECK(out[1] == 0x70);
    CHECK(out[2] == 0);
    CHECK(out[3] == 0);

    float value = 0.0f;
    std::memcpy(&value, &out[4], sizeof(value));
    CHECK(value == doctest::Approx(1.5f));
}

TEST_CASE("uint8 パラメータは値を 1 バイトで置く") {
    uint8_t out[8] = {};
    encodeParamU8(param::RUN_MODE, 2, out);

    CHECK(out[0] == 0x05);
    CHECK(out[1] == 0x70);
    CHECK(out[4] == 2);
    CHECK(out[5] == 0);
}

TEST_CASE("EL05の運転状態と異常ビットを分離して読む") {
    const uint8_t data[8] = {};
    const auto id = buildCanId(comm::FEEDBACK, 0x897F, 0xFD);
    const auto feedback = decodeFeedback(id, data);
    CHECK(feedback.fault_bits == 9);
}

TEST_CASE("EL05の通常返信は正逆の折返しを連続した移動量に変える") {
    const float unit = (POSITION_MAX - POSITION_MIN) / 65535.0f;
    CHECK(positionDelta(65000, 1000) == doctest::Approx(1536 * unit));
    CHECK(positionDelta(1000, 65000) == doctest::Approx(-1536 * unit));
    CHECK(positionDelta(30000, 40000) == doctest::Approx(10000 * unit));
    CHECK(positionDelta(40000, 30000) == doctest::Approx(-10000 * unit));
    CHECK(positionDelta(12345, 12345) == 0);
}
