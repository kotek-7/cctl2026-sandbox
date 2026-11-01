cmake_minimum_required(VERSION 3.22)
project(manual_tests CXX)
set(CMAKE_CXX_STANDARD 17)
enable_testing()
add_executable(codec_tests
    doctest_main.cpp
    cctl/c620_codec_test.cpp cctl/el05_codec_test.cpp
    cctl/manual_test.cpp
    serial_svmd/sts3215_protocol_test.cpp
    ../cctl/src/domain/c620_codec.cpp ../cctl/src/domain/el05_codec.cpp
    ../serial_svmd/src/domain/sts3215_protocol.cpp
)
target_include_directories(codec_tests PRIVATE vendor ../cctl/src ../serial_svmd/src)
target_compile_options(codec_tests PRIVATE -Wall -Wextra)
add_test(NAME codec_tests COMMAND codec_tests)
