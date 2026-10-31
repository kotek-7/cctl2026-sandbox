cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME DRC-CCTL2026)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C CXX ASM)

add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/motors.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ee.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/can_bus.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/el05_motor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/m3508_motor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/domain/pid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/domain/delta_timer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/domain/c620_codec.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/domain/el05_codec.cpp
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# STM32CubeMXのC++リンク時に不要なlibobを除く。
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)
