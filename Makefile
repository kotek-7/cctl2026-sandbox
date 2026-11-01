.DEFAULT_GOAL := help
BUILD_TYPE ?= Release
PIO ?= $(HOME)/.platformio/penv/bin/pio
CLANG_FORMAT ?= clang-format
STM32_PROGRAMMER ?= STM32_Programmer_CLI
STLINK_SN ?=
UPLOAD_PORT ?=
CPP_SOURCES := $(filter %.c %.h %.cpp %.hpp,$(shell git ls-files 'cctl/src/*' 'serial_svmd/src/*' 'svmd/src/*' 'tests/*.cpp'))
.PHONY: help build test build-cctl build-serial-svmd build-svmd flash-cctl flash-serial-svmd flash-svmd format format-check
help:
	@printf '%s\n' 'make format / format-check   自作コードの整形 / 整形確認' 'make build   PC用Rustプログラムをビルド' 'cargo run --release --locked -- DEVICE   操縦開始' 'make test    非実機テスト' 'make build-cctl / build-serial-svmd / build-svmd   基板FWビルド' 'make flash-cctl / flash-serial-svmd / flash-svmd   ビルドして基板へ書込み'
build:
	cargo build --release --locked
test:
	cmake -S tests -B tests/build -G Ninja
	cmake --build tests/build
	ctest --test-dir tests/build --output-on-failure
build-cctl:
	cmake -S cctl --preset "$(BUILD_TYPE)"
	cmake --build "cctl/build/$(BUILD_TYPE)"
build-serial-svmd:
	cmake -S serial_svmd --preset "$(BUILD_TYPE)"
	cmake --build "serial_svmd/build/$(BUILD_TYPE)"
build-svmd:
	"$(PIO)" run --project-dir svmd --environment uno_r4_minima

flash-cctl: build-cctl
	"$(STM32_PROGRAMMER)" -c port=SWD $(if $(STLINK_SN),sn="$(STLINK_SN)") -w "cctl/build/$(BUILD_TYPE)/DRC-CCTL2026.elf" -v -rst
flash-serial-svmd: build-serial-svmd
	"$(STM32_PROGRAMMER)" -c port=SWD $(if $(STLINK_SN),sn="$(STLINK_SN)") -w "serial_svmd/build/$(BUILD_TYPE)/DRC-SerialSVMD2026.elf" -v -rst
flash-svmd:
	"$(PIO)" run --project-dir svmd --environment uno_r4_minima --target upload $(if $(UPLOAD_PORT),--upload-port "$(UPLOAD_PORT)")

format:
	cargo fmt
	$(CLANG_FORMAT) -i $(CPP_SOURCES)
format-check:
	cargo fmt --check
	$(CLANG_FORMAT) --dry-run --Werror $(CPP_SOURCES)
