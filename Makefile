.DEFAULT_GOAL := help
BUILD_TYPE ?= Release
PIO ?= $(HOME)/.platformio/penv/bin/pio
.PHONY: help setup test build-cctl build-serial-svmd build-svmd
help:
	@printf '%s\n' 'make setup   Python依存を準備' 'uv run --locked operate.py DEVICE   操縦開始' 'make test    非実機テスト' 'make build-cctl / build-serial-svmd / build-svmd   基板FWビルド'
setup:
	uv sync --locked
test:
	uv run --locked python -m unittest discover -s tests/python -v
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
