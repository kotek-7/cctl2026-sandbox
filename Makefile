.DEFAULT_GOAL := help
BUILD_TYPE ?= Release
PIO ?= $(HOME)/.platformio/penv/bin/pio
.PHONY: help build test build-cctl build-serial-svmd build-svmd
help:
	@./operate help
	@printf '%s\n' 'make build-cctl / build-serial-svmd / build-svmd  基板FWをビルド'
build:
	./operate build
test:
	./operate test
build-cctl:
	cmake -S cctl --preset "$(BUILD_TYPE)"
	cmake --build "cctl/build/$(BUILD_TYPE)"
build-serial-svmd:
	cmake -S serial_svmd --preset "$(BUILD_TYPE)"
	cmake --build "serial_svmd/build/$(BUILD_TYPE)"
build-svmd:
	"$(PIO)" run --project-dir svmd --environment uno_r4_minima
