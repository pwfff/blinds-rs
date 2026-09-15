.DEFAULT_GOAL := build
.PHONY: build run test

# Run outside the project tree so Cargo does not inherit ESP build-std settings.
HOST_TARGET := $(shell rustc -vV | awk '/^host:/ {print $$2}')
HOST_TEST_MANIFEST := $(CURDIR)/tests/host/Cargo.toml

test:
	cd /tmp && cargo test --locked --manifest-path "$(HOST_TEST_MANIFEST)" --target "$(HOST_TARGET)"

build:
	CROSS_COMPILE=xtensa-esp32-elf cargo build

run:
	CROSS_COMPILE=xtensa-esp32-elf cargo run