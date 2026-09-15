# blinds-rs

ESP32-S3 / TMC2240 STEP/DIR smoke test using the local `../tmc-rs` checkout
(updated through `bcd52aa`). This is not a blinds controller: there are no
end stops, position tracking, acceleration ramps, or stall-based stopping.

## configuration

`src/motor.rs` contains a compile-time validated SpreadCycle profile, including
RMS current selection, microstepping, external clock and current-transition timing.
The complete 22-register checked image replaces the old raw-register/reset setup.

Hardware: BIGTREETECH TMC2240 module and STEPPERONLINE 17HS19-2004S1.
The [BTT V1.0 schematic][board-schematic] specifies R1 = 12 kΩ from IREF to
GND, and R3 = 4.7 kΩ pulling DRV_ENN up to VCC_IO. Confirm your module revision
matches. Use 3.3 V VCC_IO with the ESP32 and a common ground.

The [motor datasheet][motor-datasheet] specifies 2 A/phase, 1.4 Ω/phase,
3 mH/phase and 1.8° steps (200 full steps/revolution). Winding A is black/green;
winding B is red/blue. Connect each pair to one driver output pair; do not
connect or disconnect motor leads while powered.

The configured ceiling is **1400 mA RMS**, conservatively keeping the nominal
sinusoidal peak below 2 A. The motor datasheet does not explicitly call its
2 A/phase rating sinusoidal peak or RMS. Run/hold requests remain **900/450 mA
RMS** as starting values; the driver quantizes these downward. Holding torque
under the blinds' load and operating temperatures still need testing.

The demo starts automatically after configuration and readback succeed, using
the existing wiring and supplied heatsink. Keep the blinds disconnected for
testing: 500 full steps/s is 150 rpm for this motor, without an acceleration
ramp.

[board-schematic]: https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC2240/Hardware/TMC2240_V1.0-SCH.pdf
[motor-datasheet]: https://www.omc-stepperonline.com/download/17HS19-2004S1.pdf

Behavior changes:

- SpreadCycle replaces StealthChop. Checked StealthChop profiles are not yet
  supported upstream. Freewheeling and StallGuard4 configuration are removed;
  the motor now uses a nonzero regulated hold current while stopped.
- Microstepping remains 256, rising-edge STEP, at 128 kHz (500 full steps/s).
  Motion alternates five seconds moving and five seconds stopped, reversing
  direction during each stop before pulses resume.
- Current-transition timings retain the previous 16 MHz register encodings.
  SpreadCycle chopper settings use the datasheet starting point, not finished tuning.
- SPI uses mode 3 at 5 MHz. The 16 MHz clock and STEP use 50% duty cycles.
- EN stays high until the checked write and CHOPCONF readback succeed. Runtime
  SPI failures and temperature/short flags stop STEP and disable the bridge;
  there is no automatic retry. Status is polled once per second, not an emergency
  stop. Provide a hardware pull-up on DRV_ENN for reset/boot and hardware protection.

Pin assignment is unchanged:

| signal | GPIO |
| --- | --- |
| MOSI / SCLK / CS / MISO | 4 / 5 / 6 / 7 |
| DRV_ENN | 15 |
| driver clock / STEP / DIR | 16 / 17 / 18 |

## build and test

```sh
make build
make run
make test
```

Firmware needs an Espressif Xtensa-enabled Rust toolchain (`esp` in
`rust-toolchain.toml`), `ldproxy`, and ESP-IDF v5.1.2 build prerequisites.
`~/nixos-config` now defines a pinned Espressif toolchain and rustup dispatchers;
activate it with `make switch` there, then restart your editor. Check that
`rustup show active-toolchain` reports `esp` in this directory. The regular nix
Rust compiler cannot compile this Xtensa target. The nix-owned ESP Cargo wrapper
also selects Espressif libclang for bindgen. Manage toolchain changes through nix.

The Rust ESP crates use `esp-idf-svc` 0.52 / `esp-idf-hal` 0.46 for current
Xtensa C-character compatibility. HAL's `rmt-legacy` feature avoids a new RMT
queue incompatibility with the pinned IDF 5.1.2. The synchronous demo does not
enable Embassy's executor-dependent timer driver by default.
Adjust the flash port in `.cargo/config.toml` for your machine.

`make test` runs the actual `src/motor.rs` profile tests on the host without
ESP-IDF, using a small standalone manifest under `tests/host`. It checks the
staged register image, motion/timing consistency, and quantized currents.
It does not exercise ESP peripherals or establish safe motor tuning.

No Wi-Fi credentials are needed; the unused Wi-Fi scaffolding has been removed.
