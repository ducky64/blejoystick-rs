# blejoystick-rs
Rust firmware for an experimental custom BLE ring joystick

## Flashing the nRF52
In `firmware-bootloader`, `cargo run --release` (release needed to fit code within the allocated partition).

In `firmware-main`, `cargo run` (flash over SWD). This preserves the bootloader and settings partitions.

For future updates over USB DFU, in `firmware-main`, `cargo dfu` (rebuilds, generates .bin, and invokes `dfu-util`). Requires `dfu-util`.
