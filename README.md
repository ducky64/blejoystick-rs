# blejoystick-rs
Rust firmware for an experimental custom BLE ring joystick

## Flashing the nRF52
In `firmware-bootloader`, `cargo run --release` (release needed to fit code within the allocated partition).

In `firmware-main`, `cargo run` (flash over SWD). This preserves the bootloader and settings partitions.

For future updates over USB DFU, in `firmware-main`, `cargo dfu` (rebuilds without `defmt-rtt`, generates .bin, invokes `dfu-util`, then attaches the USB log console). Requires `dfu-util`, `rust-objcopy` (`cargo install cargo-binutils` and `rustup component add llvm-tools`) and `defmt-print` (`cargo install defmt-print`, 1.2+) on PATH. WinUSB binds to the DFU interface automatically.

### Logs
The device always has a USB serial port.

If the device is built with (default) `defmt-rtt` feature, defmt logs go to RTT and the USB serial port is empty.
`cargo run` and `cargo attach` build and attach in this mode.
Otherwise, defmt logs go to USB.
`cargo dfu` and `cargo console` build and attach in this mode, with the port configured by `SERIAL_PORT` in `.cargo/config.toml`.

If running defmt-print: `defmt-print -e target/thumbv7em-none-eabihf/debug/blejoystick-rs serial --path [serial-port]`
