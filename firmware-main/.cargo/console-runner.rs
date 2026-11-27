//! Cargo runner that attaches the USB log console: decodes the defmt stream from the device's USB
//! log port (src/usb_log.rs) with defmt-print and the ELF. The USB counterpart of `cargo attach`.
//! std only, so rust-script builds it once in about a second and caches it.
//!
//! The port comes from `SERIAL_PORT` (read by defmt-print, defaulted in .cargo/config.toml).
//!
//! Needs on PATH: `defmt-print` (1.2+, for `serial`).
//!
//! usage: console-runner.rs <elf>

use std::process::{Command, ExitCode};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [elf] = args.as_slice() else {
        eprintln!("usage: console-runner.rs <elf>");
        return ExitCode::FAILURE;
    };

    // --dtr: the device only sends while DTR is set
    let mut cmd = Command::new("defmt-print");
    cmd.arg("-e").arg(elf).args(["serial", "--dtr"]);
    match cmd.status() {
        Ok(status) if status.success() => ExitCode::SUCCESS,
        Ok(status) => {
            eprintln!("console-runner: {cmd:?} failed: {status}");
            ExitCode::FAILURE
        }
        Err(e) => {
            eprintln!("console-runner: failed to start {cmd:?} (is it on PATH?): {e}");
            ExitCode::FAILURE
        }
    }
}
