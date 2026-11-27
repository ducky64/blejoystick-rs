//! Cargo runner that flashes over USB DFU instead of SWD.
//! Converts the ELF to a raw binary next to it, downloads it with dfu-util, then attaches the USB
//! log console with defmt-print, like console-runner.rs (port from `SERIAL_PORT`).
//! std only, so rust-script builds it once in about a second and caches it.
//!
//! Needs on PATH: `rust-objcopy` (cargo-binutils + llvm-tools), `dfu-util` and `defmt-print`.
//!
//! usage: dfu-runner.rs <vid:pid> <elf>

use std::path::Path;
use std::process::{Command, ExitCode};
use std::thread::sleep;
use std::time::Duration;

/// After dfu-util's final reset, how long the new image takes to boot and enumerate its USB log port.
const ENUMERATE_DELAY: Duration = Duration::from_secs(2);

fn run(cmd: &mut Command) -> bool {
    match cmd.status() {
        Ok(status) if status.success() => true,
        Ok(status) => {
            eprintln!("dfu-runner: {cmd:?} failed: {status}");
            false
        }
        Err(e) => {
            eprintln!("dfu-runner: failed to start {cmd:?} (is it on PATH?): {e}");
            false
        }
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [device, elf] = args.as_slice() else {
        eprintln!("usage: dfu-runner.rs <vid:pid> <elf>");
        return ExitCode::FAILURE;
    };

    // next to the ELF in target/, so it always matches the build that produced it
    let bin = Path::new(elf).with_extension("bin");

    let ok = run(Command::new("rust-objcopy").arg(elf).args(["-O", "binary"]).arg(&bin))
        // -E 1: the app detaches immediately, so only wait 1s for the bootloader to enumerate
        // -R: final USB reset boots the new image
        && run(Command::new("dfu-util").args(["-d", device, "-E", "1", "-R", "-D"]).arg(&bin))
        && {
            sleep(ENUMERATE_DELAY);
            // --dtr: the device only sends while DTR is set
            run(Command::new("defmt-print").arg("-e").arg(elf).args(["serial", "--dtr"]))
        };

    if ok { ExitCode::SUCCESS } else { ExitCode::FAILURE }
}
