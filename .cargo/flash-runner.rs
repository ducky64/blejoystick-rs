//! ```cargo
//! [dependencies]
//! probe-rs = "0.32"
//! ```

// use std::env;
// use std::process::{exit, Command};
use probe_rs::{
    flashing::{DownloadOptions, ElfLoader, ElfOptions, FlashProgress},
    probe::list::Lister,
    MemoryInterface, Permissions,
};
use std::env;
use std::path::Path;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: custom-runner <path-to-elf>");
        std::process::exit(1);
    }
    let elf_path = Path::new(&args[1]);

    let chip = "nRF52840_xxAA";
    let speed_khz = 16_000;
    let backup_addr = 0x000F_E000;
    let backup_size = 4096;

    println!("Attaching to probe...");
    let probes = Lister::new().list_all();
    let mut probe = probes[0].open()?;
    probe.set_speed(speed_khz)?;
    let mut session = probe.attach(chip, Permissions::default())?;
    let mut core = session.core(0)?;

    println!("Backing up persistent flash {:#010X}", backup_addr);
    let mut backup_buffer = vec![0u8; backup_size];
    core.read_8(backup_addr, &mut backup_buffer)?;
    drop(core); // Release core handle before flash operation

    println!("Flashing");
    let mut options = DownloadOptions::default();
    options.progress = FlashProgress::new(|event| println!("Flashing: {:#?}", event));
    options.do_chip_erase = true; // Do fast whole chip erase
    options.keep_unwritten_bytes = false;
    probe_rs::flashing::download_file_with_options(
        &mut session,
        elf_path,
        ElfLoader(ElfOptions::default()),
        options,
    )?;

    println!("Restoring persistent flash");
    let mut loader = session.target().flash_loader();
    loader.add_data(backup_addr, &backup_buffer)?;
    let mut restore_options = DownloadOptions::default();
    restore_options.progress = FlashProgress::new(|event| println!("Restoring: {:#?}", event));
    restore_options.do_chip_erase = false;
    loader.commit(&mut session, restore_options)?;

    println!("Resetting target...");
    let mut core = session.core(0)?;
    core.reset_and_halt(std::time::Duration::from_millis(500))?;
    core.run()?;

    println!("Success!");
    Ok(())
}
