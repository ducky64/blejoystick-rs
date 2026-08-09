//! ```cargo
//! [dependencies]
//! probe-rs = "0.32"
//! indicatif = "0.17"
//! ```

use std::env;
use std::path::Path;
use std::process::{Command, ExitCode};
use std::sync::{Arc, Mutex};

use indicatif::{ProgressBar, ProgressStyle};
use probe_rs::{
    flashing::{DownloadOptions, ElfLoader, ElfOptions, FlashProgress},
    probe::list::Lister,
    MemoryInterface, Permissions,
};

fn flash_progress() -> FlashProgress<'static> {
    let pb: Arc<Mutex<Option<ProgressBar>>> = Arc::new(Mutex::new(None));

    FlashProgress::new(move |event| match event {
        probe_rs::flashing::ProgressEvent::AddProgressBar {
            operation: probe_rs::flashing::ProgressOperation::Program,
            total,
        } => {
            let bar = ProgressBar::new(total.unwrap_or(0));
            bar.set_style(
                ProgressStyle::default_bar()
                    .template("[{bar:40}] {bytes}/{total_bytes} ({bytes_per_sec})")
                    .unwrap()
                    .progress_chars("=-"),
            );
            *pb.lock().unwrap() = Some(bar);
        }
        probe_rs::flashing::ProgressEvent::Started(_) => {
            if let Some(bar) = pb.lock().unwrap().as_ref() {
                bar.reset();
            }
        }
        probe_rs::flashing::ProgressEvent::Progress { size, .. } => {
            if let Some(bar) = pb.lock().unwrap().as_ref() {
                bar.inc(size);
            }
        }
        probe_rs::flashing::ProgressEvent::Finished(
            probe_rs::flashing::ProgressOperation::Program,
        ) => {
            if let Some(bar) = pb.lock().unwrap().take() {
                bar.finish();
            }
        }
        _ => {}
    })
}

fn main() -> Result<ExitCode, Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.is_empty() {
        eprintln!("Usage: custom-runner <probe-rs args ...>");
        return Ok(ExitCode::FAILURE);
    }

    let chip = "nRF52840_xxAA";
    let speed_khz = 16_000;
    let backup_addr = 0x000F_E000;
    let backup_size = 4096;

    {
        // scoped to drop probe when done
        println!("Attaching to probe...");
        let probes = Lister::new().list_all();
        let mut probe = probes.into_iter().next().ok_or("No debug probe")?.open()?;
        probe.set_speed(speed_khz)?;
        let mut session = probe.attach(chip, Permissions::default())?;

        println!("Backing up persistent flash {:#010X}...", backup_addr);
        let mut backup_buffer = vec![0u8; backup_size];
        {
            let mut core = session.core(0)?;
            core.read_8(backup_addr, &mut backup_buffer)?;
        }

        println!("Flashing");
        let elf_path_str = args.last().unwrap();
        let elf_path = Path::new(elf_path_str);
        let mut options = DownloadOptions::default();
        options.progress = flash_progress();
        options.do_chip_erase = true;
        probe_rs::flashing::download_file_with_options(
            &mut session,
            elf_path,
            ElfLoader(ElfOptions::default()),
            options,
        )?;

        println!("Restoring persistent flash portion...");
        let mut loader = session.target().flash_loader();
        loader.add_data(backup_addr, &backup_buffer)?;
        let mut restore_options = DownloadOptions::default();
        restore_options.do_chip_erase = false;
        loader.commit(&mut session, restore_options)?;

        {
            let mut core = session.core(0)?;
            core.reset_and_halt(std::time::Duration::from_millis(500))?;
            core.run()?;
        }
    }

    println!("Running probe-rs...");
    let status = Command::new("probe-rs")
        .arg("attach")
        .args(&args)
        .status()?;

    Ok(ExitCode::from(status.code().unwrap_or(0) as u8))
}
