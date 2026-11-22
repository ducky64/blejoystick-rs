//! ```cargo
//! [dependencies]
//! probe-rs = "0.32"
//! indicatif = "0.17"
//! pico-args = "0.5"
//! parse_int = "0.9"
//! ```

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
    let mut args = pico_args::Arguments::from_env();

    // passthrough probe-rs args
    let chip: String = args.value_from_str("--chip")?;
    let speed_khz: Option<u32> = args.opt_value_from_str("--speed")?;

    // new args, repeatable: each --persist-addr is paired with the --persist-size at the same position
    let persist_addrs: Vec<u64> = args.values_from_fn("--persist-addr", parse_int::parse)?;
    let persist_sizes: Vec<usize> = args.values_from_fn("--persist-size", parse_int::parse)?;
    if persist_addrs.len() != persist_sizes.len() {
        return Err("each --persist-addr must have a matching --persist-size".into());
    }

    let elf_path_str: String = args.free_from_str()?;
    let elf_path = Path::new(&elf_path_str);
    if !elf_path.exists() {
        return Err(format!("ELF file not found: {}", elf_path.display()).into());
    }

    let remaining = args.finish();
    if !remaining.is_empty() {
        return Err(format!("Unexpected arguments: {:?}", remaining).into());
    }

    {
        // scoped to drop probe when done
        println!("Attaching to probe...");
        let probes = Lister::new().list_all();
        let mut probe = probes.into_iter().next().ok_or("No debug probe")?.open()?;
        if let Some(speed_khz) = speed_khz {
            probe.set_speed(speed_khz)?;
        }
        let mut session = probe.attach(&chip, Permissions::default())?;

        let mut persist_buffers = Vec::new();
        {
            let mut core = session.core(0)?;
            for (&addr, &size) in persist_addrs.iter().zip(persist_sizes.iter()) {
                println!("Reading persistent flash {:#010X}+{:#X}...", addr, size);
                let mut buffer = vec![0u8; size];
                core.read_8(addr, &mut buffer)?;
                persist_buffers.push((addr, buffer));
            }
        }

        // ideally we could delegate this to probe-rs as a subcommand
        // but probe-rs does not support skipping erasing
        println!("Flashing...");
        let mut options = DownloadOptions::default();
        options.progress = flash_progress();
        options.do_chip_erase = true;
        probe_rs::flashing::download_file_with_options(
            &mut session,
            elf_path,
            ElfLoader(ElfOptions::default()),
            options,
        )?;

        println!("Restoring persistent flash...");
        let mut loader = session.target().flash_loader();
        for (addr, buffer) in &persist_buffers {
            loader.add_data(*addr, buffer)?;
        }
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
    let mut cmd = Command::new("probe-rs");
    cmd.arg("attach").arg("--chip").arg(&chip);
    if let Some(speed_khz) = speed_khz {
        cmd.arg("--speed").arg(speed_khz.to_string());
    }
    cmd.arg(&elf_path_str);

    let status = cmd.status()?;

    Ok(ExitCode::from(status.code().unwrap_or(0) as u8))
}
