//! Single-slot (direct into ACTIVE) firmware update on top of embassy-boot and embassy-usb-dfu.
//!
//! See firmware-common for shared definitions of bootloader state flags.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_boot::State;
use embassy_boot_nrf::BlockingFirmwareState;
use embassy_usb::class::dfu::consts::Status;
use embassy_usb::class::dfu::dfu_mode::Handler;
use embassy_usb_dfu::dfu::FirmwareHandler;
use embassy_usb_dfu::Reset;
use embedded_storage::nor_flash::NorFlash;

/// What the bootloader should do after inspecting the state partition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Action {
    /// Jump to the image in ACTIVE.
    Boot,
    /// Stay in the bootloader and run USB DFU.
    Dfu,
}

/// Decides between booting ACTIVE and entering DFU mode, see the module docs.
pub fn decide<STATE: NorFlash>(state: &mut BlockingFirmwareState<'_, STATE>) -> Action {
    match state.get_state() {
        Ok(State::DfuDetach) => {
            defmt::info!("boot: DFU requested, or no confirmed image");
            Action::Dfu
        }
        Ok(State::Swap) => {
            // download completed: boot the new image on trial, it must confirm itself
            if let Err(e) = state.mark_dfu() {
                defmt::error!("boot: failed to mark new image as on trial: {}", e);
            }
            defmt::info!("boot: booting new image on trial");
            Action::Boot
        }
        Ok(state) => {
            // Boot, or Revert (never written in this scheme, treated like Boot)
            defmt::info!("boot: booting confirmed image ({})", state);
            Action::Boot
        }
        Err(e) => {
            defmt::error!("boot: failed to read state: {}", e);
            Action::Dfu
        }
    }
}

/// Set once a complete image has been written. The host's next USB reset (`dfu-util -R`) then
/// resets the device into the new image, see [`UsbResetHandler`].
static DOWNLOAD_COMPLETE: AtomicBool = AtomicBool::new(false);

/// Wraps embassy-usb-dfu's [`FirmwareHandler`] (writing straight into ACTIVE) so that the state is
/// `DfuDetach` for the whole download: an interrupted download always comes back in DFU mode.
pub struct OverwriteDfuHandler<
    'd,
    ACTIVE: NorFlash,
    STATE: NorFlash,
    RST: Reset,
    const BLOCK_SIZE: usize,
> {
    inner: FirmwareHandler<'d, ACTIVE, STATE, RST, BLOCK_SIZE>,
    state: BlockingFirmwareState<'d, STATE>,
    reset: RST,
    started: bool,
    bytes: usize,
}

impl<'d, ACTIVE: NorFlash, STATE: NorFlash, RST: Reset, const BLOCK_SIZE: usize>
    OverwriteDfuHandler<'d, ACTIVE, STATE, RST, BLOCK_SIZE>
{
    /// `inner` must have been created with an updater whose DFU partition is ACTIVE.
    /// `state` must be the same state partition as the updater's.
    pub fn new(
        inner: FirmwareHandler<'d, ACTIVE, STATE, RST, BLOCK_SIZE>,
        state: BlockingFirmwareState<'d, STATE>,
        reset: RST,
    ) -> Self {
        Self {
            inner,
            state,
            reset,
            started: false,
            bytes: 0,
        }
    }
}

impl<'d, ACTIVE: NorFlash, STATE: NorFlash, RST: Reset, const BLOCK_SIZE: usize> Handler
    for OverwriteDfuHandler<'d, ACTIVE, STATE, RST, BLOCK_SIZE>
{
    fn start(&mut self) -> Result<(), Status> {
        if self.started {
            // The embassy-boot updater remembers the last erased sector across downloads, so a
            // second download in the same session could skip an erase. Start over from a reset
            // instead (the state is still DfuDetach, so this comes back up in DFU mode).
            defmt::warn!("DFU: second download in one session, resetting");
            self.reset.sys_reset();
        }
        self.started = true;
        self.bytes = 0;
        DOWNLOAD_COMPLETE.store(false, Ordering::Release);

        // invalidate the image before the first erase
        self.state.mark_dfu().map_err(|e| {
            defmt::error!("DFU: failed to mark state: {}", e);
            Status::ErrWrite
        })?;
        defmt::info!("DFU: download started");
        self.inner.start()
    }

    fn write(&mut self, data: &[u8]) -> Result<(), Status> {
        let result = self.inner.write(data);
        match result {
            Ok(()) => {
                self.bytes += data.len();
                defmt::debug!("DFU: written {} KiB", self.bytes / 1024);
            }
            Err(e) => defmt::error!("DFU: write failed, status {=u8:#x}", e as u8),
        }
        result
    }

    fn finish(&mut self) -> Result<(), Status> {
        // marks the state Swap (= download complete), or fails if signature verification is on and fails
        let result = self.inner.finish();
        if result.is_ok() {
            defmt::info!(
                "DFU: download complete, {} bytes, waiting for USB reset",
                self.bytes
            );
            DOWNLOAD_COMPLETE.store(true, Ordering::Release);
        }
        result
    }

    fn system_reset(&mut self) {
        // only called by the DFU class on a USB reset during dfuMANIFEST-SYNC / dfuMANIFEST
        self.inner.system_reset()
    }
}

/// Resets into the new image on the host's USB reset after a completed download.
///
/// With MANIFESTATION_TOLERANT the DFU class returns to dfuIDLE after the download and never resets
/// by itself. (Non-tolerant mode is broken in embassy-usb 0.6: it reports dfuMANIFEST forever.)
pub struct UsbResetHandler<RST: Reset>(pub RST);

impl<RST: Reset> embassy_usb::Handler for UsbResetHandler<RST> {
    fn reset(&mut self) {
        if DOWNLOAD_COMPLETE.load(Ordering::Acquire) {
            defmt::info!("DFU: USB reset after download, resetting");
            self.0.sys_reset();
        }
    }
}
