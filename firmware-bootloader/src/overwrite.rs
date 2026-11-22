//! Single-slot (direct into ACTIVE) firmware update on top of embassy-boot and embassy-usb-dfu.
//!
//! See firmware-common for shared definitions of bootloader state flags.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_boot::State;
use embassy_boot_nrf::BlockingFirmwareState;
use embassy_usb::class::dfu::consts::Status;
use embassy_usb::class::dfu::dfu_mode::Handler;
use embassy_usb::control::{OutResponse, Recipient, Request, RequestType};
use embassy_usb_dfu::dfu::FirmwareHandler;
use embassy_usb_dfu::{Reset, ResetImmediate};
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
/// resets the device into the new image, see [`HostResetHandler`].
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

/// DFU_DETACH bRequest.
const DFU_DETACH: u8 = 0;

/// Set by DFU_DETACH, see [`HostResetHandler`]. The reset itself happens in the main loop, once the
/// request has been answered.
static DETACH_REQUESTED: AtomicBool = AtomicBool::new(false);

/// Whether DFU_DETACH has requested a reset.
pub fn detach_requested() -> bool {
    DETACH_REQUESTED.load(Ordering::Acquire)
}

/// Resets the device when the host asks for it:
///
/// - a USB reset after a completed download (the DFU 1.1 way).
/// - DFU_DETACH, at any time. Not strictly spec compliant, but a workaround for libusb on Windows,
///   which can't issue a proper USB reset.
///
/// Must be registered before the DFU class, which accepts and ignores DFU_DETACH.
pub struct HostResetHandler;

impl embassy_usb::Handler for HostResetHandler {
    fn reset(&mut self) {
        if DOWNLOAD_COMPLETE.load(Ordering::Acquire) {
            defmt::info!("DFU: USB reset after download, resetting");
            ResetImmediate.sys_reset();
        }
    }

    fn control_out(&mut self, req: Request, _data: &[u8]) -> Option<OutResponse> {
        if (req.request_type, req.recipient, req.request)
            != (RequestType::Class, Recipient::Interface, DFU_DETACH)
        {
            return None;
        }
        defmt::info!(
            "DFU: DFU_DETACH, resetting (download complete: {})",
            DOWNLOAD_COMPLETE.load(Ordering::Acquire)
        );
        DETACH_REQUESTED.store(true, Ordering::Release);
        Some(OutResponse::Accepted)
    }
}
