//! Single-slot (direct into ACTIVE) firmware update on top of embassy-boot and embassy-usb-dfu.
//!
//! See firmware-common for shared definitions of bootloader state flags.

use core::cell::Cell;

use embassy_boot::State;
use embassy_boot_nrf::BlockingFirmwareState;
use embassy_usb::class::dfu::consts::Status;
use embassy_usb::class::dfu::dfu_mode::Handler;
use embassy_usb::control::{OutResponse, Recipient, Request, RequestType};
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
    bytes: usize,
}

impl<'d, ACTIVE: NorFlash, STATE: NorFlash, RST: Reset, const BLOCK_SIZE: usize>
    OverwriteDfuHandler<'d, ACTIVE, STATE, RST, BLOCK_SIZE>
{
    /// `inner` must have been created with an updater whose DFU partition is ACTIVE (its reset is
    /// never used). `state` must be the same state partition as the updater's.
    pub fn new(inner: FirmwareHandler<'d, ACTIVE, STATE, RST, BLOCK_SIZE>, state: BlockingFirmwareState<'d, STATE>) -> Self {
        Self { inner, state, bytes: 0 }
    }
}

impl<'d, ACTIVE: NorFlash, STATE: NorFlash, RST: Reset, const BLOCK_SIZE: usize> Handler
    for OverwriteDfuHandler<'d, ACTIVE, STATE, RST, BLOCK_SIZE>
{
    fn start(&mut self) -> Result<(), Status> {
        self.bytes = 0;

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
        let result = self.inner.finish();
        if result.is_ok() {
            defmt::info!("DFU: download complete");
        }
        result
    }

    fn system_reset(&mut self) {
        // only called by the DFU class on a USB reset during dfuMANIFEST-SYNC / dfuMANIFEST, ie
        // after a completed download: HostResetHandler sees the same USB reset in Swap state
    }
}

/// DFU_DETACH bRequest.
const DFU_DETACH: u8 = 0;

/// Requests a reset (`reset_requested`) when the host asks for one:
///
/// - a USB reset with a completed image, such as indicated by `Swap` state.
/// - DFU_DETACH, at any time. Not strictly spec compliant, but a workaround for libusb on Windows,
///   which can't issue a proper USB reset.
///
/// The caller resets, after a delay that allows a DFU_DETACH transfer to finish.
///
/// Must be registered before the DFU class, which accepts and ignores DFU_DETACH.
pub struct HostResetHandler<'d, STATE: NorFlash> {
    state: BlockingFirmwareState<'d, STATE>,
    reset_requested: &'d Cell<bool>,
}

impl<'d, STATE: NorFlash> HostResetHandler<'d, STATE> {
    pub fn new(state: BlockingFirmwareState<'d, STATE>, reset_requested: &'d Cell<bool>) -> Self {
        Self {
            state,
            reset_requested,
        }
    }
}

impl<'d, STATE: NorFlash> embassy_usb::Handler for HostResetHandler<'d, STATE> {
    fn reset(&mut self) {
        if let Ok(State::Swap) = self.state.get_state() {
            defmt::info!("DFU: USB reset after download, resetting");
            self.reset_requested.set(true);
        }
    }

    fn control_out(&mut self, req: Request, _data: &[u8]) -> Option<OutResponse> {
        if (req.request_type, req.recipient, req.request)
            != (RequestType::Class, Recipient::Interface, DFU_DETACH)
        {
            return None;
        }
        defmt::info!("DFU: DFU_DETACH, resetting");
        self.reset_requested.set(true);
        Some(OutResponse::Accepted)
    }
}
