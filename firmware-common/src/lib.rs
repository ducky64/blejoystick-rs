//! Definitions shared by `firmware-main` and `firmware-bootloader`.
//!
//! # Boot state
//!
//! The bootloader writes images in place (single slot, no A/B swap) and reuses embassy-boot's state
//! magics with this meaning:
//!
//! | state       | meaning                                                              |
//! |-------------|----------------------------------------------------------------------|
//! | `DfuDetach` | no confirmed image: detach requested, download in progress or        |
//! |             | interrupted, or a new image on trial that has not confirmed itself  |
//! | `Swap`      | download complete, not yet booted                                    |
//! | `Boot`      | confirmed image                                                      |

#![no_std]

use embassy_boot::AlignedBuffer;
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::wdt::{self, HaltConfig, SleepConfig, Watchdog, WatchdogHandle};
use embassy_nrf::{peripherals, Peri};
use embassy_usb::class::dfu::consts::DfuAttributes;
use embedded_storage::nor_flash::NorFlash;

/// Starts the watchdog, or takes it over if it's already running: it survives soft resets, so the
/// bootloader's carries into the application. `None` if it's running with a different
/// configuration, which can't be changed or stopped, so a watchdog reset is coming.
pub fn try_wdt(wdt: Peri<'static, peripherals::WDT>) -> Option<WatchdogHandle> {
    let mut config = wdt::Config::default();
    config.timeout_ticks = 32768 * 5; // 5 seconds
    config.action_during_sleep = SleepConfig::Run;
    config.action_during_debug_halt = HaltConfig::Pause;

    Watchdog::try_new(wdt, config)
        .ok()
        .map(|(_, [handle])| handle)
}

/// USB vendor ID, pid.codes.
pub const USB_VID: u16 = 0x1209;
/// USB product ID, pid.codes test PID. TODO allocate a real one.
pub const USB_PID: u16 = 0x0001;

/// USB serial number of the bootloader's DFU mode. The application has none (for now).
///
/// Differs from the application's so the host sees DFU mode as a different device, and
/// dfu-util sees the bootloader's DFU mode as a different device.
pub const USB_SERIAL_DFU: &str = "DFU";

/// USB device configuration, the same for the application's DFU run-time interface and the
/// bootloader's DFU mode, apart from the product and serial number strings.
pub fn usb_config(product: &'static str, serial_number: Option<&'static str>) -> embassy_usb::Config<'static> {
    let mut config = embassy_usb::Config::new(USB_VID, USB_PID);
    config.manufacturer = Some("blejoystick-rs");
    config.product = Some(product);
    config.serial_number = serial_number;
    config.max_power = 100;
    config
}

/// DFU functional descriptor attributes
pub fn dfu_attributes() -> DfuAttributes {
    DfuAttributes::CAN_DOWNLOAD | DfuAttributes::MANIFESTATION_TOLERANT
}

/// How long to wait after accepting DFU_DETACH before resetting, so the request's status stage
/// completes and the host sees it succeed. The nRF52840 USBD completes the status stage in
/// hardware and doesn't report when it's done, so this is a bound, not a signal.
pub const DETACH_RESET_DELAY: core::time::Duration = core::time::Duration::from_millis(10);

/// Scratch buffer for embassy-boot's state partition (one NVMC write word).
pub type StateBuffer = AlignedBuffer<{ Nvmc::WRITE_SIZE }>;

/// A zeroed [`StateBuffer`].
pub const fn state_buffer() -> StateBuffer {
    AlignedBuffer([0; Nvmc::WRITE_SIZE])
}

/// The flash partition for a region defined by a pair of linker symbols `$start`/`$end` in memory.x.
///
/// The symbols are flash addresses, which on the nRF52 are also NVMC offsets (flash starts at 0),
/// so the partition's `offset()` is the region's address.
#[macro_export]
macro_rules! linker_partition {
    ($flash:expr, $start:ident, $end:ident) => {{
        unsafe extern "C" {
            static $start: u32;
            static $end: u32;
        }
        let start = unsafe { &raw const $start } as u32;
        let end = unsafe { &raw const $end } as u32;
        $crate::BlockingPartition::new($flash, start, end - start)
    }};
}

#[doc(hidden)]
pub use embassy_embedded_hal::flash::partition::BlockingPartition;
