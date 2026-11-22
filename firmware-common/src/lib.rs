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
use embassy_nrf::wdt::{self, HaltConfig, SleepConfig};
use embedded_storage::nor_flash::NorFlash;

/// Watchdog configuration. The bootloader starts the watchdog, and the application takes it over,
/// which only works with an identical configuration.
pub fn wdt_config() -> wdt::Config {
    let mut config = wdt::Config::default();
    config.timeout_ticks = 32768 * 5; // 5 seconds
    config.action_during_sleep = SleepConfig::Run;
    config.action_during_debug_halt = HaltConfig::Pause;
    config
}

/// USB vendor ID, pid.codes.
pub const USB_VID: u16 = 0x1209;
/// USB product ID, pid.codes test PID. TODO allocate a real one.
pub const USB_PID: u16 = 0x0001;

/// USB device configuration, the same for the application's DFU run-time interface and the
/// bootloader's DFU mode, apart from the product string.
pub fn usb_config(product: &'static str) -> embassy_usb::Config<'static> {
    let mut config = embassy_usb::Config::new(USB_VID, USB_PID);
    config.manufacturer = Some("blejoystick-rs");
    config.product = Some(product);
    config.serial_number = None;
    config.max_power = 100;
    config
}

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
