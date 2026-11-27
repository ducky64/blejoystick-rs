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
use embassy_sync::once_lock::OnceLock;
use embassy_usb::class::dfu::consts::DfuAttributes;
use embassy_usb::driver::Driver;
use embassy_usb::msos::{self, windows_version};
use embassy_usb::{Builder, FunctionBuilder};
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

pub const USB_VID: u16 = 0x1209;
pub const USB_PID_MAIN: u16 = 0x0001; // pid.codes test PID, TODO allocate a real one.
pub const USB_PID_BOOT: u16 = 0x0002; // requires a different one so dfu-util detects mode switch

/// Common USB device configuration across both firmwares
pub fn usb_config(product: &'static str, is_bootloader: bool) -> embassy_usb::Config<'static> {
    let mut config = embassy_usb::Config::new(
        USB_VID,
        if is_bootloader {
            USB_PID_BOOT
        } else {
            USB_PID_MAIN
        },
    );
    config.manufacturer = Some("blejoystick-rs");
    config.product = Some(product);
    config.serial_number = Some(serial_number());
    config.max_power = 100;
    config
}

/// Prefix so Vial recognizes the device
const VIAL_SERIAL_MARKER: &[u8; 14] = b"vial:f64c2b3c:";

fn serial_number() -> &'static str {
    static SERIAL: OnceLock<[u8; 30]> = OnceLock::new();
    let serial = SERIAL.get_or_init(|| {
        let ficr = embassy_nrf::pac::FICR;
        let id = (u64::from(ficr.deviceid(1).read()) << 32) | u64::from(ficr.deviceid(0).read());
        let mut serial = [0; 30];
        serial[..14].copy_from_slice(VIAL_SERIAL_MARKER);
        // can't fail: 8 bytes are exactly 16 hex digits
        hex::encode_to_slice(id.to_be_bytes(), &mut serial[14..]).unwrap();
        serial
    });
    // can't fail: only ASCII
    core::str::from_utf8(serial).unwrap()
}

/// bMS_VendorCode: the bRequest of the vendor request Windows uses to fetch the MS OS 2.0 descriptor set.
const MSOS_VENDOR_CODE: u8 = 0x01;

/// Device-specific random GUID for WinUSB function.
const MSOS_INTERFACE_GUIDS: &[&str] = &["{E1D91319-FC5A-4559-B3C5-E097E87E5DB5}"];

pub const MSOS_DESC_LEN: usize = 256;

/// Adds an MS OS 2.0 descriptor set (a BOS platform capability, needs bcdUSB >= 2.1) that has
/// Windows bind WinUSB automatically (no INF or Zadig), so dfu-util can use the device.
///
/// The features are device level: Windows only treats a device as composite (usbccgp, which reads
/// function subsets) when it has more than one interface, and the DFU interface is the only one.
/// For a device with more interfaces, use [`msos_descriptor`] and [`msos_winusb_function`] instead,
/// or WinUSB would replace the composite driver for the whole device.
pub fn msos_winusb<'d, D: Driver<'d>>(builder: &mut Builder<'d, D>) {
    msos_descriptor(builder);
    builder.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    builder.msos_feature(msos_interface_guids());
}

/// Adds an (empty) MS OS 2.0 descriptor set, for functions to add their features to with
/// [`msos_winusb_function`].
pub fn msos_descriptor<'d, D: Driver<'d>>(builder: &mut Builder<'d, D>) {
    builder.msos_descriptor(windows_version::WIN8_1, MSOS_VENDOR_CODE);
}

/// Has Windows bind WinUSB to just this function of a composite device (eg, the DFU run-time
/// interface, via `usb_dfu`'s `func_modifier`), leaving the other functions to their class drivers.
/// Needs [`msos_descriptor`].
pub fn msos_winusb_function<'d, D: Driver<'d>>(func: &mut FunctionBuilder<'_, 'd, D>) {
    func.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    func.msos_feature(msos_interface_guids());
}

fn msos_interface_guids() -> msos::RegistryPropertyFeatureDescriptor<'static> {
    msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        msos::PropertyData::RegMultiSz(MSOS_INTERFACE_GUIDS),
    )
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
