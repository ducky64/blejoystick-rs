//! Firmware update support: the USB DFU run-time interface, the bootloader watchdog, and
//! confirming a freshly downloaded image.
//!
//! Flow (DFU 1.1): host runs `dfu-util -D app.bin -R` -> DFU_DETACH + USB reset on the application's
//! run-time interface -> [`DetachHandler`] marks the state partition `DfuDetach` and resets ->
//! the bootloader (firmware-bootloader/) enumerates in DFU mode and writes the image straight
//! into ACTIVE (no A/B swap) -> USB reset -> the new image boots "on trial" and, once it has been
//! running for [`MARK_BOOTED_DELAY`], calls `mark_booted`. If it resets before that (eg, panics,
//! or hangs and the watchdog fires), the bootloader comes back up in DFU mode.

use embassy_boot::{AlignedBuffer, BlockingFirmwareState, State};
use embassy_futures::select::{select, Either};
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_nrf::wdt::{self, HaltConfig, SleepConfig, Watchdog};
use embassy_nrf::{peripherals, Peri};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch;
use embassy_time::{Duration, Ticker, Timer};
use embassy_usb::UsbDevice;
use embassy_usb_dfu::application::{self as dfu_runtime, DfuAttributes, DfuState};
use embassy_usb_dfu::{Reset, ResetImmediate};
use embedded_storage::nor_flash::NorFlash;
use nrf_sdc::mpsl::{self, Hfclk, MultiprotocolServiceLayer};
use static_cell::{ConstStaticCell, StaticCell};

use crate::bus::{linker_partition, FlashPartition, GlobalBus};
use crate::prelude::*;
use crate::Irqs;

/// Watchdog configuration, must match firmware-bootloader/src/main.rs, otherwise the
/// running watchdog (which cannot be stopped or reconfigured) cannot be taken over.
const WDT_TIMEOUT_TICKS: u32 = 32768 * 5; // 5 seconds
const WDT_PET_INTERVAL: Duration = Duration::from_secs(1);

/// How long a freshly downloaded image must run before it is confirmed.
const MARK_BOOTED_DELAY: Duration = Duration::from_secs(5);

/// wDetachTimeOut: how long after DFU_DETACH the host's USB reset still enters DFU mode.
const DFU_DETACH_TIMEOUT: Duration = Duration::from_millis(1000);

const USB_VID: u16 = 0x1209; // pid.codes
const USB_PID: u16 = 0x0001; // pid.codes test PID, TODO allocate a real one

/// The embassy-boot state uses one STATE::WRITE_SIZE scratch buffer.
type StateBuffer = AlignedBuffer<{ Nvmc::WRITE_SIZE }>;

fn state_partition(bus: &'static GlobalBus) -> FlashPartition {
    linker_partition!(bus.flash, __bootloader_state_start, __bootloader_state_end)
}

/// DFU run-time handler: on DFU_DETACH followed by a USB reset, reboot into the bootloader's DFU mode.
struct DetachHandler {
    bus: &'static GlobalBus,
}

impl dfu_runtime::Handler for DetachHandler {
    fn enter_dfu(&mut self) {
        info!("DFU: detach, resetting into bootloader DFU mode");
        let mut buf: StateBuffer = AlignedBuffer([0; Nvmc::WRITE_SIZE]);
        let mut state = BlockingFirmwareState::new(state_partition(self.bus), &mut buf.0);
        match state.mark_dfu() {
            Ok(()) => ResetImmediate.sys_reset(),
            Err(e) => error!("DFU: failed to mark DFU state: {}", e),
        }
    }
}

/// Runs the USB device (DFU run-time interface only for now) whenever VBUS is present.
#[embassy_executor::task]
pub(crate) async fn usb_task(
    bus: &'static GlobalBus,
    mpsl: &'static MultiprotocolServiceLayer<'static>,
    usbd: Peri<'static, peripherals::USBD>,
) {
    // VBUS is reported in software by charger_task, since the POWER interrupt belongs to the MPSL
    let driver = Driver::new(usbd, Irqs, &bus.vbus);

    let mut config = embassy_usb::Config::new(USB_VID, USB_PID);
    config.manufacturer = Some("blejoystick-rs");
    config.product = Some("blejoystick-rs");
    config.serial_number = None;
    config.max_power = 100;

    static CONFIG_DESC: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
    static BOS_DESC: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
    static MSOS_DESC: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
    static CONTROL_BUF: ConstStaticCell<[u8; 64]> = ConstStaticCell::new([0; 64]);
    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        CONFIG_DESC.take(),
        BOS_DESC.take(),
        MSOS_DESC.take(),
        CONTROL_BUF.take(),
    );

    static DFU_STATE: StaticCell<DfuState<DetachHandler>> = StaticCell::new();
    let dfu_state = DFU_STATE.init(DfuState::new(
        DetachHandler { bus },
        DfuAttributes::CAN_DOWNLOAD | DfuAttributes::MANIFESTATION_TOLERANT,
        DFU_DETACH_TIMEOUT,
    ));
    dfu_runtime::usb_dfu(&mut builder, dfu_state, |_| {});

    let mut device = builder.build();

    let mut usb_powered_rcv = unwrap!(bus.usb_powered.receiver());
    loop {
        usb_powered_rcv.changed_and(|powered| *powered).await;
        run_usb_session(mpsl, &mut device, &mut usb_powered_rcv).await.ok()
    }
}

type UsbPoweredReceiver = watch::Receiver<'static, CriticalSectionRawMutex, bool, 2>;

/// Runs the USB device for one VBUS-present session, returning once VBUS is removed.
/// The HFCLK is held for the duration of the session and released on return.
async fn run_usb_session(
    mpsl: &'static MultiprotocolServiceLayer<'static>,
    device: &mut UsbDevice<'static, Driver<'static, &'static SoftwareVbusDetect>>,
    usb_powered_rcv: &mut UsbPoweredReceiver,
) -> Result<(), mpsl::Error> {
    // USBD needs the HFXO, whose ownership belongs to the MPSL
    let _hfclk = mpsl.request_hfclk().await.inspect_err(|e| error!("USB: failed to request HFCLK: {}", e))?;
    Hfclk::wait().await.inspect_err(|e| error!("USB: HFCLK failed to start: {}", e))?;
    info!("USB: enabled");

    let usb_fut = async {
        loop {
            device.run_until_suspend().await;
            device.wait_resume().await;
        }
    };
    match select(usb_fut, usb_powered_rcv.changed_and(|powered| !*powered)).await {
        Either::First(never) => never,
        Either::Second(_) => Ok(()),
    }
}

/// Takes over and feeds the watchdog started by the bootloader.
#[embassy_executor::task]
pub(crate) async fn wdt_task(wdt: Peri<'static, peripherals::WDT>) {
    let mut config = wdt::Config::default();
    config.timeout_ticks = WDT_TIMEOUT_TICKS;
    config.action_during_sleep = SleepConfig::Run;
    config.action_during_debug_halt = HaltConfig::Pause;

    let (_wdt, [mut handle]) = match Watchdog::try_new(wdt, config) {
        Ok(x) => x,
        Err(_) => {
            // running with a different configuration, nothing we can do but let it reset us
            error!("WDT: already running with a different config, a reset is imminent");
            return;
        }
    };

    let mut ticker = Ticker::every(WDT_PET_INTERVAL);
    loop {
        handle.pet();
        ticker.next().await;
    }
}

/// Confirms the running image after it has been up for a while, so the bootloader won't revert it.
#[embassy_executor::task]
pub(crate) async fn mark_booted_task(bus: &'static GlobalBus) {
    let mut buf: StateBuffer = AlignedBuffer([0; Nvmc::WRITE_SIZE]);
    let mut state = BlockingFirmwareState::new(state_partition(bus), &mut buf.0);

    match state.get_state() {
        Ok(State::Boot) => {
            info!("firmware: booted confirmed image");
            return;  // no need to re-confirm
        },
        // the bootloader marks a freshly downloaded image DfuDetach before booting it
        Ok(State::DfuDetach) => info!("firmware: booted new image, on trial"),
        Ok(state) => warn!("firmware: unexpected boot state: {:?}", state),
        Err(e) => {
            error!("firmware: failed to read boot state: {}", e);
            return;
        }
    }

    Timer::after(MARK_BOOTED_DELAY).await;

    match state.mark_booted() {
        Ok(()) => info!("firmware: image confirmed"),
        Err(e) => error!("firmware: failed to mark booted: {}", e),
    }
}
