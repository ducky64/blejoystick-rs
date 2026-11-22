//! Firmware update support: the USB DFU run-time interface, the bootloader watchdog, and
//! confirming a freshly downloaded image.
//!
//! Flow (DFU 1.1, see [`firmware_common::dfu_attributes`]): host runs `dfu-util -D app.bin -R` ->
//! DFU_DETACH, then the host's USB reset within wDetachTimeOut (the device does not detach by
//! itself, no WILL_DETACH) on the application's run-time interface -> [`DetachHandler`] marks the state partition `DfuDetach` and resets ->
//! the bootloader (firmware-bootloader/) enumerates in DFU mode and writes the image straight
//! into ACTIVE (no A/B swap) -> USB reset -> the new image boots "on trial" and, once the host has
//! configured its USB DFU run-time interface (so it can take the next update), [`ConfirmHandler`]
//! calls `mark_booted`. If it resets before that (eg, panics, hangs and the watchdog fires, or its
//! USB never enumerates), the bootloader comes back up in DFU mode.

use embassy_boot::{BlockingFirmwareState, State};
use embassy_futures::select::{select, Either};
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_nrf::wdt::Watchdog;
use embassy_nrf::{peripherals, Peri};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch;
use embassy_time::{Duration, Ticker};
use embassy_usb::UsbDevice;
use embassy_usb_dfu::application::{self as dfu_runtime, DfuState};
use embassy_usb_dfu::{Reset, ResetImmediate};
use firmware_common::{dfu_attributes, linker_partition, state_buffer, usb_config, wdt_config};
use nrf_sdc::mpsl::{self, Hfclk, MultiprotocolServiceLayer};
use static_cell::{ConstStaticCell, StaticCell};

use crate::bus::{FlashPartition, GlobalBus};
use crate::prelude::*;
use crate::Irqs;

/// How often the watchdog (started by the bootloader, see [`wdt_config`]) is fed.
const WDT_PET_INTERVAL: Duration = Duration::from_secs(1);

/// wDetachTimeOut: how long after DFU_DETACH the host's USB reset still enters DFU mode. Generous,
/// since a reset by port cycle (eg, on Windows) can take over a second.
const DFU_DETACH_TIMEOUT: Duration = Duration::from_millis(5000);

fn state_partition(bus: &'static GlobalBus) -> FlashPartition {
    linker_partition!(bus.flash, __bootloader_state_start, __bootloader_state_end)
}

/// DFU run-time handler: on DFU_DETACH followed by a USB reset, reboot into the bootloader's DFU mode.
struct DetachHandler {
    bus: &'static GlobalBus,
}

impl dfu_runtime::Handler for DetachHandler {
    fn enter_dfu(&mut self) {
        let mut buf = state_buffer();
        let mut state = BlockingFirmwareState::new(state_partition(self.bus), &mut buf.0);
        match state.mark_dfu() {
            Ok(()) => {
                info!("DFU: detach + USB reset, resetting into bootloader DFU mode");
                ResetImmediate.sys_reset()
            }
            Err(e) => error!("DFU: failed to mark DFU state, staying in run-time mode: {}", e),
        }
    }
}

/// Confirms a trial image (state `DfuDetach`, set by the bootloader before booting a new image) once
/// the host has configured the device, which proves the DFU run-time interface works.
struct ConfirmHandler {
    bus: &'static GlobalBus,
}

impl embassy_usb::Handler for ConfirmHandler {
    fn configured(&mut self, configured: bool) {
        if !configured {
            return;
        }
        let mut buf = state_buffer();
        let mut state = BlockingFirmwareState::new(state_partition(self.bus), &mut buf.0);
        match state.get_state() {
            Ok(State::Boot) => {}  // already confirmed
            Ok(_) => match state.mark_booted() {
                Ok(()) => info!("firmware: USB configured, image confirmed"),
                Err(e) => error!("firmware: failed to mark booted: {}", e),
            },
            Err(e) => error!("firmware: failed to read boot state: {}", e),
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

    let config = usb_config("blejoystick-rs");

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
        dfu_attributes(),
        DFU_DETACH_TIMEOUT,
    ));
    dfu_runtime::usb_dfu(&mut builder, dfu_state, |_| {});

    static CONFIRM_HANDLER: StaticCell<ConfirmHandler> = StaticCell::new();
    builder.handler(CONFIRM_HANDLER.init(ConfirmHandler { bus }));

    let mut device = builder.build();

    let mut usb_powered_rcv = unwrap!(bus.usb_powered.receiver());
    loop {
        usb_powered_rcv.changed_and(|powered| *powered).await;
        run_usb_session(mpsl, &mut device, &mut usb_powered_rcv).await.ok();
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
        Either::Second(_) => {
            info!("USB: VBUS removed, stopping USB device");
            Ok(())
        }
    }
}

/// Takes over and feeds the watchdog started by the bootloader.
#[embassy_executor::task]
pub(crate) async fn wdt_task(wdt: Peri<'static, peripherals::WDT>) {
    let (_wdt, [mut handle]) = match Watchdog::try_new(wdt, wdt_config()) {
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
