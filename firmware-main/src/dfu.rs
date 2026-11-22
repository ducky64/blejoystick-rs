//! Firmware update support: USB DFU (in-application download), the bootloader watchdog,
//! and confirming a freshly swapped image.
//!
//! Flow: host runs `dfu-util -D app.bin -R` -> the DFU class writes the image into the DFU
//! partition and marks the state partition as "swap" -> the device returns to dfuIDLE
//! (manifestation tolerant) -> host issues a USB reset -> system reset -> the bootloader
//! (firmware-bootloader/) swaps ACTIVE and DFU -> new image boots and, once it has been
//! running for [`MARK_BOOTED_DELAY`], calls `mark_booted`. If it resets before that
//! (eg, hangs and the watchdog fires), the bootloader reverts to the previous image.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_boot::{AlignedBuffer, BlockingFirmwareState, BlockingFirmwareUpdater, FirmwareUpdaterConfig, State};
use embassy_futures::select::{select, Either};
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_nrf::wdt::{self, HaltConfig, SleepConfig, Watchdog};
use embassy_nrf::{peripherals, Peri};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch;
use embassy_time::{Duration, Ticker, Timer};
use embassy_usb::class::dfu::consts::{DfuAttributes, Status};
use embassy_usb::class::dfu::dfu_mode::{self, DfuState, Handler};
use embassy_usb_dfu::dfu::FirmwareHandler;
use embassy_usb::UsbDevice;
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

/// Set once a complete image is written and marked for swap. The next USB bus reset from the
/// host then performs a system reset into the bootloader (DFU 1.1, manifestation tolerant).
static SWAP_PENDING: AtomicBool = AtomicBool::new(false);

/// How long a freshly swapped image must run before it is confirmed.
const MARK_BOOTED_DELAY: Duration = Duration::from_secs(5);

/// DFU transfer size (wTransferSize), one flash page.
const DFU_BLOCK_SIZE: usize = Nvmc::ERASE_SIZE;

const USB_VID: u16 = 0x1209; // pid.codes
const USB_PID: u16 = 0x0001; // pid.codes test PID, TODO allocate a real one

/// The DFU state and the firmware updater both use one STATE::WRITE_SIZE scratch buffer each.
type StateBuffer = AlignedBuffer<{ Nvmc::WRITE_SIZE }>;

fn updater_config(bus: &'static GlobalBus) -> FirmwareUpdaterConfig<FlashPartition, FlashPartition> {
    FirmwareUpdaterConfig {
        dfu: linker_partition!(bus.flash, __bootloader_dfu_start, __bootloader_dfu_end),
        state: linker_partition!(bus.flash, __bootloader_state_start, __bootloader_state_end),
    }
}

/// Wraps the embassy-usb-dfu firmware handler to keep the device awake and log progress.
struct DfuHandler {
    inner: FirmwareHandler<'static, FlashPartition, FlashPartition, ResetImmediate, DFU_BLOCK_SIZE>,
    bus: &'static GlobalBus,
    bytes: usize,
}

impl Handler for DfuHandler {
    fn start(&mut self) -> Result<(), Status> {
        info!("DFU: download started");
        self.bytes = 0;
        self.bus.activity();
        self.inner.start()
    }

    fn write(&mut self, data: &[u8]) -> Result<(), Status> {
        self.bus.activity();
        let result = self.inner.write(data);
        match result {
            Ok(()) => {
                self.bytes += data.len();
                info!("DFU: written {} KiB", self.bytes / 1024);
            },
            Err(e) => error!("DFU: write failed, status {=u8:#x}", e as u8),  // DFU1.1 bStatus code
        }
        result
    }

    fn finish(&mut self) -> Result<(), Status> {
        info!("DFU: download complete, {} bytes", self.bytes);
        let result = self.inner.finish();
        if result.is_ok() {
            SWAP_PENDING.store(true, Ordering::Release);
        }
        result
    }

    fn system_reset(&mut self) {
        info!("DFU: resetting into bootloader");
        self.inner.system_reset()
    }
}

/// Performs the system reset into the bootloader on the host's USB reset after a completed
/// download. The DFU class only does this itself in the (non-tolerant) manifestation states.
struct UsbResetHandler;

impl embassy_usb::Handler for UsbResetHandler {
    fn reset(&mut self) {
        if SWAP_PENDING.load(Ordering::Acquire) {
            info!("DFU: USB reset with swap pending, resetting into bootloader");
            ResetImmediate.sys_reset();
        }
    }
}

/// Runs the USB device (DFU class only for now) whenever VBUS is present.
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
    config.product = Some("blejoystick-rs DFU");
    config.serial_number = None;
    config.max_power = 100;

    static CONFIG_DESC: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
    static BOS_DESC: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
    static MSOS_DESC: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
    // must fit a whole DFU_DNLOAD block
    static CONTROL_BUF: ConstStaticCell<[u8; DFU_BLOCK_SIZE]> = ConstStaticCell::new([0; DFU_BLOCK_SIZE]);
    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        CONFIG_DESC.take(),
        BOS_DESC.take(),
        MSOS_DESC.take(),
        CONTROL_BUF.take(),
    );

    static UPDATER_BUF: ConstStaticCell<StateBuffer> = ConstStaticCell::new(AlignedBuffer([0; Nvmc::WRITE_SIZE]));
    let updater = BlockingFirmwareUpdater::new(updater_config(bus), &mut UPDATER_BUF.take().0);
    static DFU_STATE: StaticCell<DfuState<DfuHandler>> = StaticCell::new();
    let dfu_state = DFU_STATE.init(DfuState::new(
        DfuHandler {
            inner: FirmwareHandler::new(updater, ResetImmediate),
            bus,
            bytes: 0,
        },
        // Manifestation tolerant: returns to dfuIDLE on completion and the host's USB reset ends the session.
        DfuAttributes::CAN_DOWNLOAD | DfuAttributes::MANIFESTATION_TOLERANT,
    ));
    dfu_mode::usb_dfu(&mut builder, dfu_state, DFU_BLOCK_SIZE, |_| {});

    static USB_RESET_HANDLER: StaticCell<UsbResetHandler> = StaticCell::new();
    builder.handler(USB_RESET_HANDLER.init(UsbResetHandler));

    let mut device = builder.build();

    let mut usb_powered_rcv = unwrap!(bus.usb_powered.receiver());
    loop {
        usb_powered_rcv.changed_and(|powered| *powered).await;
        match run_usb_session(mpsl, &mut device, &mut usb_powered_rcv).await {
            Ok(()) => info!("USB: VBUS removed"),
            Err(e) => error!("USB: HFCLK error: {}", e),
        }
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
    let mut state = BlockingFirmwareState::from_config(updater_config(bus), &mut buf.0);

    match state.get_state() {
        Ok(State::Boot) => {
            info!("firmware: booted comfirmed image"); 
            return;  // no need to re-confirm
        },
        Ok(State::Swap) => info!("firmware: booted new image"),
        Ok(State::Revert) => warn!("firmware: reverted to previous image"),
        Ok(state) => warn!("firmware: unknown boot state: {:?}", state),
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
