//! Single-slot bootloader with USB DFU (DFU 1.1 DFU mode), built from embassy-boot and embassy-usb-dfu.
//!
//! The application exposes only the DFU run-time interface. On DFU_DETACH it marks the state
//! partition `DfuDetach` and resets into this bootloader, which enumerates as a DFU-mode device and
//! streams the new image straight into ACTIVE. See `overwrite.rs` for the state machine.
//!
//! DFU mode is manifestation tolerant without WILL_DETACH (see [`firmware_common::dfu_attributes`]):
//! after the download it returns to dfuIDLE and waits for the host to reset it into the new image,
//! with a USB reset or DFU_DETACH (see [`HostResetHandler`]).
#![no_std]
#![no_main]

mod overwrite;

use core::cell::{Cell, RefCell};
use core::future::poll_fn;
use core::task::Poll;

use cortex_m_rt::{entry, exception};
use defmt_rtt as _;
use embassy_boot_nrf::{
    BlockingFirmwareState, BlockingFirmwareUpdater, BootLoader, FirmwareUpdaterConfig,
};
use embassy_embedded_hal::flash::partition::BlockingPartition;
use embassy_futures::block_on;
use embassy_futures::select::select;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::usb::vbus_detect::{self, HardwareVbusDetect};
use embassy_nrf::usb::{self, Driver};
use embassy_nrf::wdt::WatchdogHandle;
use embassy_nrf::{bind_interrupts, pac, peripherals, Peri};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_usb::class::dfu::dfu_mode::{self, DfuState};
use embassy_usb_dfu::dfu::FirmwareHandler;
use embassy_usb_dfu::ResetImmediate;
use embedded_storage::nor_flash::NorFlash;
use firmware_common::{
    dfu_attributes, linker_partition, msos_winusb, state_buffer, try_wdt, usb_config,
    DETACH_RESET_DELAY, MSOS_DESC_LEN, USB_DEVICE_RELEASE_BOOTLOADER, USB_SERIAL_DFU,
};

use crate::overwrite::{Action, HostResetHandler, OverwriteDfuHandler};

/// DFU transfer size (wTransferSize), one flash page.
const DFU_BLOCK_SIZE: usize = Nvmc::ERASE_SIZE;

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => vbus_detect::InterruptHandler;
});

type SharedFlash = Mutex<NoopRawMutex, RefCell<Nvmc<'static>>>;
type FlashPartition<'a> = BlockingPartition<'a, NoopRawMutex, Nvmc<'static>>;

fn state_partition(flash: &SharedFlash) -> FlashPartition<'_> {
    linker_partition!(flash, __bootloader_state_start, __bootloader_state_end)
}

/// The application image, also where it executes from (its `offset()` is its address).
fn active_partition(flash: &SharedFlash) -> FlashPartition<'_> {
    linker_partition!(flash, __bootloader_active_start, __bootloader_active_end)
}

#[entry]
fn main() -> ! {
    let p = embassy_nrf::init(Default::default());
    // keep power active during DFU / bootloader
    let _pwr_gate = Output::new(p.P0_07, Level::High, OutputDrive::Standard);

    let Some(mut wdt) = try_wdt(p.WDT) else {
        defmt::error!("WDT: already running with a different config, waiting for reset");
        loop {
            cortex_m::asm::wfe();
        }
    };

    let flash: SharedFlash = Mutex::new(RefCell::new(Nvmc::new(p.NVMC)));
    let action = {
        let mut buf = state_buffer();
        let mut state = BlockingFirmwareState::new(state_partition(&flash), &mut buf.0);
        overwrite::decide(&mut state)
    };

    match action {
        Action::Boot => {
            wdt.pet();
            let bl: BootLoader = BootLoader;
            unsafe { bl.load(active_partition(&flash).offset()) }
        }
        Action::Dfu => run_dfu(p.USBD, &flash, wdt),
    }
}

/// Runs USB DFU until the device resets: after a completed download (on the host's USB reset),
/// or on a second download attempt. Never jumps to the application, so the clocks and peripherals
/// set up here never leak into it.
fn run_dfu(
    usbd: Peri<'static, peripherals::USBD>,
    flash: &SharedFlash,
    mut wdt: WatchdogHandle,
) -> ! {
    // USBD needs the HFXO. There's no MPSL in the bootloader, so start it directly.
    let clock = pac::CLOCK;
    clock.events_hfclkstarted().write_value(0);
    clock.tasks_hfclkstart().write_value(1);
    while clock.events_hfclkstarted().read() == 0 {}

    defmt::info!("DFU: entering DFU mode");

    // everything the USB device borrows, declared before it
    let mut config_desc = [0u8; 256];
    let mut bos_desc = [0u8; 256];
    let mut msos_desc = [0u8; MSOS_DESC_LEN];
    let mut control_buf = [0u8; DFU_BLOCK_SIZE]; // must fit a whole DFU_DNLOAD block
    let mut updater_buf = state_buffer();
    let mut state_buf = state_buffer();
    let mut reset_state_buf = state_buffer();
    // set by the handlers below, see HostResetHandler
    let reset_requested = Cell::new(false);
    let mut reset_handler = HostResetHandler::new(
        BlockingFirmwareState::new(state_partition(flash), &mut reset_state_buf.0),
        &reset_requested,
    );

    // embassy-boot's updater, with ACTIVE as its "DFU" partition: the image is written in place
    let updater = BlockingFirmwareUpdater::new(
        FirmwareUpdaterConfig {
            dfu: active_partition(flash),
            state: state_partition(flash),
        },
        &mut updater_buf.0,
    );
    let state = BlockingFirmwareState::new(state_partition(flash), &mut state_buf.0);
    let mut dfu_state = DfuState::new(
        OverwriteDfuHandler::<_, _, _, DFU_BLOCK_SIZE>::new(
            FirmwareHandler::new(updater, ResetImmediate),
            state,
        ),
        dfu_attributes(),
    );

    let driver = Driver::new(usbd, Irqs, HardwareVbusDetect::new(Irqs));
    let config = usb_config("blejoystick-rs bootloader", Some(USB_SERIAL_DFU), USB_DEVICE_RELEASE_BOOTLOADER);
    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        &mut config_desc,
        &mut bos_desc,
        &mut msos_desc,
        &mut control_buf,
    );
    msos_winusb(&mut builder);
    // before the DFU class, which would otherwise accept (and ignore) DFU_DETACH
    builder.handler(&mut reset_handler);
    dfu_mode::usb_dfu(&mut builder, &mut dfu_state, DFU_BLOCK_SIZE, |_| {});
    let mut device = builder.build();

    // block_on busy-polls, so this feeds the watchdog continuously while USB is idle, and picks
    // up a requested reset after the USB stack has finished handling what requested it
    let pet_wdt = poll_fn(|_| {
        wdt.pet();
        if reset_requested.get() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    });
    block_on(select(device.run(), pet_wdt));

    // Let a DFU_DETACH status stage complete (a host's USB reset lasts at least 10 ms anyway).
    // The SysTick runs from the 64 MHz CPU clock.
    let syst = defmt::unwrap!(cortex_m::Peripherals::take()).SYST;
    cortex_m::delay::Delay::new(syst, 64_000_000).delay_us(DETACH_RESET_DELAY.as_micros() as u32);
    cortex_m::peripheral::SCB::sys_reset()
}

#[unsafe(no_mangle)]
#[cfg_attr(target_os = "none", unsafe(link_section = ".HardFault.user"))]
unsafe extern "C" fn HardFault() {
    cortex_m::peripheral::SCB::sys_reset();
}

#[exception]
unsafe fn DefaultHandler(_: i16) -> ! {
    const SCB_ICSR: *const u32 = 0xE000_ED04 as *const u32;
    let irqn = unsafe { core::ptr::read_volatile(SCB_ICSR) } as u8 as i16 - 16;

    panic!("DefaultHandler #{:?}", irqn);
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));
    cortex_m::asm::udf();
}
