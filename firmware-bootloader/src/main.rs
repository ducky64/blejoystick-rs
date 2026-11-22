//! Minimal swap bootloader, based on embassy's examples/boot/bootloader/nrf.
#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m_rt::{entry, exception};
use defmt_rtt as _;
use embassy_boot_nrf::*;
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::wdt::{self, HaltConfig, SleepConfig};
use embassy_sync::blocking_mutex::Mutex;

/// Watchdog configuration, must match `firmware-main/src/dfu.rs`: WDT_CONFIG,
/// otherwise the application cannot take over the running watchdog.
const WDT_TIMEOUT_TICKS: u32 = 32768 * 5; // 5 seconds

#[entry]
fn main() -> ! {
    let p = embassy_nrf::init(Default::default());

    let mut wdt_config = wdt::Config::default();
    wdt_config.timeout_ticks = WDT_TIMEOUT_TICKS;
    wdt_config.action_during_sleep = SleepConfig::Run;
    wdt_config.action_during_debug_halt = HaltConfig::Pause;

    let flash = WatchdogFlash::start(Nvmc::new(p.NVMC), p.WDT, wdt_config);
    let flash = Mutex::new(RefCell::new(flash));

    let config = BootLoaderConfig::from_linkerfile_blocking(&flash, &flash, &flash);
    let active_offset = config.active.offset();
    let bl: BootLoader = BootLoader::prepare(config);

    unsafe { bl.load(active_offset) }
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
