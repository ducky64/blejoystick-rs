#![no_std]
#![no_main]

/// pinning from edg
/// [
/// io0=PC7, 17,
/// dpad_0=PD4, 1,
/// dpad_1=PD5, 2,
/// dpad_2=PD6, 3,
/// dpad_3=PD0, 8,
/// dpad_4=PC0, 10,
/// dpad_5=PC3, 13,
/// dpad_6=PC4, 14,
/// dpad_7=PC5, 15,
/// npx=PC6, 16,
/// i2c=I2C_T,
/// i2c.scl=PC2, 12,
/// i2c.sda=PC1, 11
/// ]
use ch32_hal as hal;
use hal::gpio::{Level, Output};

use defmt::{debug, error, info, warn};
use defmt_rtt as _;

use core::panic::PanicInfo;
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // This blows up flash usage
    // This will print the panic message, file, and line number via defmt!
    // defmt::error!("{}", defmt::Display2Format(info));
    defmt::error!("panic");
    loop {
        core::hint::spin_loop();
    }
}

use embassy_executor::Spawner;
use embassy_time::Timer;

// #[embassy_executor::task(pool_size = 2)]
// async fn blink(pin: Peri<'static, AnyPin>, interval_ms: u64) {}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);

    // info!("CHIP signature => {}", hal::signature::chip_id().name());
    // info!("Clocks {:?}", hal::rcc::clocks());

    // let mut led = Output::new(p.PC4, Level::Low, Default::default());

    loop {
        Timer::after_millis(1000).await;
        info!("tick");
    }
}

// #[qingke_rt::entry]
// fn main() -> ! {
//     use hal::delay::Delay;

//     let mut config = hal::Config::default();
//     config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
//     let p = hal::init(config);

//     // info!("CHIP signature => {}", hal::signature::chip_id().name());

//     let mut delay = Delay;

//     let mut led1 = Output::new(p.PC0, Level::Low, Default::default());
//     led1.toggle();
//     let mut led2 = Output::new(p.PD0, Level::Low, Default::default());
//     loop {
//         led1.toggle();
//         led2.toggle();

//         delay.delay_ms(50);

//         info!("loop {}", hal::pac::SYSTICK.cnt().read());
//     }
// }
