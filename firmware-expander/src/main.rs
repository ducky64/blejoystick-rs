#![no_std]
#![no_main]

/// pinning from edg [
/// pwr_sense=PC4, 14, 
/// eth_grn=PD0, 8, 
/// eth_yel=PC0, 10, 
/// i2c=I2C_T, 
/// i2c.scl=PC2, 12, 
/// i2c.sda=PC1, 11
/// ]

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

use {ch32_hal as hal};


// use hal::Peri;
// use embassy_executor::Spawner;
// use embassy_time::Timer;
// use hal::gpio::{AnyPin, Level, Output};
// use hal::println;


// #[embassy_executor::task(pool_size = 2)]
// async fn blink(pin: Peri<'static, AnyPin>, interval_ms: u64) {
//     let mut led = Output::new(pin, Level::Low, Default::default());

//     loop {
//         led.set_high();
//         Timer::after_millis(interval_ms).await;
//         led.set_low();
//         Timer::after_millis(interval_ms).await;
//     }
// }

// #[embassy_executor::main(entry = "qingke_rt::entry")]
// async fn main(spawner: Spawner) -> ! {
//     hal::debug::SDIPrint::enable();
//     let mut config = hal::Config::default();
//     config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
//     let p = hal::init(config);

//     println!("CHIP signature => {}", hal::signature::chip_id().name());
//     println!("Clocks {:?}", hal::rcc::clocks());

//     // let mut led = Output::new(p.PC4, Level::Low, Default::default());

//     spawner.spawn(blink(p.PD0.into(), 50).unwrap());
//     spawner.spawn(blink(p.PC0.into(), 110).unwrap());

//     loop {
//         Timer::after_millis(1000).await;
//         println!("tick");
//     }
// }



use hal::delay::Delay;
use hal::gpio::{Level, Output};

#[qingke_rt::entry]
fn main() -> ! {
    // hal::debug::SDIPrint::enable();

    info!("start");

    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);

    let mut delay = Delay;

    let mut led1 = Output::new(p.PC0, Level::Low, Default::default());
    led1.toggle();
    let mut led2 = Output::new(p.PD0, Level::Low, Default::default());
    loop {
        led1.toggle();
        led2.toggle();

        delay.delay_ms(50);

        info!("loop {}", hal::pac::SYSTICK.cnt().read());
    }
}
