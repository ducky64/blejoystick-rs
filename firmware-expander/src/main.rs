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
use hal::prelude::Hertz;
use hal::spi::Spi;

use defmt_rtt as _;
mod prelude;
use prelude::*;

// use ws2812_spi::Ws2812;
use smart_leds::SmartLedsWrite;
use ws2812_spi::prerendered::Ws2812;

use core::panic::PanicInfo;
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
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
async fn main(_spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);

    info!("Init");

    let mut spi_config = hal::spi::Config::default();
    spi_config.frequency = Hertz::khz(3200);
    let spi = Spi::new_txonly::<0>(p.SPI1, p.PC5, p.PC6, p.DMA1_CH3, spi_config);

    use smart_leds::RGB8;

    let mut colors = [RGB8 { r: 2, g: 0, b: 2 }; 11];
    let mut npx_buf: [u8; 256] = [0; 256];
    let mut npx = Ws2812::new(spi, &mut npx_buf);

    let mut i = 0;

    loop {
        if i % 2 == 0 {
            colors[0] = RGB8 { r: 0, g: 2, b: 2 };
            colors[2] = RGB8 { r: 0, g: 2, b: 2 };
            colors[4] = RGB8 { r: 0, g: 2, b: 2 };
            colors[9] = RGB8 { r: 0, g: 0, b: 0 };
        } else {
            colors[0] = RGB8 { r: 2, g: 2, b: 0 };
            colors[2] = RGB8 { r: 2, g: 2, b: 0 };
            colors[4] = RGB8 { r: 2, g: 2, b: 0 };
            colors[9] = RGB8 { r: 0, g: 4, b: 0 };
        }
        npx.write(colors.into_iter()).unwrap();
        // TODO wait on SmartLEDs or SPI instead of guess waiting
        Timer::after_millis(250).await;
        i = i + 1;
    }
}
