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
use hal::{bind_interrupts, peripherals};
use hal::gpio::{Level, Output};
use hal::i2c::slave::{I2cSlave, SlaveCommandKind, SlaveConfig};
use hal::prelude::Hertz;
use hal::spi::Spi;

use defmt_rtt as _;
mod prelude;
use prelude::*;
use ws2812_async::{Grb, Ws2812};

use smart_leds::{RGB8, SmartLedsWriteAsync};

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

bind_interrupts!(
    struct Irqs {
        I2C1_EV => hal::i2c::EventInterruptHandler<peripherals::I2C1>;
        I2C1_ER => hal::i2c::ErrorInterruptHandler<peripherals::I2C1>;
    }
);

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

    let mut i2c_config = SlaveConfig::default();
    i2c_config.addr = 0x42;
    let mut i2c = I2cSlave::new::<0>(p.I2C1, p.PC2, p.PC1, Irqs, i2c_config);

    let mut spi_config = hal::spi::Config::default();
    spi_config.frequency = Hertz::khz(3000);
    let spi = Spi::new_txonly_nosck::<0>(p.SPI1, p.PC6, p.DMA1_CH3, spi_config);

    let mut colors = [RGB8 { r: 0, g: 0, b: 0 }; 11];
    let mut ws: Ws2812<_, Grb, 11> = Ws2812::new(spi);
    // flash on start
    colors[0] = RGB8 { r: 0, g: 7, b: 0 };
    ws.write(colors.into_iter()).await.unwrap();  // first write may contain garbage
    ws.write(colors.into_iter()).await.unwrap();
    Timer::after_millis(10).await;
    colors[0] = RGB8 { r: 0, g: 0, b: 0 };
    ws.write(colors.into_iter()).await.unwrap();  // clear LEDs on start

    let mut i = 0;

    loop {
        let cmd = match i2c.listen().await {
            Ok(cmd) => cmd,
            Err(e) => {
                error!("listen error: {:?}", e);
                continue;
            }
        };

        match cmd.kind {
            SlaveCommandKind::Write => {
                let mut buf = [0u8; 8];
                match i2c.respond_to_write(&mut buf).await {
                    Ok(_) => i = i + 1,
                    Err(e) => error!("write error: {:?}", e),
                }
            }

            SlaveCommandKind::Read => {
                let mut buf = [0x42u8; 8];
                match i2c.respond_to_read(&buf).await {
                    Ok(status) => (),
                    Err(e) => error!("read error: {:?}", e),
                }
            }
        }

        if i % 2 == 0 {
            colors[0] = RGB8 { r: 0, g: 3, b: 2 };
            colors[2] = RGB8 { r: 0, g: 3, b: 2 };
            colors[4] = RGB8 { r: 0, g: 3, b: 2 };
            colors[9] = RGB8 { r: 0, g: 0, b: 0 };
        } else {
            colors[0] = RGB8 { r: 2, g: 3, b: 0 };
            colors[2] = RGB8 { r: 2, g: 3, b: 0 };
            colors[4] = RGB8 { r: 2, g: 3, b: 0 };
            colors[9] = RGB8 { r: 0, g: 3, b: 0 };
        }
        ws.write(colors.into_iter()).await.unwrap();
        i = i + 1;
    }
}
