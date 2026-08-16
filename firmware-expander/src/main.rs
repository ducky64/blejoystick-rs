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
use hal::gpio::{Input, Pull};
use hal::i2c::slave::{I2cSlave, SlaveCommandKind, SlaveConfig};
use hal::prelude::Hertz;
use hal::spi::Spi;

use defmt_rtt as _;
mod prelude;
use prelude::*;
use ws2812_async::{Grb, Ws2812};

use smart_leds::{RGB8, SmartLedsWriteAsync};

use num_enum::TryFromPrimitive;

use core::panic::PanicInfo;
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // This blows up flash usage
    // This will print the panic message, file, and line number via defmt!
    // defmt::error!("{}", defmt::Display2Format(info));
    error!("panic");
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


// TODO keep sync'd with main expander.rs
#[derive(PartialEq, TryFromPrimitive, defmt::Format)]
#[repr(u8)]
enum Opcode {
    Resrved = 0x00,
    ReadBtns = 0x01, 
    WriteRgbIndex = 0x12,
    UpdateRgbs = 0x19,
}


#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);

    info!("Init");

    let mut i2c_config = SlaveConfig::default();
    i2c_config.addr = 0x42;
    let mut i2c = I2cSlave::new::<0>(p.I2C1, p.PC2, p.PC1, Irqs, i2c_config);
    let mut last_read_opcode: Opcode = Opcode::Resrved;

    let mut spi_config = hal::spi::Config::default();
    spi_config.frequency = Hertz::khz(3000);
    let spi = Spi::new_txonly_nosck::<0>(p.SPI1, p.PC6, p.DMA1_CH3, spi_config);

    let btns: [Input; 8] = [
        Input::new(p.PD4, Pull::Up),
        Input::new(p.PD5, Pull::Up),
        Input::new(p.PD6, Pull::Up),
        Input::new(p.PD0, Pull::Up),
        Input::new(p.PC0, Pull::Up),
        Input::new(p.PC3, Pull::Up),
        Input::new(p.PC4, Pull::Up),
        Input::new(p.PC5, Pull::Up),
    ];

    let mut colors = [RGB8 { r: 0, g: 0, b: 0 }; 11];
    let mut ws: Ws2812<_, Grb, 11> = Ws2812::new(spi);
    // flash on start
    colors[0] = RGB8 { r: 0, g: 7, b: 0 };
    ws.write(colors.into_iter()).await.unwrap();
    Timer::after_millis(10).await;
    colors[0] = RGB8 { r: 0, g: 0, b: 0 };
    ws.write(colors.into_iter()).await.unwrap();  // clear LEDs on start

    let mut update_ws = false;

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
                match i2c.respond_to_write(&mut buf).await.map(|n| (buf.first().and_then(|x| Opcode::try_from(*x).ok()), n)) {
                    Ok((Some(Opcode::WriteRgbIndex), 5)) => {
                        let index = buf[1] as usize;
                        if index < colors.len() {
                            colors[index] = RGB8 { r: buf[2], g: buf[3], b: buf[4] };
                        } else {
                            error!("i2c write: index {} out of bounds", index);
                        }
                        info!("i2c write: write rgb index {}", index);
                    },
                    Ok((Some(Opcode::UpdateRgbs), 1)) => {
                        update_ws = true;
                        info!("i2c write: update leds");
                    },
                    Ok((Some(Opcode::ReadBtns), 1)) => {
                        last_read_opcode = Opcode::ReadBtns;
                        info!("i2c write: read btns");
                    },
                    Ok((_, 0)) => {
                        info!("i2c write: no data");
                    }
                    Ok((_, n)) => {
                        error!("i2c write: unknown opcode {} len {}", buf[0], n);
                    }
                    Err(e) => {
                        error!("i2c write error: {:?}", e);
                    }
                }
            },
            SlaveCommandKind::Read => {
                match last_read_opcode {
                    Opcode::ReadBtns => {
                        let mut btns_state = 0u8;
                        for (i, btn) in btns.iter().enumerate() {
                            if btn.is_low() {
                                btns_state |= 1 << i;
                            }
                        }
                        info!("i2c read: read buttons, state {:08b}", btns_state);
                        i2c.respond_to_read(&[btns_state]).await.inspect_err(|e| error!("i2c read error: {:?}", e)).ok();
                    },
                    _ => {
                        error!("i2c read: unknown last opcode {:?}", last_read_opcode);
                        i2c.respond_to_read(&[]).await.inspect_err(|e| error!("i2c read error: {:?}", e)).ok();
                    }
                }
            }
        }

        if update_ws {
            match ws.write(colors.into_iter()).await {
                Ok(()) => {
                    update_ws = false;
                }
                Err(e) => {
                    error!("ws write error: {:?}", e);
                }
            }
        }
    }
}
