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
use hal::gpio::{Flex, Input, Pull, Speed};
use hal::i2c::slave::{I2cSlave, SlaveCommandKind, SlaveConfig};
use hal::prelude::Hertz;
use hal::spi::Spi;
use hal::pac::rcc::vals::{Hpre as AHBPrescaler, Pllsrc as PllSource, Ppre as APBPrescaler, Sw as Sysclk};

use defmt_rtt as _;
mod prelude;
use prelude::*;
mod ws2812;
use ws2812::Ws2812SpiCustom;

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
    ReadBtns = 0x01,
    WriteRgbIndex = 0x12,
    UpdateRgbs = 0x19,
}


#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config {  // effectively 12MHz
        hse: None,
        sys: Sysclk::HSI,
        pll_src: PllSource::HSI,
        ahb_pre: AHBPrescaler::DIV2,
        apb2_pre: APBPrescaler::DIV4,
    };
    let mut p = hal::init(config);

    info!("Init");

    let mut i2c_config = SlaveConfig::default();
    i2c_config.addr = 0x42;
    let mut i2c = I2cSlave::new::<0>(p.I2C1, p.PC2, p.PC1, Irqs, i2c_config);
    let mut last_read_opcode: Opcode = Opcode::ReadBtns;

    {   // this probably doesn't actually do anything useful
        let mut pc6 = Flex::new(p.PC6.reborrow());
        pc6.set_as_output(Speed::Low);
        pc6.set_low();
    }

    let mut spi_config = hal::spi::Config::default();
    spi_config.frequency = Hertz::khz(3000);
    // required to avoid an extra leading edge on SPI when not at 48 MHz
    spi_config.mode = embedded_hal::spi::MODE_1;
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
    let ws_lut = ws2812::MultiBitWs2812Lookup::<u16, 16>::new(0b100, 3, 0b1100, 4);
    let mut ws = Ws2812SpiCustom::<u8, _, _, 11, 4>::new(spi, ws_lut);
    // flash on start
    colors[0] = RGB8 { r: 0, g: 7, b: 0 };
    ws.write(colors.into_iter()).await.unwrap();
    Timer::after_millis(5).await;
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
                        let btn_state_highbit = 1u8 << (btns.len() - 1);
                        for btn in btns.iter() {
                            btns_state >>= 1;
                            if btn.is_low() {
                                btns_state |= btn_state_highbit;
                            }
                        }
                        i2c.respond_to_read(&[btns_state]).await.inspect_err(|e| error!("i2c read error: {:?}", e)).ok();
                        info!("i2c read: read buttons, state {:08b}", btns_state);
                    },
                    _ => {
                        i2c.respond_to_read(&[]).await.inspect_err(|e| error!("i2c read error: {:?}", e)).ok();
                        error!("i2c read: unknown last opcode {:?}", last_read_opcode);
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
