#![no_std]
#![no_main]

/// [ax1=P0.28, 8,
/// ax2=P0.29, 9,
/// trig=P0.03, 6,
/// i2c=TWIM0,
/// i2c.scl=P0.22, 26,
/// i2c.sda=P1.00, 27,
/// led=P0.30, 10,
/// sw=P0.05, 13,
/// gate_ctl=P0.07, 15,
/// bumper=P0.31, 11,
/// stick_pwr_gate=P1.10, 3,
/// trig_pwr_gate=P0.04, 12,
/// chg=P1.11, 2,
/// btns_io0=P0.19, 20,
/// swd=SWD,
/// swd.swclk=SWCLK, 31,
/// swd.swdio=SWDIO, 32,
/// 0=USBD,
/// 0.dp=D+, 24,
/// 0.dm=D-, 23]
use defmt_rtt as _;
use panic_probe as _;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

use usbd_hid::descriptor::SerializedDescriptor;
mod ble_descriptors;
mod ble_peripheral;
mod bus;
mod prelude;
mod util;
use crate::bus::{GlobalBus, JoystickState};
use crate::prelude::*;

use fixed::types::{I16F16, I1F15, U0F16};

use embassy_executor::Spawner;

// App-specific imports
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::saadc::{ChannelConfig, Config, Saadc};
use embassy_time::{Duration, Timer};

static ADC_MUTEX: StaticCell<Mutex<CriticalSectionRawMutex, Saadc<'static, 3>>> = StaticCell::new();

use apache_nimble::controller::NimbleController;
use apache_nimble::controller::NimbleControllerTask;
use embassy_nrf::mode::Async;
// use embassy_nrf::peripherals::RNG;
use embassy_nrf::{bind_interrupts, rng, saadc};
use rand_chacha::ChaCha12Rng;
use rand_core::SeedableRng;

bind_interrupts!(struct Irqs {
    // RNG => rng::InterruptHandler<RNG>;
    SAADC => saadc::InterruptHandler;
});

/// How many outgoing L2CAP buffers per link
const L2CAP_TXQ: u8 = 3;

/// How many incoming L2CAP buffers per link
const L2CAP_RXQ: u8 = 3;

/// Size of L2CAP packets
const L2CAP_MTU: usize = 72;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = embassy_nrf::init(Default::default());
    info!("Starting!");

    info!(
        "descriptor ({}) {:02x}",
        ble_descriptors::MouseReport::desc().len(),
        ble_descriptors::MouseReport::desc()
    );

    // initialize global state and shared peripherals
    let flash = Nvmc::new(p.NVMC);
    let bus = bus::init(flash);

    // initialise peripherals and tasks
    // pull high to enable charging, low to disable
    let mut chg_en = Output::new(p.P1_11, Level::High, OutputDrive::Standard);

    let led = Output::new(p.P0_30, Level::Low, OutputDrive::Standard);
    spawner.spawn(blinky(led).unwrap());

    let mut stick_gate = Output::new(p.P1_10, Level::High, OutputDrive::Standard);
    let mut trig_gate = Output::new(p.P0_04, Level::High, OutputDrive::Standard);

    let bumper = Input::new(p.P0_31, Pull::Up);
    let stick_sw = Input::new(p.P0_05, Pull::Up);

    let x_pin = ChannelConfig::single_ended(p.P0_28.reborrow());
    let y_pin = ChannelConfig::single_ended(p.P0_29.reborrow());
    let trig_pin = ChannelConfig::single_ended(p.P0_03.reborrow());
    let adc = Saadc::new(p.SAADC, Irqs, Config::default(), [x_pin, y_pin, trig_pin]);
    spawner.spawn(read_ui(bus, stick_gate, trig_gate, adc, stick_sw).unwrap());

    // initialize BLE - black magic from trouble example
    // let mut rng = rng::Rng::new(p.RNG, Irqs);
    // let mut rng_2 = ChaCha12Rng::from_rng(&mut rng).unwrap();

    apache_nimble::initialize_nimble();
    let controller = NimbleController::new();
    spawner.spawn(run_controller(controller.create_task()).unwrap());

    let stack = {
        // ble_peripheral::build_stack(controller, &mut rng_2)
        ble_peripheral::build_stack(controller)
    };
    ble_peripheral::run(bus, &stack).await;
}

#[embassy_executor::task]
async fn run_controller(controller_task: NimbleControllerTask) {
    controller_task.run().await
}

fn adc12_to_u0f16(adc: i16) -> U0F16 {
    // converts a unsigned raw 12-bit adc reading to U0F16, using the entire range
    U0F16::from_bits(((adc as u16) << 4) | ((adc as u16) >> 8))
}

fn scale_bipolar(adc: U0F16, center: U0F16, fullscale: U0F16, deadzone: U0F16) -> I1F15 {
    // takes a adc reading [0, 1) and maps it to a bipolar range [-1, 1), with a configurable center, full-scale, and deadzone
    // full-scale and deadzone are both half-span
    // convert everything to larger working type
    let adc = I16F16::from_num(adc);
    let center = I16F16::from_num(center);
    let fullscale = I16F16::from_num(fullscale);
    let deadzone = I16F16::from_num(deadzone);

    let scale_except_deadzone = fullscale - deadzone;
    let adc_offset = if adc > center + deadzone {
        adc - center - deadzone
    } else if adc < center - deadzone {
        adc - (center - deadzone)
    } else {
        I16F16::ZERO
    };
    I1F15::saturating_from_num(I16F16::saturating_div(adc_offset, scale_except_deadzone))
}

// the ADC API seems to be a bit of a dumpster fire
// https://github.com/esp-rs/esp-hal/issues/449
// such that it was removed int he embedded hal 1.0 API
// https://github.com/rust-embedded/embedded-hal/pull/376
#[embassy_executor::task]
async fn read_ui(
    bus: &'static GlobalBus,
    mut stick_gate: Output<'static>,
    mut trig_gate: Output<'static>,
    mut adc: Saadc<'static, 3>,
    stick_sw: Input<'static>,
) {
    const CENTER_X: U0F16 = U0F16::lit("0.70");
    const CENTER_Y: U0F16 = U0F16::lit("0.71");
    const FULLSCALE_XY: U0F16 = U0F16::lit("0.20"); // half-span, including of deadzone
    const DEADZONE_XY: U0F16 = U0F16::lit("0.02");

    let josytick_state_sender = bus.joystick_state.sender();

    stick_gate.set_low();
    trig_gate.set_high();

    loop {
        let mut buf = [0; 3];
        adc.sample(&mut buf).await;
        let x_adc = buf[0];
        let y_adc = buf[1];
        let trig_adc = buf[2];

        let x_linear = scale_bipolar(adc12_to_u0f16(x_adc), CENTER_X, FULLSCALE_XY, DEADZONE_XY);
        let y_linear = scale_bipolar(adc12_to_u0f16(y_adc), CENTER_Y, FULLSCALE_XY, DEADZONE_XY);
        let trig_linear = adc12_to_u0f16(trig_adc)
            .saturating_sub(U0F16::lit("0.55"))
            .saturating_div(U0F16::lit("0.2"));

        let btn_value = stick_sw.is_low();

        info!(
            "JX {} {}    JY {} {}    Tr {} {}    Btn {}",
            x_adc, x_linear, y_adc, y_linear, trig_adc, trig_linear, btn_value,
        );

        let joystick_state = JoystickState {
            x: x_linear,
            y: y_linear,
            trig: I1F15::from_num(trig_linear),
            btn: false, //btn_value,
        };
        josytick_state_sender.send(joystick_state);
        Timer::after(Duration::from_millis(20)).await;
    }
}

// #[embassy_executor::task]
// async fn read_bat(
//     bus: &'static GlobalBus,
//     adc_mutex: &'static Mutex<CriticalSectionRawMutex, Adc<'static, ADC1<'static>, Async>>,
//     mut vbat_pin: AdcPin<GPIO0<'static>, ADC1<'static>>,
// ) {
//     let vbus_sender = bus.vbat.sender();
//     loop {
//         let vbat_value = {
//             let mut adc = adc_mutex.lock().await;
//             adc.read_oneshot(&mut vbat_pin).await
//         };
//         debug!("read vbat {}", vbat_value);
//         vbus_sender.send(vbat_value); // TODO SCALING
//         Timer::after(Duration::from_millis(1000)).await;
//     }
// }

#[embassy_executor::task]
async fn blinky(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(25)).await;
        led.set_low();
        Timer::after(Duration::from_millis(225)).await;
    }
}
