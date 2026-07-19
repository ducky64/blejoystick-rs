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

use embassy_nrf::mode::Async;
use embassy_nrf::peripherals::RNG;
use embassy_nrf::{bind_interrupts, rng, saadc};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    EGU0_SWI0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_sdc::mpsl::ClockInterruptHandler;
    RADIO => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    SAADC => saadc::InterruptHandler;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

/// How many outgoing L2CAP buffers per link
const L2CAP_TXQ: u8 = 3;

/// How many incoming L2CAP buffers per link
const L2CAP_RXQ: u8 = 3;

/// Size of L2CAP packets
const L2CAP_MTU: usize = 72;

fn build_sdc<'d, const N: usize>(
    p: nrf_sdc::Peripherals<'d>,
    rng: &'d mut rng::Rng<RNG, Async>,
    mpsl: &'d MultiprotocolServiceLayer,
    mem: &'d mut sdc::Mem<N>,
) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
    sdc::Builder::new()?
        .support_adv()?
        .support_peripheral()?
        .peripheral_count(1)?
        .buffer_cfg(L2CAP_MTU as u16, L2CAP_MTU as u16, L2CAP_TXQ, L2CAP_RXQ)?
        .build(p, rng, mpsl, mem)
}

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

    // initialize BLE - black magic from trouble example
    let mpsl_p =
        mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(mpsl::MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.spawn(mpsl_task(&*mpsl).unwrap());

    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    let mut rng = rng::Rng::new(p.RNG, Irqs);

    let mut sdc_mem = sdc::Mem::<3312>::new();
    let sdc = build_sdc(sdc_p, &mut rng, mpsl, &mut sdc_mem).unwrap();

    // let radio = esp_radio::init().unwrap();
    // let bluetooth = peripherals.BT;
    // let connector = BleConnector::new(&radio, bluetooth, Default::default()).unwrap();
    // let controller: ExternalController<_, 20> = ExternalController::new(connector);
    // let stack = {
    //     let _trng_source = TrngSource::new(peripherals.RNG, peripherals.ADC1.reborrow());
    //     let mut trng = Trng::try_new().unwrap();
    //     ble_peripheral::build_stack(controller, &mut trng)
    // };

    // initialize IO
    let led = Output::new(p.P0_30, Level::Low, OutputDrive::Standard);
    let button = Input::new(p.P0_31, Pull::Up);

    let x_pin = ChannelConfig::single_ended(p.P0_28.reborrow());
    let y_pin = ChannelConfig::single_ended(p.P0_29.reborrow());
    let trig_pin = ChannelConfig::single_ended(p.P0_03.reborrow());

    let adc = Saadc::new(p.SAADC, Irqs, Config::default(), [x_pin, y_pin, trig_pin]);

    // build and run tasks
    spawner.spawn(blinky(led).unwrap());
    spawner.spawn(read_ui(bus, adc).unwrap());

    // ble_peripheral::run(bus, &stack).await;

    // Deep sleep code for power benchmarking
    // let mut rtc = Rtc::new(peripherals.LPWR); // LPWR is the low power peripheral
    // let timer_wakeup =
    //     esp_hal::rtc_cntl::sleep::TimerWakeupSource::new(core::time::Duration::from_secs(10));
    // rtc.sleep_deep(&[&timer_wakeup]);
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
async fn read_ui(bus: &'static GlobalBus, mut adc: Saadc<'static, 3>) {
    const CENTER_X: U0F16 = U0F16::lit("0.70");
    const CENTER_Y: U0F16 = U0F16::lit("0.71");
    const FULLSCALE_XY: U0F16 = U0F16::lit("0.20"); // half-span, including of deadzone
    const DEADZONE_XY: U0F16 = U0F16::lit("0.02");

    let josytick_state_sender = bus.joystick_state.sender();
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

        // let btn_value = button.is_low();

        // info!(
        //     "JX {}    JY {}    Tr {}    Btn {}",
        //     x_linear, y_linear, trig_linear, btn_value,
        // );

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
        led.set_high(); // LED off
        Timer::after(Duration::from_millis(250)).await;
        led.set_low();
        Timer::after(Duration::from_millis(250)).await;
    }
}
