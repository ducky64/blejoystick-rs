#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use esp_hal::Async;
use static_cell::StaticCell;

use {esp_backtrace as _, esp_println as _};

use usbd_hid::descriptor::SerializedDescriptor;
mod ble_descriptors;
mod ble_peripheral;
mod bus;
mod prelude;
mod util;
use crate::bus::{GlobalBus, JoystickState};
use crate::prelude::*;

// application-specific imports
use fixed::types::{I16F16, I1F15, U0F16, U16F16};

// TrouBLE example imports
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    analog::adc::AdcPin,
    clock::CpuClock,
    peripherals::{ADC1, GPIO0, GPIO1, GPIO3, GPIO4},
};
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::ExternalController;

// BAS bonding imports
use esp_hal::rng::{Trng, TrngSource};
use esp_storage::FlashStorage;

// App-specific imports
use embassy_time::{Duration, Timer};
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static ADC_MUTEX: StaticCell<Mutex<CriticalSectionRawMutex, Adc<'static, ADC1<'static>, Async>>> =
    StaticCell::new();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    // based on examples from https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    // in particular ble_bas_peripheral.rs
    // esp_println::logger::init_logger_from_env();
    let mut peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    esp_alloc::heap_allocator!(size: 72 * 1024);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    #[cfg(target_arch = "riscv32")]
    let software_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        software_interrupt.software_interrupt0,
    );

    info!(
        "descriptor ({}) {:02x}",
        ble_descriptors::MouseReport::desc().len(),
        ble_descriptors::MouseReport::desc()
    );

    // initialize global state and shared peripherals
    let flash = FlashStorage::new(peripherals.FLASH);
    let bus = bus::init(flash);

    // initialize BLE
    let radio = esp_radio::init().unwrap();
    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(&radio, bluetooth, Default::default()).unwrap();
    let controller: ExternalController<_, 20> = ExternalController::new(connector);
    let stack = {
        let _trng_source = TrngSource::new(peripherals.RNG, peripherals.ADC1.reborrow());
        let mut trng = Trng::try_new().unwrap();
        ble_peripheral::build_stack(controller, &mut trng)
    };

    // initialize IO
    let led = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());
    let button = Input::new(
        peripherals.GPIO6,
        InputConfig::default().with_pull(Pull::Up),
    );

    let mut adc_config = AdcConfig::new();
    let x_pin = adc_config.enable_pin(peripherals.GPIO4, Attenuation::_11dB);
    let y_pin = adc_config.enable_pin(peripherals.GPIO3, Attenuation::_11dB);
    let trig_pin = adc_config.enable_pin(peripherals.GPIO1, Attenuation::_11dB);
    let vbat_pin = adc_config.enable_pin(peripherals.GPIO0, Attenuation::_11dB);

    let adc = Adc::new(peripherals.ADC1, adc_config).into_async();
    let adc_mutex = ADC_MUTEX.init(Mutex::new(adc));

    // build and run tasks
    spawner.must_spawn(blinky(led));
    spawner.must_spawn(read_ui(bus, adc_mutex, x_pin, y_pin, button, trig_pin));
    spawner.must_spawn(read_bat(bus, adc_mutex, vbat_pin));

    ble_peripheral::run(bus, &stack).await;
}

fn adc12_to_u0f16(adc: u16) -> U0F16 {
    // converts a unsigned raw 12-bit adc reading to U0F16, using the entire range
    U0F16::from_bits((adc << 4) | (adc >> 8))
}

fn linearize_joystick(adc: U0F16) -> U0F16 {
    // the joystick is a 10k potentiometer, followed by a 10k-10k divider for the ESP32's restricted analog signal range
    // this produces a nonlinear mapping, which is corrected here
    // using formula from https://electronics.stackexchange.com/questions/328212/voltage-divider-equation-with-load-on-output
    // for x = position of joystick, using inverse of their notation
    // D = x / (1 + 10k / 20k * x * (1-x))
    // plug'n'chug into a CAS yields
    // (sqrt(9*d^2 - 4*d + 4) + d - 2) / (2 * d)
    // which is implemented here in fixed point
    if adc == 0 {
        return U0F16::ZERO;
    }
    let adc = U16F16::from_num(adc);
    U0F16::from_num(
        (U16F16::saturating_sub(9 * adc * adc + U16F16::from_num(4), 4 * adc).sqrt() + adc
            - U16F16::from_num(2))
            / (2 * adc),
    )
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
// so for now, the GPIO # is hardcoded instead of generic, yuck!
#[embassy_executor::task]
async fn read_ui(
    bus: &'static GlobalBus,
    adc_mutex: &'static Mutex<CriticalSectionRawMutex, Adc<'static, ADC1<'static>, Async>>,
    mut x_pin: AdcPin<GPIO4<'static>, ADC1<'static>>,
    mut y_pin: AdcPin<GPIO3<'static>, ADC1<'static>>,
    button: Input<'static>,
    mut trig_pin: AdcPin<GPIO1<'static>, ADC1<'static>>,
) {
    const CENTER_X: U0F16 = U0F16::lit("0.70");
    const CENTER_Y: U0F16 = U0F16::lit("0.71");
    const FULLSCALE_XY: U0F16 = U0F16::lit("0.20"); // half-span, including of deadzone
    const DEADZONE_XY: U0F16 = U0F16::lit("0.02");

    let josytick_state_sender = bus.joystick_state.sender();
    loop {
        let (x_adc, y_adc, trig_adc) = {
            let mut adc = adc_mutex.lock().await;
            (
                adc.read_oneshot(&mut x_pin).await,
                adc.read_oneshot(&mut y_pin).await,
                adc.read_oneshot(&mut trig_pin).await,
            )
        };

        let x_linear = scale_bipolar(
            linearize_joystick(adc12_to_u0f16(x_adc)),
            CENTER_X,
            FULLSCALE_XY,
            DEADZONE_XY,
        );
        let y_linear = scale_bipolar(
            linearize_joystick(adc12_to_u0f16(y_adc)),
            CENTER_Y,
            FULLSCALE_XY,
            DEADZONE_XY,
        );

        let btn_value = button.is_low();

        debug!(
            "JX {}    JY {}    Btn {}    Tr {}",
            x_linear, y_linear, btn_value, trig_adc
        );

        let joystick_state = JoystickState {
            x: x_linear,
            y: y_linear,
            btn: btn_value,
        };
        josytick_state_sender.send(joystick_state);
        Timer::after(Duration::from_millis(20)).await;
    }
}

#[embassy_executor::task]
async fn read_bat(
    bus: &'static GlobalBus,
    adc_mutex: &'static Mutex<CriticalSectionRawMutex, Adc<'static, ADC1<'static>, Async>>,
    mut vbat_pin: AdcPin<GPIO0<'static>, ADC1<'static>>,
) {
    let vbus_sender = bus.vbat.sender();
    loop {
        let vbat_value = {
            let mut adc = adc_mutex.lock().await;
            adc.read_oneshot(&mut vbat_pin).await
        };
        debug!("read vbat {}", vbat_value);
        vbus_sender.send(vbat_value); // TODO SCALING
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
async fn blinky(mut led: Output<'static>) {
    loop {
        led.set_high(); // LED off
        Timer::after(Duration::from_millis(250)).await;
        led.set_low();
        Timer::after(Duration::from_millis(250)).await;
    }
}
