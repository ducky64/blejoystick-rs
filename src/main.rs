#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

#[cfg(feature = "defmt")]
use defmt::{debug, info, warn, error};
use esp_hal::Async;
#[cfg(feature = "log")]
use log::{debug, info, warn, error};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex};
use embassy_sync::mutex::Mutex;
use num_traits::{AsPrimitive, Bounded, PrimInt};
use static_cell::StaticCell;

use {esp_backtrace as _, esp_println as _};


mod ble_peripheral;
mod ble_descriptors;
mod bus;
use crate::bus::{GlobalBus, JoystickState};

// TrouBLE example imports
use embassy_executor::Spawner;
use esp_hal::{analog::adc::{AdcPin}, clock::CpuClock, peripherals::{ADC1, GPIO0, GPIO1, GPIO3, GPIO4}};
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::ExternalController;
use esp_backtrace as _;

// BAS bonding imports
use esp_hal::rng::{Trng, TrngSource};
use esp_storage::FlashStorage;

// App-specific imports
use embassy_time::{Duration, Timer};
use esp_hal::{
    gpio::{Input, InputConfig, Pull, Level, Output, OutputConfig},
    analog::adc::{AdcConfig, Adc, Attenuation},
};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();


static ADC_MUTEX: StaticCell<Mutex<CriticalSectionRawMutex, Adc<'static, ADC1<'static>, Async>>> = StaticCell::new();


#[esp_rtos::main]
async fn main(spawner: Spawner) {
    // based on examples from https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    // in particular ble_bas_peripheral.rs
    // esp_println::logger::init_logger_from_env();
    let mut peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    esp_alloc::heap_allocator!(size: 72 * 1024);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    #[cfg(target_arch = "riscv32")]
    let software_interrupt = esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        software_interrupt.software_interrupt0,
    );

    info!("report length = {}", 
        <ble_descriptors::MouseReport as usbd_hid::descriptor::SerializedDescriptor>::desc().len());

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
    let button = Input::new(peripherals.GPIO6, InputConfig::default().with_pull(Pull::Up));

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


fn scale_adc_bipolar<InType, ResultType, IntermediateType>(
    adc_value: InType, center: InType, fullscale: InType, deadzone: InType
) -> ResultType
where
    InType: PrimInt + AsPrimitive<IntermediateType>,
    ResultType: PrimInt + Bounded + AsPrimitive<IntermediateType>,
    IntermediateType: PrimInt + 'static,
{
    let adc_i: IntermediateType = adc_value.as_();
    let center_i: IntermediateType = center.as_();
    let deadzone_i: IntermediateType = deadzone.as_();
    let scale_except_deadzone: IntermediateType = fullscale.as_() - deadzone_i;

    let result = if adc_value > center + deadzone {
        adc_i - center_i - deadzone_i
    } else if adc_value < center - deadzone {
        IntermediateType::zero() - (center_i - deadzone_i - adc_i)
    } else {
        IntermediateType::zero()
    } * ResultType::max_value().as_() / scale_except_deadzone;

    return ResultType::from(
        result.clamp(ResultType::min_value().as_(), ResultType::max_value().as_())
    ).unwrap_or(ResultType::zero());
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
        mut x_pin: AdcPin<GPIO4<'static>, ADC1<'static>>, mut y_pin: AdcPin<GPIO3<'static>, ADC1<'static>>, 
        button: Input<'static>,
        mut trig_pin: AdcPin<GPIO1<'static>, ADC1<'static>>) {
    const CENTER_X: u16 = 2610;
    const CENTER_Y: u16 = 2650;
    const FULLSCALE_XY: u16 = 1000;  // adc counts for full scale, half-span
    const DEADZONE_XY: u16 = 32;  // in ADC counts, half-span

    let josytick_state_sender = bus.joystick_state.sender();
    loop {
        let (x_adc, y_adc, trig_adc) = {
            let mut adc = adc_mutex.lock().await;
            (adc.read_oneshot(&mut x_pin).await,
            adc.read_oneshot(&mut y_pin).await,
            adc.read_oneshot(&mut trig_pin).await)
        };
        let btn_value = button.is_low();
        debug!("JX {}    JY {}    Btn {}    Tr {}",
            x_adc, y_adc, btn_value, trig_adc);

        let joystick_state = JoystickState {
            x: scale_adc_bipolar::<u16, i16, i32>(x_adc, CENTER_X, FULLSCALE_XY, DEADZONE_XY),
            y: scale_adc_bipolar::<u16, i16, i32>(y_adc, CENTER_Y, FULLSCALE_XY, DEADZONE_XY),
            btn: btn_value,
        };
        josytick_state_sender.send(joystick_state);
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[embassy_executor::task]
async fn read_bat(
        bus: &'static GlobalBus,
        adc_mutex: &'static Mutex<CriticalSectionRawMutex, Adc<'static, ADC1<'static>, Async>>, 
        mut vbat_pin: AdcPin<GPIO0<'static>, ADC1<'static>>) {
    let vbus_sender = bus.vbat.sender();
    loop {
        let vbat_value = {
            let mut adc = adc_mutex.lock().await;
            adc.read_oneshot(&mut vbat_pin).await
        };
        debug!("read vbat {}", vbat_value);
        vbus_sender.send(vbat_value);  // TODO SCALING
        Timer::after(Duration::from_millis(1000)).await;
    }
}


#[embassy_executor::task]
async fn blinky(mut led: Output<'static>) {
    loop {
        led.set_high();  // LED off
        Timer::after(Duration::from_millis(250)).await;
        led.set_low();
        Timer::after(Duration::from_millis(250)).await;
    }
}
