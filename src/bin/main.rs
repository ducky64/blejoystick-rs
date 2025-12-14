#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

// TrouBLE example imports
use embassy_executor::Spawner;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::ExternalController;
use esp_backtrace as _;

// App-specific imports
use embedded_hal_nb::nb;
use embassy_time::{Duration, Timer};
use esp_hal::{
    gpio::{Input, InputConfig, Pull, Level, Output, OutputConfig},
    analog::adc::{AdcConfig, Adc, Attenuation},
};
use esp_println::println;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    // based on examples from https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    // in particular ble_bas_peripheral.rs
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    esp_alloc::heap_allocator!(size: 72 * 1024);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    #[cfg(target_arch = "riscv32")]
    let software_interrupt = esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        software_interrupt.software_interrupt0,
    );

    let radio = esp_radio::init().unwrap();
    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(&radio, bluetooth, Default::default()).unwrap();
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    // App-specific code starts here
    println!("Quack quack quack world");

    let led = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());
    let button = Input::new(peripherals.GPIO6, InputConfig::default().with_pull(Pull::Up));

    let mut adc_config = esp_hal::analog::adc::AdcConfig::new();
    let mut joystick_x_pin = adc_config.enable_pin(peripherals.GPIO4, Attenuation::_11dB);
    let mut joystick_y_pin = adc_config.enable_pin(peripherals.GPIO3, Attenuation::_11dB);
    let mut trig_pin = adc_config.enable_pin(peripherals.GPIO1, Attenuation::_11dB);
    let mut vbat_pin = adc_config.enable_pin(peripherals.GPIO0, Attenuation::_11dB);
    let mut adc = esp_hal::analog::adc::Adc::new(peripherals.ADC1, adc_config);


    spawner.spawn(blinky(led, button)).ok();

    loop {
        let joystick_x_value = nb::block!(adc.read_oneshot(&mut joystick_x_pin)).unwrap();
        let joystick_y_value = nb::block!(adc.read_oneshot(&mut joystick_y_pin)).unwrap();
        let trig_value = nb::block!(adc.read_oneshot(&mut trig_pin)).unwrap();
        let vbat_value = nb::block!(adc.read_oneshot(&mut vbat_pin)).unwrap();
        println!("JX {joystick_x_value}    JY {joystick_y_value}    Tr {trig_value}    VB {vbat_value}");
        Timer::after(Duration::from_millis(100)).await;
    }
}


#[embassy_executor::task]
async fn blinky(mut led: Output<'static>, button: Input<'static>) {
    loop {
        if button.is_high() {  // button open
            led.set_high();  // LED off
        } else {
            led.set_low();
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}
