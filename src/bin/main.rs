#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_wifi::ble::controller::BleConnector;

use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Pull, Level, Output, OutputConfig},
    analog::adc::{AdcConfig, Adc, Attenuation},
};
use esp_println::println;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.5.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    let rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init =
        esp_wifi::init(timer1.timer0, rng).expect("Failed to initialize WIFI/BLE controller");
    let (mut _wifi_controller, _interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(&wifi_init, peripherals.BT);
    let _ble_controller = ExternalController::<_, 20>::new(transport);

    // TODO: Spawn some tasks
    let _ = spawner;

    println!("Quack quack quack world");

    let mut led = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());
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
        println!("JX {joystick_x_value}    JY {joystick_y_value}    T {trig_value}    VB {vbat_value}");
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
