#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::cell::RefCell;

#[cfg(feature = "defmt")]
use defmt::{info, warn, error};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::Mutex;
#[cfg(feature = "log")]
use log::{info, warn, error};
use static_cell::StaticCell;

use {esp_backtrace as _, esp_println as _};


mod ble_peripheral;


// TrouBLE example imports
use embassy_executor::Spawner;
use esp_hal::{Blocking, analog::adc::{AdcChannel, AdcPin}, clock::CpuClock, gpio::{AnalogPin, AnyPin}, peripherals::{ADC1, GPIO0, GPIO1, GPIO3, GPIO4}};
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::ExternalController;
use trouble_host::prelude::*;
use esp_backtrace as _;

// BAS bonding imports
use esp_hal::rng::{Trng, TrngSource};
use esp_storage::FlashStorage;

// App-specific imports
use embedded_hal_nb::nb;
use embassy_time::{Duration, Timer};
use esp_hal::{
    gpio::{Input, InputConfig, Pull, Level, Output, OutputConfig, Pin},
    analog::adc::{AdcConfig, Adc, Attenuation},
};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();


static ADC_MUTEX: StaticCell<Mutex<NoopRawMutex, RefCell<Adc<'static, ADC1<'static>, Blocking>>>> = StaticCell::new();


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


    let radio = esp_radio::init().unwrap();
    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(&radio, bluetooth, Default::default()).unwrap();
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    // Initialize the flash
    let mut flash = embassy_embedded_hal::adapter::BlockingAsync::new(FlashStorage::new(peripherals.FLASH));

    // ==== BLE INIT CODE BELOW TODO REFACTOR OUt ====

    /// Max number of connections
    const CONNECTIONS_MAX: usize = 1;

    /// Max number of L2CAP channels.
    const L2CAP_CHANNELS_MAX: usize = 4; // Signal + att


    // Using a fixed "random" address can be useful for testing. In real scenarios, one would
    // use e.g. the MAC 6 byte array as the address (how to get that varies by the platform).
    let address: Address = Address::random([0xff, 0x8f, 0x08, 0x05, 0xe4, 0xff]);
    info!("Our address = {}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> = HostResources::new();
 
    let stack = {
        let _trng_source = TrngSource::new(peripherals.RNG, peripherals.ADC1.reborrow());
        let mut trng = Trng::try_new().unwrap();
        
        trouble_host::new(controller, &mut resources)
            .set_random_address(address)
            .set_random_generator_seed(&mut trng)
    };

    // App-specific code starts here
    info!("Quack quack quack world");

    let led = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());
    let button = Input::new(peripherals.GPIO6, InputConfig::default().with_pull(Pull::Up));

    let mut adc_config = AdcConfig::new();
    let x_pin = adc_config.enable_pin(peripherals.GPIO4, Attenuation::_11dB);
    let y_pin = adc_config.enable_pin(peripherals.GPIO3, Attenuation::_11dB);
    let trig_pin = adc_config.enable_pin(peripherals.GPIO1, Attenuation::_11dB);
    let vbat_pin = adc_config.enable_pin(peripherals.GPIO0, Attenuation::_11dB);

    let mut adc = Adc::new(peripherals.ADC1, adc_config);
    let adc_mutex = ADC_MUTEX.init(Mutex::new(adc.into()));

    // ble_peripheral::run(controller, peripherals.RNG, adc_mutex, &mut flash);
    // TODO run BLE peripheral here
    spawner.spawn(blinky(led, button)).ok();
    spawner.spawn(read_ui(adc_mutex, x_pin, y_pin, trig_pin)).ok();
    spawner.spawn(read_bat(adc_mutex, vbat_pin)).ok();

    loop {
        Timer::after(Duration::from_millis(1000)).await;
    }
}


// the ADC API seems to be a bit of a dumpster fire
// https://github.com/esp-rs/esp-hal/issues/449
// such that it was removed int he embedded hal 1.0 API
// https://github.com/rust-embedded/embedded-hal/pull/376
// so for now, the GPIO # is hardcoded instead of generic, yuck!
#[embassy_executor::task]
async fn read_ui(adc_mutex: &'static Mutex<NoopRawMutex, RefCell<Adc<'static, ADC1<'static>, Blocking>>>, 
        mut x_pin: AdcPin<GPIO4<'static>, ADC1<'static>>, mut y_pin: AdcPin<GPIO3<'static>, ADC1<'static>>, 
        mut trig_pin: AdcPin<GPIO1<'static>, ADC1<'static>>) {
    loop {
        let x_value = adc_mutex.lock(|adc| 
            nb::block!(adc.borrow_mut().read_oneshot(&mut x_pin)).unwrap());
        let y_value = adc_mutex.lock(|adc| 
            nb::block!(adc.borrow_mut().read_oneshot(&mut y_pin)).unwrap());
        let trig_value = adc_mutex.lock(|adc| 
            nb::block!(adc.borrow_mut().read_oneshot(&mut trig_pin)).unwrap());
        info!("JX {}    JY {}    Tr {}",
            x_value,
            y_value,
            trig_value);
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn read_bat(adc_mutex: &'static Mutex<NoopRawMutex, RefCell<Adc<'static, ADC1<'static>, Blocking>>>, 
        mut vbat_pin: AdcPin<GPIO0<'static>, ADC1<'static>>) {
    loop {
        let vbat_value = adc_mutex.lock(|adc| 
            nb::block!(adc.borrow_mut().read_oneshot(&mut vbat_pin)).unwrap());
        info!("VB {}",
            vbat_value);
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
