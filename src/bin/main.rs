#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]


// mod ble_peripheral;


// TrouBLE example imports
use embassy_executor::Spawner;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::ExternalController;
use esp_backtrace as _;

// BAS bonding imports
use esp_hal::rng::{Trng, TrngSource};
use esp_storage::FlashStorage;

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

    let _trng_source = TrngSource::new(peripherals.RNG, peripherals.ADC1.reborrow());
    let mut rng = Trng::try_new().unwrap();
    // let mut rng = Rng::new();

    let radio = esp_radio::init().unwrap();
    let bluetooth = peripherals.BT;
    let connector = BleConnector::new(&radio, bluetooth, Default::default()).unwrap();
    let controller: ExternalController<_, 20> = ExternalController::new(connector);

    // Initialize the flash
    let mut flash = embassy_embedded_hal::adapter::BlockingAsync::new(FlashStorage::new(peripherals.FLASH));

    run(controller, &mut rng, &mut flash).await;

    rng.downgrade();
    core::mem::drop(_trng_source);  // drop the Trng to allow ADC1 t obe used

    // App-specific code starts here
    println!("Quack quack quack world");

    let led = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());
    let button = Input::new(peripherals.GPIO6, InputConfig::default().with_pull(Pull::Up));

    let mut adc_config = AdcConfig::new();
    let mut joystick_x_pin = adc_config.enable_pin(peripherals.GPIO4, Attenuation::_11dB);
    let mut joystick_y_pin = adc_config.enable_pin(peripherals.GPIO3, Attenuation::_11dB);
    let mut trig_pin = adc_config.enable_pin(peripherals.GPIO1, Attenuation::_11dB);
    let mut vbat_pin = adc_config.enable_pin(peripherals.GPIO0, Attenuation::_11dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);


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









// ble_bas_peripheral_bonding code

use core::ops::Range;
use embassy_futures::join::join;
use embassy_futures::select::select;
// use embassy_time::Timer;
use embedded_storage_async::nor_flash::NorFlash;
use rand_core::{CryptoRng, RngCore};
use sequential_storage::cache::NoCache;
use sequential_storage::map::{Key, SerializationError, Value};
use trouble_host::prelude::*;

use log::{info, warn, error};

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 4; // Signal + att

// GATT Server definition
#[gatt_server]
struct Server {
    battery_service: BatteryService,
    hid_service: HidService,
}

static DESC: [u8; 67] = [
    5u8, 1u8, 9u8, 6u8, 161u8, 1u8, 5u8, 7u8, 25u8, 224u8, 41u8, 231u8, 21u8, 0u8, 37u8, 1u8, 117u8, 1u8, 149u8, 8u8,
    129u8, 2u8, 21u8, 0u8, 38u8, 255u8, 0u8, 117u8, 8u8, 149u8, 1u8, 129u8, 3u8, 5u8, 8u8, 25u8, 1u8, 41u8, 5u8, 37u8,
    1u8, 117u8, 1u8, 149u8, 5u8, 145u8, 2u8, 149u8, 3u8, 145u8, 3u8, 5u8, 7u8, 25u8, 0u8, 41u8, 221u8, 38u8, 255u8,
    0u8, 117u8, 8u8, 149u8, 6u8, 129u8, 0u8, 192u8,
];

#[gatt_service(uuid = service::HUMAN_INTERFACE_DEVICE)]
pub(crate) struct HidService {
    #[characteristic(uuid = "2a4a", read, value = [0x01, 0x01, 0x00, 0x03])]
    pub(crate) hid_info: [u8; 4],
    #[characteristic(uuid = "2a4b", read, value = DESC)]
    pub(crate) report_map: [u8; 67],
    #[characteristic(uuid = "2a4c", write_without_response)]
    pub(crate) hid_control_point: u8,
    #[characteristic(uuid = "2a4e", read, write_without_response, value = 1)]
    pub(crate) protocol_mode: u8,
    #[descriptor(uuid = "2908", read, value = [0u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) input_keyboard: [u8; 8],
    #[descriptor(uuid = "2908", read, value = [0u8, 2u8])]
    #[characteristic(uuid = "2a4d", read, write, write_without_response)]
    pub(crate) output_keyboard: [u8; 1],
}

/// Battery service
#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    /// Battery Level
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, name = "hello", read, value = "Battery Level")]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 10)]
    level: u8,
    #[characteristic(uuid = "408813df-5dd4-1f87-ec11-cdb001100000", write, read, notify)]
    status: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct StoredAddr(BdAddr);

impl Key for StoredAddr {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 6 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0..6].copy_from_slice(self.0.raw());
        Ok(6)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        if buffer.len() < 6 {
            Err(SerializationError::BufferTooSmall)
        } else {
            Ok((StoredAddr(BdAddr::new(buffer[0..6].try_into().unwrap())), 6))
        }
    }
}

struct StoredBondInformation {
    ltk: LongTermKey,
    security_level: SecurityLevel,
}

impl<'a> Value<'a> for StoredBondInformation {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 17 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0..16].copy_from_slice(self.ltk.to_le_bytes().as_slice());
        buffer[16] = match self.security_level {
            SecurityLevel::NoEncryption => 0,
            SecurityLevel::Encrypted => 1,
            SecurityLevel::EncryptedAuthenticated => 2,
        };
        Ok(17)
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < 17 {
            Err(SerializationError::BufferTooSmall)
        } else {
            let ltk = LongTermKey::from_le_bytes(buffer[0..16].try_into().unwrap());
            let security_level = match buffer[16] {
                0 => SecurityLevel::NoEncryption,
                1 => SecurityLevel::Encrypted,
                2 => SecurityLevel::EncryptedAuthenticated,
                _ => return Err(SerializationError::InvalidData),
            };
            Ok(StoredBondInformation { ltk, security_level })
        }
    }
}

fn flash_range<S: NorFlash>() -> Range<u32> {
    0..2 * S::ERASE_SIZE as u32
}

async fn store_bonding_info<S: NorFlash>(
    storage: &mut S,
    info: &BondInformation,
) -> Result<(), sequential_storage::Error<S::Error>> {
    // Use flash range from 640KB, should be good for both ESP32 & nRF52840 examples
    let start_addr = 0xA0000 as u32;
    let storage_range = start_addr..(start_addr + 8 * S::ERASE_SIZE as u32);
    sequential_storage::erase_all(storage, storage_range.clone()).await?;
    let mut buffer = [0; 32];
    let key = StoredAddr(info.identity.bd_addr);
    let value = StoredBondInformation {
        ltk: info.ltk,
        security_level: info.security_level,
    };
    sequential_storage::map::store_item(storage, storage_range, &mut NoCache::new(), &mut buffer, &key, &value).await?;
    Ok(())
}

async fn load_bonding_info<S: NorFlash>(storage: &mut S) -> Option<BondInformation> {
    let mut buffer = [0; 32];
    let mut cache = NoCache::new();
    let mut iter = sequential_storage::map::fetch_all_items::<StoredAddr, _, _>(
        storage,
        flash_range::<S>(),
        &mut cache,
        &mut buffer,
    )
    .await
    .ok()?;
    while let Some((key, value)) = iter.next::<StoredBondInformation>(&mut buffer).await.ok()? {
        return Some(BondInformation {
            identity: Identity {
                bd_addr: key.0,
                irk: None,
            },
            security_level: value.security_level,
            is_bonded: true,
            ltk: value.ltk,
        });
    }
    None
}

/// Run the BLE stack.
pub async fn run<C, RNG, S>(controller: C, random_generator: &mut RNG, storage: &mut S)
where
    C: Controller,
    RNG: RngCore + CryptoRng,
    S: NorFlash,
{
    // Using a fixed "random" address can be useful for testing. In real scenarios, one would
    // use e.g. the MAC 6 byte array as the address (how to get that varies by the platform).
    let address: Address = Address::random([0xff, 0x8f, 0x08, 0x05, 0xe4, 0xff]);
    info!("Our address = {}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .set_random_generator_seed(random_generator);

    let mut bond_stored = if let Some(bond_info) = load_bonding_info(storage).await {
        info!("Loaded bond information");
        stack.add_bond_information(bond_info).unwrap();
        true
    } else {
        info!("No bond information found");
        false
    };

    let Host {
        mut peripheral, runner, ..
    } = stack.build();

    info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "TrouBLE",
        appearance: &appearance::human_interface_device::GENERIC_HUMAN_INTERFACE_DEVICE,
    }))
    .unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            match advertise("Trouble Example", &mut peripheral, &server).await {
                Ok(conn) => {
                    // Allow bondable if no bond is stored.
                    conn.raw().set_bondable(!bond_stored).unwrap();
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    let a = gatt_events_task(storage, &server, &conn, &mut bond_stored);
                    let b = custom_task(&server, &conn, &stack);
                    // run until any task ends (usually because the connection has been closed),
                    // then return to advertising state.
                    select(a, b).await;
                    info!("Connection dropped");
                }
                Err(e) => {
                    #[cfg(feature = "defmt")]
                    let e = defmt::Debug2Format(&e);
                    panic!("[adv] error: {:?}", e);
                }
            }
        }
    })
    .await;
}

/// This is a background task that is required to run forever alongside any other BLE tasks.
///
/// ## Alternative
///
/// If you didn't require this to be generic for your application, you could statically spawn this with i.e.
///
/// ```rust,ignore
///
/// #[embassy_executor::task]
/// async fn ble_task(mut runner: Runner<'static, SoftdeviceController<'static>>) {
///     runner.run().await;
/// }
///
/// spawner.must_spawn(ble_task(runner));
/// ```
async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            #[cfg(feature = "defmt")]
            let e = defmt::Debug2Format(&e);
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

/// Stream Events until the connection closes.
///
/// This function will handle the GATT events and process them.
/// This is how we interact with read and write requests.
async fn gatt_events_task<S: NorFlash>(
    storage: &mut S,
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    bond_stored: &mut bool,
) -> Result<(), Error> {
    let level = server.battery_service.level;
    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            #[cfg(feature = "security")]
            GattConnectionEvent::PairingComplete { security_level, bond } => {
                info!("[gatt] pairing complete: {:?}", security_level);
                if let Some(bond) = bond {
                    store_bonding_info(storage, &bond).await.unwrap();
                    *bond_stored = true;
                    info!("Bond information stored");
                }
            }
            #[cfg(feature = "security")]
            GattConnectionEvent::PairingFailed(err) => {
                error!("[gatt] pairing error: {:?}", err);
            }
            GattConnectionEvent::Gatt { event } => {
                let result = match &event {
                    GattEvent::Read(event) => {
                        if event.handle() == level.handle {
                            let value = server.get(&level);
                            info!("[gatt] Read Event to Level Characteristic: {:?}", value);
                        }
                        #[cfg(feature = "security")]
                        if conn.raw().security_level()?.encrypted() {
                            None
                        } else {
                            Some(AttErrorCode::INSUFFICIENT_ENCRYPTION)
                        }
                        #[cfg(not(feature = "security"))]
                        None
                    }
                    GattEvent::Write(event) => {
                        if event.handle() == level.handle {
                            info!("[gatt] Write Event to Level Characteristic: {:?}", event.data());
                        }
                        #[cfg(feature = "security")]
                        if conn.raw().security_level()?.encrypted() {
                            None
                        } else {
                            Some(AttErrorCode::INSUFFICIENT_ENCRYPTION)
                        }
                        #[cfg(not(feature = "security"))]
                        None
                    }
                    _ => None,
                };

                let reply_result = if let Some(code) = result {
                    event.reject(code)
                } else {
                    event.accept()
                };
                match reply_result {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[gatt] error sending response: {:?}", e),
                }
            }
            _ => {} // ignore other Gatt Connection Events
        }
    };
    info!("[gatt] disconnected: {:?}", reason);
    Ok(())
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[
                service::BATTERY.to_le_bytes(),
                service::HUMAN_INTERFACE_DEVICE.to_le_bytes(),
            ]),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    info!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[adv] connection established");
    Ok(conn)
}

/// Example task to use the BLE notifier interface.
/// This task will notify the connected central of a counter value every 2 seconds.
/// It will also read the RSSI value every 2 seconds.
/// and will stop when the connection is closed by the central or an error occurs.
async fn custom_task<C: Controller, P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    stack: &Stack<'_, C, P>,
) {
    let mut tick: u8 = 0;
    let level = server.battery_service.level;
    loop {
        tick = tick.wrapping_add(1);
        info!("[custom_task] notifying connection of tick {}", tick);
        if level.notify(conn, &tick).await.is_err() {
            info!("[custom_task] error notifying connection");
            break;
        };
        // read RSSI (Received Signal Strength Indicator) of the connection.
        if let Ok(rssi) = conn.raw().rssi(stack).await {
            info!("[custom_task] RSSI: {:?}", rssi);
        } else {
            info!("[custom_task] error getting RSSI");
            break;
        };
        Timer::after_secs(2).await;
    }
}
