use defmt::Format;
use embassy_futures::join::join;
use embassy_futures::select::select;
use rand_core::{CryptoRng, RngCore};

use serde::{Deserialize, Serialize};
use static_cell::StaticCell;
use trouble_host::prelude::*;

use crate::bus::{GlobalBus, StorageKey};
use crate::ble_descriptors::{DEVICE_NAME, MouseReport, Server};
use crate::serialization_util::StorageSerde;


#[cfg(feature = "defmt")]
use defmt::{debug, info, warn, error};
#[cfg(feature = "log")]
use log::{debug, info, warn, error};


/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 4; // Signal + att


#[derive(Debug, Format, Clone, Serialize, Deserialize)]
struct StoredBondInformation {
    addr: [u8; 6],
    ltk: u128,
    irk: Option<u128>,
    security_level: u8,
}

impl StoredBondInformation {
    pub fn from_bond_info(bond_info: &BondInformation) -> Self {
        Self {
            addr: bond_info.identity.bd_addr.raw().try_into().unwrap(),
            ltk: bond_info.ltk.0,
            irk: bond_info.identity.irk.map(|i| i.0),
            security_level: match bond_info.security_level {
                SecurityLevel::Encrypted => 1,
                SecurityLevel::EncryptedAuthenticated => 2,
                SecurityLevel::NoEncryption => 0,
        }
        }
    }

    pub fn security_level(self) -> SecurityLevel {
        match self.security_level {
            1 => SecurityLevel::Encrypted,
            2 => SecurityLevel::EncryptedAuthenticated,
            _ => SecurityLevel::NoEncryption,
        }
    }

    pub fn bond_info(self) -> BondInformation {
        BondInformation { 
            ltk: LongTermKey(self.ltk),
            identity: Identity {
                bd_addr: BdAddr::new(self.addr),
                irk: self.irk.map(|i| IdentityResolvingKey(i)),
            },
            is_bonded: true, 
            security_level: self.security_level() 
        }
    }
}


static RESOURCES: StaticCell<HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX>> = StaticCell::new();

/// Build the BLE stack, temporarily acquiring resources needed for construction
pub fn build_stack<'a, C, TRNG>(controller: C, trng: &mut TRNG) -> Stack<'a, C, DefaultPacketPool> 
where 
    C: Controller + 'a,
    TRNG: RngCore + CryptoRng
{
    // Using a fixed "random" address can be useful for testing. In real scenarios, one would
    // use e.g. the MAC 6 byte array as the address (how to get that varies by the platform).
    let address: Address = Address::random([0xff, 0x8f, 0x08, 0x05, 0xe4, 0xff]);
    info!("Our address = {}", address);

    let resources = RESOURCES.init(HostResources::new());
 
    return trouble_host::new(controller, resources)
        .set_random_address(address)
        .set_random_generator_seed(trng)
}

/// Run the BLE stack.
pub async fn run<'a, C>(
    bus: &'static GlobalBus,
    stack: &'a Stack<'a, C, DefaultPacketPool>)
where
    C: Controller + 'a,
{
    let stored_bond_info = {
        let mut storage = bus.storage.lock().await;
        let mut buf = [0u8; 128];
        storage.fetch_item::<StorageSerde<StoredBondInformation>>(&mut buf, &(StorageKey::BondingInfo as u8)).await.unwrap()
    };
    if let Some(stored_bond_info) = stored_bond_info {
        info!("Loaded bond information: {}", stored_bond_info.0);
        stack.add_bond_information(stored_bond_info.0.bond_info()).unwrap();
    } else {
        info!("No bond information found");
    };

    let Host {
        mut peripheral, runner, ..
    } = stack.build();

    info!("Starting advertising and GATT service");
    let server: Server<'_> = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: DEVICE_NAME,
        appearance: &appearance::human_interface_device::GAMEPAD,
    }))
    .unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            match advertise(DEVICE_NAME, &mut peripheral, &server).await {
                Ok(conn) => {
                    // Allow bondable if no bond is stored.
                    conn.raw().set_bondable(true).unwrap();
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    let a = gatt_events_task(bus, &server, &conn);
                    let b = custom_task(bus, &server, &conn, &stack);
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
async fn gatt_events_task(
    bus: &'static GlobalBus,
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
) -> Result<(), Error> {
    let level = server.battery_service.level;
    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            #[cfg(feature = "security")]
            GattConnectionEvent::PairingComplete { security_level, bond } => {
                let cccd_table = server.get_cccd_table(conn.raw()).unwrap();
                info!("cccd {}", cccd_table);

                info!("[gatt] pairing complete: {:?}", security_level);
                if let Some(bond) = bond {
                    let _ = {
                        let mut storage = bus.storage.lock().await;
                        let mut buf = [0u8; 128];
                        storage.store_item(
                            &mut buf, 
                            &(StorageKey::BondingInfo as u8), 
                            &StorageSerde(StoredBondInformation::from_bond_info(&bond))).await
                    }.inspect_err(|_| error!("Failed to store bond info"));
                    info!("Bond information stored: {}", bond);
                }
            }
            #[cfg(feature = "security")]
            GattConnectionEvent::PairingFailed(err) => {
                error!("[gatt] pairing error: {:?}", err);
            }
            GattConnectionEvent::Gatt { event } => {
                let result = match &event {
                    GattEvent::Read(event) => {
                        info!("[gatt] Read Event to Characteristic {}", event.handle());

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
                        info!("[gatt] Write Event to Characteristic {}: {:?}", event.handle(), event.data());

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
    bus: &'static GlobalBus,
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    stack: &Stack<'_, C, P>,
) {
    let mut joystick_reader = bus.joystick_state.receiver().unwrap();
    
    let level = server.battery_service.level;  // TODO move out
    let hid_report = server.mouse_service.report;

    loop {
        let joystick = joystick_reader.changed().await;

        let report = MouseReport {
            buttons: if joystick.btn {1} else {0},
            x: (joystick.x / (i16::MAX/127)) as i8,
            y: 0,
            wheel: -(joystick.y / (i16::MAX/127)) as i8,
            pan: 0,
        };

        let _ = hid_report.notify(conn, &report.serialize()).await
            .inspect_err(|e| error!("failed to notify: {}", e));

        // read RSSI (Received Signal Strength Indicator) of the connection.
        if let Ok(rssi) = conn.raw().rssi(stack).await {
            debug!("[custom_task] RSSI: {:?}", rssi);
        } else {
            info!("[custom_task] error getting RSSI");
            break;
        };
    }
}
