use defmt::Format;
use embassy_futures::join::join;
use embassy_futures::select::select3;
use embassy_time::{Duration, Instant, Timer};
use fixed::types::{I16F16, I1F15};
use heapless::LinearMap;

use sequential_storage::map::PostcardValue;
use serde::{Deserialize, Serialize};
use trouble_host::prelude::*;
use usbd_hid::descriptor::AsInputReport;

use crate::ble_descriptors::{MouseReport, Server, DEVICE_NAME};
use crate::bus::{GlobalBus, StorageKey};
use crate::prelude::*;
use crate::util::FormatLinearMap;

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 4; // Signal + att

const STORED_CCCD_MAX: usize = 8;

#[derive(Debug, PartialEq, Clone, Serialize, Deserialize, Format)]
struct StoredBondInformation {
    bond_info: BondInformation,
    cccds: FormatLinearMap<u16, u16, STORED_CCCD_MAX>,
}
impl<'a> PostcardValue<'a> for StoredBondInformation {}

impl StoredBondInformation {
    pub fn from_bond_info(bond_info: BondInformation) -> Self {
        Self {
            bond_info: bond_info,
            cccds: FormatLinearMap(LinearMap::new()),
        }
    }

    pub fn added_cccd(&self, handle: u16, value: u16) -> Self {
        let mut new = self.clone();
        new.cccds.0.insert(handle, value).ok();
        new
    }

    pub fn bond_info(&self) -> &BondInformation {
        &self.bond_info
    }

    pub fn cccds(&self) -> &LinearMap<u16, u16, STORED_CCCD_MAX> {
        &self.cccds.0
    }
}

/// Run the BLE stack.
pub async fn run<'a, C>(bus: &'static GlobalBus, controller: C)
where
    C: Controller + 'a,
{
    // Using a fixed "random" address can be useful for testing. In real scenarios, one would
    // use e.g. the MAC 6 byte array as the address (how to get that varies by the platform).
    let address: Address = Address::random([0xff, 0x8f, 0x08, 0x05, 0xe4, 0xff]);
    info!("Our address = {}", address);

    let mut resources: HostResources<_, DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .build();

    let stored_bond_info = {
        let mut storage = bus.storage.lock().await;
        let mut buf = [0u8; 128];
        storage
            .fetch_item::<StoredBondInformation>(&mut buf, &StorageKey::BondingInfo)
            .await
            .inspect_err(|e| error!("error reading bonding info {}", defmt::Debug2Format(e)))
            .ok()
            .flatten()
    };
    if let Some(stored_bond_info) = stored_bond_info {
        info!("Loaded bond information: {}", stored_bond_info);
        stack
            .add_bond_information(stored_bond_info.bond_info().clone())
            .unwrap();
    } else {
        info!("No bond information found");
    };

    let runner = stack.runner();
    let mut peripheral = stack.peripheral();

    info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: DEVICE_NAME,
        appearance: &appearance::human_interface_device::GENERIC_HUMAN_INTERFACE_DEVICE,
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
                    let b = joystick_task(bus, &server, &conn, &stack);
                    let c = battery_task(bus, &server, &conn);
                    // run until any task ends (usually because the connection has been closed),
                    // then return to advertising state.
                    select3(a, b, c).await;
                    info!("Connection dropped");
                }
                Err(e) => {
                    error!("[adv] error: {:?}", defmt::Debug2Format(&e));
                    Timer::after_millis(100).await; // wait a bit before retrying
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
            panic!("[ble_task] error: {:?}", defmt::Debug2Format(&e));
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
    let mut stored_bond_info = {
        let mut storage = bus.storage.lock().await;
        let mut buf = [0u8; 128];
        storage
            .fetch_item::<StoredBondInformation>(&mut buf, &StorageKey::BondingInfo)
            .await
            .inspect_err(|e| error!("error reading bonding info {}", defmt::Debug2Format(e)))
            .ok()
            .flatten()
            .filter(|value| {
                value
                    .bond_info()
                    .identity
                    .match_address(&conn.raw().peer_address())
            })
    };

    // TODO delay restoring cccd until encryption confirmed
    stored_bond_info.as_ref().map(|stored_bond_info| {
        let mut att_table = server.get_client_att_table(conn.raw()).unwrap();
        stored_bond_info.cccds().iter().for_each(|(handle, value)| {
            att_table.write(*handle, 0, &value.to_le_bytes()).unwrap();
        });
        server.set_client_att_table(conn.raw(), &att_table.view());
        info!("[gatt] restored stored bond info: {:?}", stored_bond_info);
    });

    let reason = loop {
        let conn_event = conn.next().await;
        let mut new_stored_bond_info: Option<StoredBondInformation> = None;

        match conn_event {
            GattConnectionEvent::Disconnected { reason } => {
                info!("[gatt] disconnected: {}", reason);
                break reason;
            }
            GattConnectionEvent::PairingComplete {
                security_level,
                bond,
            } => {
                info!("[gatt] pairing complete: {:?}", security_level);
                if let Some(bond) = bond {
                    new_stored_bond_info = Some(StoredBondInformation::from_bond_info(bond));
                }
            }
            GattConnectionEvent::PairingFailed(err) => {
                error!("[gatt] pairing error: {:?}", err);
            }
            GattConnectionEvent::Gatt { event } => {
                let mut cccd_changed_handle: Option<u16> = None;

                let result = match &event {
                    GattEvent::Read(event) => {
                        info!("[gatt] Read Event to Characteristic {}", event.handle());

                        if conn.raw().security_level()?.encrypted() {
                            None
                        } else {
                            Some(AttErrorCode::INSUFFICIENT_ENCRYPTION)
                        }
                    }
                    GattEvent::Write(event) => {
                        info!("[gatt] Write Event to Characteristic {}", event.handle());

                        const CLIENT_CHARACTERISTIC_CONFIGURATION: Uuid = Uuid::Uuid16(
                            bt_hci::uuid::descriptors::CLIENT_CHARACTERISTIC_CONFIGURATION
                                .to_le_bytes(),
                        );
                        if Some(CLIENT_CHARACTERISTIC_CONFIGURATION)
                            == server.table().uuid(event.handle())
                        {
                            cccd_changed_handle = Some(event.handle());
                        }

                        if conn.raw().security_level()?.encrypted() {
                            None
                        } else {
                            Some(AttErrorCode::INSUFFICIENT_ENCRYPTION)
                        }
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

                // This needs to be processed after event.accept, otherwise the new values aren't ready
                if let Some(cccd_handle) = cccd_changed_handle {
                    if let Some(stored_bond_info) = stored_bond_info.as_ref() {
                        let att_table = server.get_client_att_table(conn.raw()).unwrap();
                        let value = att_table.get(cccd_handle);
                        if let Some([b0, b1]) = value {
                            let value_u16 = u16::from_le_bytes([*b0, *b1]);
                            info!("cccd changed => {}={}", cccd_handle, value_u16);
                            new_stored_bond_info =
                                Some(stored_bond_info.added_cccd(cccd_handle, value_u16));
                        }
                    }
                }
            }
            _ => {} // ignore other Gatt Connection Events
        }

        if let Some(updated_bond_info) = new_stored_bond_info.as_ref() {
            // check for a potential bond change
            if Some(updated_bond_info) == stored_bond_info.as_ref() {
                continue; // data not changed
            }
            let result = {
                let mut storage = bus.storage.lock().await;
                let mut buf = [0u8; 128];
                storage
                    .store_item(&mut buf, &StorageKey::BondingInfo, updated_bond_info)
                    .await
            };
            if result.is_ok() {
                info!("[gatt] wrote updated bond info: {}", updated_bond_info);
                stored_bond_info = new_stored_bond_info;
            } else {
                error!("[gatt] failed to write updated bond info")
            }
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
            AdStructure::IncompleteServiceUuids16(&[
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

// Given some absolute input (eg, joystick), produce a time-relative output (eg, mouse movement) by accumulating the input
// and producing output ticks when a threshold is passed.
pub struct RelativeAccummulator {
    accum: I16F16,
    ticks_per_second: I16F16, // if the input is constant 1, this many ticks are produced per second
}

impl RelativeAccummulator {
    pub fn new(ticks_per_second: I16F16) -> Self {
        Self {
            accum: I16F16::ZERO,
            ticks_per_second: ticks_per_second,
        }
    }

    pub fn update(&mut self, input: I1F15, dt: Duration) -> i8 {
        self.accum += I16F16::from_num(input).saturating_mul(
            self.ticks_per_second
                .saturating_mul(I16F16::saturating_from_num(dt.as_millis()) / 1000),
        );
        let output = self.accum.to_num::<i8>();
        self.accum -= I16F16::from_num(output);
        output
    }
}

fn sensitivity_mapping(input: I1F15) -> I1F15 {
    // a sensitivity mapping that is finer near the center but allows full-range at max
    (input / 2).saturating_add(input.saturating_mul(input.saturating_abs() / 2))
}

async fn joystick_task<C: Controller, P: PacketPool>(
    bus: &'static GlobalBus,
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    stack: &Stack<'_, C, P>,
) {
    let mut joystick_reader = bus.joystick_state.receiver().unwrap();

    let hid_report = server.mouse_service.report;

    let mut last_tick = Instant::now();
    let mut trig_accum = RelativeAccummulator::new(I16F16::from_num(127));
    let mut x_accum = RelativeAccummulator::new(I16F16::from_num(63));
    let mut y_accum = RelativeAccummulator::new(I16F16::from_num(63));

    let mut last_report = MouseReport::default();

    loop {
        // handle joystick
        let joystick = joystick_reader.changed().await;
        let now = Instant::now();
        let dt = now - last_tick;
        last_tick = now;

        let trig_delta = trig_accum.update(sensitivity_mapping(joystick.trig), dt);
        let x_delta = x_accum.update(sensitivity_mapping(joystick.x), dt);
        let y_delta = y_accum.update(sensitivity_mapping(joystick.y), dt);

        // handle buttons
        let mut report_buttons: u8 = 0;

        if joystick.btn {
            report_buttons |= 1 << 0; // left click
        }

        let btns = bus.buttons_state.try_get().unwrap_or(0);
        if (btns & (1 << 1)) != 0 {
            report_buttons |= 1 << 3; // back button
        }
        if (btns & (1 << 7)) != 0 {
            report_buttons |= 1 << 4; // forward button
        }

        // assembe and send report
        let report = MouseReport {
            buttons: report_buttons,
            x: 0,
            y: 0,
            wheel: -y_delta - trig_delta,
            pan: x_delta,
        };

        if report == last_report && !report.relative_changed() {
            continue; // don't send reports if nothing changed
        }
        last_report = report;

        let mut buf = [0u8; MouseReport::SIZE];
        report.serialize(&mut buf).unwrap();
        let _ = hid_report
            .notify(conn, &buf, false)
            .await
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

async fn battery_task<P: PacketPool>(
    bus: &'static GlobalBus,
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) {
    let level = server.battery_service.level;
    let mut vbat_soc_reader = bus.vbat_soc.receiver().unwrap();

    loop {
        let vbat_soc = vbat_soc_reader.changed().await;
        let _ = level
            .notify(conn, &vbat_soc, true)
            .await
            .inspect_err(|e| error!("failed to notify: {}", e));
    }
}
