use trouble_host::prelude::*;
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::SerializedDescriptor;

pub const DEVICE_NAME: &str = "Ducky RingJoystick";

#[gatt_service(uuid = service::DEVICE_INFORMATION)]
pub(crate) struct DeviceInformationService {
    #[characteristic(uuid = characteristic::MANUFACTURER_NAME_STRING, read, value = *b"Ducky")]
    pub manufacturer: [u8; 5],
    #[characteristic(uuid = characteristic::MODEL_NUMBER_STRING, read, value = *b"RingJoystick")]
    pub model: [u8; 12],
    #[characteristic(uuid = characteristic::PNP_ID, read, value = [0x01, 0x0d, 0x00, 0x47, 0x00, 0x01, 0x00])]
    pub pnp_id: [u8; 7],
}

#[gatt_service(uuid = service::BATTERY)]
pub(crate) struct BatteryService {
    /// Battery Level
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, read, value = "Battery Level")]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 10)]
    pub(crate) level: u8,
}

// The characteristic buffer length for the descriptor must exactly match the descriptor length,
// or it will crash when the BLE stack starts.
// As of usbd-hid 0.9.0, the descriptor is not available at compiler time.
// Workaround: get the length at runtime, then update the code
// - in main, before the BLE stack starts (and crashes), print the length, for example
//     info!("report length = {}",
//         <ble_descriptors::MouseReport as usbd_hid::descriptor::SerializedDescriptor>::desc().len());
// - paste the result into the characteristic buffer length

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = MOUSE) = {
        (collection = PHYSICAL, usage = POINTER) = {
            (usage_page = BUTTON, usage_min = BUTTON_1, usage_max = BUTTON_8) = {
                #[packed_bits = 8] #[item_settings(data,variable,absolute)] buttons=input;
            };
            (usage_page = GENERIC_DESKTOP,) = {
                (usage = X,) = {
                    #[item_settings(data,variable,relative)] x=input;
                };
                (usage = Y,) = {
                    #[item_settings(data,variable,relative)] y=input;
                };
                (usage = WHEEL,) = {
                    #[item_settings(data,variable,relative)] wheel=input;
                };
            };
            (usage_page = CONSUMER,) = {
                (usage = AC_PAN,) = {
                    #[item_settings(data,variable,relative)] pan=input;
                };
            };
        };
    }
)]
#[allow(dead_code)]
#[derive(defmt::Format)]
pub struct MouseReport {
    pub buttons: u8,
    pub x: i8,
    pub y: i8,
    pub wheel: i8, // Scroll down (negative) or up (positive) this many units
    pub pan: i8,   // Scroll left (negative) or right (positive) this many units
}

impl MouseReport {
    pub const SIZE: usize = core::mem::size_of::<MouseReport>();
    pub const DESC_SIZE: usize = MouseReport::DESC.len(); //69; // IMPORTANT: length MUST EXACTLY equal the descriptor size, see note at top of file
}

#[gatt_service(uuid = service::HUMAN_INTERFACE_DEVICE)]
pub(crate) struct MouseService {
    #[characteristic(uuid = "2a4a", read, value = [0x01, 0x01, 0x00, 0x03])]
    pub(crate) hid_info: [u8; 4],
    #[characteristic(uuid = "2a4b", read, value = MouseReport::DESC)]
    pub(crate) report_map: [u8; MouseReport::DESC_SIZE],
    #[characteristic(uuid = "2a4c", write_without_response)]
    pub(crate) hid_control_point: u8,
    #[characteristic(uuid = "2a4e", read, write_without_response, value = 1)]
    pub(crate) protocol_mode: u8,
    #[descriptor(uuid = "2908", read, value = [0u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) report: [u8; MouseReport::SIZE],
}

// GATT Server definition
#[gatt_server]
#[allow(dead_code)]
pub(crate) struct Server {
    pub(crate) device_information: DeviceInformationService,
    pub(crate) battery_service: BatteryService,
    pub(crate) mouse_service: MouseService,
}
