use trouble_host::prelude::*;
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::SerializedDescriptor;
use serde::Serialize;


// The characteristic buffer length for the descriptor must exactly match the descriptor length,
// or it will crash before main starts.
// As of usbd-hid 0.9.0, the descriptor is not available at compiler time.
// Workaround: get the length at runtime, then update the code
// - comment out references to the server
// - in main, add
//     use usbd_hid::descriptor::SerializedDescriptor;
//   so the descriptor is visiible
// - in main, print the length, for example
//     info!("report length = {}", CompositeReport::desc().len());
// - paste the result into the characteristic buffer length


/// A composite hid report which contains mouse, consumer, system reports.
/// Report id is used to distinguish from them.
#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = MOUSE) = {
        (collection = PHYSICAL, usage = POINTER) = {
            (report_id = 0x01,) = {
                (usage_page = BUTTON, usage_min = BUTTON_1, usage_max = BUTTON_8) = {
                    #[packed_bits 8] #[item_settings data,variable,absolute] buttons=input;
                };
                (usage_page = GENERIC_DESKTOP,) = {
                    (usage = X,) = {
                        #[item_settings data,variable,relative] x=input;
                    };
                    (usage = Y,) = {
                        #[item_settings data,variable,relative] y=input;
                    };
                    (usage = WHEEL,) = {
                        #[item_settings data,variable,relative] wheel=input;
                    };
                };
                (usage_page = CONSUMER,) = {
                    (usage = AC_PAN,) = {
                        #[item_settings data,variable,relative] pan=input;
                    };
                };
            };
        };
    },
)]
#[derive(Default, Serialize)]
pub struct CompositeReport {
    pub(crate) buttons: u8, // MouseButtons
    pub(crate) x: i8,
    pub(crate) y: i8,
    pub(crate) wheel: i8, // Scroll down (negative) or up (positive) this many units
    pub(crate) pan: i8,   // Scroll left (negative) or right (positive) this many units
}

fn _check_len() {
    let _: [u8; 1] = *CompositeReport::desc(); 
}

#[gatt_service(uuid = service::HUMAN_INTERFACE_DEVICE)]
pub(crate) struct CompositeService {
    #[characteristic(uuid = "2a4a", read, value = [0x01, 0x01, 0x00, 0x03])]
    pub(crate) hid_info: [u8; 4],
    #[characteristic(uuid = "2a4b", read, value = CompositeReport::desc().try_into().expect("Failed to serialize CompositeReport"))]
    pub(crate) report_map: [u8; 62],  // IMPORTANT: length MUST EXACTLY equal the descriptor size, see note at top of file
    #[characteristic(uuid = "2a4c", write_without_response)]
    pub(crate) hid_control_point: u8,
    #[characteristic(uuid = "2a4e", read, write_without_response, value = 1)]
    pub(crate) protocol_mode: u8,
    #[descriptor(uuid = "2908", read, value = [0u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) report: [u8; 5],
}


/// Battery service
#[gatt_service(uuid = service::BATTERY)]
pub(crate) struct BatteryService {
    /// Battery Level
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, read, value = "Battery Level")]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 10)]
    pub(crate) level: u8,
}


// GATT Server definition
#[gatt_server]
pub(crate) struct Server {
    pub(crate) battery_service: BatteryService,
    pub(crate) hid_service: CompositeService,
}
