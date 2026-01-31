use core::cell::RefCell;

use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_sync::{blocking_mutex::{Mutex, raw::CriticalSectionRawMutex}, watch::Watch};
use esp_storage::FlashStorage;
use sequential_storage::{cache::NoCache, map::MapStorage};
use static_cell::StaticCell;


#[derive(Clone, Copy, Default)]
pub struct JoystickState {
    pub x: i16,  // up-scaled to i16, deadzone pre-applied
    pub y: i16,
    pub btn: bool,
}


pub struct GlobalBus {
    pub storage: Mutex<CriticalSectionRawMutex, RefCell<MapStorage<u8, BlockingAsync<FlashStorage<'static>>, NoCache>>>,

    pub joystick_state: Watch<CriticalSectionRawMutex, JoystickState, 2>,
    pub vbat: Watch<CriticalSectionRawMutex, u16, 2>,  // in mV
}


static BUS: StaticCell<GlobalBus> = StaticCell::new();


pub fn init(
  storage: MapStorage<u8, BlockingAsync<FlashStorage<'static>>, NoCache>
) -> &'static GlobalBus {
  BUS.init(GlobalBus {
    storage: Mutex::new(storage.into()),

    joystick_state: Watch::new(),
    vbat: Watch::new(),
  })
}
