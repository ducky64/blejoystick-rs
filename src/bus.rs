use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use static_cell::StaticCell;

#[derive(Clone, Copy, Default)]
pub struct JoystickState {
    pub x: i16,  // up-scaled to i16
    pub y: i16,
    pub btn: bool,
}


pub struct GlobalBus {
    pub joystick_state: Watch<CriticalSectionRawMutex, JoystickState, 2>,
    pub vbat: Watch<CriticalSectionRawMutex, u16, 2>,  // in mV
}


static BUS: StaticCell<GlobalBus> = StaticCell::new();

pub fn init() -> &'static GlobalBus {
  BUS.init(GlobalBus {
    joystick_state: Watch::new(),
    vbat: Watch::new(),
  })
}
