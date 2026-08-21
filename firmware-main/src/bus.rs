use core::sync::atomic::Ordering;

use defmt::Format;
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_nrf::nvmc::Nvmc;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, watch::Watch};
use fixed::types::I1F15;
use portable_atomic::{AtomicBool, AtomicU64};
use sequential_storage::{
    cache::{Cache, Uncached},
    map::{Key, MapConfig, MapStorage, SerializationError},
};
use static_cell::StaticCell;
use strum::FromRepr;

#[derive(Clone, Copy, Default)]
pub struct JoystickState {
    pub x: I1F15,
    pub y: I1F15,
    pub trig: I1F15,
    pub btn: bool,
}

#[derive(Clone, Copy, Default)]
pub struct ImuUpdate {
    pub dx: I1F15,
    pub dy: I1F15,
    pub trig: I1F15,
    pub btn: bool,
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Format, FromRepr)]
#[repr(u8)]
pub enum StorageKey {
    // 0 is reserved
    BondingInfo = 1,

    Unknown = 127,
}

impl Key for StorageKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 1 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0] = *self as u8;
        Ok(1) // used 1 byte
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        if buffer.len() < 1 {
            return Err(SerializationError::BufferTooSmall);
        }
        let key = Self::from_repr(buffer[0]);

        if let Some(key) = key {
            Ok((key, 1))
        } else {
            Ok((StorageKey::Unknown, 1))
        }
    }

    fn get_len(_buffer: &[u8]) -> Result<usize, SerializationError> {
        Ok(1)
    }
}

pub struct GlobalBus {
    pub storage: Mutex<
        CriticalSectionRawMutex,
        MapStorage<
            StorageKey,
            BlockingAsync<Nvmc<'static>>,
            Cache<Uncached, Uncached, Uncached, StorageKey>,
        >,
    >,

    pub joystick_state: Watch<CriticalSectionRawMutex, JoystickState, 2>,
    pub vbat_mv: Watch<CriticalSectionRawMutex, u16, 2>,
    pub vbat_soc: Watch<CriticalSectionRawMutex, u8, 2>,  // 0 - 100 inclusive
    pub usb_powered: AtomicBool,
    pub charging: AtomicBool,
    pub last_activity: AtomicU64,  // timestamp of last activity for power keep-alice
}

static BUS: StaticCell<GlobalBus> = StaticCell::new();

pub fn init(flash: Nvmc<'static>) -> &'static GlobalBus {
    let storage = MapStorage::<StorageKey, _, _>::new(
        BlockingAsync::new(flash),
        const { MapConfig::new(0x000F_E000..0x0010_0000) },
        Cache::new_uncached(),
    );

    BUS.init(GlobalBus {
        storage: Mutex::new(storage),

        joystick_state: Watch::new(),
        vbat_mv: Watch::new(),
        vbat_soc: Watch::new(),

        usb_powered: AtomicBool::new(false),
        charging: AtomicBool::new(false),
        last_activity: AtomicU64::new(0),
    })
}

impl GlobalBus {
    pub fn activity(&mut self) {
        let now = embassy_time::Instant::now().as_micros() as u64;
        self.last_activity.store(now, Ordering::Relaxed);
    }
}
