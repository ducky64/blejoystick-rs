use defmt::Format;
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_nrf::nvmc::Nvmc;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, watch::Watch};
use fixed::types::I1F15;
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

    fn get_len(buffer: &[u8]) -> Result<usize, SerializationError> {
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
    pub vbat: Watch<CriticalSectionRawMutex, u16, 2>, // in mV
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
        vbat: Watch::new(),
    })
}
