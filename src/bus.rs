use defmt::Format;
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, watch::Watch};
use esp_storage::FlashStorage;
use sequential_storage::{cache::NoCache, map::{Key, MapConfig, MapStorage, SerializationError}};
use static_cell::StaticCell;
use strum::FromRepr;

#[derive(Clone, Copy, Default)]
pub struct JoystickState {
    pub x: i16,  // up-scaled to i16, deadzone pre-applied
    pub y: i16,
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
            return Err(SerializationError::BufferTooSmall)
        }
        buffer[0] = *self as u8;
        Ok(1)  // used 1 byte
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        if buffer.len() < 1 {
            return Err(SerializationError::BufferTooSmall)
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
    pub storage: Mutex<CriticalSectionRawMutex, MapStorage<StorageKey, BlockingAsync<FlashStorage<'static>>, NoCache>>,

    pub joystick_state: Watch<CriticalSectionRawMutex, JoystickState, 2>,
    pub vbat: Watch<CriticalSectionRawMutex, u16, 2>,  // in mV
}


static BUS: StaticCell<GlobalBus> = StaticCell::new();


pub fn init(
    flash: FlashStorage<'static>
) -> &'static GlobalBus {
  let storage = MapStorage::<StorageKey, BlockingAsync<FlashStorage<'static>>, NoCache>::new(
      BlockingAsync::new(flash),
      const { MapConfig::new(0x10_0000..0x12_0000) }, 
      NoCache::new());

  BUS.init(GlobalBus {
    storage: Mutex::new(storage),

    joystick_state: Watch::new(),
    vbat: Watch::new(),
  })
}
