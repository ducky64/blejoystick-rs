use core::cell::RefCell;
use core::sync::atomic::Ordering;

use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_embedded_hal::flash::partition::BlockingPartition;
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_sync::{blocking_mutex, blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch, semaphore::GreedySemaphore};
use fixed::types::I1F15;
use portable_atomic::{AtomicU64};
use embedded_nvm_settings::{SeqNvmSettings, SeqStorageBackend, VersionedPostcardFormat};
use sequential_storage::{
    cache::NoCache,
    map::{MapConfig, MapStorage},
};
use static_cell::StaticCell;

use crate::prelude::*;
use crate::settings::{Settings, SETTINGS_BUF_SIZE};

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

/// The internal flash, shared between persistent storage and the firmware updater.
/// All flash operations are blocking, so this lock is never held across an await.
pub type SharedFlash = blocking_mutex::Mutex<CriticalSectionRawMutex, RefCell<Nvmc<'static>>>;

/// A region of the shared flash, addressed relative to the start of the region.
pub type FlashPartition = BlockingPartition<'static, CriticalSectionRawMutex, Nvmc<'static>>;

type StorageFlash = BlockingAsync<FlashPartition>;

/// Persistent settings, stored in the STORAGE partition.
/// The NVM_START / NVM_SIZE const parameters are 0 since they are unused: the storage is built
/// with `SeqStorageBackend::from_storage` using the partition range from the linker symbols.
pub type NvSettings = SeqNvmSettings<Settings, StorageFlash, NoCache, 0, 0, SETTINGS_BUF_SIZE>;

pub struct GlobalBus {
    pub flash: &'static SharedFlash,

    /// Persistent settings. Must be loaded with [`GlobalBus::load_settings`] before use.
    pub settings: NvSettings,

    pub joystick_state: Watch<CriticalSectionRawMutex, JoystickState, 2>,
    pub buttons_state: Watch<CriticalSectionRawMutex, u8, 2>,  // bitmask of buttons
    pub vbat_mv: Watch<CriticalSectionRawMutex, u16, 2>,
    pub vbat_soc: Watch<CriticalSectionRawMutex, u8, 2>,  // 0 - 100 inclusive
    pub usb_powered: Watch<CriticalSectionRawMutex, bool, 2>,
    pub vbus: SoftwareVbusDetect,  // VBUS state for the USB driver, fed by charger_task
    pub charging: Watch<CriticalSectionRawMutex, bool, 2>,
    pub last_activity: AtomicU64,  // timestamp of last activity for power keep-alive

    pub shutdown_requested: Watch<CriticalSectionRawMutex, bool, 2>,  // set to indicate shutdown is requested or in progress
    pub shutdown_locks: GreedySemaphore<CriticalSectionRawMutex>,  // locks indicating processes waiting to complete before shutdown
}

static BUS: StaticCell<GlobalBus> = StaticCell::new();
static FLASH: StaticCell<SharedFlash> = StaticCell::new();

pub fn init(flash: Nvmc<'static>) -> &'static GlobalBus {
    let flash: &'static SharedFlash = FLASH.init(blocking_mutex::Mutex::new(RefCell::new(flash)));

    // STORAGE region from memory.x, unchanged from the pre-bootloader layout (0xFE000..0x100000)
    let storage_partition = firmware_common::linker_partition!(flash, __storage_start, __storage_end);
    let storage_size = storage_partition.size();
    let settings = NvSettings::new(
        SeqStorageBackend::from_storage(MapStorage::new(
            BlockingAsync::new(storage_partition),
            MapConfig::new(0..storage_size),
            NoCache::new(),
        )),
        VersionedPostcardFormat,
    );

    BUS.init(GlobalBus {
        flash,
        settings,

        joystick_state: Watch::new(),
        buttons_state: Watch::new(),
        vbat_mv: Watch::new(),
        vbat_soc: Watch::new(),

        usb_powered: Watch::new(),
        vbus: SoftwareVbusDetect::new(false, false),
        charging: Watch::new(),
        last_activity: AtomicU64::new(0),

        shutdown_requested: Watch::new(),
        shutdown_locks: GreedySemaphore::new(GlobalBus::SHUTDOWN_LOCKS),
    })
}

impl GlobalBus {
    pub const SHUTDOWN_LOCKS: usize = 4;

    pub fn activity(&self) {
        let now = embassy_time::Instant::now().as_micros() as u64;
        self.last_activity.store(now, Ordering::Relaxed);
    }

    /// Loads persistent settings from flash into the settings cache.
    /// On failure, settings are left at defaults.
    pub async fn load_settings(&self) {
        match self.settings.load().await {
            Ok(true) => info!("Loaded settings: {}", self.settings.get()),
            Ok(false) => info!("No stored settings found, using defaults"),
            Err(e) => error!("Failed to load settings: {}", defmt::Debug2Format(&e)),
        }
    }
}
