//! Persistent (non-volatile) device settings.
//!
//! Settings are a single in-memory-cached record managed by `embedded-nvm-settings`,
//! serialized with `versioned-postcard` and stored with `sequential-storage` in the
//! STORAGE flash region (see memory.x).
//!
//! To change the format (see the versioned-postcard README):
//! - appending fields: add a separate `Wire` struct with the new fields as `Option`s
//! - otherwise: keep the current struct as the `Prev` version, and bump `TAG`

use defmt::Format;
use postcard::experimental::max_size::MaxSize;
use serde::{Deserialize, Serialize};
use versioned_postcard::{Base, Version};

use crate::ble_peripheral::StoredBondInformation;

/// All persistent settings.
#[derive(Clone, Copy, Default, PartialEq, Debug, Format, Serialize, Deserialize, MaxSize)]
pub struct Settings {
    pub bond: Option<StoredBondInformation>,
}

impl Version for Settings {
    const TAG: u16 = 1;
    type Wire = Settings;
    type Prev = Base;
    fn from_prev(p: Base) -> Self {
        match p {}
    }
}

/// Serialization buffer size: the largest record plus the 1-byte sequential-storage key,
/// rounded up to the flash word size as required by sequential-storage.
pub const SETTINGS_BUF_SIZE: usize = (<Settings as Version>::RECORD_MAX + 1).next_multiple_of(4);
