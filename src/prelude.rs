#[cfg(feature = "defmt")]
pub use defmt::{debug, error, info, warn};
#[cfg(feature = "log")]
pub use log::{debug, error, info, warn};
