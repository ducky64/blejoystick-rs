//! Device-side [VIA](https://caniusevia.com/) / [Vial](https://get.vial.today/) keyboard
//! configuration protocol, for any firmware that wants to be configured by the Vial GUI.
//!
//! - [`protocol`]: a transport-agnostic, `no_std`, allocation-free command handler. It answers one
//!   32-byte request in place, against your device through the [`Keyboard`] trait.
//! - [`hid`]: the raw HID report descriptor and constants.
//! - `build` (feature `build`, for build scripts): minifies and LZMA-compresses `vial.json` into
//!   `OUT_DIR`, and [`include_definition!`] embeds the result.
//!
//! To use this in your device:
//! - put [`hid::VIAL_SERIAL_MARKER`] at the start of its USB serial number, which is how Vial
//!   recognizes Vial devices
//! - persist keymap changes, and act on the other [`Effect`]s
//!
//! Protocol notes: VIA values are big endian, the Vial extension (command `0xFE`) little endian.
//! The keyboard is always reported as unlocked (Vial's unlock combo is not implemented).
#![cfg_attr(not(any(test, feature = "build")), no_std)]

pub mod hid;
pub mod protocol;

#[cfg(feature = "build")]
pub mod build;

pub use protocol::{handle, Effect, Keyboard, VialConfig};

/// Embeds a definition compressed by `build::compress_definition_file` as a `&'static [u8]`.
/// `$name` is the output file name given to it.
#[macro_export]
macro_rules! include_definition {
    ($name:literal) => {
        include_bytes!(concat!(env!("OUT_DIR"), "/", $name))
    };
}
