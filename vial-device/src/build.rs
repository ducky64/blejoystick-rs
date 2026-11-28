//! Build-script helpers (feature `build`): turn `vial.json` into the compressed definition that
//! Vial downloads from the device.
//!
//! ```toml
//! [build-dependencies]
//! vial-device = { path = "...", features = ["build"] }
//! ```
//! ```ignore
//! // build.rs
//! vial_device::build::compress_definition_file("vial.json", "vial.json.lzma");
//! // firmware
//! static DEFINITION: &[u8] = vial_device::include_definition!("vial.json.lzma");
//! ```

use std::io::Read;
use std::path::{Path, PathBuf};

/// Minifies a vial.json definition and LZMA-compresses, to the format VIAL GUI expects
pub fn compress_definition(json: &str) -> Result<Vec<u8>, String> {
    let value: serde_json::Value =
        serde_json::from_str(json).map_err(|e| format!("invalid JSON: {e}"))?;
    let minified = serde_json::to_vec(&value).map_err(|e| e.to_string())?;

    let options = xz2::stream::LzmaOptions::new_preset(9).map_err(|e| e.to_string())?;
    let stream = xz2::stream::Stream::new_lzma_encoder(&options).map_err(|e| e.to_string())?;
    let mut compressed = Vec::new();
    xz2::read::XzEncoder::new_stream(&minified[..], stream)
        .read_to_end(&mut compressed)
        .map_err(|e| e.to_string())?;
    Ok(compressed)
}

/// For build scripts: compresses the definition at `json_path` (relative to the crate root) into
/// `$OUT_DIR/<out_name>`, for [`crate::include_definition!`].
/// Reruns on changes. Returns the output path.
pub fn compress_definition_file(json_path: impl AsRef<Path>, out_name: &str) -> PathBuf {
    let json_path = json_path.as_ref();
    println!("cargo:rerun-if-changed={}", json_path.display());
    let json = std::fs::read_to_string(json_path).unwrap();
    let compressed =
        compress_definition(&json).unwrap_or_else(|e| panic!("{}: {e}", json_path.display()));
    let out = PathBuf::from(std::env::var_os("OUT_DIR").unwrap()).join(out_name);
    std::fs::write(&out, compressed).unwrap();
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decompress(data: &[u8]) -> Vec<u8> {
        let stream = xz2::stream::Stream::new_lzma_decoder(u64::MAX).unwrap();
        let mut out = Vec::new();
        xz2::read::XzDecoder::new_stream(data, stream)
            .read_to_end(&mut out)
            .unwrap();
        out
    }

    #[test]
    fn round_trip_minified() {
        let json = "{\n  \"name\": \"Test\",\n  \"matrix\": { \"rows\": 1, \"cols\": 2 }\n}\n";
        let compressed = compress_definition(json).unwrap();
        assert_eq!(&compressed[..1], &[0x5D]); // LZMA-alone header: lc=3, lp=0, pb=2
        assert_eq!(
            decompress(&compressed),
            br#"{"matrix":{"cols":2,"rows":1},"name":"Test"}"#
        );
    }

    #[test]
    fn rejects_invalid_json() {
        assert!(compress_definition("{ nope").is_err());
    }
}
