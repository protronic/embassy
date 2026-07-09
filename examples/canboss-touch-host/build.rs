//! Host-Build: teilt den EDS->Rust-Codegen mit der Firmware.
//!
//! `build_codegen.rs` liegt bei der Firmware (gen4-ft813-70ctp-h573i-dk) und
//! wird per `include!` eingebunden; als Quelle dienen deren EDS-Dateien, damit
//! Host und Target garantiert dieselben `NODES`-Tabellen sehen.

use std::env;
use std::path::{Path, PathBuf};

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let eds_dir = manifest_dir.join("../gen4-ft813-70ctp-h573i-dk/eds");
    generate_canboss_tables(&eds_dir);
}

include!("../gen4-ft813-70ctp-h573i-dk/build_codegen.rs");
