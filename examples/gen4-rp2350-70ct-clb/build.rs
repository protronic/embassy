use std::env;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=c/display.c");
    println!("cargo:rerun-if-changed=c/display.h");
    println!("cargo:rerun-if-env-changed=GEN4_GRAPHICS4D_SDK");

    build_display_driver();

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}

fn build_display_driver() {
    let mut build = cc::Build::new();
    build.file("c/display.c").include("c");

    if let Ok(sdk) = env::var("GEN4_GRAPHICS4D_SDK") {
        let sdk = Path::new(&sdk);
        let include = sdk.join("include");
        let src = sdk.join("src");
        if include.is_dir() && src.is_dir() {
            build.define("GEN4_USE_GRAPHICS4D", None);
            build.include(&include);
            println!("cargo:rustc-link-search=native={}", sdk.join("lib").display());
            println!("cargo:rustc-link-lib=static=graphics4d_rp2350");
            println!(
                "cargo:warning=Linking Graphics4D from GEN4_GRAPHICS4D_SDK={}",
                sdk.display()
            );
        } else {
            println!(
                "cargo:warning=GEN4_GRAPHICS4D_SDK is set but {}/{{include,src}} were not found; using stub scanout",
                sdk.display()
            );
        }
    } else {
        println!(
            "cargo:warning=GEN4_GRAPHICS4D_SDK not set; RGB panel scanout is stubbed (LVGL still renders into PSRAM)"
        );
    }

    build.compile("gen4_display");
}
