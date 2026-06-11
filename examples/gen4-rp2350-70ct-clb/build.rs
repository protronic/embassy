//! Build script for gen4-RP2350-70CT-CLB examples.
//!
//! RGB panel scan-out is provided by 4D Systems' proprietary **Graphics4D**
//! library. By default we look for it in `vendor/Graphics4D-pico`
//! (clone via `scripts/init-graphics4d-pico.sh`).
//! Override with `GEN4_GRAPHICS4D_SDK` if the library lives elsewhere
//! (e.g. a local Workshop5 install).

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};

fn main() {
    println!("cargo::rustc-check-cfg=cfg(gen4_graphics4d)");

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=c/display_stub.c");
    println!("cargo:rerun-if-changed=c/display_gfx4d.cpp");
    println!("cargo:rerun-if-changed=c/display.h");
    println!("cargo:rerun-if-env-changed=GEN4_GRAPHICS4D_SDK");
    println!(
        "cargo:rerun-if-changed={}",
        manifest_dir.join("vendor/Graphics4D-pico").display()
    );

    build_display_driver(&manifest_dir);

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}

/// Resolve the Graphics4D SDK root directory.
fn resolve_graphics4d_sdk(manifest_dir: &Path) -> Option<PathBuf> {
    if let Ok(sdk) = env::var("GEN4_GRAPHICS4D_SDK") {
        let sdk = PathBuf::from(sdk);
        if validate_sdk_root(&sdk) {
            return Some(sdk);
        }
        println!(
            "cargo:warning=GEN4_GRAPHICS4D_SDK={} is set but no Graphics4D headers/static lib were found",
            sdk.display()
        );
        return None;
    }

    let vendor = manifest_dir.join("vendor/Graphics4D-pico");
    if validate_sdk_root(&vendor) {
        return Some(vendor);
    }

    None
}

/// Accept Workshop5 / Graphics4D-pico layouts with headers in `include/` or at the root.
fn validate_sdk_root(sdk: &Path) -> bool {
    graphics4d_header(sdk).is_some() && find_static_lib(sdk).is_some()
}

fn graphics4d_header(sdk: &Path) -> Option<PathBuf> {
    let candidates = [
        sdk.join("include/Graphics4D.h"),
        sdk.join("Graphics4D.h"),
        sdk.join("src/Graphics4D.h"),
    ];
    candidates.into_iter().find(|p| p.is_file())
}

fn include_dirs(sdk: &Path) -> Vec<PathBuf> {
    let mut dirs = Vec::new();
    for dir in [sdk.join("include"), sdk.to_path_buf(), sdk.join("src")] {
        if dir.is_dir() {
            dirs.push(dir);
        }
    }
    dirs
}

fn static_lib_names() -> [&'static str; 3] {
    [
        "libgraphics4d_rp2350.a",
        "libgraphics4d.a",
        "libGraphics4D.a",
    ]
}

/// Locate a prebuilt Graphics4D archive (Workshop5 export or `build-graphics4d-lib.sh` output).
fn find_static_lib(sdk: &Path) -> Option<(PathBuf, &'static str)> {
    let preferred_dirs = [
        sdk.join("lib"),
        sdk.join("build-embassy"),
        sdk.join("build"),
        sdk.to_path_buf(),
    ];

    for dir in preferred_dirs {
        if !dir.is_dir() {
            continue;
        }
        for name in static_lib_names() {
            let path = dir.join(name);
            if path.is_file() {
                let stem = name.strip_prefix("lib").and_then(|s| s.strip_suffix(".a"))?;
                return Some((dir, stem));
            }
        }
    }

    // CMake / manual builds may place the archive deeper under build-*/.
    find_static_lib_recursive(sdk)
}

fn find_static_lib_recursive(sdk: &Path) -> Option<(PathBuf, &'static str)> {
    let mut stack = vec![sdk.join("build-embassy"), sdk.join("build")];
    while let Some(dir) = stack.pop() {
        let entries = std::fs::read_dir(&dir).ok()?;
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                if path.file_name().is_some_and(|n| n != "build-embassy-obj") {
                    stack.push(path);
                }
                continue;
            }
            let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
                continue;
            };
            if static_lib_names().contains(&name) {
                let stem = name.strip_prefix("lib").and_then(|s| s.strip_suffix(".a"))?;
                return Some((path.parent()?.to_path_buf(), stem));
            }
        }
    }
    None
}

fn has_graphics4d_sources(sdk: &Path) -> bool {
    let src = sdk.join("src");
    if !src.is_dir() {
        return false;
    }
    has_source_files(&src)
}

fn has_source_files(dir: &Path) -> bool {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return false;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            if has_source_files(&path) {
                return true;
            }
            continue;
        }
        if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
            if matches!(ext, "c" | "cc" | "cpp") {
                return true;
            }
        }
    }
    false
}

fn diagnose_missing_sdk(manifest_dir: &Path) {
    if let Ok(sdk) = env::var("GEN4_GRAPHICS4D_SDK") {
        let path = PathBuf::from(&sdk);
        if !path.is_dir() {
            println!("cargo:warning=GEN4_GRAPHICS4D_SDK={sdk} is not a directory");
            return;
        }
        if graphics4d_header(&path).is_none() {
            println!("cargo:warning=GEN4_GRAPHICS4D_SDK={sdk} — Graphics4D.h not found");
        }
        if find_static_lib(&path).is_none() {
            println!("cargo:warning=GEN4_GRAPHICS4D_SDK={sdk} — libgraphics4d_rp2350.a not found");
        }
        return;
    }

    let vendor = manifest_dir.join("vendor/Graphics4D-pico");
    if vendor.is_dir() {
        if graphics4d_header(&vendor).is_none() {
            println!(
                "cargo:warning={} exists but Graphics4D.h is missing",
                vendor.display()
            );
        }
        if find_static_lib(&vendor).is_none() {
            println!(
                "cargo:warning={} exists but libgraphics4d_rp2350.a is missing",
                vendor.display()
            );
        }
    } else {
        println!(
            "cargo:warning={} not present — run scripts/init-graphics4d-pico.sh or set GEN4_GRAPHICS4D_SDK",
            vendor.display()
        );
    }
}

fn build_display_driver(manifest_dir: &Path) {
    if let Some(sdk) = resolve_graphics4d_sdk(manifest_dir) {
        link_graphics4d(&sdk);
        build_gfx4d_glue(&sdk);
    } else {
        println!(
            "cargo:warning=Graphics4D not linked — RGB scanout STUB active (blank panel). Run scripts/check-graphics4d.sh"
        );
        diagnose_missing_sdk(manifest_dir);
        build_stub_glue();
    }
}

fn link_graphics4d(sdk: &Path) {
    let (lib_dir, lib_name) = find_static_lib(sdk).expect("validated above");

    println!("cargo:rustc-cfg=gen4_graphics4d");
    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=static={lib_name}");
    println!("cargo:rustc-link-lib=static=stdc++");
    println!("cargo:rustc-link-lib=static=m");
    println!("cargo:rustc-link-lib=static=c");
    println!(
        "cargo:warning=Linking Graphics4D RGB scanout from {}",
        sdk.display()
    );

    // Optional extra linker flags maintained inside Graphics4D-pico for Embassy.
    let link_file = sdk.join("embassy/link.txt");
    if link_file.is_file() {
        if let Ok(text) = std::fs::read_to_string(&link_file) {
            for line in text.lines() {
                let line = line.trim();
                if line.is_empty() || line.starts_with('#') {
                    continue;
                }
                println!("cargo:rustc-link-arg-bins={line}");
            }
        }
    }

    if let Ok(pico_sdk) = env::var("PICO_SDK_PATH") {
        let pico_lib = Path::new(&pico_sdk).join("build");
        if pico_lib.is_dir() {
            println!("cargo:rustc-link-search=native={}", pico_lib.display());
        }
    }
}

fn build_gfx4d_glue(sdk: &Path) {
    let embassy_glue = sdk.join("embassy/gen4_lcd_glue.cpp");
    let glue_file = if embassy_glue.is_file() {
        println!(
            "cargo:warning=Using Embassy glue from {}",
            embassy_glue.display()
        );
        embassy_glue
    } else {
        PathBuf::from("c/display_gfx4d.cpp")
    };

    let mut build = cc::Build::new();
    build
        .cpp(true)
        .file(&glue_file)
        .include("c")
        .define("GEN4_USE_GRAPHICS4D", None)
        .flag_if_supported("-fno-exceptions")
        .flag_if_supported("-fno-rtti");

    for dir in include_dirs(sdk) {
        build.include(dir);
    }

    // Workshop5 targets Cortex-M33 hard-float — match the Embassy target.
    build.compile("gen4_display");
}

fn build_stub_glue() {
    cc::Build::new()
        .file("c/display_stub.c")
        .include("c")
        .compile("gen4_display");
}
