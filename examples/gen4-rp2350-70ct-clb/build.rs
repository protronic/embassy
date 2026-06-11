//! Build script for gen4-RP2350-70CT-CLB examples.
//!
//! RGB panel scan-out is provided by 4D Systems' proprietary **Graphics4D**
//! library. Resolution order:
//! 1. `GEN4_GRAPHICS4D_SDK` env override
//! 2. `vendor/graphics4d-rp2350/` — prebuilt lib committed in this repo
//! 3. `vendor/Graphics4D-pico/` — local source clone + `build-graphics4d-lib.sh`

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
    println!("cargo:rerun-if-changed=c/graphics4d_scanout.h");
    println!("cargo:rerun-if-changed=c/pico_sdk_stubs.c");
    println!("cargo:rerun-if-changed=c/display.h");
    println!("cargo:rerun-if-changed=scripts/filter-graphics4d-embassy-lib.sh");
    println!("cargo:rerun-if-changed=scripts/build-pico-embassy-lib.sh");
    println!("cargo:rerun-if-changed=cmake/pico-embassy-lib/CMakeLists.txt");
    println!("cargo:rerun-if-env-changed=PICO_SDK_PATH");
    println!("cargo:rerun-if-env-changed=GEN4_GRAPHICS4D_SDK");
    println!(
        "cargo:rerun-if-changed={}",
        manifest_dir.join("vendor/graphics4d-rp2350").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        manifest_dir.join("vendor/Graphics4D-pico").display()
    );

    build_display_driver(&manifest_dir, &out);

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

    let prebuilt = manifest_dir.join("vendor/graphics4d-rp2350");
    if validate_sdk_root(&prebuilt) {
        return Some(prebuilt);
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
            for lib_name in static_lib_names() {
                if name == lib_name {
                    let stem = lib_name.strip_prefix("lib").and_then(|s| s.strip_suffix(".a"))?;
                    return Some((path.parent()?.to_path_buf(), stem));
                }
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
            if has_graphics4d_sources(&path) {
                println!(
                    "cargo:warning=GEN4_GRAPHICS4D_SDK={sdk} — sources found but libgraphics4d_rp2350.a missing; run scripts/build-graphics4d-lib.sh"
                );
            } else {
                println!("cargo:warning=GEN4_GRAPHICS4D_SDK={sdk} — libgraphics4d_rp2350.a not found");
            }
        }
        return;
    }

    let prebuilt = manifest_dir.join("vendor/graphics4d-rp2350");
    if prebuilt.is_dir() && !validate_sdk_root(&prebuilt) {
        if graphics4d_header(&prebuilt).is_none() {
            println!(
                "cargo:warning={} exists but Graphics4D.h is missing",
                prebuilt.display()
            );
        }
        if find_static_lib(&prebuilt).is_none() {
            println!(
                "cargo:warning={} exists but libgraphics4d_rp2350.a is missing — run scripts/vendor-graphics4d-into-repo.sh",
                prebuilt.display()
            );
        }
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
            if has_graphics4d_sources(&vendor) {
                println!(
                    "cargo:warning={} has Graphics4D sources but no libgraphics4d_rp2350.a — run scripts/build-graphics4d-lib.sh (needs PICO_SDK_PATH)",
                    vendor.display()
                );
            } else {
                println!(
                    "cargo:warning={} exists but libgraphics4d_rp2350.a is missing",
                    vendor.display()
                );
            }
        }
    } else {
        println!(
            "cargo:warning={} not present — run scripts/init-graphics4d-pico.sh or set GEN4_GRAPHICS4D_SDK",
            vendor.display()
        );
    }
}

fn build_display_driver(manifest_dir: &Path, out_dir: &Path) {
    if let Some(sdk) = resolve_graphics4d_sdk(manifest_dir) {
        link_graphics4d(&sdk, out_dir);
        build_gfx4d_glue(&sdk);
    } else {
        println!(
            "cargo:warning=Graphics4D not linked — using Embassy PIO+DPI scanout (see src/dpi.rs)"
        );
        diagnose_missing_sdk(manifest_dir);
        build_stub_glue();
    }
}

/// Link newlib + libstdc++ for `thumbv8m.main-none-eabihf` (RP2350 / Cortex-M33 hard-float).
fn link_arm_eabihf_toolchain(bin: &str) {
    let Some(libstdcxx) = arm_none_eabi_lib("libstdc++.a") else {
        println!("cargo:warning=arm-none-eabi libstdc++.a not found — Graphics4D C++ link may fail");
        return;
    };
    if let Some(lib_dir) = libstdcxx.parent() {
        println!("cargo:rustc-link-search=native={}", lib_dir.display());
    }
    if let Some(libgcc) = arm_none_eabi_lib("libgcc.a") {
        if let Some(gcc_dir) = libgcc.parent() {
            println!("cargo:rustc-link-search=native={}", gcc_dir.display());
        }
    }

    for lib in ["stdc++", "c", "m", "gcc", "nosys"] {
        println!("cargo:rustc-link-arg-bin={bin}=-l{lib}");
    }
}

fn arm_none_eabi_lib(name: &str) -> Option<PathBuf> {
    let out = std::process::Command::new("arm-none-eabi-gcc")
        .args([
            "-mcpu=cortex-m33",
            "-mthumb",
            "-mfloat-abi=hard",
            "-mfpu=fpv5-sp-d16",
            &format!("-print-file-name={name}"),
        ])
        .output()
        .ok()?;
    let path = String::from_utf8(out.stdout).ok()?;
    let path = path.trim();
    let path = PathBuf::from(path);
    if path.is_file() {
        return Some(path);
    }
    path.canonicalize().ok().filter(|p| p.is_file())
}

fn prepare_embassy_graphics4d_lib(sdk: &Path, out_dir: &Path) -> (PathBuf, String) {
    let (lib_dir, lib_name) = find_static_lib(sdk).expect("validated above");
    let src_lib = lib_dir.join(format!("lib{lib_name}.a"));
    let filtered = out_dir.join("libgraphics4d_rp2350_embassy.a");
    let filter_script = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap())
        .join("scripts/filter-graphics4d-embassy-lib.sh");

    if filter_script.is_file() {
        let ok = std::process::Command::new("bash")
            .arg(&filter_script)
            .arg(&src_lib)
            .arg(&filtered)
            .status()
            .map(|s| s.success())
            .unwrap_or(false);
        if ok && filtered.is_file() {
            println!(
                "cargo:warning=Using Embassy-filtered Graphics4D archive ({})",
                filtered.display()
            );
            return (out_dir.to_path_buf(), "graphics4d_rp2350_embassy".to_string());
        }
        println!("cargo:warning=Embassy filter failed — linking vendored archive as-is");
    }

    (lib_dir, lib_name.to_string())
}

fn build_pico_embassy_lib(out_dir: &Path, bin: &str) {
    let Ok(pico_sdk) = env::var("PICO_SDK_PATH") else {
        println!(
            "cargo:warning=PICO_SDK_PATH unset — slim Graphics4D lib needs libpico_embassy.a (set PICO_SDK_PATH or use fat vendored .a)"
        );
        return;
    };
    if !Path::new(&pico_sdk).is_dir() {
        println!("cargo:warning=PICO_SDK_PATH={pico_sdk} is not a directory");
        return;
    }

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let script = manifest_dir.join("scripts/build-pico-embassy-lib.sh");
    if !script.is_file() {
        return;
    }

    let pico_out = out_dir.join("pico-embassy");
    let board_dir = manifest_dir.join("board");
    let status = std::process::Command::new("bash")
        .arg(&script)
        .env("PICO_SDK_PATH", &pico_sdk)
        .env("OUT_DIR", &pico_out)
        .env("BOARD_DIR", &board_dir)
        .status();

    let lib = pico_out.join("libpico_embassy.a");
    match status {
        Ok(s) if s.success() && lib.is_file() => {
            println!("cargo:rustc-link-search=native={}", pico_out.display());
            println!("cargo:rustc-link-arg-bin={bin}=-lpico_embassy");
            println!(
                "cargo:warning=Linking Pico SDK support from {}",
                lib.display()
            );
        }
        _ => println!("cargo:warning=libpico_embassy.a build failed — Graphics4D hardware symbols may be missing"),
    }
}

fn link_graphics4d(sdk: &Path, out_dir: &Path) {
    let (lib_dir, lib_name) = prepare_embassy_graphics4d_lib(sdk, out_dir);
    let bin = "oxivgl_widget_demo";

    println!("cargo:rustc-cfg=gen4_graphics4d");
    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    // Link into the firmware binary only — avoids embedding the whole .a in the Rust rlib.
    println!("cargo:rustc-link-arg-bin={bin}=-l{lib_name}");
    build_pico_embassy_lib(out_dir, bin);
    link_arm_eabihf_toolchain(bin);
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

    cc::Build::new()
        .file("c/pico_sdk_stubs.c")
        .compile("gen4_pico_stubs");
}

fn build_stub_glue() {
    cc::Build::new()
        .file("c/display_stub.c")
        .include("c")
        .compile("gen4_display");
}
