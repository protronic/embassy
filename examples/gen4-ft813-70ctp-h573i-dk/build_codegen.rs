// ---------------------------------------------------------------------------
// EDS -> Rust-Codegen (Gegenstueck zu CANbossTouch tools/eds2lvgl.py)
// ---------------------------------------------------------------------------

/// Widget-Auswahl je Datenpunkt — muss dem C-Generator entsprechen.
const SLIDER_MAX_SPAN: i64 = 2000;
const F32_SCALE: f64 = 1000.0;
const F32_DEFAULT_RANGE: (i64, i64) = (-2_000_000, 2_000_000);
const DEFAULT_INCLUDE: &[(u16, u16)] = &[(0x1000, 0x1001), (0x1008, 0x100A), (0x2000, 0x9FFF)];

#[derive(Clone, Copy, PartialEq)]
enum DType {
    Bool,
    I8,
    I16,
    I32,
    U8,
    U16,
    U32,
    F32,
    Str,
    Octet,
    I64,
    U64,
}

impl DType {
    fn from_eds(raw: u32) -> Option<Self> {
        Some(match raw {
            0x01 => Self::Bool,
            0x02 => Self::I8,
            0x03 => Self::I16,
            0x04 => Self::I32,
            0x05 => Self::U8,
            0x06 => Self::U16,
            0x07 => Self::U32,
            0x08 => Self::F32,
            0x09 => Self::Str,
            0x0A => Self::Octet,
            0x15 => Self::I64,
            0x1B => Self::U64,
            _ => return None,
        })
    }

    fn is_int(self) -> bool {
        matches!(
            self,
            Self::I8 | Self::I16 | Self::I32 | Self::U8 | Self::U16 | Self::U32 | Self::I64 | Self::U64
        )
    }

    /// Vorgabe-Grenzen (i32-Spinbox-Bereich) je Datentyp.
    fn default_range(self) -> (i64, i64) {
        match self {
            Self::I8 => (-128, 127),
            Self::I16 => (-32768, 32767),
            Self::U8 => (0, 255),
            Self::U16 => (0, 65535),
            Self::U32 | Self::U64 => (0, 2147483647),
            _ => (-2147483648, 2147483647),
        }
    }

    fn rust_name(self) -> &'static str {
        match self {
            Self::Bool => "Bool",
            Self::I8 => "I8",
            Self::I16 => "I16",
            Self::I32 => "I32",
            Self::U8 => "U8",
            Self::U16 => "U16",
            Self::U32 => "U32",
            Self::F32 => "F32",
            Self::Str => "Str",
            Self::Octet => "Octet",
            Self::I64 => "I64",
            Self::U64 => "U64",
        }
    }
}

struct GenDp {
    index: u16,
    sub: u8,
    name: String,
    dtype: DType,
    access: &'static str, // "Ro" | "Wo" | "Rw"
    min: i64,
    max: i64,
    has_limits: bool,
    eds_limits: bool,
}

impl GenDp {
    fn widget(&self) -> &'static str {
        if self.access == "Rw" {
            if self.dtype == DType::Bool {
                return "Switch";
            }
            if self.dtype.is_int() {
                if self.eds_limits && self.min == 0 && self.max == 1 {
                    return "Switch";
                }
                if self.eds_limits && self.max > self.min && self.max - self.min <= SLIDER_MAX_SPAN {
                    return "Slider";
                }
                return "Spinbox";
            }
            if self.dtype == DType::F32 {
                return "SpinboxF32";
            }
            if self.dtype == DType::Str {
                return "Text";
            }
        } else if self.dtype.is_int() && self.eds_limits && self.min < self.max {
            return "Bar";
        }
        "Value"
    }
}

/// Minimaler INI-Reader fuer EDS-Dateien (CiA 306): Sektion -> Key -> Wert.
fn read_eds_sections(path: &Path) -> Vec<(String, Vec<(String, String)>)> {
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("EDS {path:?}: {e}"));
    let mut sections: Vec<(String, Vec<(String, String)>)> = Vec::new();
    for raw in text.lines() {
        let line = raw.trim_start_matches('\u{feff}').trim();
        if line.is_empty() || line.starts_with(';') || line.starts_with('#') {
            continue;
        }
        if let Some(name) = line.strip_prefix('[').and_then(|s| s.strip_suffix(']')) {
            sections.push((name.trim().to_string(), Vec::new()));
        } else if let Some((k, v)) = line.split_once('=') {
            if let Some(last) = sections.last_mut() {
                last.1.push((k.trim().to_string(), v.trim().to_string()));
            }
        }
    }
    sections
}

fn section_get<'a>(kv: &'a [(String, String)], key: &str) -> Option<&'a str> {
    kv.iter()
        .find(|(k, _)| k.eq_ignore_ascii_case(key))
        .map(|(_, v)| v.as_str())
}

/// EDS-Zahlen: dezimal oder 0x-Hex; `$NODEID`-Formeln liefern None.
fn parse_eds_int(v: Option<&str>) -> Option<i64> {
    let v = v?.trim();
    if v.is_empty() || v.contains('$') {
        return None;
    }
    if let Some(hex) = v.strip_prefix("0x").or_else(|| v.strip_prefix("0X")) {
        i64::from_str_radix(hex, 16).ok()
    } else {
        v.parse::<i64>().ok()
    }
}

fn parse_eds_float(v: Option<&str>) -> Option<f64> {
    let v = v?.trim();
    if v.is_empty() || v.contains('$') {
        return None;
    }
    parse_eds_int(Some(v))
        .map(|i| i as f64)
        .or_else(|| v.parse::<f64>().ok())
}

fn make_dp(index: u16, sub: u8, name: &str, kv: &[(String, String)]) -> Option<GenDp> {
    let dtype = DType::from_eds(parse_eds_int(section_get(kv, "DataType"))? as u32)?;
    let access = match section_get(kv, "AccessType")
        .unwrap_or("ro")
        .trim()
        .to_ascii_lowercase()
        .as_str()
    {
        "wo" => "Wo",
        "rw" | "rww" | "rwr" => "Rw",
        _ => "Ro",
    };

    let (mut min, mut max, mut has_limits, mut eds_limits) = (0i64, 0i64, false, false);
    if dtype.is_int() {
        let lo = parse_eds_int(section_get(kv, "LowLimit"));
        let hi = parse_eds_int(section_get(kv, "HighLimit"));
        eds_limits = lo.is_some() && hi.is_some();
        let (dlo, dhi) = dtype.default_range();
        min = lo.map_or(dlo, |v| v.max(dlo));
        max = hi.map_or(dhi, |v| v.min(dhi));
        has_limits = true;
    } else if dtype == DType::F32 {
        let lo = parse_eds_float(section_get(kv, "LowLimit"));
        let hi = parse_eds_float(section_get(kv, "HighLimit"));
        eds_limits = lo.is_some() && hi.is_some();
        let (dlo, dhi) = F32_DEFAULT_RANGE;
        min = lo.map_or(dlo, |v| ((v * F32_SCALE) as i64).max(dlo));
        max = hi.map_or(dhi, |v| ((v * F32_SCALE) as i64).min(dhi));
        has_limits = true;
    }

    Some(GenDp {
        index,
        sub,
        name: name.trim().to_string(),
        dtype,
        access,
        min,
        max,
        has_limits,
        eds_limits,
    })
}

fn parse_include_ranges(node: &serde_json::Value) -> Vec<(u16, u16)> {
    let Some(list) = node.get("include").and_then(|v| v.as_array()) else {
        return DEFAULT_INCLUDE.to_vec();
    };
    list.iter()
        .filter_map(|spec| {
            let s = spec.as_str()?.replace(' ', "");
            let parse = |t: &str| -> Option<u16> {
                let t = t.trim();
                if let Some(h) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
                    u16::from_str_radix(h, 16).ok()
                } else {
                    t.parse().ok()
                }
            };
            if let Some((lo, hi)) = s.split_once('-') {
                Some((parse(lo)?, parse(hi)?))
            } else {
                let v = parse(&s)?;
                Some((v, v))
            }
        })
        .collect()
}

fn extract_datapoints(eds_path: &Path, include: &[(u16, u16)]) -> Vec<GenDp> {
    let sections = read_eds_sections(eds_path);

    // Sektionsnamen: "1234" (Objekt, hex) oder "1234sub5" (Subindex, hex)
    let mut objects: Vec<(u16, usize)> = Vec::new();
    let mut subs: Vec<(u16, u8, usize)> = Vec::new();
    for (i, (name, _)) in sections.iter().enumerate() {
        let n = name.trim().to_ascii_lowercase();
        if let Some((idx_part, sub_part)) = n.split_once("sub") {
            if idx_part.len() == 4 {
                if let (Ok(idx), Ok(sub)) = (u16::from_str_radix(idx_part, 16), u8::from_str_radix(sub_part, 16)) {
                    subs.push((idx, sub, i));
                    continue;
                }
            }
        }
        if n.len() == 4 {
            if let Ok(idx) = u16::from_str_radix(&n, 16) {
                objects.push((idx, i));
            }
        }
    }
    objects.sort_by_key(|(idx, _)| *idx);

    let in_ranges = |idx: u16| include.iter().any(|(lo, hi)| (*lo..=*hi).contains(&idx));
    let mut dps = Vec::new();

    for (idx, sec_i) in objects {
        if !in_ranges(idx) {
            continue;
        }
        let kv = sections[sec_i].1.clone();
        let obj_name = section_get(&kv, "ParameterName")
            .map(|s| s.to_string())
            .unwrap_or_else(|| format!("Object {idx:04X}"));
        let obj_type = parse_eds_int(section_get(&kv, "ObjectType")).unwrap_or(0x07);

        let mut sub_secs: Vec<(u8, usize)> = subs
            .iter()
            .filter(|(i, _, _)| *i == idx)
            .map(|(_, s, si)| (*s, *si))
            .collect();
        sub_secs.sort_by_key(|(s, _)| *s);

        if obj_type == 0x07 || sub_secs.is_empty() {
            if let Some(dp) = make_dp(idx, 0, &obj_name, &kv) {
                dps.push(dp);
            }
            continue;
        }
        // ARRAY/RECORD: Subindex 0 (Anzahl Eintraege) ueberspringen
        for (sub, si) in sub_secs.into_iter().filter(|(s, _)| *s != 0) {
            let skv = sections[si].1.clone();
            let sname = section_get(&skv, "ParameterName")
                .map(|s| s.to_string())
                .unwrap_or_else(|| format!("Sub {sub}"));
            let label = if obj_name.to_lowercase().contains(&sname.to_lowercase()) {
                sname.clone()
            } else {
                format!("{obj_name}: {sname}")
            };
            if let Some(dp) = make_dp(idx, sub, &label, &skv) {
                dps.push(dp);
            }
        }
    }
    dps
}

/// EDS-Tabellen (`NODES`) nach OUT_DIR/canboss_gen.rs generieren.
/// `eds_dir` enthaelt network.json + die EDS-Dateien.
fn generate_canboss_tables(eds_dir: &Path) {
    let network_path = eds_dir.join("network.json");
    println!("cargo:rerun-if-changed={}", network_path.display());

    let network: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&network_path).expect("read eds/network.json"))
            .expect("parse eds/network.json");
    let nodes = network["nodes"].as_array().expect("nodes array");

    let esc = |s: &str| s.replace('\\', "\\\\").replace('"', "\\\"");
    let mut out = String::from("// @generated by build.rs aus eds/network.json — nicht editieren\n\n");
    let mut node_entries = String::new();

    for node in nodes {
        let node_id = node["node_id"].as_u64().expect("node_id") as u8;
        let name = node["name"].as_str().unwrap_or("Node");
        let eds_file = node["eds"].as_str().expect("eds");
        let eds_path = eds_dir.join(eds_file);
        println!("cargo:rerun-if-changed={}", eds_path.display());

        let include = parse_include_ranges(node);
        let dps = extract_datapoints(&eds_path, &include);

        out.push_str(&format!("static NODE_{node_id}_DPS: &[Dp] = &[\n"));
        for dp in &dps {
            out.push_str(&format!(
                "    Dp {{ index: 0x{:04X}, sub: 0x{:02X}, name: \"{}\", dtype: DType::{}, \
                 access: Access::{}, widget: WidgetKind::{}, min: {}, max: {}, has_limits: {} }},\n",
                dp.index,
                dp.sub,
                esc(&dp.name),
                dp.dtype.rust_name(),
                dp.access,
                dp.widget(),
                dp.min.clamp(i32::MIN as i64, i32::MAX as i64),
                dp.max.clamp(i32::MIN as i64, i32::MAX as i64),
                dp.has_limits,
            ));
        }
        out.push_str("];\n\n");
        node_entries.push_str(&format!(
            "    NodeDesc {{ node_id: {node_id}, name: \"{}\", eds_file: \"{}\", dps: NODE_{node_id}_DPS }},\n",
            esc(name),
            esc(eds_file),
        ));
    }

    out.push_str(&format!(
        "/// Alle Knoten aus eds/network.json.\npub static NODES: &[NodeDesc] = &[\n{node_entries}];\n"
    ));

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap()).join("canboss_gen.rs");
    std::fs::write(&out_path, out).expect("write canboss_gen.rs");
}
