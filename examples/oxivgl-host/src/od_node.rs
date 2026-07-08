//! JSON-driven CANopen-style node for the OxivGL host platform.
//!
//! Combines the pieces of the CANbossTouch PoC series on the PC:
//!
//! - **Object dictionary** from JSON — either inline `datapoints` (compact PoC
//!   schema, same as the STM32 `json_node` examples) and/or imported from a
//!   **CANopenNode / CANopenEditor JSON export** (`od_file`, e.g.
//!   `DS301_profile.json` from the CANopenNode repo) filtered by `include`
//!   index ranges (CANbossTouch `network.json` semantics).
//! - **Rhai scripts** (`once` / `cyclic`) run against the OD via
//!   `od_read`/`od_write`/`get`/`set` — see the STM32 examples.
//! - **SocketCAN**: OD changes go out as "TPDO" frames, received "RPDO"
//!   frames write into the OD (bus view: only `rw`/`wo` writable), plus
//!   CANopen bootup + heartbeat (0x700 + node id, state = operational).
//! - The **OxivGL GUI** (`od_view`) shows the OD live and writes through the
//!   same access checks (`Writer::Ui`).
//!
//! LVGL is single-threaded: scripts and GUI run on the main task; only the
//! SocketCAN TX/RX threads run in the background and touch the shared OD.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU32, Ordering};
use std::sync::{Mutex, OnceLock, mpsc};

use log::{debug, error, info, warn};
use rhai::packages::{BasicIteratorPackage, BasicMathPackage, BasicStringPackage, MoreStringPackage, Package};
use rhai::{AST, Dynamic, Engine, EvalAltResult};
use serde::Deserialize;

/// Abort a runaway script (endless loop) after this many Rhai operations.
const MAX_SCRIPT_OPS: u64 = 500_000;

// ---------------------------------------------------------------------------
// JSON schema — PoC config (node + scripts + CAN) and CANopenNode OD import
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
pub struct NodeConfig {
    pub node_id: u8,
    pub name: String,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub can: Option<CanSection>,
    /// Optional object dictionary import: CANopenNode/CANopenEditor JSON
    /// export (`objects` / `subObjects`), path relative to this config file.
    #[serde(default)]
    pub od_file: Option<String>,
    /// Index ranges for `od_file`, e.g. `["0x1000-0x1001", "0x1017"]`.
    /// Default: 0x1000-0x1001, 0x1008-0x100A, 0x1017, 0x2000-0x9FFF.
    #[serde(default)]
    pub include: Option<Vec<String>>,
    #[serde(default)]
    pub datapoints: Vec<DpConfig>,
    #[serde(default)]
    pub scripts: Vec<ScriptConfig>,
}

#[derive(Deserialize)]
pub struct CanSection {
    /// "socketcan" or "off".
    pub mode: String,
    #[serde(default = "default_channel")]
    pub channel: String,
    /// COB-ID bases; the node id is added (CANopen style).
    #[serde(default = "default_tpdo")]
    pub tpdo: String,
    #[serde(default = "default_rpdo")]
    pub rpdo: String,
    /// Send CANopen bootup + heartbeat on 0x700 + node id
    /// (period from OD 0x1017.00 in ms; 0 disables).
    #[serde(default = "default_true")]
    pub heartbeat: bool,
}

fn default_channel() -> String {
    "vcan0".to_string()
}
fn default_tpdo() -> String {
    "0x180".to_string()
}
fn default_rpdo() -> String {
    "0x200".to_string()
}
fn default_true() -> bool {
    true
}

#[derive(Deserialize)]
pub struct DpConfig {
    /// Object dictionary index, hex string ("0x6200") or decimal.
    pub index: String,
    #[serde(default)]
    pub sub: u8,
    pub name: String,
    #[serde(rename = "type")]
    pub ty: DpType,
    #[serde(default)]
    pub access: Access,
    pub value: JsonValue,
    /// GUI increment/decrement step for integer types.
    #[serde(default = "default_step")]
    pub step: i32,
}

fn default_step() -> i32 {
    1
}

#[derive(Deserialize, Clone, Copy, PartialEq, Eq, Debug)]
#[serde(rename_all = "lowercase")]
pub enum DpType {
    Bool,
    U8,
    I8,
    U16,
    I16,
    U32,
    I32,
    F32,
    Str,
}

impl DpType {
    pub fn name(self) -> &'static str {
        match self {
            Self::Bool => "bool",
            Self::U8 => "u8",
            Self::I8 => "i8",
            Self::U16 => "u16",
            Self::I16 => "i16",
            Self::U32 => "u32",
            Self::I32 => "i32",
            Self::F32 => "f32",
            Self::Str => "str",
        }
    }

    /// Value range for the integer types (clamped on write, like an OD limit).
    fn int_range(self) -> Option<(i32, i32)> {
        match self {
            Self::U8 => Some((0, u8::MAX as i32)),
            Self::I8 => Some((i8::MIN as i32, i8::MAX as i32)),
            Self::U16 => Some((0, u16::MAX as i32)),
            Self::I16 => Some((i16::MIN as i32, i16::MAX as i32)),
            // With Rhai `only_i32` the scripting side cannot express the full
            // u32 range; clamp to 0..=i32::MAX (imports wrap the bit pattern).
            Self::U32 => Some((0, i32::MAX)),
            Self::I32 => Some((i32::MIN, i32::MAX)),
            _ => None,
        }
    }

    /// Map a CANopenNode/CANopenEditor `dataType` string.
    fn from_canopen(s: &str) -> Option<Self> {
        Some(match s {
            "BOOLEAN" => Self::Bool,
            "UNSIGNED8" => Self::U8,
            "INTEGER8" => Self::I8,
            "UNSIGNED16" => Self::U16,
            "INTEGER16" => Self::I16,
            "UNSIGNED32" => Self::U32,
            "INTEGER32" => Self::I32,
            "REAL32" => Self::F32,
            "VISIBLE_STRING" => Self::Str,
            _ => return None, // DOMAIN, OCTET_STRING, 64-bit types: skipped
        })
    }
}

#[derive(Deserialize, Clone, Copy, PartialEq, Eq, Default, Debug)]
#[serde(rename_all = "lowercase")]
pub enum Access {
    Ro,
    Wo,
    #[default]
    Rw,
    Const,
}

impl Access {
    pub fn name(self) -> &'static str {
        match self {
            Self::Ro => "ro",
            Self::Wo => "wo",
            Self::Rw => "rw",
            Self::Const => "const",
        }
    }

    fn from_canopen(s: &str) -> Self {
        match s {
            "ACCESS_SDO_RW" => Self::Rw,
            "ACCESS_SDO_WO" => Self::Wo,
            _ => Self::Ro, // RO and "no SDO access"
        }
    }

    /// Writable from bus (RPDO) or GUI.
    pub fn bus_writable(self) -> bool {
        matches!(self, Self::Rw | Self::Wo)
    }
}

/// Initial value in JSON — type is resolved against the datapoint's `type`.
#[derive(Deserialize)]
#[serde(untagged)]
pub enum JsonValue {
    Bool(bool),
    Int(i64),
    Float(f64),
    Str(String),
}

#[derive(Deserialize)]
pub struct ScriptConfig {
    pub name: String,
    /// "once" (default when no period is given) or "cyclic".
    #[serde(default)]
    pub run: Option<String>,
    #[serde(default)]
    pub period_ms: Option<u32>,
    pub script: ScriptSource,
}

/// Rhai source as a single string or an array of lines.
#[derive(Deserialize)]
#[serde(untagged)]
pub enum ScriptSource {
    One(String),
    Lines(Vec<String>),
}

impl ScriptSource {
    fn join(&self) -> String {
        match self {
            Self::One(s) => s.clone(),
            Self::Lines(lines) => lines.join("\n"),
        }
    }
}

// CANopenNode/CANopenEditor JSON export (subset we need).

#[derive(Deserialize)]
struct CoFile {
    #[serde(default)]
    objects: BTreeMap<String, CoObject>,
}

#[derive(Deserialize)]
struct CoObject {
    #[serde(default)]
    disabled: bool,
    #[serde(default)]
    name: String,
    #[serde(default)]
    alias: String,
    #[serde(default, rename = "subObjects")]
    sub_objects: BTreeMap<String, CoSub>,
}

#[derive(Deserialize)]
struct CoSub {
    #[serde(default)]
    name: String,
    #[serde(default)]
    alias: String,
    #[serde(default, rename = "dataType")]
    data_type: String,
    #[serde(default)]
    sdo: String,
    #[serde(default, rename = "defaultValue")]
    default_value: String,
    #[serde(default, rename = "actualValue")]
    actual_value: String,
}

// ---------------------------------------------------------------------------
// Object dictionary (runtime)
// ---------------------------------------------------------------------------

#[derive(Clone, PartialEq, Debug)]
pub enum Value {
    Int(i32),
    Float(f32),
    Bool(bool),
    Str(String),
}

impl Value {
    fn from_json(v: &JsonValue) -> Self {
        match v {
            JsonValue::Bool(b) => Self::Bool(*b),
            JsonValue::Int(i) => Self::Int((*i).clamp(i32::MIN as i64, i32::MAX as i64) as i32),
            JsonValue::Float(f) => Self::Float(*f as f32),
            JsonValue::Str(s) => Self::Str(s.clone()),
        }
    }

    fn to_dynamic(&self) -> Dynamic {
        match self {
            Self::Int(i) => Dynamic::from(*i),
            Self::Float(f) => Dynamic::from(*f as rhai::FLOAT),
            Self::Bool(b) => Dynamic::from(*b),
            Self::Str(s) => Dynamic::from(s.clone()),
        }
    }

    pub fn to_display(&self) -> String {
        match self {
            Self::Int(i) => format!("{}", i),
            Self::Float(f) => format!("{}", f),
            Self::Bool(b) => format!("{}", b),
            Self::Str(s) => format!("\"{}\"", s),
        }
    }

    /// Wire encoding for the CAN "PDO" payload: (dtype, 4 bytes LE).
    fn to_wire(&self) -> Option<(u8, [u8; 4])> {
        match self {
            Self::Int(i) => Some((0, i.to_le_bytes())),
            Self::Float(f) => Some((1, f.to_le_bytes())),
            Self::Bool(b) => Some((2, [*b as u8, 0, 0, 0])),
            Self::Str(_) => None, // strings do not fit a classic frame; not sent
        }
    }

    fn from_wire(dtype: u8, raw: [u8; 4]) -> Self {
        match dtype {
            1 => Self::Float(f32::from_le_bytes(raw)),
            2 => Self::Bool(raw[0] != 0),
            _ => Self::Int(i32::from_le_bytes(raw)),
        }
    }
}

pub struct Datapoint {
    pub index: u16,
    pub sub: u8,
    pub name: String,
    pub ty: DpType,
    pub access: Access,
    pub value: Value,
    pub step: i32,
}

/// Who is writing: scripts are device-internal (may update `ro` inputs);
/// the bus (RPDO) and the GUI only get `rw`/`wo` entries.
#[derive(Clone, Copy, PartialEq)]
pub enum Writer {
    Script,
    Bus,
    Ui,
}

static OD: Mutex<Vec<Datapoint>> = Mutex::new(Vec::new());
static NODE_ID: AtomicU8 = AtomicU8::new(0);
static NODE_NAME: Mutex<String> = Mutex::new(String::new());

/// Bumped on every observable change (OD write, LED, print, CAN counter) so
/// the GUI only rebuilds labels when something happened.
static UI_GEN: AtomicU32 = AtomicU32::new(0);

// GUI bridge: virtual process data ("hardware" of the host node).
pub static LED_MASK: AtomicI32 = AtomicI32::new(0);
pub static BUTTON_HELD: AtomicBool = AtomicBool::new(false);
static LAST_PRINT: Mutex<String> = Mutex::new(String::new());
pub static CAN_TX_COUNT: AtomicU32 = AtomicU32::new(0);
pub static CAN_RX_COUNT: AtomicU32 = AtomicU32::new(0);
static CAN_STATUS: Mutex<String> = Mutex::new(String::new());

fn bump_gen() {
    UI_GEN.fetch_add(1, Ordering::Relaxed);
}

pub fn ui_generation() -> u32 {
    UI_GEN.load(Ordering::Relaxed)
}

pub fn node_id() -> u8 {
    NODE_ID.load(Ordering::Relaxed)
}

pub fn node_name() -> String {
    NODE_NAME.lock().unwrap().clone()
}

pub fn last_print() -> String {
    LAST_PRINT.lock().unwrap().clone()
}

pub fn can_status() -> String {
    CAN_STATUS.lock().unwrap().clone()
}

fn set_can_status(s: String) {
    *CAN_STATUS.lock().unwrap() = s;
    bump_gen();
}

fn od_with<R>(index: u16, sub: u8, f: impl FnOnce(&mut Datapoint) -> R) -> Option<R> {
    let mut od = OD.lock().unwrap();
    od.iter_mut().find(|dp| dp.index == index && dp.sub == sub).map(f)
}

fn od_with_name<R>(name: &str, f: impl FnOnce(&mut Datapoint) -> R) -> Option<R> {
    let mut od = OD.lock().unwrap();
    od.iter_mut().find(|dp| dp.name == name).map(f)
}

/// Coerce a script/JSON/bus value to the datapoint's declared type.
fn coerce(ty: DpType, v: Value) -> Result<Value, String> {
    match ty {
        DpType::Bool => match v {
            Value::Bool(b) => Ok(Value::Bool(b)),
            Value::Int(i) => Ok(Value::Bool(i != 0)),
            _ => Err("expected bool".to_string()),
        },
        DpType::F32 => match v {
            Value::Float(f) => Ok(Value::Float(f)),
            Value::Int(i) => Ok(Value::Float(i as f32)),
            _ => Err("expected f32".to_string()),
        },
        DpType::Str => match v {
            Value::Str(s) => Ok(Value::Str(s)),
            Value::Int(i) => Ok(Value::Str(format!("{}", i))),
            Value::Float(f) => Ok(Value::Str(format!("{}", f))),
            Value::Bool(b) => Ok(Value::Str(format!("{}", b))),
        },
        _ => {
            let (min, max) = ty.int_range().unwrap();
            let i = match v {
                Value::Int(i) => i,
                Value::Bool(b) => b as i32,
                Value::Float(f) => f as i32,
                Value::Str(_) => return Err(format!("expected {}", ty.name())),
            };
            let clamped = i.clamp(min, max);
            if clamped != i {
                warn!("od: value {} clamped to {} ({})", i, clamped, ty.name());
            }
            Ok(Value::Int(clamped))
        }
    }
}

// "TPDO" queue towards the SocketCAN TX thread.
struct Tpdo {
    index: u16,
    sub: u8,
    dtype: u8,
    raw: [u8; 4],
}

static TPDO_TX: OnceLock<mpsc::SyncSender<Tpdo>> = OnceLock::new();

/// Write a datapoint; logs every change and queues a "TPDO" CAN frame.
fn write_dp(dp: &mut Datapoint, new: Value, writer: Writer) -> Result<(), String> {
    if dp.access == Access::Const {
        return Err(format!("od 0x{:04x}.{:02x} '{}' is const", dp.index, dp.sub, dp.name));
    }
    if writer != Writer::Script && !dp.access.bus_writable() {
        return Err(format!(
            "od 0x{:04x}.{:02x} '{}' is read-only from {}",
            dp.index,
            dp.sub,
            dp.name,
            if writer == Writer::Bus { "the bus" } else { "the UI" }
        ));
    }
    let coerced =
        coerce(dp.ty, new).map_err(|e| format!("od 0x{:04x}.{:02x} '{}': {}", dp.index, dp.sub, dp.name, e))?;
    if coerced != dp.value {
        info!(
            "od 0x{:04x}.{:02x} {} = {} (was {})",
            dp.index,
            dp.sub,
            dp.name,
            coerced.to_display(),
            dp.value.to_display()
        );
        if let Some(tx) = TPDO_TX.get() {
            if let Some((dtype, raw)) = coerced.to_wire() {
                let msg = Tpdo {
                    index: dp.index,
                    sub: dp.sub,
                    dtype,
                    raw,
                };
                if tx.try_send(msg).is_err() {
                    warn!("can: tx queue full, tpdo dropped");
                }
            }
        }
        dp.value = coerced;
        bump_gen();
    }
    Ok(())
}

fn dump_od() {
    let od = OD.lock().unwrap();
    info!("object dictionary ({} entries):", od.len());
    for dp in od.iter() {
        info!(
            "  0x{:04x}.{:02x} {:<28} {:>5} {:<5} = {}",
            dp.index,
            dp.sub,
            dp.name,
            dp.ty.name(),
            dp.access.name(),
            dp.value.to_display()
        );
    }
}

/// Parse an OD index / COB-ID given as "0x6200" (hex) or "25088" (decimal).
fn parse_index(s: &str) -> Option<u16> {
    let s = s.trim();
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u16::from_str_radix(hex, 16).ok()
    } else {
        s.parse().ok()
    }
}

/// Parse "0x1000-0x1FFF" or a single index into an inclusive range.
fn parse_range(s: &str) -> Option<(u16, u16)> {
    match s.split_once('-') {
        Some((a, b)) => Some((parse_index(a)?, parse_index(b)?)),
        None => {
            let i = parse_index(s)?;
            Some((i, i))
        }
    }
}

/// CANopenNode default/actual value: hex, decimal, `$NODEID+0x…`, or empty.
fn parse_co_number(s: &str, node_id: u8) -> Option<i64> {
    let s = s.trim();
    if s.is_empty() {
        return Some(0);
    }
    let (base, rest) = if let Some(r) = s.to_uppercase().strip_prefix("$NODEID") {
        (node_id as i64, r.trim_start_matches('+').to_string())
    } else {
        (0, s.to_string())
    };
    if rest.is_empty() {
        return Some(base);
    }
    let v = if let Some(hex) = rest.strip_prefix("0X").or_else(|| rest.strip_prefix("0x")) {
        i64::from_str_radix(hex, 16).ok()?
    } else {
        rest.parse::<i64>().ok()?
    };
    Some(base + v)
}

fn co_initial_value(ty: DpType, sub: &CoSub, node_id: u8) -> Result<Value, String> {
    let raw = if !sub.actual_value.trim().is_empty() {
        sub.actual_value.trim()
    } else {
        sub.default_value.trim()
    };
    match ty {
        DpType::Str => Ok(Value::Str(raw.to_string())),
        DpType::Bool => Ok(Value::Bool(matches!(raw, "1" | "true" | "TRUE"))),
        DpType::F32 => Ok(Value::Float(raw.parse::<f32>().unwrap_or(0.0))),
        DpType::U32 => {
            // Keep the 32-bit pattern (COB-IDs use bit 31); display may be negative.
            let v = parse_co_number(raw, node_id).ok_or_else(|| format!("bad value '{}'", raw))?;
            Ok(Value::Int(v as u32 as i32))
        }
        _ => {
            let v = parse_co_number(raw, node_id).ok_or_else(|| format!("bad value '{}'", raw))?;
            coerce(ty, Value::Int(v.clamp(i32::MIN as i64, i32::MAX as i64) as i32))
        }
    }
}

/// Import datapoints from a CANopenNode/CANopenEditor JSON export.
fn import_canopen_od(
    path: &Path,
    include: &[(u16, u16)],
    node_id: u8,
) -> Result<Vec<Datapoint>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {}", path.display(), e))?;
    let co: CoFile = serde_json::from_str(&text).map_err(|e| format!("{}: {}", path.display(), e))?;

    let mut out = Vec::new();
    for (idx_str, obj) in &co.objects {
        if obj.disabled {
            continue;
        }
        let Ok(index) = u16::from_str_radix(idx_str, 16) else {
            warn!("od import: bad index '{}'", idx_str);
            continue;
        };
        if !include.iter().any(|(a, b)| index >= *a && index <= *b) {
            continue;
        }
        let single_var = obj.sub_objects.len() == 1;
        for (sub_str, sub) in &obj.sub_objects {
            let Ok(sub_idx) = u8::from_str_radix(sub_str, 16) else {
                continue;
            };
            let Some(ty) = DpType::from_canopen(&sub.data_type) else {
                debug!(
                    "od import: 0x{:04x}.{:02x} skipped (dataType {})",
                    index, sub_idx, sub.data_type
                );
                continue;
            };
            let access = Access::from_canopen(&sub.sdo);
            let value = match co_initial_value(ty, sub, node_id) {
                Ok(v) => v,
                Err(e) => {
                    warn!("od import: 0x{:04x}.{:02x}: {}", index, sub_idx, e);
                    continue;
                }
            };
            let obj_name = if obj.alias.is_empty() { &obj.name } else { &obj.alias };
            let sub_name = if sub.alias.is_empty() { &sub.name } else { &sub.alias };
            let name = if single_var || sub_name.is_empty() {
                obj_name.clone()
            } else {
                format!("{}.{}", obj_name, sub_name)
            };
            out.push(Datapoint {
                index,
                sub: sub_idx,
                name,
                ty,
                access,
                value,
                step: 1,
            });
        }
    }
    Ok(out)
}

/// Load a node config file (compact PoC schema).
pub fn load_config(path: &Path) -> Result<NodeConfig, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {}", path.display(), e))?;
    serde_json::from_str(&text).map_err(|e| format!("{}: {}", path.display(), e))
}

const DEFAULT_INCLUDE: &[&str] = &["0x1000-0x1001", "0x1008-0x100A", "0x1017", "0x2000-0x9FFF"];

/// Build the OD from `od_file` import (optional) plus inline `datapoints`,
/// store it in the global OD and publish node identity.
pub fn init_node(cfg: &NodeConfig, config_dir: &Path) -> Result<(), String> {
    NODE_ID.store(cfg.node_id, Ordering::Relaxed);
    *NODE_NAME.lock().unwrap() = cfg.name.clone();

    let mut od: Vec<Datapoint> = Vec::new();

    if let Some(od_file) = &cfg.od_file {
        let ranges: Vec<(u16, u16)> = cfg
            .include
            .as_ref()
            .map(|v| v.iter().map(String::as_str).collect::<Vec<_>>())
            .unwrap_or_else(|| DEFAULT_INCLUDE.to_vec())
            .iter()
            .filter_map(|s| parse_range(s))
            .collect();
        let path: PathBuf = config_dir.join(od_file);
        let imported = import_canopen_od(&path, &ranges, cfg.node_id)?;
        info!("od import: {} datapoints from {}", imported.len(), path.display());
        od.extend(imported);
    }

    for d in &cfg.datapoints {
        let index = parse_index(&d.index).ok_or_else(|| format!("invalid index '{}'", d.index))?;
        let value = coerce(d.ty, Value::from_json(&d.value)).map_err(|e| format!("'{}': {}", d.name, e))?;
        // Inline datapoints override imported ones with the same index/sub.
        od.retain(|dp| !(dp.index == index && dp.sub == d.sub));
        od.push(Datapoint {
            index,
            sub: d.sub,
            name: d.name.clone(),
            ty: d.ty,
            access: d.access,
            value,
            step: d.step,
        });
    }

    od.sort_by_key(|dp| (dp.index, dp.sub));

    let mut seen = std::collections::HashSet::new();
    for dp in &od {
        if !seen.insert(dp.name.clone()) {
            warn!("od: duplicate name '{}' — get()/set() use the first entry", dp.name);
        }
    }

    info!("node '{}' (id {}): {} datapoints", cfg.name, cfg.node_id, od.len());
    *OD.lock().unwrap() = od;
    bump_gen();
    Ok(())
}

// ---------------------------------------------------------------------------
// GUI bridge
// ---------------------------------------------------------------------------

/// Row snapshot for the OxivGL view.
pub struct DpRow {
    pub index: u16,
    pub sub: u8,
    pub name: String,
    pub ty: DpType,
    pub access: Access,
    pub value: String,
    pub step: i32,
}

pub fn od_rows() -> Vec<DpRow> {
    let od = OD.lock().unwrap();
    od.iter()
        .map(|dp| DpRow {
            index: dp.index,
            sub: dp.sub,
            name: dp.name.clone(),
            ty: dp.ty,
            access: dp.access,
            value: dp.value.to_display(),
            step: dp.step,
        })
        .collect()
}

/// GUI write: add `delta` to an integer datapoint (Writer::Ui semantics).
pub fn ui_adjust(index: u16, sub: u8, delta: i32) {
    let result = od_with(index, sub, |dp| {
        let new = match &dp.value {
            Value::Int(i) => Value::Int(i.saturating_add(delta)),
            Value::Float(f) => Value::Float(f + delta as f32),
            Value::Bool(b) => Value::Bool(!b),
            Value::Str(_) => return Err("string not editable".to_string()),
        };
        write_dp(dp, new, Writer::Ui)
    });
    match result {
        None => warn!("ui: 0x{:04x}.{:02x} not in object dictionary", index, sub),
        Some(Err(e)) => warn!("ui: {}", e),
        Some(Ok(())) => {}
    }
}

/// GUI write: toggle a bool datapoint (Writer::Ui semantics).
pub fn ui_toggle(index: u16, sub: u8) {
    ui_adjust(index, sub, 1);
}

// ---------------------------------------------------------------------------
// Rhai engine + script scheduler
// ---------------------------------------------------------------------------

type RhaiResult<T> = Result<T, Box<EvalAltResult>>;

fn od_write_rhai(index: i32, sub: i32, v: Value) -> RhaiResult<i32> {
    match od_with(index as u16, sub as u8, |dp| write_dp(dp, v, Writer::Script)) {
        None => Err(format!("od_write: 0x{:04x}.{:02x} not in object dictionary", index, sub).into()),
        Some(Err(e)) => Err(e.into()),
        Some(Ok(())) => Ok(0),
    }
}

fn set_rhai(name: &str, v: Value) -> RhaiResult<i32> {
    match od_with_name(name, |dp| write_dp(dp, v, Writer::Script)) {
        None => Err(format!("set: '{}' not in object dictionary", name).into()),
        Some(Err(e)) => Err(e.into()),
        Some(Ok(())) => Ok(0),
    }
}

fn register_api(engine: &mut Engine) {
    // Object dictionary — by index/subindex.
    engine.register_fn("od_read", |index: i32, sub: i32| -> RhaiResult<Dynamic> {
        od_with(index as u16, sub as u8, |dp| dp.value.to_dynamic())
            .ok_or_else(|| format!("od_read: 0x{:04x}.{:02x} not in object dictionary", index, sub).into())
    });
    engine.register_fn("od_write", |i: i32, s: i32, v: i32| od_write_rhai(i, s, Value::Int(v)));
    engine.register_fn("od_write", |i: i32, s: i32, v: bool| od_write_rhai(i, s, Value::Bool(v)));
    engine.register_fn("od_write", |i: i32, s: i32, v: rhai::FLOAT| {
        od_write_rhai(i, s, Value::Float(v as f32))
    });
    engine.register_fn("od_write", |i: i32, s: i32, v: &str| od_write_rhai(i, s, Value::Str(v.to_string())));

    // Object dictionary — by datapoint name.
    engine.register_fn("get", |name: &str| -> RhaiResult<Dynamic> {
        od_with_name(name, |dp| dp.value.to_dynamic())
            .ok_or_else(|| format!("get: '{}' not in object dictionary", name).into())
    });
    engine.register_fn("set", |name: &str, v: i32| set_rhai(name, Value::Int(v)));
    engine.register_fn("set", |name: &str, v: bool| set_rhai(name, Value::Bool(v)));
    engine.register_fn("set", |name: &str, v: rhai::FLOAT| set_rhai(name, Value::Float(v as f32)));
    engine.register_fn("set", |name: &str, v: &str| set_rhai(name, Value::Str(v.to_string())));

    engine.register_fn("od_dump", || -> i32 {
        dump_od();
        0
    });

    // Node identity + time.
    engine.register_fn("node_id", || -> i32 { NODE_ID.load(Ordering::Relaxed) as i32 });
    engine.register_fn("node_name", || -> String { node_name() });
    engine.register_fn("uptime_ms", || -> i32 { embassy_time::Instant::now().as_millis() as i32 });
    engine.register_fn("sleep", |ms: i32| -> i32 {
        // Blocks the UI thread — keep short; prefer cyclic scripts over sleeps.
        std::thread::sleep(std::time::Duration::from_millis(ms.clamp(0, 500) as u64));
        0
    });

    // Virtual process data, rendered by the OxivGL view.
    engine.register_fn("led", |index: i32, on: bool| -> i32 {
        let mask = LED_MASK.load(Ordering::Relaxed);
        let bit = 1 << index.clamp(0, 30);
        let new = if on { mask | bit } else { mask & !bit };
        if new != mask {
            LED_MASK.store(new, Ordering::Relaxed);
            bump_gen();
        }
        if on { 1 } else { 0 }
    });
    engine.register_fn("led", |index: i32, on: i32| -> i32 {
        let mask = LED_MASK.load(Ordering::Relaxed);
        let bit = 1 << index.clamp(0, 30);
        let new = if on != 0 { mask | bit } else { mask & !bit };
        if new != mask {
            LED_MASK.store(new, Ordering::Relaxed);
            bump_gen();
        }
        if on != 0 { 1 } else { 0 }
    });
    engine.register_fn("leds", |mask: i32| -> i32 {
        if LED_MASK.swap(mask, Ordering::Relaxed) != mask {
            bump_gen();
        }
        mask
    });
    engine.register_fn("button", || -> i32 { BUTTON_HELD.load(Ordering::Relaxed) as i32 });

    engine.on_print(|s| {
        info!("[script] {}", s);
        *LAST_PRINT.lock().unwrap() = s.to_string();
        bump_gen();
    });
    engine.on_debug(|s, _src, _pos| debug!("[script] {}", s));
}

struct CyclicScript {
    name: String,
    period: embassy_time::Duration,
    next: embassy_time::Instant,
    ast: AST,
}

/// Owns the Rhai engine and the cyclic scripts. Not `Send` — poll it from the
/// main (LVGL) task only.
pub struct NodeRuntime {
    engine: Engine,
    cyclics: Vec<CyclicScript>,
}

impl NodeRuntime {
    /// Build the engine, run all `once` scripts, compile the `cyclic` ones.
    pub fn new(cfg: &NodeConfig) -> Self {
        let mut engine = Engine::new_raw();
        BasicMathPackage::new().register_into_engine(&mut engine);
        BasicIteratorPackage::new().register_into_engine(&mut engine);
        BasicStringPackage::new().register_into_engine(&mut engine);
        MoreStringPackage::new().register_into_engine(&mut engine);
        engine.set_max_operations(MAX_SCRIPT_OPS);
        register_api(&mut engine);

        let mut cyclics = Vec::new();
        let now = embassy_time::Instant::now();
        for s in &cfg.scripts {
            let cyclic = match s.run.as_deref() {
                Some("cyclic") => true,
                Some("once") => false,
                None => s.period_ms.is_some(),
                Some(other) => {
                    error!("script '{}': unknown run mode '{}' — skipped", s.name, other);
                    continue;
                }
            };
            let src = s.script.join();
            if cyclic {
                let period = embassy_time::Duration::from_millis(s.period_ms.unwrap_or(1000).max(10) as u64);
                match engine.compile(&src) {
                    Ok(ast) => cyclics.push(CyclicScript {
                        name: s.name.clone(),
                        period,
                        next: now,
                        ast,
                    }),
                    Err(e) => error!("script '{}': compile error: {}", s.name, e),
                }
            } else {
                info!("running once script '{}'", s.name);
                if let Err(e) = engine.run(&src) {
                    error!("script '{}': {}", s.name, e);
                }
            }
        }
        info!("scheduler: {} cyclic scripts", cyclics.len());
        Self { engine, cyclics }
    }

    /// Run all due cyclic scripts (PLC-style, to completion). Call once per
    /// GUI frame — periods are quantized to the frame rate.
    pub fn poll(&mut self) {
        let now = embassy_time::Instant::now();
        for c in self.cyclics.iter_mut() {
            if now >= c.next {
                if let Err(e) = self.engine.run_ast(&c.ast) {
                    error!("script '{}': {}", c.name, e);
                }
                c.next += c.period;
                if c.next <= now {
                    // Fell behind — skip missed cycles instead of bursting.
                    c.next = now + c.period;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// SocketCAN: "PDO" TX/RX + CANopen bootup/heartbeat
// ---------------------------------------------------------------------------

/// Received "RPDO" payload → OD write with bus semantics.
fn handle_rpdo(data: &[u8]) {
    if data.len() < 8 {
        warn!("can: rpdo too short ({} bytes)", data.len());
        return;
    }
    let index = u16::from_le_bytes([data[0], data[1]]);
    let sub = data[2];
    let value = Value::from_wire(data[3], [data[4], data[5], data[6], data[7]]);
    match od_with(index, sub, |dp| write_dp(dp, value, Writer::Bus)) {
        None => warn!("can: rpdo for unknown od 0x{:04x}.{:02x}", index, sub),
        Some(Err(e)) => warn!("can: rpdo rejected: {}", e),
        Some(Ok(())) => {}
    }
}

/// Producer heartbeat period from OD 0x1017.00 (ms); 0 = disabled.
fn heartbeat_period_ms() -> u64 {
    od_with(0x1017, 0, |dp| match dp.value {
        Value::Int(i) => i.max(0) as u64,
        _ => 0,
    })
    .unwrap_or(0)
}

/// Start the SocketCAN TX/RX threads. Degrades gracefully (UI + scripts keep
/// running) when the interface cannot be opened.
pub fn start_can(cfg: &NodeConfig) {
    use socketcan::{CanFrame, CanSocket, EmbeddedFrame, Socket, StandardId};

    let Some(can) = &cfg.can else {
        info!("can: not configured");
        set_can_status("CAN: aus".to_string());
        return;
    };
    if can.mode != "socketcan" {
        info!("can: mode '{}' — disabled", can.mode);
        set_can_status("CAN: aus".to_string());
        return;
    }

    let tpdo_id = parse_index(&can.tpdo).unwrap_or(0x180) + cfg.node_id as u16;
    let rpdo_id = parse_index(&can.rpdo).unwrap_or(0x200) + cfg.node_id as u16;
    let hb_id = 0x700u16 + cfg.node_id as u16;
    let channel = can.channel.clone();
    let heartbeat = can.heartbeat;

    let tx_socket = match CanSocket::open(&channel) {
        Ok(s) => s,
        Err(e) => {
            warn!("can: cannot open {} ({}) — running without CAN", channel, e);
            set_can_status(format!("CAN: {} nicht verfügbar", channel));
            return;
        }
    };
    let rx_socket = match CanSocket::open(&channel) {
        Ok(s) => s,
        Err(e) => {
            warn!("can: cannot open {} ({}) — running without CAN", channel, e);
            set_can_status(format!("CAN: {} nicht verfügbar", channel));
            return;
        }
    };

    let (tx, rx) = mpsc::sync_channel::<Tpdo>(64);
    let _ = TPDO_TX.set(tx);
    set_can_status(format!(
        "CAN: {} · TPDO 0x{:03x} · RPDO 0x{:03x} · HB 0x{:03x}",
        channel, tpdo_id, rpdo_id, hb_id
    ));
    info!(
        "can: {} tpdo=0x{:03x} rpdo=0x{:03x} heartbeat=0x{:03x}",
        channel, tpdo_id, rpdo_id, hb_id
    );

    // TX thread: TPDO queue + CANopen bootup/heartbeat.
    std::thread::spawn(move || {
        let send = |id: u16, data: &[u8]| -> bool {
            let Some(sid) = StandardId::new(id) else { return false };
            let Some(frame) = CanFrame::new(sid, data) else { return false };
            let ok = tx_socket.write_frame(&frame).is_ok();
            if ok {
                CAN_TX_COUNT.fetch_add(1, Ordering::Relaxed);
                bump_gen();
            }
            ok
        };

        if heartbeat {
            // CANopen bootup message (NMT state 0 = initialising).
            send(hb_id, &[0x00]);
        }
        let mut next_hb = std::time::Instant::now();

        loop {
            match rx.recv_timeout(std::time::Duration::from_millis(20)) {
                Ok(pdo) => {
                    let mut data = [0u8; 8];
                    data[0..2].copy_from_slice(&pdo.index.to_le_bytes());
                    data[2] = pdo.sub;
                    data[3] = pdo.dtype;
                    data[4..8].copy_from_slice(&pdo.raw);
                    if send(tpdo_id, &data) {
                        debug!("can: tpdo 0x{:03x} -> 0x{:04x}.{:02x}", tpdo_id, pdo.index, pdo.sub);
                    } else {
                        warn!("can: tpdo write failed");
                    }
                }
                Err(mpsc::RecvTimeoutError::Timeout) => {}
                Err(mpsc::RecvTimeoutError::Disconnected) => return,
            }
            if heartbeat {
                let period = heartbeat_period_ms();
                if period > 0 && std::time::Instant::now() >= next_hb {
                    // NMT state 0x05 = operational.
                    send(hb_id, &[0x05]);
                    next_hb = std::time::Instant::now() + std::time::Duration::from_millis(period.max(50));
                }
            }
        }
    });

    // RX thread: RPDO → OD (bus access checks apply).
    std::thread::spawn(move || {
        let timeout = std::time::Duration::from_millis(100);
        loop {
            match rx_socket.read_frame_timeout(timeout) {
                Ok(frame) => {
                    let id = match frame.id() {
                        socketcan::Id::Standard(id) => id.as_raw(),
                        _ => continue,
                    };
                    CAN_RX_COUNT.fetch_add(1, Ordering::Relaxed);
                    bump_gen();
                    if id == rpdo_id {
                        handle_rpdo(frame.data());
                    } else {
                        // Own TPDO/heartbeat loop back via the second socket.
                        debug!("can: rx 0x{:03x} ignored", id);
                    }
                }
                Err(_) => {}
            }
        }
    });
}

// ---------------------------------------------------------------------------
// Tests (no LVGL/SDL needed): schema, scripts, access semantics, OD import
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Everything shares the global OD, so keep all runtime checks in ONE test.
    #[test]
    fn node_runtime_end_to_end() {
        let json = r#"{
            "node_id": 16,
            "name": "TestNode",
            "datapoints": [
                { "index": "0x1017", "sub": 0, "name": "heartbeat_ms", "type": "u16", "access": "rw", "value": 2000 },
                { "index": "0x2100", "sub": 0, "name": "cycle_counter", "type": "i32", "access": "ro", "value": 0 },
                { "index": "0x2101", "sub": 0, "name": "blink_on", "type": "bool", "access": "rw", "value": true },
                { "index": "0x1008", "sub": 0, "name": "device_name", "type": "str", "access": "const", "value": "Test" },
                { "index": "0x6000", "sub": 1, "name": "din_button", "type": "u8", "access": "ro", "value": 0 },
                { "index": "0x6200", "sub": 1, "name": "dout_leds", "type": "u8", "access": "rw", "value": 0 }
            ],
            "scripts": [
                { "name": "init", "run": "once", "script": "set(\"dout_leds\", 1);" },
                { "name": "logic", "run": "cyclic", "period_ms": 10, "script": [
                    "let c = od_read(0x2100, 0) + 1;",
                    "od_write(0x2100, 0, c);",
                    "od_write(0x6000, 1, button());",
                    "if get(\"blink_on\") { set(\"dout_leds\", 1 << (c % 3)); } else { set(\"dout_leds\", 0); }",
                    "leds(get(\"dout_leds\"));"
                ] }
            ]
        }"#;
        let cfg: NodeConfig = serde_json::from_str(json).expect("config parses");
        init_node(&cfg, Path::new(".")).expect("init");
        assert_eq!(node_id(), 16);

        let mut rt = NodeRuntime::new(&cfg);
        // init script ran:
        assert_eq!(od_with_name("dout_leds", |dp| dp.value.clone()), Some(Value::Int(1)));

        // Two scheduler polls with a gap > period.
        rt.poll();
        std::thread::sleep(std::time::Duration::from_millis(15));
        rt.poll();
        let counter = od_with_name("cycle_counter", |dp| dp.value.clone()).unwrap();
        assert_eq!(counter, Value::Int(2));
        assert_ne!(LED_MASK.load(Ordering::Relaxed), 0);

        // Virtual button feeds din via script.
        BUTTON_HELD.store(true, Ordering::Relaxed);
        std::thread::sleep(std::time::Duration::from_millis(15));
        rt.poll();
        assert_eq!(od_with_name("din_button", |dp| dp.value.clone()), Some(Value::Int(1)));

        // RPDO: rw write accepted, ro write rejected.
        let mut frame = [0u8; 8];
        frame[0..2].copy_from_slice(&0x2101u16.to_le_bytes());
        frame[3] = 2; // bool false
        handle_rpdo(&frame);
        assert_eq!(od_with_name("blink_on", |dp| dp.value.clone()), Some(Value::Bool(false)));
        let mut frame = [0u8; 8];
        frame[0..2].copy_from_slice(&0x6000u16.to_le_bytes());
        frame[2] = 1;
        frame[4] = 1;
        handle_rpdo(&frame); // must be rejected (ro)
        assert_eq!(od_with_name("din_button", |dp| dp.value.clone()), Some(Value::Int(1)));

        // UI: const not editable, rw editable with step.
        ui_adjust(0x1008, 0, 1); // const → rejected, no panic
        ui_adjust(0x1017, 0, 100);
        assert_eq!(od_with_name("heartbeat_ms", |dp| dp.value.clone()), Some(Value::Int(2100)));
        assert_eq!(heartbeat_period_ms(), 2100);

        // Clamping.
        ui_adjust(0x6200, 1, 1000);
        assert_eq!(od_with_name("dout_leds", |dp| dp.value.clone()), Some(Value::Int(255)));
    }

    #[test]
    fn canopen_import_parses_ds301_profile() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("od/DS301_profile.json");
        let ranges = [(0x1000u16, 0x1001u16), (0x1014, 0x1014), (0x1017, 0x1017)];
        let od = import_canopen_od(&path, &ranges, 48).expect("import");
        assert!(od.iter().any(|dp| dp.index == 0x1000 && dp.ty == DpType::U32));
        assert!(od.iter().any(|dp| dp.index == 0x1001 && dp.access == Access::Ro));
        let hb = od.iter().find(|dp| dp.index == 0x1017).expect("0x1017 present");
        assert_eq!(hb.access, Access::Rw);
        // $NODEID+0x80 with node 48 → 0x80 + 48
        let emcy = od.iter().find(|dp| dp.index == 0x1014).expect("0x1014 present");
        assert_eq!(emcy.value, Value::Int(0x80 + 48));
    }

    #[test]
    fn co_number_parsing() {
        assert_eq!(parse_co_number("0x80", 16), Some(0x80));
        assert_eq!(parse_co_number("$NODEID+0x180", 16), Some(0x190));
        assert_eq!(parse_co_number("$NODEID", 5), Some(5));
        assert_eq!(parse_co_number("", 5), Some(0));
        assert_eq!(parse_co_number("42", 5), Some(42));
        assert_eq!(parse_range("0x1000-0x1FFF"), Some((0x1000, 0x1FFF)));
        assert_eq!(parse_range("0x1017"), Some((0x1017, 0x1017)));
    }
}
