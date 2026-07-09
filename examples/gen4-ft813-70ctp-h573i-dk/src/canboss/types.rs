//! Gemeinsame Datentypen des CANbossTouch-Rust-Ports.
//!
//! Von Firmware und Host-Port (`examples/canboss-touch-host`) geteilt —
//! keine Plattformabhaengigkeiten, damit die generierten Tabellen und die
//! Views auf beiden Zielen dieselben Typen sehen.

/// CANopen-Datentyp eines EDS-Datenpunkts (CiA 301 `DataType`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DType {
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
    /// Feste Nutzdatenlaenge in Bytes (0 = variabel, z.B. Strings).
    pub fn size(self) -> usize {
        match self {
            Self::Bool | Self::I8 | Self::U8 => 1,
            Self::I16 | Self::U16 => 2,
            Self::I32 | Self::U32 | Self::F32 => 4,
            Self::I64 | Self::U64 => 8,
            Self::Str | Self::Octet => 0,
        }
    }

    pub fn is_signed(self) -> bool {
        matches!(self, Self::I8 | Self::I16 | Self::I32 | Self::I64)
    }
}

/// Zugriffsart (`AccessType` aus der EDS).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Access {
    Ro,
    Wo,
    Rw,
}

/// Vom Generator gewaehltes Widget (verfeinerte Zuordnung wie im C-Port).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WidgetKind {
    /// Nur-Lese-Wertanzeige
    Value,
    /// Nur-Lese-Wert + Balken im Limitbereich
    Bar,
    /// BOOLEAN rw oder Integer rw mit Limits 0..1
    Switch,
    /// Integer rw mit EDS-Limit-Spanne <= 2000
    Slider,
    /// Integer rw
    Spinbox,
    /// REAL32 rw als Festkomma-Spinbox (Wert x1000, 3 Nachkommastellen)
    SpinboxF32,
    /// VISIBLE_STRING rw mit Bildschirmtastatur
    Text,
}

/// Festkomma-Skalierung der REAL32-Spinbox.
pub const F32_SCALE: f32 = 1000.0;

/// Ein Datenpunkt = ein (Index, Subindex)-Blatt aus der EDS.
pub struct Dp {
    pub index: u16,
    pub sub: u8,
    pub name: &'static str,
    pub dtype: DType,
    pub access: Access,
    pub widget: WidgetKind,
    /// Grenzen fuer Eingabe-Widgets (Integer; REAL32 als Festkomma x1000)
    pub min: i32,
    pub max: i32,
    pub has_limits: bool,
}

/// Ein CANopen-Knoten aus eds/network.json.
pub struct NodeDesc {
    pub node_id: u8,
    pub name: &'static str,
    pub eds_file: &'static str,
    pub dps: &'static [Dp],
}

/// Little-endian-Integer aus SDO-Nutzdaten dekodieren (mit Vorzeichen).
pub fn decode_int(data: &[u8], signed: bool) -> i64 {
    let len = data.len().min(8);
    let mut u: u64 = 0;
    for (i, b) in data.iter().take(len).enumerate() {
        u |= (*b as u64) << (8 * i);
    }
    if signed && len > 0 && len < 8 && (data[len - 1] & 0x80) != 0 {
        u |= !0u64 << (8 * len); // Vorzeichen erweitern
    }
    u as i64
}

/// Integer little-endian in `len` Bytes kodieren.
pub fn encode_int(out: &mut [u8], len: usize, v: i64) {
    for (i, b) in out.iter_mut().take(len).enumerate() {
        *b = (v >> (8 * i)) as u8;
    }
}
