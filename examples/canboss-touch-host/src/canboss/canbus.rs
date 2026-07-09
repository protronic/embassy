//! Host-CAN-Backend: In-Memory-**SDO-Server-Mock** statt echter FDCAN.
//!
//! Erfuellt dieselbe Schnittstelle wie das Firmware-`canbus`
//! ([`send_request_raw`]), damit der geteilte SDO-Client
//! ([`crate::canboss::sdo`]) unveraendert laeuft. Statt Frames auf den Bus zu
//! legen, beantwortet dieser Mock die Client-Requests wie ein CANopen-Knoten:
//! Upload-Requests liefern simulierte, zum EDS-Datentyp passende Werte
//! (Dreieckskurven im Limitbereich, BOOL-Toggle, REAL32, "Host …"-Strings),
//! Downloads werden gespeichert und danach zurueckgelesen. So exerziert der
//! Host-Build den **echten** Client-Protokollcode (expedited **und**
//! segmented) und rendert vollstaendig befuellte Screens.

use std::sync::Mutex;
use std::sync::atomic::{AtomicU32, Ordering};

use crate::canboss::{DType, Dp, NODES, sdo};

/// Geschriebene Werte (überschreiben die Simulation beim Zurücklesen).
static STORE: Mutex<Vec<(u8, u16, u8, Vec<u8>)>> = Mutex::new(Vec::new());
/// Laufender Zaehler, damit simulierte Werte sich ueber die Zeit bewegen.
static TICK: AtomicU32 = AtomicU32::new(0);

/// Laufender segmentierter Upload (Server -> Client): Restdaten + Toggle.
static UP_XFER: Mutex<Option<(Vec<u8>, usize, u8)>> = Mutex::new(None);
/// Laufender segmentierter Download (Client -> Server): (node,index,sub) + Puffer + Toggle.
static DOWN_XFER: Mutex<Option<(u8, u16, u8, Vec<u8>, u8)>> = Mutex::new(None);

/// SDO-Request des geteilten Clients — vom Mock-Server beantwortet.
///
/// Signaturgleich zum Firmware-`canbus`: der Client kennt nur diese Funktion.
/// Die Antwort geht synchron ueber [`sdo::on_response`] in die Antwort-Queue,
/// die der Client-Worker anschliessend abholt (ein Transfer nach dem anderen).
pub async fn send_request_raw(id: u16, payload: &[u8; 8]) -> Result<(), ()> {
    if !(0x600..=0x67F).contains(&id) {
        return Ok(());
    }
    let node = (id - 0x600) as u8;
    let cmd = payload[0];
    let ccs = cmd >> 5; // command specifier des Clients

    match ccs {
        2 => upload_init(node, payload),      // initiate upload request (0x40)
        3 => upload_segment(node),            // upload segment request (0x60)
        1 => download_init(node, payload),    // initiate download request (0x2x)
        0 => download_segment(node, payload), // download segment request (0x0x)
        _ => {}
    }
    Ok(())
}

fn reply(node: u8, data: [u8; 8]) {
    sdo::on_response(0x580 + node as u16, &data);
}

fn abort(node: u8, index: u16, sub: u8, code: u32) {
    let mut d = [0u8; 8];
    d[0] = 0x80;
    d[1] = index as u8;
    d[2] = (index >> 8) as u8;
    d[3] = sub;
    d[4..8].copy_from_slice(&code.to_le_bytes());
    reply(node, d);
}

// ── Upload (Server -> Client) ───────────────────────────────────────────────

fn upload_init(node: u8, payload: &[u8; 8]) {
    let index = u16::from_le_bytes([payload[1], payload[2]]);
    let sub = payload[3];
    let value = get_or_simulate(node, index, sub);

    if value.len() <= 4 {
        // Expedited-Upload-Antwort (scs=2, e=1, s=1): 0x43 | ((4-len)<<2)
        let n = value.len();
        let mut d = [0u8; 8];
        d[0] = 0x43 | (((4 - n) as u8) << 2);
        d[1] = index as u8;
        d[2] = (index >> 8) as u8;
        d[3] = sub;
        d[4..4 + n].copy_from_slice(&value);
        reply(node, d);
    } else {
        // Segmented-Upload-Init (scs=2, e=0, s=1): 0x41, Groesse in 4..8
        let mut d = [0u8; 8];
        d[0] = 0x41;
        d[1] = index as u8;
        d[2] = (index >> 8) as u8;
        d[3] = sub;
        d[4..8].copy_from_slice(&(value.len() as u32).to_le_bytes());
        *UP_XFER.lock().unwrap() = Some((value, 0, 0));
        reply(node, d);
    }
}

fn upload_segment(node: u8) {
    let mut guard = UP_XFER.lock().unwrap();
    let Some((data, pos, toggle)) = guard.as_mut() else {
        drop(guard);
        abort(node, 0, 0, 0x0800_0000);
        return;
    };
    let n = (data.len() - *pos).min(7);
    let last = *pos + n >= data.len();
    // Upload-Segment-Antwort (scs=0): toggle<<4 | (7-n)<<1 | last
    let mut d = [0u8; 8];
    d[0] = (*toggle << 4) | (((7 - n) as u8) << 1) | last as u8;
    d[1..1 + n].copy_from_slice(&data[*pos..*pos + n]);
    *pos += n;
    *toggle ^= 1;
    let done = last;
    drop(guard);
    if done {
        *UP_XFER.lock().unwrap() = None;
    }
    reply(node, d);
}

// ── Download (Client -> Server) ─────────────────────────────────────────────

fn download_init(node: u8, payload: &[u8; 8]) {
    let index = u16::from_le_bytes([payload[1], payload[2]]);
    let sub = payload[3];
    let cmd = payload[0];

    if cmd & 0x02 != 0 {
        // Expedited download (e=1): n Bytes in 4..8, Groesse in Bits 3..2 (s=1)
        let n = if cmd & 0x01 != 0 {
            4 - ((cmd >> 2) & 0x03) as usize
        } else {
            4
        };
        store_put(node, index, sub, &payload[4..4 + n]);
    } else {
        // Segmented download init (0x21): Puffer anlegen
        *DOWN_XFER.lock().unwrap() = Some((node, index, sub, Vec::new(), 0));
    }
    // Download-Init-Antwort (scs=3): 0x60 + Mux
    let mut d = [0u8; 8];
    d[0] = 0x60;
    d[1] = index as u8;
    d[2] = (index >> 8) as u8;
    d[3] = sub;
    reply(node, d);
}

fn download_segment(node: u8, payload: &[u8; 8]) {
    let cmd = payload[0];
    let toggle = (cmd >> 4) & 0x01;
    let n = 7 - ((cmd >> 1) & 0x07) as usize;
    let last = cmd & 0x01 != 0;

    let mut guard = DOWN_XFER.lock().unwrap();
    if let Some((snode, sindex, ssub, buf, _)) = guard.as_mut() {
        buf.extend_from_slice(&payload[1..1 + n]);
        if last {
            let (sn, si, ss, data) = (*snode, *sindex, *ssub, buf.clone());
            *guard = None;
            drop(guard);
            store_put(sn, si, ss, &data);
        }
    }
    // Download-Segment-Antwort (scs=1): 0x20 | toggle<<4
    let mut d = [0u8; 8];
    d[0] = 0x20 | (toggle << 4);
    reply(node, d);
}

// ── Wertspeicher + Simulation ───────────────────────────────────────────────

fn find_dp(node: u8, index: u16, sub: u8) -> Option<&'static Dp> {
    NODES
        .iter()
        .find(|n| n.node_id == node)?
        .dps
        .iter()
        .find(|d| d.index == index && d.sub == sub)
}

fn store_put(node: u8, index: u16, sub: u8, data: &[u8]) {
    let mut store = STORE.lock().unwrap();
    if let Some(e) = store
        .iter_mut()
        .find(|(n, i, s, _)| *n == node && *i == index && *s == sub)
    {
        e.3 = data.to_vec();
    } else {
        store.push((node, index, sub, data.to_vec()));
    }
}

fn get_or_simulate(node: u8, index: u16, sub: u8) -> Vec<u8> {
    if let Some(e) = STORE
        .lock()
        .unwrap()
        .iter()
        .find(|(n, i, s, _)| *n == node && *i == index && *s == sub)
    {
        return e.3.clone();
    }
    simulate(find_dp(node, index, sub), index, sub)
}

/// Simulierten, zum Datentyp passenden Wert erzeugen (wie der C-Host-Mock).
fn simulate(dp: Option<&Dp>, index: u16, sub: u8) -> Vec<u8> {
    let t = TICK.fetch_add(1, Ordering::Relaxed) / 4;
    let seed = ((index as u32) << 8) ^ sub as u32;

    let Some(dp) = dp else {
        return (seed.wrapping_add(t)).to_le_bytes().to_vec();
    };

    match dp.dtype {
        DType::Bool => vec![(((t / 4) + seed) & 1) as u8],
        DType::Str => format!("Host {index:04X}.{sub:02X}").into_bytes(),
        DType::Octet => (0..4u32).map(|i| (seed + t + i) as u8).collect(),
        DType::F32 => {
            let phase = (t + seed) % 200;
            let f = if phase < 100 {
                phase as f32
            } else {
                (200 - phase) as f32
            };
            f.to_le_bytes().to_vec()
        }
        _ => {
            // Integer: Dreieckskurve im (ggf. gekappten) Limitbereich
            let (mut lo, mut hi) = (dp.min as i64, dp.max as i64);
            if hi - lo > 100_000 || hi <= lo {
                lo = 0;
                hi = 1000;
            }
            let span = (hi - lo) as u64;
            let phase = ((t as u64) * (1 + span / 60) + seed as u64) % (2 * span);
            let v = if phase < span {
                lo + phase as i64
            } else {
                hi - (phase - span) as i64
            };
            let len = dp.dtype.size().max(1);
            (0..len).map(|i| (v >> (8 * i)) as u8).collect()
        }
    }
}
