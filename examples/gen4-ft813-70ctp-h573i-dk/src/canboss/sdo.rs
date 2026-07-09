//! Asynchroner CiA-301-SDO-Client (Default-Server-SDO, 11-Bit-COB-IDs).
//!
//! Rust-Gegenstueck zu `App/canboss_sdo.c` im C-Projekt: Die LVGL-Views
//! stellen Lese-/Schreibauftraege in Slots ein ([`submit_read`] /
//! [`submit_write`]) und pollen den Zustand ([`state`]); der Worker-Task
//! arbeitet die Auftraege sequenziell ab (Expedited- und Segmented-
//! Transfers, Requests 0x600+Node, Antworten 0x580+Node ueber den
//! RX-Router in [`super::canbus`]).

use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, Ordering};

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, with_timeout};

/// Maximale Nutzdatengroesse eines Datenpunkts (Strings werden gekappt) —
/// wie `CANBOSS_DP_DATA_MAX` im C-Port.
pub const SDO_DATA_MAX: usize = 32;
/// Ein Slot je Widget-Zeile des aktiven Screens.
pub const SLOT_COUNT: usize = 64;
/// Antwort-Timeout je SDO-Teilschritt.
const SDO_TIMEOUT: Duration = Duration::from_millis(500);

/// Slot-Zustand (Pendant zu `canboss_sdo_state_t`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum SdoState {
    Idle = 0,
    Pending = 1,
    Done = 2,
    Error = 3,
}

#[derive(Clone, Copy)]
struct SlotData {
    data: [u8; SDO_DATA_MAX],
    len: usize,
    abort: u32,
}

const EMPTY_SLOT: SlotData = SlotData {
    data: [0; SDO_DATA_MAX],
    len: 0,
    abort: 0,
};

static SLOT_STATE: [AtomicU8; SLOT_COUNT] = [const { AtomicU8::new(0) }; SLOT_COUNT];
static SLOT_DATA: Mutex<CriticalSectionRawMutex, RefCell<[SlotData; SLOT_COUNT]>> =
    Mutex::new(RefCell::new([EMPTY_SLOT; SLOT_COUNT]));

#[derive(Clone, Copy)]
struct Job {
    slot: usize,
    node_id: u8,
    index: u16,
    sub: u8,
    write: bool,
    data: [u8; SDO_DATA_MAX],
    len: usize,
}

static JOBS: Channel<CriticalSectionRawMutex, Job, 8> = Channel::new();
/// SDO-Antworten vom RX-Router: (NodeID, 8 Datenbytes).
static RESPONSES: Channel<CriticalSectionRawMutex, (u8, [u8; 8]), 4> = Channel::new();

/// Vom RX-Router fuer COB-IDs 0x581..=0x5FF aufgerufen (Task-Kontext).
pub fn on_response(cob_id: u16, data: &[u8]) {
    let mut bytes = [0u8; 8];
    let n = data.len().min(8);
    bytes[..n].copy_from_slice(&data[..n]);
    let _ = RESPONSES.try_send(((cob_id - 0x580) as u8, bytes));
}

pub fn state(slot: usize) -> SdoState {
    match SLOT_STATE[slot].load(Ordering::Acquire) {
        1 => SdoState::Pending,
        2 => SdoState::Done,
        3 => SdoState::Error,
        _ => SdoState::Idle,
    }
}

/// Ergebnis eines Done/Error-Slots kopieren; Rueckgabe (Laenge, Abortcode).
pub fn read_result(slot: usize, out: &mut [u8; SDO_DATA_MAX]) -> (usize, u32) {
    SLOT_DATA.lock(|slots| {
        let slots = slots.borrow();
        out.copy_from_slice(&slots[slot].data);
        (slots[slot].len, slots[slot].abort)
    })
}

/// Slot nach Ergebnisabholung wieder freigeben.
pub fn release(slot: usize) {
    SLOT_STATE[slot].store(SdoState::Idle as u8, Ordering::Release);
}

fn submit(job: Job) -> bool {
    let slot = job.slot;
    if state(slot) != SdoState::Idle {
        return false;
    }
    SLOT_STATE[slot].store(SdoState::Pending as u8, Ordering::Release);
    if JOBS.try_send(job).is_err() {
        SLOT_STATE[slot].store(SdoState::Idle as u8, Ordering::Release);
        return false;
    }
    true
}

/// SDO-Upload (lesen) einreihen. false = Slot belegt oder Queue voll.
pub fn submit_read(slot: usize, node_id: u8, index: u16, sub: u8) -> bool {
    submit(Job {
        slot,
        node_id,
        index,
        sub,
        write: false,
        data: [0; SDO_DATA_MAX],
        len: 0,
    })
}

/// SDO-Download (schreiben) einreihen. false = Slot belegt oder Queue voll.
pub fn submit_write(slot: usize, node_id: u8, index: u16, sub: u8, payload: &[u8]) -> bool {
    let mut data = [0u8; SDO_DATA_MAX];
    let len = payload.len().min(SDO_DATA_MAX);
    data[..len].copy_from_slice(&payload[..len]);
    submit(Job {
        slot,
        node_id,
        index,
        sub,
        write: true,
        data,
        len,
    })
}

// ---------------------------------------------------------------------------
// Worker: sequenzielle Transfers ueber die TX-Queue
// ---------------------------------------------------------------------------

#[embassy_executor::task]
pub async fn worker_task() -> ! {
    loop {
        let job = JOBS.receive().await;

        // Veraltete Antworten frueherer (abgelaufener) Transfers verwerfen
        while RESPONSES.try_receive().is_ok() {}

        let result = if job.write {
            download(&job).await
        } else {
            upload(&job).await
        };

        SLOT_DATA.lock(|slots| {
            let mut slots = slots.borrow_mut();
            match &result {
                Ok((data, len)) => {
                    slots[job.slot].data = *data;
                    slots[job.slot].len = *len;
                    slots[job.slot].abort = 0;
                }
                Err(abort) => {
                    slots[job.slot].len = 0;
                    slots[job.slot].abort = *abort;
                }
            }
        });
        let new_state = if result.is_ok() {
            SdoState::Done
        } else {
            SdoState::Error
        };
        SLOT_STATE[job.slot].store(new_state as u8, Ordering::Release);
    }
}

/// SDO-Abortcode fuer lokale Fehler (0 = Timeout, wie im C-Port geloggt).
const ABORT_TIMEOUT: u32 = 0;
const ABORT_GENERAL: u32 = 0x0800_0000;

async fn send_request(node_id: u8, payload: &[u8; 8]) -> Result<(), u32> {
    // RSDO (Empfangs-SDO des Servers) = 0x600 + NodeID; die Frame-
    // Konstruktion liegt im plattformspezifischen `canbus` (Firmware:
    // embassy-FDCAN, Host: Mock-Server), damit dieser Client geteilt bleibt.
    super::canbus::send_request_raw(0x600 + node_id as u16, payload)
        .await
        .map_err(|_| ABORT_GENERAL)
}

/// Naechste Antwort dieses Knotens abwarten (fremde NodeIDs verwerfen).
async fn wait_response(node_id: u8) -> Result<[u8; 8], u32> {
    loop {
        match with_timeout(SDO_TIMEOUT, RESPONSES.receive()).await {
            Ok((node, data)) => {
                if node == node_id {
                    return Ok(data);
                }
            }
            Err(_) => return Err(ABORT_TIMEOUT),
        }
    }
}

fn check_abort(resp: &[u8; 8]) -> Result<(), u32> {
    if resp[0] == 0x80 {
        Err(u32::from_le_bytes([resp[4], resp[5], resp[6], resp[7]]))
    } else {
        Ok(())
    }
}

fn check_mux(resp: &[u8; 8], index: u16, sub: u8) -> Result<(), u32> {
    if resp[1] == (index & 0xFF) as u8 && resp[2] == (index >> 8) as u8 && resp[3] == sub {
        Ok(())
    } else {
        Err(ABORT_GENERAL)
    }
}

/// SDO-Upload: expedited oder segmented (CiA 301 §7.2.4.3.5/.6).
async fn upload(job: &Job) -> Result<([u8; SDO_DATA_MAX], usize), u32> {
    let mut req = [0u8; 8];
    req[0] = 0x40; // initiate upload request
    req[1] = (job.index & 0xFF) as u8;
    req[2] = (job.index >> 8) as u8;
    req[3] = job.sub;
    send_request(job.node_id, &req).await?;

    let resp = wait_response(job.node_id).await?;
    check_abort(&resp)?;
    let cmd = resp[0];
    if cmd & 0xE0 != 0x40 {
        return Err(ABORT_GENERAL);
    }
    check_mux(&resp, job.index, job.sub)?;

    let mut out = [0u8; SDO_DATA_MAX];

    if cmd & 0x02 != 0 {
        // Expedited: bis 4 Bytes direkt in der Initiate-Antwort
        let n = if cmd & 0x01 != 0 {
            4 - ((cmd >> 2) & 0x03) as usize
        } else {
            4
        };
        out[..n].copy_from_slice(&resp[4..4 + n]);
        return Ok((out, n));
    }

    // Segmented Upload: Segmente zu je 7 Bytes anfordern
    let mut len = 0usize;
    let mut toggle = 0u8;
    loop {
        let mut seg_req = [0u8; 8];
        seg_req[0] = 0x60 | (toggle << 4);
        send_request(job.node_id, &seg_req).await?;

        let seg = wait_response(job.node_id).await?;
        check_abort(&seg)?;
        let scmd = seg[0];
        if scmd & 0xE0 != 0x00 || (scmd >> 4) & 0x01 != toggle {
            return Err(ABORT_GENERAL);
        }
        let n = 7 - ((scmd >> 1) & 0x07) as usize;
        for b in &seg[1..1 + n] {
            if len < SDO_DATA_MAX {
                out[len] = *b;
                len += 1;
            }
        }
        if scmd & 0x01 != 0 {
            return Ok((out, len)); // letztes Segment
        }
        toggle ^= 1;
    }
}

/// SDO-Download: expedited (<= 4 Bytes) oder segmented (CiA 301 §7.2.4.3.3/.4).
async fn download(job: &Job) -> Result<([u8; SDO_DATA_MAX], usize), u32> {
    let mut req = [0u8; 8];
    req[1] = (job.index & 0xFF) as u8;
    req[2] = (job.index >> 8) as u8;
    req[3] = job.sub;

    if job.len <= 4 {
        // Expedited download, Groesse angegeben
        req[0] = 0x23 | (((4 - job.len) as u8) << 2);
        req[4..4 + job.len].copy_from_slice(&job.data[..job.len]);
        send_request(job.node_id, &req).await?;

        let resp = wait_response(job.node_id).await?;
        check_abort(&resp)?;
        if resp[0] & 0xE0 != 0x60 {
            return Err(ABORT_GENERAL);
        }
        return Ok((job.data, job.len));
    }

    // Segmented download: Initiate mit Groesse, dann 7-Byte-Segmente
    req[0] = 0x21;
    req[4..8].copy_from_slice(&(job.len as u32).to_le_bytes());
    send_request(job.node_id, &req).await?;
    let resp = wait_response(job.node_id).await?;
    check_abort(&resp)?;
    if resp[0] & 0xE0 != 0x60 {
        return Err(ABORT_GENERAL);
    }

    let mut sent = 0usize;
    let mut toggle = 0u8;
    while sent < job.len {
        let n = (job.len - sent).min(7);
        let last = sent + n >= job.len;
        let mut seg = [0u8; 8];
        seg[0] = (toggle << 4) | (((7 - n) as u8) << 1) | last as u8;
        seg[1..1 + n].copy_from_slice(&job.data[sent..sent + n]);
        send_request(job.node_id, &seg).await?;

        let ack = wait_response(job.node_id).await?;
        check_abort(&ack)?;
        if ack[0] & 0xE0 != 0x20 || (ack[0] >> 4) & 0x01 != toggle {
            return Err(ABORT_GENERAL);
        }
        sent += n;
        toggle ^= 1;
    }
    Ok((job.data, job.len))
}
