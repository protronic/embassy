//! Geteilter FDCAN-Bus fuer SDO-Client und PoC-Hallenlicht.
//!
//! Ein TX-Pump-Task besitzt den [`CanTx`] und sendet alles, was ueber
//! [`TX_QUEUE`] hereinkommt (SDO-Requests aus [`super::sdo`] und
//! PoC-Bitmasken aus [`crate::touch_can`]). Der RX-Router besitzt den
//! [`CanRx`] und verteilt: SDO-Antworten (COB-ID 0x581..0x5FF) an den
//! SDO-Client, alles andere an die PoC-Verarbeitung.

use embassy_futures::select::{Either, select};
use embassy_stm32::can::frame::Frame;
use embassy_stm32::can::{CanRx, CanTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use touch_hall_common::CAN_RX_POLL_MS;

use crate::canboss::sdo;
use crate::touch_can;

/// Ausgangs-Queue: alle Sender reihen fertige Frames ein.
pub static TX_QUEUE: Channel<CriticalSectionRawMutex, Frame, 8> = Channel::new();

/// Frame einreihen, ohne zu blockieren (false = Queue voll).
pub fn try_send(frame: Frame) -> bool {
    TX_QUEUE.try_send(frame).is_ok()
}

/// Frame einreihen, wartet bei voller Queue.
pub async fn send(frame: Frame) {
    TX_QUEUE.send(frame).await;
}

#[embassy_executor::task]
pub async fn tx_pump_task(tx: &'static mut CanTx<'static>) -> ! {
    loop {
        let frame = TX_QUEUE.receive().await;
        let _ = tx.write(&frame).await;
    }
}

#[embassy_executor::task]
pub async fn rx_router_task(rx: &'static mut CanRx<'static>) -> ! {
    let poll = Duration::from_millis(CAN_RX_POLL_MS);

    loop {
        // Zeitbasis der PoC-RX-Entprellung weiterziehen (ersetzt den
        // Poll-Anteil von touch_can::rx_task)
        touch_can::rx_poll_tick();

        match select(rx.read(), Timer::after(poll)).await {
            Either::First(Ok(envelope)) => {
                let (frame, _) = envelope.parts();
                if let embedded_can::Id::Standard(id) = frame.header().id() {
                    let id = id.as_raw();
                    if (0x581..=0x5FF).contains(&id) {
                        // SDO-Antwort (TSDO 0x580 + NodeID)
                        sdo::on_response(id, frame.data());
                    } else {
                        touch_can::process_rx_frame(id, frame.data());
                    }
                }
            }
            Either::First(Err(_)) => {}
            Either::Second(()) => {}
        }
    }
}
