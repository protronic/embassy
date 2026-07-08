//! CANopen node on the STM32H573I-DK, using the CANopenNode Rust port.
//!
//! The discovery kit exposes FDCAN2 on the Arduino connector: PB5 = RX on
//! D3, PB6 = TX on D15 — attach a CAN transceiver breakout there (raw
//! TXD/RXD type like SN65HVD230/MCP2562, not an SPI shield). FDCAN1 is not
//! usable on this board: every FDCAN1_RX-capable pin is hard-wired to
//! another function (PA11 USB, PB8 I2C4, PD0 LCD, PE0/PH14 detect pins,
//! PI9 LED, PI10 ETH). Classic CAN at 500 kbit/s.
//!
//! The device boots as a CANopen slave with the DS301 example object
//! dictionary: boot-up message, cyclic heartbeat, NMT slave, SDO server and
//! event-driven PDOs. Test from a Linux host with a (USB) SocketCAN adapter
//! and the canopen-demo CLI from the CANopenNode repo:
//!
//! ```sh
//! canopen-demo sdo-read  can0 10 0x1200 2        # -> 0x58A ($NODEID+0x580)
//! canopen-demo sdo-write can0 10 0x1017 0 250 2  # heartbeat -> 250 ms
//! canopen-demo nmt       can0 start 10           # PDOs active
//! canopen-demo sdo-write can0 10 0x2000 5 0x42 1 # triggers TPDO1 (0x18A)
//! ```

#![no_std]
#![no_main]

use canopen_core::{CanFrame, Node, NodeId, ResetCommand};
use canopen_embassy::NodeBus;
use canopen_example_od::Od;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::peripherals::FDCAN2;
use embassy_stm32::{bind_interrupts, can, rcc};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    FDCAN2_IT0 => can::IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => can::IT1InterruptHandler<FDCAN2>;
});

const NODE_ID: u8 = 10;
const BITRATE: u32 = 500_000;

/// CANopen bus adapter for the embassy-stm32 FDCAN driver (classic frames).
struct FdcanBus<'d> {
    can: can::Can<'d>,
}

impl NodeBus for FdcanBus<'_> {
    async fn recv(&mut self) -> CanFrame {
        loop {
            match self.can.read().await {
                // Extended-id and remote frames are not used by CANopen.
                Ok(envelope) => {
                    if let Some(frame) = CanFrame::from_embedded(&envelope.frame) {
                        return frame;
                    }
                }
                // Bus error (e.g. bus-off recovery in progress): keep
                // listening; EMCY reporting is a later port milestone. The
                // error counters tell wiring problems apart: TEC rising with
                // no RX traffic means our frames are not being ACKed.
                Err(err) => warn!(
                    "CAN rx error: {} (TEC={} REC={})",
                    err,
                    self.can.properties().tx_error_count(),
                    self.can.properties().rx_error_count(),
                ),
            }
        }
    }

    async fn send(&mut self, frame: CanFrame) {
        self.can.write(&frame.to_embedded::<can::Frame>()).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // The DK feeds HSE from an active 25 MHz oscillator (X3) on OSC_IN, not
    // a crystal: digital bypass mode, like ST's RCC_HSE_BYPASS_DIGITAL in the
    // Cube templates. Use it as the FDCAN kernel clock so bit timing is exact.
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(rcc::Hse {
        freq: embassy_stm32::time::Hertz(25_000_000),
        mode: rcc::HseMode::BypassDigital,
    });
    config.rcc.mux.fdcan12sel = rcc::mux::Fdcansel::Hse;
    let p = embassy_stm32::init(config);
    info!("CANopen node {} starting (FDCAN2 on PB5/PB6)", NODE_ID);

    let mut configurator = can::CanConfigurator::new(p.FDCAN2, p.PB5, p.PB6, Irqs);
    configurator.set_bitrate(BITRATE);
    configurator.properties().set_extended_filter(
        can::filter::ExtendedFilterSlot::_0,
        can::filter::ExtendedFilter::reject_all(),
    );
    configurator.properties().set_standard_filter(
        can::filter::StandardFilterSlot::_0,
        can::filter::StandardFilter::accept_all_into_fifo0(),
    );
    let mut bus = FdcanBus {
        can: configurator.start(can::OperatingMode::NormalOperationMode),
    };

    let node_id = NodeId::new(NODE_ID).unwrap();
    // OD values survive a communication reset (like RAM values in the C
    // stack), so SDO-written configuration - e.g. PDO event timers - takes
    // effect in the re-created node. Factory defaults return with the
    // system reset below.
    let mut od = Od::new(node_id);
    loop {
        let mut node = Node::new(node_id, od.clone());
        info!("boot-up, entering pre-operational");
        match canopen_embassy::run(&mut node, &mut bus).await {
            ResetCommand::Communication => {
                info!("NMT reset communication");
                od = node.od().clone();
            }
            ResetCommand::Node => {
                info!("NMT reset node -> system reset");
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
    }
}
