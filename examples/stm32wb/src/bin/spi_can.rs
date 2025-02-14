#![no_std]
#![no_main]

use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{GPDMA1_CH3, I2C1, SPI1};
use embassy_stm32::spi::Spi;
use embassy_stm32::{i2c, spi};
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Instant, Ticker, Timer};
use embedded_can::{ExtendedId, Id, StandardId};
use heapless::Vec;
use mcp25xxfd::frame::Frame;
use mcp25xxfd::registers::PayloadSize;
use mcp25xxfd::{
    config::{BitRate, Clock, Config, FIFOConfig, FilterConfig, MaskConfig},
    registers, MCP25xxFD,
};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

type SpiBusRef = StaticCell<Mutex<CriticalSectionRawMutex, Spi<'static, Async>>>;
static SPI1_SHARED: SpiBusRef = StaticCell::new();

type SpiSevRef = StaticCell<Mutex<CriticalSectionRawMutex, >>;

static FORWARDING_CHANNEL: Channel<CriticalSectionRawMutex, (StandardId, Vec<u8, 64>), 10> = Channel::new();

static OBD_CONTROLLER: StaticCell<
    Mutex<CriticalSectionRawMutex, MCP25xxFD<SpiDevice<CriticalSectionRawMutex, SpiBusRef, Output>>>,
> = StaticCell::new();
fn construct_uds_query(command: &[u8]) -> [u8; 8] {
    let mut query = [0u8; 8];
    if command.len() <= 6 {
        query[0] = command.len() as u8 + 1; // Length of UDS command byte + ECU command
        query[1] = 0x22; // UDS command = diagnostic read
                         // Copy over the ECU subcommand
        for (i, byte) in command.iter().enumerate() {
            query[i + 2] = *byte;
        }
    }
    query
}

struct ECUAddresses {
    bms: Id,
    tpms: Id,
    hvac: Id,
    adas: Id,
    iccu: Id,
    vcms: Id,
    dash: Id,
    igpm: Id,
}

impl ECUAddresses {
    fn new() -> (Self, Self) {
        let tx = Self {
            bms: StandardId::new(0x7E4).unwrap().into(),
            tpms: StandardId::new(0x7A0).unwrap().into(),
            hvac: StandardId::new(0x7B3).unwrap().into(),
            adas: StandardId::new(0x730).unwrap().into(),
            iccu: StandardId::new(0x7E5).unwrap().into(),
            vcms: StandardId::new(0x744).unwrap().into(),
            dash: StandardId::new(0x7C6).unwrap().into(),
            igpm: StandardId::new(0x770).unwrap().into(),
        };
        let rx = Self {
            bms: Self::rx_address(tx.bms),
            tpms: Self::rx_address(tx.tpms),
            hvac: Self::rx_address(tx.hvac),
            adas: Self::rx_address(tx.adas),
            iccu: Self::rx_address(tx.iccu),
            vcms: Self::rx_address(tx.vcms),
            dash: Self::rx_address(tx.dash),
            igpm: Self::rx_address(tx.igpm),
        };
        (tx, rx)
    }

    fn address_offset<const O: i32>(ecu_addr: impl Into<Id>) -> Id {
        let ecu_addr = ecu_addr.into();
        match ecu_addr {
            Id::Standard(addr) => StandardId::new(((addr.as_raw() as i32) + O) as u16).unwrap().into(),
            Id::Extended(addr) => ExtendedId::new(((addr.as_raw() as i32) + O) as u32).unwrap().into(),
        }
    }

    fn rx_address(ecu_addr: impl Into<Id>) -> Id {
        Self::address_offset::<8>(ecu_addr)
    }

    fn tx_address(ecu_addr: impl Into<Id>) -> Id {
        Self::address_offset::<-8>(ecu_addr)
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let miso = p.PA6;
    let mosi = p.PA7;
    let sclk = p.PA5;
    let spi1 = Spi::new(
        p.SPI1,
        sclk,
        mosi,
        miso,
        p.GPDMA1_CH3,
        p.GPDMA1_CH2,
        embassy_stm32::spi::Config::default(),
    );
    let spi1 = SPI1_SHARED.init(Mutex::<CriticalSectionRawMutex, _>::new(spi1));

    let obd_cs = Output::new(p.PA4, Level::High, Speed::High);
    let obd_int = Input::new(p.PB0, Pull::Up);
    let mut obd_stby = Output::new(p.PB1, Level::Low, Speed::High);
    obd_stby.set_low();

    let comma_cs = Output::new(p.PB10, Level::High, Speed::High);
    let comma_int = Input::new(p.PB13, Pull::Up);
    let mut comma_stby = Output::new(p.PB12, Level::Low, Speed::High);
    comma_stby.set_low();

    let obd_device = SpiDevice::new(spi1, obd_cs);

    let obd_controller = OBD_CONTROLLER.init(Mutex::new(MCP25xxFD::new(obd_device)));
    spawner.must_spawn(obd_task(spawner, obd_controller, obd_int));
}

async fn wait_for_low(pin: &mut Input<'static>) {
    while pin.is_high() {
        Timer::after(Duration::from_millis(10)).await;
    }
}

const TRANSMIT_FIFO: u8 = 1;
const RX_BATTERY_FIFO: u8 = 2;
const RX_TPMS_FIFO: u8 = 3;
const RX_HVAC_FIFO: u8 = 4;
const RX_ADAS_FIFO: u8 = 5;
const RX_ICCU_FIFO: u8 = 6;
const RX_VCMS_FIFO: u8 = 7;
const RX_DASH_FIFO: u8 = 8;
const RX_IGPM_FIFO: u8 = 9;

#[embassy_executor::task]
async fn obd_task(
    spawner: Spawner,
    obd_controller: &'static Mutex<
        CriticalSectionRawMutex,
        MCP25xxFD<SpiDevice<'static, CriticalSectionRawMutex, SpiBusRef, Output<'static>>>,
    >,
    mut int: Input<'static>,
) {
    let (tx_addrs, rx_addrs) = ECUAddresses::new();

    {
        let mut obd_controller = obd_controller.lock().await;
        obd_controller
            .reset_and_apply_config(&Config {
                clock: Clock::Clock20MHz,
                bit_rate: BitRate::default(),
                ecc_enabled: true,
                restrict_retx_attempts: false,
                txq_enabled: false,
                tx_event_fifo_enabled: false,
                iso_crc_enabled: true,
            })
            .await
            .unwrap();

        obd_controller
            .configure_fifo(FIFOConfig::<TRANSMIT_FIFO>::tx_with_size(8, PayloadSize::Bytes8))
            .await
            .unwrap();

        obd_controller
            .configure_fifo(FIFOConfig::<RX_BATTERY_FIFO>::rx_with_size(8, PayloadSize::Bytes8))
            .await
            .unwrap();
        obd_controller
            .configure_filter(
                FilterConfig::<RX_BATTERY_FIFO, RX_BATTERY_FIFO>::from_id(rx_addrs.bms),
                MaskConfig::<RX_BATTERY_FIFO>::match_exact(),
            )
            .await
            .unwrap();

        obd_controller
            .configure_fifo(FIFOConfig::<RX_TPMS_FIFO>::rx_with_size(8, PayloadSize::Bytes8))
            .await
            .unwrap();
        obd_controller
            .configure_filter(
                FilterConfig::<RX_TPMS_FIFO, RX_TPMS_FIFO>::from_id(rx_addrs.tpms),
                MaskConfig::<RX_TPMS_FIFO>::match_exact(),
            )
            .await
            .unwrap();

        obd_controller
            .configure_fifo(FIFOConfig::<RX_HVAC_FIFO>::rx_with_size(8, PayloadSize::Bytes8))
            .await
            .unwrap();
        obd_controller
            .configure_filter(
                FilterConfig::<RX_HVAC_FIFO, RX_HVAC_FIFO>::from_id(rx_addrs.hvac),
                MaskConfig::<RX_HVAC_FIFO>::match_exact(),
            )
            .await
            .unwrap();

        obd_controller
            .configure_fifo(FIFOConfig::<RX_ADAS_FIFO>::rx_with_size(8, PayloadSize::Bytes8))
            .await
            .unwrap();
        obd_controller
            .configure_filter(
                FilterConfig::<RX_ADAS_FIFO, RX_ADAS_FIFO>::from_id(rx_addrs.adas),
                MaskConfig::<RX_ADAS_FIFO>::match_exact(),
            )
            .await
            .unwrap();

        obd_controller
            .configure_fifo(FIFOConfig::<RX_ICCU_FIFO>::rx_with_size(8, PayloadSize::Bytes8))
            .await
            .unwrap();
        obd_controller
            .configure_filter(
                FilterConfig::<RX_ICCU_FIFO, RX_ICCU_FIFO>::from_id(rx_addrs.iccu),
                MaskConfig::<RX_ICCU_FIFO>::match_exact(),
            )
            .await
            .unwrap();

        obd_controller
            .configure_fifo(FIFOConfig::<RX_VCMS_FIFO>::rx_with_size(8, PayloadSize::Bytes8))
            .await
            .unwrap();
        obd_controller
            .configure_filter(
                FilterConfig::<RX_VCMS_FIFO, RX_VCMS_FIFO>::from_id(rx_addrs.vcms),
                MaskConfig::<RX_VCMS_FIFO>::match_exact(),
            )
            .await
            .unwrap();

        obd_controller
            .configure_fifo(FIFOConfig::<RX_DASH_FIFO>::rx_with_size(8, PayloadSize::Bytes8))
            .await
            .unwrap();
        obd_controller
            .configure_filter(
                FilterConfig::<RX_DASH_FIFO, RX_DASH_FIFO>::from_id(rx_addrs.dash),
                MaskConfig::<RX_DASH_FIFO>::match_exact(),
            )
            .await
            .unwrap();

        obd_controller
            .configure_fifo(FIFOConfig::<RX_IGPM_FIFO>::rx_with_size(8, PayloadSize::Bytes8))
            .await
            .unwrap();
        obd_controller
            .configure_filter(
                FilterConfig::<RX_IGPM_FIFO, RX_IGPM_FIFO>::from_id(rx_addrs.igpm),
                MaskConfig::<RX_IGPM_FIFO>::match_exact(),
            )
            .await
            .unwrap();

        obd_controller.set_mode(registers::OperationMode::Normal).await.unwrap();
        Timer::after_millis(500).await;
    }
    spawner.must_spawn(obd_sender_task(obd_controller, tx_addrs));

    #[derive(Format)]
    struct ISOTPTransfer {
        rx_addr: Id,
        raw_data: Vec<u8, 80>,
        length: u16,
        rx_fifo: u8,
    }
    impl ISOTPTransfer {
        fn new(rx_addr: Id, data: &[u8], length: u16, rx_fifo: u8) -> Self {
            Self {
                rx_addr,
                raw_data: Vec::from_slice(data).unwrap(),
                length,
                rx_fifo,
            }
        }
        fn pid(&self) -> &[u8] {
            // First byte is UDS response type
            // Next two bytes are requested PID
            &self.raw_data[1..3]
        }
        fn data(&self) -> &[u8] {
            &self.raw_data[3..]
        }
        fn raw_rx_addr(&self) -> u32 {
            match self.rx_addr {
                Id::Standard(id) => id.as_raw() as u32,
                Id::Extended(id) => id.as_raw(),
            }
        }
    }

    // Receive loop
    loop {
        // Wait for interrupt pin to go low (aka active) before calling receive so we don't spinlock
        wait_for_low(&mut int).await;
        // Lock the mutex for this receive cycle (sender thread must wait until we're done receiving)
        let mut obd_controller = obd_controller.lock().await;
        let mut transfer: Option<ISOTPTransfer> = None;
        let transfer_start = Instant::now();

        loop {
            let rx_fifo = transfer.as_ref().map(|t| t.rx_fifo); // Hold the RX FIFO number if there is an active transfer
            match obd_controller.receive(rx_fifo).await {
                Ok(Some((fifo, frame))) => {
                    trace!(
                        "Received message from FIFO{}: {:x} ({} bytes): {:x}",
                        fifo,
                        frame.raw_id(),
                        frame.data().len(),
                        frame.data()
                    );

                    match frame.data()[0] >> 4 {
                        0 => {
                            // Single ISO-TP frame
                            trace!("Single frame of data");
                            // ISO-TP transmission complete
                            transfer = Some(ISOTPTransfer::new(frame.id(), &frame.data()[1..], 8 - 3, fifo));
                            break;
                        }
                        1 => {
                            // First ISO-TP frame
                            let length = frame.data()[1] as u16 + ((frame.data()[0] as u16 & 0b1111) << 8);
                            trace!("First frame of data with total length {}", length);
                            if length >= 80 {
                                warn!(
                                    "Unable to handle ISO-TP transmission with length {} (ECU: {:x}, PID: {:x})",
                                    length,
                                    frame.raw_id(),
                                    &frame.data()
                                );
                                transfer = None;
                                break;
                            }
                            transfer = Some(ISOTPTransfer::new(frame.id(), &frame.data()[2..], length, fifo));

                            // Send flow control message to receive the rest of the data
                            let flow_control_frame = Frame::new(
                                ECUAddresses::tx_address(frame.id()),
                                &[0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                            )
                            .unwrap();
                            obd_controller
                                .transmit::<TRANSMIT_FIFO>(&flow_control_frame)
                                .await
                                .unwrap();
                        }
                        2 => {
                            // Consecutive ISO-TP frame
                            let frame_number = frame.data()[0] & 0b1111;
                            trace!("Consecutive frame #{}", frame_number);

                            match transfer {
                                Some(ref mut transfer) => {
                                    let remaining_bytes: usize = transfer.length as usize - transfer.raw_data.len();
                                    if remaining_bytes > 7 {
                                        transfer.raw_data.extend_from_slice(&frame.data()[1..]).unwrap();
                                    } else {
                                        // Don't copy more bytes than the transfer size
                                        transfer
                                            .raw_data
                                            .extend_from_slice(&frame.data()[1..1 + remaining_bytes])
                                            .unwrap();
                                    }

                                    if transfer.raw_data.len() as u16 >= transfer.length {
                                        // ISO-TP transmission complete
                                        break;
                                    }
                                }
                                None => warn!("Received consecutive frame without an active transfer!"),
                            }
                        }
                        _ => {}
                    }
                }
                Ok(None) => {
                    // No message in the specified RX FIFO, wait for another RX interrupt before continuing (but not more than 250 ms)
                    let _ = embassy_time::with_timeout(Duration::from_millis(250), wait_for_low(&mut int)).await;
                }
                Err(mcp25xxfd::Error::ControllerError(description)) => {
                    error!("{} Transfer: {}", description, transfer);
                    FORWARDING_CHANNEL
                        .send((
                            StandardId::new(0x700).unwrap(),
                            Vec::from_slice(description.as_bytes()).unwrap(),
                        ))
                        .await;
                    transfer = None;
                    break;
                }
                Err(err) => {
                    dbg!(err);
                    transfer = None;
                    break;
                }
            }
            // If we've been receiving for more than 250 milliseconds, abort so that we don't hold the mutex lock forever
            if transfer_start.elapsed().as_millis() > 250 {
                if let Some(transfer) = transfer {
                    warn!("Transfer from {:x} timed out: {:?}", transfer.raw_rx_addr(), transfer);
                } else {
                    warn!("Unknown transfer timed out");
                }
                transfer = None;
                break;
            }
        }

        if let Some(transfer) = transfer.take() {
            let forwarding_address = match transfer.rx_addr {
                addr if addr == rx_addrs.bms && transfer.pid() == [0x01, 0x01] => 0x701,
                addr if addr == rx_addrs.bms && transfer.pid() == [0x01, 0x05] => 0x705,
                addr if addr == rx_addrs.bms && transfer.pid() == [0x01, 0x06] => 0x706,
                addr if addr == rx_addrs.bms && transfer.pid() == [0x01, 0x11] => 0x70B,
                addr if addr == rx_addrs.tpms && transfer.pid() == [0xC0, 0x0B] => 0x710,
                addr if addr == rx_addrs.hvac && transfer.pid() == [0x01, 0x00] => 0x720,
                addr if addr == rx_addrs.adas && transfer.pid() == [0xF0, 0x10] => 0x730,
                addr if addr == rx_addrs.iccu && transfer.pid() == [0xE0, 0x01] => 0x741,
                addr if addr == rx_addrs.iccu && transfer.pid() == [0xE0, 0x02] => 0x742,
                addr if addr == rx_addrs.iccu && transfer.pid() == [0xE0, 0x03] => 0x743,
                addr if addr == rx_addrs.iccu && transfer.pid() == [0xE0, 0x11] => 0x74B,
                addr if addr == rx_addrs.vcms && transfer.pid() == [0xE0, 0x01] => 0x751,
                addr if addr == rx_addrs.vcms && transfer.pid() == [0xE0, 0x02] => 0x752,
                addr if addr == rx_addrs.vcms && transfer.pid() == [0xE0, 0x03] => 0x753,
                addr if addr == rx_addrs.vcms && transfer.pid() == [0xE0, 0x04] => 0x754,
                addr if addr == rx_addrs.dash && transfer.pid() == [0xB0, 0x02] => 0x760,
                addr if addr == rx_addrs.igpm && transfer.pid() == [0xBC, 0x03] => 0x773,
                addr if addr == rx_addrs.igpm && transfer.pid() == [0xBC, 0x04] => 0x774,
                _ => {
                    warn!(
                        "Unhandled ISO-TP response from address {:x} to PID {:x}: {:x}",
                        transfer.raw_rx_addr(),
                        transfer.pid(),
                        transfer.data()
                    );
                    continue;
                }
            };
            let forwarding_address = StandardId::new(forwarding_address).unwrap();
            FORWARDING_CHANNEL
                .send((
                    forwarding_address,
                    Vec::from_slice(&transfer.data().chunks(64).next().unwrap()).unwrap(),
                ))
                .await;
        }
    }
}

#[embassy_executor::task]
async fn obd_sender_task(
    obd_controller: &'static Mutex<
        CriticalSectionRawMutex,
        MCP25xxFD<SpiDevice<'static, CriticalSectionRawMutex, SpiBusRef, Output<'static>>>,
    >,
    tx_addrs: ECUAddresses,
) {
    let queries = [
        Frame::new(tx_addrs.bms, &construct_uds_query(&[0x01, 0x01])).unwrap(),
        Frame::new(tx_addrs.bms, &construct_uds_query(&[0x01, 0x05])).unwrap(),
        // Frame::new(tx_addrs.bms, &construct_uds_query(&[0x01, 0x06])).unwrap(),
        Frame::new(tx_addrs.bms, &construct_uds_query(&[0x01, 0x11])).unwrap(),
        Frame::new(tx_addrs.tpms, &construct_uds_query(&[0xC0, 0x0B])).unwrap(),
        Frame::new(tx_addrs.hvac, &construct_uds_query(&[0x01, 0x00])).unwrap(),
        // Frame::new(tx_addrs.adas, &construct_uds_query(&[0xF0, 0x10])).unwrap(),
        Frame::new(tx_addrs.iccu, &construct_uds_query(&[0xE0, 0x01])).unwrap(),
        Frame::new(tx_addrs.iccu, &construct_uds_query(&[0xE0, 0x02])).unwrap(),
        Frame::new(tx_addrs.iccu, &construct_uds_query(&[0xE0, 0x03])).unwrap(),
        Frame::new(tx_addrs.iccu, &construct_uds_query(&[0xE0, 0x11])).unwrap(),
        Frame::new(tx_addrs.vcms, &construct_uds_query(&[0xE0, 0x01])).unwrap(),
        Frame::new(tx_addrs.vcms, &construct_uds_query(&[0xE0, 0x02])).unwrap(),
        Frame::new(tx_addrs.vcms, &construct_uds_query(&[0xE0, 0x03])).unwrap(),
        Frame::new(tx_addrs.vcms, &construct_uds_query(&[0xE0, 0x04])).unwrap(),
        Frame::new(tx_addrs.dash, &construct_uds_query(&[0xB0, 0x02])).unwrap(),
        Frame::new(tx_addrs.igpm, &construct_uds_query(&[0xBC, 0x03])).unwrap(),
        Frame::new(tx_addrs.igpm, &construct_uds_query(&[0xBC, 0x04])).unwrap(),
    ];

    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        for frame in queries.iter() {
            obd_controller
                .lock()
                .await
                .transmit::<TRANSMIT_FIFO>(frame)
                .await
                .unwrap();
            Timer::after_millis(30).await;
        }
        Timer::after_millis(1000).await;
        ticker.next().await;
    }
}
