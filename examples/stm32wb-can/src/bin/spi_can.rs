#![no_std]
#![no_main]

use core::fmt::Write;
// use core::str;
use defmt::panic;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can::{
    Rx0InterruptHandler, Rx1InterruptHandler, SceInterruptHandler, TxInterruptHandler,
};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::CAN1;
use embassy_stm32::spi::{Config, Spi};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_time::{Delay, Timer};

use embedded_can::{Frame, Id, StandardId};
use heapless::String;
use mcp2515::regs::CanStat;
use mcp2515::{error::Error, frame::CanFrame, regs::OpMode, CanSpeed, McpSpeed, MCP2515};
use slcan::Transmit;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    CAN1_RX0 => Rx0InterruptHandler<CAN1>;
    CAN1_RX1 => Rx1InterruptHandler<CAN1>;
    CAN1_SCE => SceInterruptHandler<CAN1>;
    CAN1_TX => TxInterruptHandler<CAN1>;
    USART2  => usart::InterruptHandler<peripherals::USART2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(Default::default());
    let spi_config = Config::default();
    let cs = Output::new(p.PB6, Level::High, Speed::VeryHigh);
    let spi = Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, Some(cs), spi_config);
    let mut can = MCP2515::new(spi);

    can.init(
        &mut Delay,
        mcp2515::Settings {
            mode: OpMode::Normal,
            can_speed: CanSpeed::Kbps100,
            mcp_speed: McpSpeed::MHz8,
            clkout_en: true,
        },
    )
        .unwrap();
    let stat: CanStat = can.read_register().unwrap();
    info!("Can stat mode: {:?}", stat.opmod());

    let button = Input::new(p.PC13, Pull::Up);

    let config_uart = UartConfig::default();
    let mut usart = Uart::new(
        p.USART2,
        p.PA3,
        p.PA2,
        Irqs,
        p.DMA1_CH7,
        p.DMA1_CH6,
        config_uart,
    )
        .unwrap();
    unwrap!(usart.blocking_write(b"Let's start canbusing !! \r\n"));
    let mut usart_rx = [0_u8; 32];
    let mut _s = "no message";

    let mut err_cache = [0_u8; 10];

    let mut mess_cache: String<128> = String::new();

    loop {
        // read and forward can messages to uart
        // read_mode = button.is_high();

        match can.read_message() {
            Ok(frame) => {
                let _f = Transmit::new(&frame);

                let mut s: String<32> = String::new();
                core::write!(s, "{}", _f).unwrap();

                unwrap!(usart.write(s.as_bytes()).await);
            }
            Err(Error::NoMessage) => info!("No message to read!"),
            Err(_) => panic!("Error reading can frame"),
        }

        // read uart and forward can messages to canbus
        if button.is_low() {
            loop {
                for (id, b) in err_cache.iter().enumerate() {
                    if *b > 0 {
                        let mut s: String<32> = String::new();

                        core::write!(s, "id: {} count: {}\r\n", id, *b).unwrap();

                        unwrap!(usart.write(s.as_bytes()).await);
                    }
                }

                if !mess_cache.is_empty() {
                    unwrap!(usart.write(mess_cache.as_bytes()).await);
                    if mess_cache.len() > 100 {
                        mess_cache.clear();
                    }
                }

                match usart.read_until_idle(&mut usart_rx).await {
                    Ok(_) => {
                        let s = core::str::from_utf8(&usart_rx).unwrap_or_else(|_err| {
                            error!("Error during usart read");
                            err_cache[0] += 1;
                            "t1230\r"
                        });

                        unwrap!(usart.write(s.as_bytes()).await);
                        info!("{:?}", s);

                        let (_, _fr) = Transmit::try_parse(s).unwrap_or_else(|_err| {
                            error!("Error during usart read");
                            err_cache[1] += 1;
                            (
                                "_",
                                Transmit::new(
                                    &CanFrame::new(
                                        Id::Standard(StandardId::new(0x0).unwrap()),
                                        &[0; 8],
                                    )
                                        .unwrap(),
                                ),
                            )
                        });

                        info!("sending can packet {}", s);

                        // can.send_message(_fr.frame);
                        let frame_to_send = CanFrame::new(_fr.frame.id(), _fr.frame.data())
                            .unwrap_or_else(|| {
                                error!("Error during usart read");
                                err_cache[2] += 1;
                                CanFrame::new(Id::Standard(StandardId::new(0x0).unwrap()), &[0; 8])
                                    .unwrap()
                            });
                        info!("sending frame {:?}", frame_to_send.data());

                        unwrap!(usart.write(s.as_bytes()).await); // echo to test that correct frame was received/sent
                        mess_cache.push_str(s).unwrap();
                        mess_cache.push_str("\r\n").unwrap();

                        match can.send_message(frame_to_send) {
                            Ok(_) => {
                                info!("Message sent");
                            }

                            Err(Error::NewModeTimeout) => {
                                err_cache[5] += 1;
                            }
                            Err(Error::TxBusy) => {
                                err_cache[6] += 1;
                            }
                            Err(Error::TxFailed) => {
                                err_cache[7] += 1;
                            }
                            Err(Error::NoMessage) => {
                                err_cache[8] += 1;
                            }
                            Err(Error::InvalidFrameId) => {
                                err_cache[9] += 1;
                            }
                            Err(Error::InvalidDlc) => {
                                err_cache[10] += 1;
                            }
                            Err(_) => {
                                err_cache[3] += 1;
                            }
                        }
                        Timer::after_millis(100).await;
                        usart_rx = [0_u8; 32];
                    }
                    Err(_) => {
                        error!("Error during usart read");
                        err_cache[4] += 1;
                    }
                }
            }
        }
    }
}
