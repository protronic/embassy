#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::usart::{Config, Uart};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    defmt::info!("Starting system");

    let mut config1 = Config::default();
    config1.baudrate = 115600;

    //RX/TX connected to USB/UART VCP of ST-Link
    let mut usart1 = Uart::new_blocking(p.USART1, p.PA8, p.PB12, config1).unwrap();

    let _ = usart1.blocking_write(b"Hello Embassy World!\r\n");
}
