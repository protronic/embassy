#![no_std]
#![no_main]

extern crate alloc;
use alloc::format;
use embedded_alloc::Heap;

use core::str;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Speed},
    peripherals,
    usart::{self, Config, Uart},
};
use {defmt_rtt as _, panic_probe as _};

use rhai::{packages::BasicMathPackage, packages::Package, Dynamic, Engine};

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[global_allocator]
static HEAP: Heap = Heap::empty();

static mut LED: Option<Output> = None;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 92 * 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    info!("Starting system");

    unsafe {
        LED = Some(Output::new(p.PC2, Level::High, Speed::Low));
    }

    let config = usart::Config::default();
    let mut usart = Uart::new(p.USART1, p.PB7, p.PB6, Irqs, p.GPDMA1_CH0, p.GPDMA1_CH1, config).unwrap();
    let (mut uart_tx, mut uart_rx) = usart.split();

    let mut engine = Engine::new_raw();
    let package = BasicMathPackage::new();

    // Register the package into the 'Engine'.
    package.register_into_engine(&mut engine);

    // this bit gets commented out to test size without Core
    let std = rhai::packages::CorePackage::new();
    std.register_into_engine(&mut engine);

    engine.register_fn("led", control_led);

    uart_tx.write(b"Hello Embassy World!\r\n").await.unwrap();

    let mut pos = 0;
    let mut buffer = [0u8; 128];
    let mut buf = [0u8; 1];
    loop {
        uart_rx.read(&mut buf).await.unwrap();
        buffer[pos] = buf[0];

        uart_tx.write(&buf).await.unwrap();

        // Check for newline characters
        if buf[0] == b'\n' || buf[0] == b'\r' {
            // Convert buffer to &str
            if let Ok(line) = str::from_utf8(&buffer[..pos]) {
                // Process the received line
                info!("Received line: {}", line);
                match engine.eval_expression::<Dynamic>(line) {
                    Ok(res) => {
                        uart_tx.write(format!("{:?},\r\n{:?}\n\r", line, res).as_bytes()).await.unwrap();
                    }
                    Err(e) => {
                        let mes = format!("Failed to process line: {:?}\n\rError: {:?}\n\r", line, e);
                        uart_tx.write(mes.as_bytes()).await.unwrap();
                    }
                }
            } else {
                error!("Failed to convert buffer to string");
            }
            pos = 0; // Reset the buffer position
        } else {
            pos += 1;
        }
    }

    fn control_led(state: bool) {
        unsafe {
            if let Some(ref mut led) = LED {
                if state {
                    info!("high");
                    led.set_high();
                } else {
                    info!("low");
                    led.set_low();
                }
            }
        }
    }
}