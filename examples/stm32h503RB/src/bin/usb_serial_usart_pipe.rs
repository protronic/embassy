#![no_std]
#![no_main]

use defmt::{panic, *};
use embassy_executor::Spawner;
use embassy_futures::join::{join, join4};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{Uart, UartRx, UartTx};
use embassy_stm32::usb::{Driver, Instance};
use embassy_stm32::{bind_interrupts, peripherals, usart, usb, Config};
use embassy_sync::blocking_mutex::raw::{NoopRawMutex};
use embassy_sync::pipe::{Pipe, Reader};
use embassy_sync::pubsub::{PubSubChannel, Publisher, Subscriber};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USB_DRD_FS => usb::InterruptHandler<peripherals::USB>;
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true }); // needed for USB
        config.rcc.mux.usbsel = mux::Usbsel::HSI48;
    }
    let p = embassy_stm32::init(config);

    info!("Hello World!");
    // Create a subscriber for the blink task
    // Define a PubSubChannel for led
    let led_signal: PubSubChannel<NoopRawMutex, u8, 1, 1, 2> = PubSubChannel::new();
    let blink_future = async { blinky(p.PB0, led_signal.subscriber().unwrap()).await };

    let config = usart::Config::default();
    let usart = Uart::new(p.USART3, p.PA3, p.PA4, Irqs, p.GPDMA1_CH0, p.GPDMA1_CH1, config).unwrap();
    let (uart_tx, uart_rx) = usart.split();

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs, p.PA12, p.PA11);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Protronic");
    config.product = Some("PROT-USB-serial");
    config.serial_number = Some("12345678");

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Pipe setup
    let mut usb_pipe: Pipe<NoopRawMutex, 20> = Pipe::new();
    let (mut usb_pipe_reader, mut usb_pipe_writer) = usb_pipe.split();

    let mut uart_pipe: Pipe<NoopRawMutex, 20> = Pipe::new();
    let (mut uart_pipe_reader, mut uart_pipe_writer) = uart_pipe.split();

    let (mut usb_tx, mut usb_rx) = class.split();

    // Read + write from USB
    let usb_future = async {
        loop {
            info!("Wait for USB connection");
            usb_rx.wait_connection().await;
            info!("Connected");
            let _ = join(
                usb_read(&mut usb_rx, &mut uart_pipe_writer),
                usb_write(&mut usb_tx, &mut usb_pipe_reader, led_signal.publisher().unwrap()),
            )
                .await;
            info!("Disconnected");
        }
    };

    // Read + write from UART
    let uart_future = join(
        uart_read(uart_rx, &mut usb_pipe_writer),
        uart_write(uart_tx, &mut uart_pipe_reader, led_signal.publisher().unwrap()),
    );

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join4(usb_fut, usb_future, uart_future, blink_future).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

/// Read from the USB and write it to the UART TX pipe
async fn usb_read<'d, T: Instance + 'd>(
    usb_rx: &mut Receiver<'d, Driver<'d, T>>,
    uart_pipe_writer: &mut embassy_sync::pipe::Writer<'_, NoopRawMutex, 20>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = usb_rx.read_packet(&mut buf).await?;
        let data = &buf[..n];
        trace!("USB IN: {:x}", data);
        (*uart_pipe_writer).write(data).await;
    }
}

/// Read from the USB TX pipe and write it to the USB
async fn usb_write<'d, T: Instance + 'd>(
    usb_tx: &mut Sender<'d, Driver<'d, T>>,
    usb_pipe_reader: &mut Reader<'_, NoopRawMutex, 20>,
    publisher: Publisher<'_, NoopRawMutex, u8, 1, 1, 2>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = (*usb_pipe_reader).read(&mut buf).await;
        publisher.publish_immediate(n as u8);
        let data = &buf[..n];
        trace!("USB OUT: {:x}", data);
        usb_tx.write_packet(&data).await?;
    }
}

/// Read from the UART and write it to the USB TX pipe
async fn uart_read(
    mut uart_rx: UartRx<'static, Async>,
    usb_pipe_writer: &mut embassy_sync::pipe::Writer<'_, NoopRawMutex, 20>,
) -> ! {
    let mut buf = [0; 64];
    loop {
        let n = uart_rx.read_until_idle(&mut buf).await.expect("UART read error");
        if n == 0 {
            continue;
        }
        let data = &buf[..n];
        trace!("UART IN: {:x}", buf);
        (*usb_pipe_writer).write(data).await;
    }
}

/// Read from the UART TX pipe and write it to the UART
async fn uart_write(
    mut uart_tx: UartTx<'static, Async>,
    uart_pipe_reader: &mut Reader<'_, NoopRawMutex, 20>,
    publisher: Publisher<'_, NoopRawMutex, u8, 1, 1, 2>,
) -> ! {
    let mut buf = [0; 64];
    loop {
        let n = (*uart_pipe_reader).read(&mut buf).await;
        publisher.publish_immediate(n as u8);
        let data = &buf[..n];
        trace!("UART OUT: {:x}", data);
        let _ = uart_tx.write(&data).await;
    }
}

async fn blinky(led: peripherals::PB0, mut subscriber: Subscriber<'_, NoopRawMutex, u8, 1, 1, 2>) {
    let mut led = Output::new(led, Level::High, Speed::Low);
    loop {
        if let Some(times) = subscriber.try_next_message_pure() {
            for _ in 0..times {
                led.toggle();
                Timer::after_millis(50).await;
                led.toggle();
                Timer::after_millis(50).await;
            }
        } else {
            led.toggle();
            Timer::after_millis(500).await;
        }
    }
}