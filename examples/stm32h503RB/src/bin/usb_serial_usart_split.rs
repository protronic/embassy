#![no_std]
#![no_main]

use defmt::{panic, *};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::{Driver, Instance};
use embassy_stm32::{bind_interrupts, peripherals, usb, usart, Config};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{Uart, UartRx};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USB_DRD_FS => usb::InterruptHandler<peripherals::USB>;
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

static RX_CHANNEL: Channel<ThreadModeRawMutex, [u8; 64], 1> = Channel::new();
static TX_CHANNEL: Channel<ThreadModeRawMutex, [u8; 64], 1> = Channel::new();


#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        //config.rcc.hsi = None;
        config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true }); // needed for USB
        // config.rcc.hse = Some(Hse {
        //     freq: Hertz(24_000_000),
        //     mode: HseMode::BypassDigital,
        // });
        // config.rcc.pll1 = Some(Pll {
        //     source: PllSource::HSE,
        //     prediv: PllPreDiv::DIV3,
        //     mul: PllMul::MUL62,
        //     divp: Some(PllDiv::DIV2), // 250mhz
        //     divq: Some(PllDiv::DIV2),
        //     divr: Some(PllDiv::DIV2),
        // });
        // config.rcc.ahb_pre = AHBPrescaler::DIV1;
        // config.rcc.apb1_pre = APBPrescaler::DIV1;
        // config.rcc.apb2_pre = APBPrescaler::DIV1;
        // config.rcc.apb3_pre = APBPrescaler::DIV1;
        // config.rcc.sys = Sysclk::PLL1_P;
        // config.rcc.voltage_scale = VoltageScale::Scale0;
        config.rcc.mux.usbsel = mux::Usbsel::HSI48;
    }
    let p = embassy_stm32::init(config);

    info!("Hello World!");

    let config = embassy_stm32::usart::Config::default();
    let mut usart = Uart::new(p.USART1, p.PB15, p.PB14, Irqs, p.GPDMA1_CH0, p.GPDMA1_CH1, config).unwrap();
    let (mut tx, rx) = usart.split();

    unwrap!(_spawner.spawn(reader(rx)));

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
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = echo(&mut class).await;
            info!("Disconnected");
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, echo_fut).await;

    loop {
        let tx_buf = TX_CHANNEL.receive().await;
        info!("writing...");
        unwrap!(tx.write(&tx_buf).await);
        let rx_buf = RX_CHANNEL.receive().await;
        info!("writing...");
        unwrap!(class.write_packet(&rx_buf).await);
    }


}

#[embassy_executor::task]
async fn reader(mut rx: UartRx<'static, Async>) {
    let mut buf = [0; 64];
    loop {
        info!("reading...");
        unwrap!(rx.read(&mut buf).await);
        RX_CHANNEL.send(buf).await;
    }
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

async fn echo<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        TX_CHANNEL.send(buf);
    }
}


