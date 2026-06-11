//! USB CDC serial logging (for picotool / BOOTSEL flashing without SWD).

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::Peri;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

/// Start the USB logger task. Call once with the `USB` peripheral before other init.
pub fn spawn(spawner: &Spawner, usb: Peri<'static, USB>) {
    let driver = Driver::new(usb, Irqs);
    spawner.spawn(logger_task(driver).unwrap());
}

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}
