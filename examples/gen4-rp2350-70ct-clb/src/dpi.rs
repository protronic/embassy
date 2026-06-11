//! PIO + DMA parallel RGB (DPI) scan-out for the gen4-RP2350 800×480 panel.
//!
//! Pin map from `board/gen4_rp2350_70ct.h`:
//! - **DE** = GPIO18, **PCLK** = GPIO21 (PIO side-set)
//! - **DATA0..15** = GPIO22..GPIO37
//! - **HSYNC** = GPIO20, **VSYNC** = GPIO19 — parked inactive (DE-only mode)
//!
//! One DMA channel streams a whole frame from the PSRAM framebuffer into the
//! PIO TX FIFO. Each row starts with one 32-bit *prefix word* discarded by the
//! PIO while DE is low, keeping the state machine stalled safely between lines.
//!
//! ```text
//!  PSRAM frame layout (per row): [prefix:u32][800 × RGB565:u16] = 1604 B
//!  Full frame: 480 rows = 769 920 B
//! ```

use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_rp::{Peri, dma, pac, peripherals};
use embassy_time::Timer;
use fixed::FixedU32;
use fixed::types::extra::U8;

use crate::gen4_board::{DISPLAY_HEIGHT, DISPLAY_WIDTH};

/// Pixel clock during the active part of a line, in Hz.
pub const PCLK_HZ: u32 = 10_000_000;

/// 32-bit words per framebuffer row (1 prefix word + 800 pixels as u16 pairs).
pub const ROW_WORDS: usize = 1 + DISPLAY_WIDTH / 2;
/// Bytes per framebuffer row.
pub const ROW_BYTES: usize = ROW_WORDS * 4;
/// Byte offset of the first pixel inside a row (skips the prefix word).
pub const ROW_PIXEL_OFFSET: usize = 4;
/// Total framebuffer size in bytes.
pub const FRAME_BYTES: usize = ROW_BYTES * DISPLAY_HEIGHT;
/// Words streamed to the PIO per frame.
pub const FRAME_WORDS: u32 = (ROW_WORDS * DISPLAY_HEIGHT) as u32;

/// Debug snapshot of the scan-out DMA + PIO state (see [`Dpi::stats`]).
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct DpiStats {
    pub busy: bool,
    pub read_offset: u32,
    pub remaining_words: u32,
    pub sm_pc: u8,
    pub tx_fifo_level: u8,
    pub tx_stalled: bool,
}

/// LCD data + sync pins per `gen4_rp2350_70ct.h`.
pub struct LcdPins {
    pub de: Peri<'static, peripherals::PIN_18>,
    pub pclk: Peri<'static, peripherals::PIN_21>,
    pub d0: Peri<'static, peripherals::PIN_22>,
    pub d1: Peri<'static, peripherals::PIN_23>,
    pub d2: Peri<'static, peripherals::PIN_24>,
    pub d3: Peri<'static, peripherals::PIN_25>,
    pub d4: Peri<'static, peripherals::PIN_26>,
    pub d5: Peri<'static, peripherals::PIN_27>,
    pub d6: Peri<'static, peripherals::PIN_28>,
    pub d7: Peri<'static, peripherals::PIN_29>,
    pub d8: Peri<'static, peripherals::PIN_30>,
    pub d9: Peri<'static, peripherals::PIN_31>,
    pub d10: Peri<'static, peripherals::PIN_32>,
    pub d11: Peri<'static, peripherals::PIN_33>,
    pub d12: Peri<'static, peripherals::PIN_34>,
    pub d13: Peri<'static, peripherals::PIN_35>,
    pub d14: Peri<'static, peripherals::PIN_36>,
    pub d15: Peri<'static, peripherals::PIN_37>,
}

/// DPI scan-out driver: PIO state machine + frame DMA channel.
pub struct Dpi<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
    dma: dma::Channel<'d>,
    ctrl: pac::dma::regs::Ctrl,
    fb_addr: u32,
}

impl<'d, PIO: Instance, const SM: usize> Dpi<'d, PIO, SM> {
    /// Set up scan-out. `fb_addr` is the word-aligned framebuffer base in PSRAM.
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        dma: dma::Channel<'d>,
        pins: LcdPins,
        fb_addr: u32,
    ) -> Self {
        // Side-set bit 0 = DE (GPIO18), bit 1 = PCLK (GPIO21).
        let prg = pio_asm!(
            ".side_set 2",
            ".wrap_target",
            "out null, 32      side 0b00",
            "mov x, isr        side 0b00",
            "pixels:",
            "out pins, 16      side 0b01",
            "jmp x-- pixels    side 0b11",
            "set y, 7          side 0b00",
            "hblank:",
            "nop               side 0b10 [7]",
            "jmp y-- hblank    side 0b00 [7]",
            ".wrap",
        );

        let pio_pins = [
            common.make_pio_pin(pins.d0),
            common.make_pio_pin(pins.d1),
            common.make_pio_pin(pins.d2),
            common.make_pio_pin(pins.d3),
            common.make_pio_pin(pins.d4),
            common.make_pio_pin(pins.d5),
            common.make_pio_pin(pins.d6),
            common.make_pio_pin(pins.d7),
            common.make_pio_pin(pins.d8),
            common.make_pio_pin(pins.d9),
            common.make_pio_pin(pins.d10),
            common.make_pio_pin(pins.d11),
            common.make_pio_pin(pins.d12),
            common.make_pio_pin(pins.d13),
            common.make_pio_pin(pins.d14),
            common.make_pio_pin(pins.d15),
        ];
        let de_pin = common.make_pio_pin(pins.de);
        let pclk_pin = common.make_pio_pin(pins.pclk);

        let mut cfg = Config::default();
        cfg.use_program(&common.load_program(&prg.program), &[&de_pin, &pclk_pin]);
        {
            let pin_refs: [&_; 16] = core::array::from_fn(|i| &pio_pins[i]);
            cfg.set_out_pins(&pin_refs);
        }
        let sys_hz = embassy_rp::clocks::clk_sys_freq() as u64;
        let divider = (sys_hz * 256 / (2 * PCLK_HZ as u64)) as u32;
        cfg.clock_divider = FixedU32::<U8>::from_bits(divider);
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            direction: ShiftDirection::Right,
        };
        cfg.fifo_join = FifoJoin::TxOnly;
        sm.set_config(&cfg);

        {
            let data_refs: [&_; 16] = core::array::from_fn(|i| &pio_pins[i]);
            sm.set_pins(embassy_rp::gpio::Level::Low, &data_refs);
            sm.set_pin_dirs(Direction::Out, &data_refs);
            sm.set_pins(embassy_rp::gpio::Level::Low, &[&de_pin, &pclk_pin]);
            sm.set_pin_dirs(Direction::Out, &[&de_pin, &pclk_pin]);
        }

        use embassy_rp::pio::program::{InstructionOperands, OutDestination};
        sm.tx().push(DISPLAY_WIDTH as u32 - 1);
        unsafe {
            sm.exec_instr(
                InstructionOperands::PULL {
                    if_empty: false,
                    block: true,
                }
                .encode(),
            );
            sm.exec_instr(
                InstructionOperands::OUT {
                    destination: OutDestination::ISR,
                    bit_count: 32,
                }
                .encode(),
            );
        }

        let regs = dma.regs();
        regs.write_addr().write_value(sm.tx_fifo_ptr() as u32);
        let mut ctrl = pac::dma::regs::Ctrl(0);
        ctrl.set_treq_sel(sm.tx_treq());
        ctrl.set_data_size(pac::dma::vals::DataSize::SizeWord);
        ctrl.set_incr_read(true);
        ctrl.set_incr_write(false);
        ctrl.set_high_priority(true);
        ctrl.set_chain_to(dma.number());
        ctrl.set_irq_quiet(true);
        ctrl.set_en(true);

        sm.set_enable(true);

        Self {
            sm,
            dma,
            ctrl,
            fb_addr,
        }
    }

    pub fn busy(&self) -> bool {
        self.dma.regs().ctrl_trig().read().busy()
    }

    /// Kick one frame from the framebuffer base.
    pub async fn present(&mut self) {
        while self.busy() {
            Timer::after_micros(500).await;
        }
        let regs = self.dma.regs();
        regs.read_addr().write_value(self.fb_addr);
        regs.trans_count().write(|w| {
            w.set_mode(pac::dma::vals::TransCountMode::Normal);
            w.set_count(FRAME_WORDS);
        });
        regs.ctrl_trig().write_value(self.ctrl);
    }

    pub fn stats(&mut self) -> DpiStats {
        let regs = self.dma.regs();
        DpiStats {
            busy: regs.ctrl_trig().read().busy(),
            read_offset: regs.read_addr().read().wrapping_sub(self.fb_addr),
            remaining_words: regs.trans_count().read().count(),
            sm_pc: self.sm.get_addr(),
            tx_fifo_level: self.sm.tx().level(),
            tx_stalled: self.sm.tx().stalled(),
        }
    }
}

/// Initialise row prefix words and fill with a solid colour.
///
/// # Safety
/// `fb` must point at least `FRAME_BYTES` of writable PSRAM.
pub unsafe fn init_framebuffer(fb: *mut u8, fill_rgb565: u16) {
    let px = fill_rgb565.to_le_bytes();
    for row in 0..DISPLAY_HEIGHT {
        let row_base = unsafe { fb.add(row * ROW_BYTES) };
        unsafe {
            core::ptr::write_bytes(row_base, 0, ROW_PIXEL_OFFSET);
            for x in 0..DISPLAY_WIDTH {
                let p = row_base.add(ROW_PIXEL_OFFSET + x * 2);
                core::ptr::copy_nonoverlapping(px.as_ptr(), p, 2);
            }
        }
    }
}
