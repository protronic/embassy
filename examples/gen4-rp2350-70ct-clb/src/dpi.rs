//! PIO + DMA parallel RGB (DPI) scan-out for the gen4-RP2350 800×480 panel.
//!
//! A single PIO state machine shifts 16-bit RGB565 pixels out of GPIO18..33
//! and generates DE + PCLK on its two side-set pins. The panel runs in
//! **DE-only sync mode**: HSYNC/VSYNC are parked at their inactive level and
//! the panel latches lines purely from the DE envelope.
//!
//! One DMA channel streams a whole frame from the PSRAM framebuffer into the
//! PIO TX FIFO. Each framebuffer row starts with one 32-bit *prefix word*
//! that the PIO program discards while DE is low; this keeps the state
//! machine stalled at a safe point (DE low, PCLK frozen) between lines,
//! frames, and on FIFO underruns. Vertical blanking is simply the pause
//! between two DMA frame kicks, paced by [`Dpi::present`].
//!
//! ```text
//!  PSRAM frame layout (per row): [prefix:u32][800 × RGB565:u16] = 1604 B
//!  Full frame: 480 rows = 769 920 B, streamed as 192 480 words per kick.
//! ```

use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{Common, Config, Direction, FifoJoin, Instance, ShiftConfig, ShiftDirection, StateMachine};
use embassy_rp::{Peri, dma, pac, peripherals};
use embassy_time::Timer;
use fixed::FixedU32;
use fixed::types::extra::U8;

use crate::gen4_board::{DISPLAY_HEIGHT, DISPLAY_WIDTH};

/// Pixel clock during the active part of a line, in Hz.
///
/// The scan-out streams straight from QSPI PSRAM, which tops out well below
/// the panel's nominal 33 MHz dot clock, so the panel is driven slowly (TN
/// DPI panels are shift registers and tolerate low dot clocks in DE mode).
/// 10 MHz leaves PSRAM bandwidth for the LVGL flush writes:
/// `800×480×2 B × ~20 fps ≈ 15 MB/s` of the ~18 MB/s practical QSPI budget.
pub const PCLK_HZ: u32 = 10_000_000;

/// 32-bit words per framebuffer row (1 prefix word + 800 pixels).
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
///
/// Healthy scan-out: `read_offset`/`remaining_words` advance between
/// snapshots while a frame streams, `sm_pc` sits in the pixel loop or at the
/// row-prefix stall, and `tx_fifo_level` is mostly non-zero. A permanently
/// `busy` channel with a frozen `read_offset` points at PSRAM reads stalling;
/// `tx_stalled = true` with `busy = false` is normal between frames.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct DpiStats {
    /// Frame DMA channel busy flag.
    pub busy: bool,
    /// Current DMA read position, in bytes from the framebuffer base.
    pub read_offset: u32,
    /// Words left in the current frame transfer.
    pub remaining_words: u32,
    /// PIO state machine program counter (instruction memory address).
    pub sm_pc: u8,
    /// TX FIFO fill level (0..8 with the joined FIFO).
    pub tx_fifo_level: u8,
    /// Sticky TX FIFO underrun/stall flag (cleared on read).
    pub tx_stalled: bool,
}

/// The LCD interface pins (board-fixed GPIO18..GPIO35).
pub struct LcdPins {
    pub d0: Peri<'static, peripherals::PIN_18>,
    pub d1: Peri<'static, peripherals::PIN_19>,
    pub d2: Peri<'static, peripherals::PIN_20>,
    pub d3: Peri<'static, peripherals::PIN_21>,
    pub d4: Peri<'static, peripherals::PIN_22>,
    pub d5: Peri<'static, peripherals::PIN_23>,
    pub d6: Peri<'static, peripherals::PIN_24>,
    pub d7: Peri<'static, peripherals::PIN_25>,
    pub d8: Peri<'static, peripherals::PIN_26>,
    pub d9: Peri<'static, peripherals::PIN_27>,
    pub d10: Peri<'static, peripherals::PIN_28>,
    pub d11: Peri<'static, peripherals::PIN_29>,
    pub d12: Peri<'static, peripherals::PIN_30>,
    pub d13: Peri<'static, peripherals::PIN_31>,
    pub d14: Peri<'static, peripherals::PIN_32>,
    pub d15: Peri<'static, peripherals::PIN_33>,
    pub de: Peri<'static, peripherals::PIN_34>,
    pub pclk: Peri<'static, peripherals::PIN_35>,
}

/// DPI scan-out driver: owns the PIO state machine and the frame DMA channel.
pub struct Dpi<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
    dma: dma::Channel<'d>,
    ctrl: pac::dma::regs::Ctrl,
    fb_addr: u32,
}

impl<'d, PIO: Instance, const SM: usize> Dpi<'d, PIO, SM> {
    /// Set up the scan-out state machine and DMA channel.
    ///
    /// `fb_addr` is the (word-aligned) framebuffer base inside PSRAM.
    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        dma: dma::Channel<'d>,
        pins: LcdPins,
        fb_addr: u32,
    ) -> Self {
        // Side-set bit 0 = DE, bit 1 = PCLK. The panel samples data on the
        // rising PCLK edge, which lands mid-pixel (data changes on `out`,
        // PCLK rises on the `jmp`). During horizontal blanking DE is held
        // low while PCLK keeps toggling; between lines/frames the program
        // stalls on `out null` with DE low and PCLK frozen.
        let prg = pio_asm!(
            ".side_set 2",
            ".wrap_target",
            "out null, 32      side 0b00", // row prefix word; stalls here when idle
            "mov x, isr        side 0b00", // X = pixels per line - 1 (preloaded)
            "pixels:",
            "out pins, 16      side 0b01", // drive RGB565, DE high, PCLK low
            "jmp x-- pixels    side 0b11", // PCLK high: panel samples the pixel
            "set y, 7          side 0b00", // horizontal blanking: 8 × 16 cycles
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
        // SM runs at 2 cycles per pixel.
        let sys_hz = embassy_rp::clocks::clk_sys_freq() as u64;
        let divider = (sys_hz * 256 / (2 * PCLK_HZ as u64)) as u32;
        cfg.clock_divider = FixedU32::<U8>::from_bits(divider);
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 32,
            // Shift right: pin 18 gets bit 0 of each RGB565 pixel, and the
            // first pixel of each 32-bit word is the low half-word.
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

        // Preload ISR with the per-line pixel count: push the value, then
        // execute `pull` + `out isr, 32` on the (still disabled) SM.
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

        // One-time DMA channel setup: read words from PSRAM (incrementing),
        // write to the PIO TX FIFO (fixed address), paced by the TX DREQ.
        let regs = dma.regs();
        regs.write_addr().write_value(sm.tx_fifo_ptr() as u32);
        let mut ctrl = pac::dma::regs::Ctrl(0);
        ctrl.set_treq_sel(sm.tx_treq());
        ctrl.set_data_size(pac::dma::vals::DataSize::SizeWord);
        ctrl.set_incr_read(true);
        ctrl.set_incr_write(false);
        ctrl.set_high_priority(true);
        ctrl.set_chain_to(dma.number()); // chain to self = no chaining
        ctrl.set_irq_quiet(true);
        ctrl.set_en(true);

        sm.set_enable(true);

        Self { sm, dma, ctrl, fb_addr }
    }

    /// `true` while the current frame is still streaming out.
    pub fn busy(&self) -> bool {
        self.dma.regs().ctrl_trig().read().busy()
    }

    /// Kick one frame: wait for the previous frame to finish, then restart
    /// the DMA stream from the framebuffer base.
    ///
    /// Call this at the desired refresh cadence; the gap between two calls
    /// is the vertical blanking interval (the PIO sits stalled with DE low).
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

    /// Snapshot of the scan-out state for debug logging.
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

    /// Stop the scan-out (aborts the DMA stream and disables the SM).
    pub fn stop(&mut self) {
        pac::DMA
            .chan_abort()
            .write(|w| w.set_chan_abort(1 << self.dma.number()));
        while pac::DMA.chan_abort().read().chan_abort() != 0 {}
        self.sm.set_enable(false);
    }
}

/// Fill the framebuffer rows' prefix words (call once before scan-out).
///
/// # Safety
/// `fb` must point at least `FRAME_BYTES` of writable memory.
pub unsafe fn init_framebuffer(fb: *mut u8, fill_rgb565: u16) {
    let px = fill_rgb565.to_le_bytes();
    // SAFETY: caller guarantees the framebuffer region is valid.
    unsafe {
        for row in 0..DISPLAY_HEIGHT {
            let row_base = fb.add(row * ROW_BYTES);
            // Prefix word content is discarded by the PIO (`out null, 32`).
            core::ptr::write_bytes(row_base, 0, ROW_PIXEL_OFFSET);
            for x in 0..DISPLAY_WIDTH {
                let p = row_base.add(ROW_PIXEL_OFFSET + x * 2);
                core::ptr::copy_nonoverlapping(px.as_ptr(), p, 2);
            }
        }
    }
}

/// RGB565 color of the 8-bar test pattern at horizontal position `x`
/// (white, yellow, cyan, green, magenta, red, blue, black).
pub fn test_pattern_color(x: usize) -> u16 {
    const BARS: [u16; 8] = [
        0xFFFF, // white
        0xFFE0, // yellow
        0x07FF, // cyan
        0x07E0, // green
        0xF81F, // magenta
        0xF800, // red
        0x001F, // blue
        0x0000, // black
    ];
    BARS[(x * BARS.len() / DISPLAY_WIDTH).min(BARS.len() - 1)]
}

/// Draw the 8-bar color test pattern into the framebuffer (and initialize
/// the row prefix words). Useful to verify the scan-out and the RGB data
/// pin mapping independently of any UI library.
///
/// # Safety
/// `fb` must point at least `FRAME_BYTES` of writable memory.
pub unsafe fn fill_test_pattern(fb: *mut u8) {
    // SAFETY: caller guarantees the framebuffer region is valid.
    unsafe {
        for row in 0..DISPLAY_HEIGHT {
            let row_base = fb.add(row * ROW_BYTES);
            core::ptr::write_bytes(row_base, 0, ROW_PIXEL_OFFSET);
            for x in 0..DISPLAY_WIDTH {
                let px = test_pattern_color(x).to_le_bytes();
                let p = row_base.add(ROW_PIXEL_OFFSET + x * 2);
                core::ptr::copy_nonoverlapping(px.as_ptr(), p, 2);
            }
        }
    }
}
