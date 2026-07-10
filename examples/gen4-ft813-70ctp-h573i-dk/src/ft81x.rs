//! Minimal FT81x (EVE2) host driver over blocking SPI.
//!
//! Targets the FT813 inside 4D Systems **gen4-FT813-xx** display modules
//! (here: gen4-FT813-70CTP-CLB, 800×480 with capacitive touch). LVGL renders
//! RGB565 partial stripes on the host; pixels are uploaded into `RAM_G` with
//! fast host SPI bursts ([`Ft81x::blit`]). The EVE co-processor builds the
//! bitmap display list once ([`Ft81x::co_show_framebuffer`]) and the FT813
//! continuously scans `RAM_G` out to the panel. Touch is read from the
//! built-in capacitive engine (`REG_TOUCH_SCREEN_XY`) over the same SPI bus.
//!
//! References: FT81x datasheet + BRT_AN_033 (EVE2 programming guide); panel
//! timings from the 4D Systems gen4 module datasheet ("7.0" LCD Timing
//! Characteristic (TN)"). The module fits a 12 MHz crystal → `CLKEXT`
//! (with a one-shot `CLKINT` fallback for crystal-less revisions).

use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::{self, Spi};
use embassy_time::{Duration, Instant, Timer};

/// Horizontal panel resolution in pixels.
pub const DISPLAY_WIDTH: usize = 800;
/// Vertical panel resolution in pixels.
pub const DISPLAY_HEIGHT: usize = 480;

/// Framebuffer byte stride (RGB565).
pub const LINE_STRIDE: usize = DISPLAY_WIDTH * 2;
/// Framebuffer size in `RAM_G` (768 000 B of the 1 MiB available).
pub const FRAME_BYTES: usize = LINE_STRIDE * DISPLAY_HEIGHT;

// ── FT81x memory map ────────────────────────────────────────────────────────
/// Graphics RAM (1 MiB) — holds the RGB565 framebuffer at offset 0.
pub const RAM_G: u32 = 0x00_0000;
#[allow(dead_code)]
const RAM_DL: u32 = 0x30_0000;
/// Co-processor command ring buffer (4 KiB).
const RAM_CMD: u32 = 0x30_8000;
const RAM_CMD_SIZE: u16 = 4096;

const REG_ID: u32 = 0x30_2000;
const REG_FRAMES: u32 = 0x30_2004;
const REG_CLOCK: u32 = 0x30_2008;
const REG_FREQUENCY: u32 = 0x30_200C;
const REG_CPURESET: u32 = 0x30_2020;
const REG_HCYCLE: u32 = 0x30_202C;
const REG_HOFFSET: u32 = 0x30_2030;
const REG_HSIZE: u32 = 0x30_2034;
const REG_HSYNC0: u32 = 0x30_2038;
const REG_HSYNC1: u32 = 0x30_203C;
const REG_VCYCLE: u32 = 0x30_2040;
const REG_VOFFSET: u32 = 0x30_2044;
const REG_VSIZE: u32 = 0x30_2048;
const REG_VSYNC0: u32 = 0x30_204C;
const REG_VSYNC1: u32 = 0x30_2050;
#[allow(dead_code)]
const REG_DLSWAP: u32 = 0x30_2054;
const REG_DITHER: u32 = 0x30_2060;
const REG_SWIZZLE: u32 = 0x30_2064;
const REG_CSPREAD: u32 = 0x30_2068;
const REG_PCLK_POL: u32 = 0x30_206C;
const REG_PCLK: u32 = 0x30_2070;
const REG_GPIOX_DIR: u32 = 0x30_2098;
const REG_GPIOX: u32 = 0x30_209C;
const REG_PWM_HZ: u32 = 0x30_20D0;
const REG_PWM_DUTY: u32 = 0x30_20D4;
const REG_TOUCH_MODE: u32 = 0x30_2104;
const REG_CTOUCH_EXTENDED: u32 = 0x30_2108;
const REG_TOUCH_SCREEN_XY: u32 = 0x30_2124;
const REG_CMD_READ: u32 = 0x30_20F8;
const REG_CMD_WRITE: u32 = 0x30_20FC;

// ── Host commands (3-byte frames) ───────────────────────────────────────────
const HCMD_ACTIVE: u8 = 0x00;
const HCMD_CLKEXT: u8 = 0x44;
const HCMD_CLKINT: u8 = 0x48;
const HCMD_RST_PULSE: u8 = 0x68;

// ── Co-processor command opcodes ────────────────────────────────────────────
const CMD_DLSTART: u32 = 0xFFFF_FF00;
const CMD_SWAP: u32 = 0xFFFF_FF01;
const CMD_TEXT: u32 = 0xFFFF_FF0C;
/// Write bytes into FT81x graphics memory (`RAM_G`, …).
const CMD_MEMWRITE: u32 = 0xFFFF_FF1A;
/// Zero a block of memory.
const CMD_MEMZERO: u32 = 0xFFFF_FF1C;
/// Max payload bytes per `CMD_MEMWRITE` submission (4 KiB FIFO minus headers).
const CO_MEMWRITE_CHUNK: usize = 4000;
/// `REG_CMD_READ` reports this sentinel when the co-processor has faulted.
const CMD_FAULT: u16 = 0x0FFF;

// ── Panel timings: 4D Systems gen4-FT813-70 (7.0" 800×480), from the module
// datasheet's "7.0" LCD Timing Characteristic (TN)" table (TYP values).
pub const PANEL_H_CYCLE: u16 = 928;
pub const PANEL_H_OFFSET: u16 = 44;
pub const PANEL_H_SYNC0: u16 = 0;
pub const PANEL_H_SYNC1: u16 = 4;
pub const PANEL_V_CYCLE: u16 = 525;
pub const PANEL_V_OFFSET: u16 = 32;
pub const PANEL_V_SYNC0: u16 = 0;
pub const PANEL_V_SYNC1: u16 = 1;
pub const PANEL_PCLK_DIV: u8 = 2;
pub const PANEL_PCLK_POL: u8 = 1;
pub const PANEL_SWIZZLE: u8 = 0;
pub const PANEL_CSPREAD: u8 = 0;

const H_CYCLE: u16 = PANEL_H_CYCLE;
const H_OFFSET: u16 = PANEL_H_OFFSET;
const H_SYNC0: u16 = PANEL_H_SYNC0;
const H_SYNC1: u16 = PANEL_H_SYNC1;
const V_CYCLE: u16 = PANEL_V_CYCLE;
const V_OFFSET: u16 = PANEL_V_OFFSET;
const V_SYNC0: u16 = PANEL_V_SYNC0;
const V_SYNC1: u16 = PANEL_V_SYNC1;
const PCLK_DIV: u8 = PANEL_PCLK_DIV;
const PCLK_POL: u8 = PANEL_PCLK_POL;
const SWIZZLE: u8 = PANEL_SWIZZLE;
const CSPREAD: u8 = PANEL_CSPREAD;

/// `RECTS` primitive selector for [`dl_begin`].
pub const RECTS: u32 = 9;
/// `POINTS` primitive selector for [`dl_begin`] (filled circles, radius set by
/// [`dl_point_size`]).
pub const POINTS: u32 = 2;
/// `CMD_TEXT` option: centre the string on the given anchor point
/// (`OPT_CENTERX | OPT_CENTERY`).
pub const OPT_CENTER: u16 = 0x0600;

// ── Display-list encoding (subset) ──────────────────────────────────────────
/// `CLEAR_COLOR_RGB` — background colour used by the next [`dl_clear`].
pub const fn dl_clear_color_rgb(r: u8, g: u8, b: u8) -> u32 {
    0x0200_0000 | ((r as u32) << 16) | ((g as u32) << 8) | b as u32
}
/// `CLEAR` — clear colour + stencil + tag buffers.
pub const fn dl_clear() -> u32 {
    0x2600_0007
}
/// `COLOR_RGB` — drawing colour for subsequent primitives / text.
pub const fn dl_color_rgb(r: u8, g: u8, b: u8) -> u32 {
    0x0400_0000 | ((r as u32) << 16) | ((g as u32) << 8) | b as u32
}
/// `BEGIN(prim)` — start a primitive (e.g. [`RECTS`]).
pub const fn dl_begin(prim: u32) -> u32 {
    0x1F00_0000 | (prim & 0x0F)
}
/// `VERTEX_FORMAT(frac)` — number of sub-pixel fraction bits in
/// [`dl_vertex2f`] (0 = whole pixels, default after reset is 4).
pub const fn dl_vertex_format(frac: u8) -> u32 {
    0x2700_0000 | (frac as u32 & 0x07)
}
/// `VERTEX2F(x, y)` — a vertex in the units set by [`dl_vertex_format`].
pub const fn dl_vertex2f(x: i16, y: i16) -> u32 {
    (1 << 30) | ((x as u32 & 0x7FFF) << 15) | (y as u32 & 0x7FFF)
}
/// `POINT_SIZE(radius)` — radius of [`POINTS`] in 1/16 pixel.
pub const fn dl_point_size(radius: u16) -> u32 {
    0x0D00_0000 | (radius as u32 & 0x1FFF)
}
const fn dl_bitmap_handle(h: u32) -> u32 {
    0x0500_0000 | (h & 0x1F)
}
const fn dl_bitmap_source(addr: u32) -> u32 {
    0x0100_0000 | (addr & 0x3F_FFFF)
}
/// `BITMAP_LAYOUT` — format 7 = RGB565; low 10/9 bits of stride/height.
const fn dl_bitmap_layout(format: u32, stride: u32, height: u32) -> u32 {
    0x0700_0000 | (format << 19) | ((stride & 0x3FF) << 9) | (height & 0x1FF)
}
const fn dl_bitmap_layout_h(stride: u32, height: u32) -> u32 {
    0x2800_0000 | (((stride >> 10) & 0x3) << 2) | ((height >> 9) & 0x3)
}
/// `BITMAP_SIZE` — NEAREST filter, BORDER wrap; low 9 bits of w/h.
const fn dl_bitmap_size(width: u32, height: u32) -> u32 {
    0x0800_0000 | ((width & 0x1FF) << 9) | (height & 0x1FF)
}
const fn dl_bitmap_size_h(width: u32, height: u32) -> u32 {
    0x2900_0000 | (((width >> 9) & 0x3) << 2) | ((height >> 9) & 0x3)
}
const fn dl_begin_bitmaps() -> u32 {
    0x1F00_0001
}
const fn dl_vertex2ii(x: u32, y: u32) -> u32 {
    0x8000_0000 | ((x & 0x1FF) << 21) | ((y & 0x1FF) << 12)
}
/// `END` — finish the current primitive.
pub const fn dl_end() -> u32 {
    0x2100_0000
}
/// `DISPLAY` — end of the display list.
pub const fn dl_display() -> u32 {
    0x0000_0000
}
#[allow(dead_code)]
const DLSWAP_FRAME: u32 = 2;

/// One capacitive touch sample in panel coordinates.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, defmt::Format)]
pub struct TouchPoint {
    /// Horizontal position in pixels (valid while `pressed`).
    pub x: i32,
    /// Vertical position in pixels (valid while `pressed`).
    pub y: i32,
    /// `true` while the panel reports contact.
    pub pressed: bool,
}

/// FT81x bring-up / transport errors.
#[derive(Debug, defmt::Format)]
pub enum Error {
    /// SPI transfer failed.
    Spi(spi::Error),
    /// `REG_ID` never reported `0x7C` — wiring or clock-source problem.
    ChipIdTimeout,
    /// The graphics engine never left reset.
    ResetTimeout,
    /// The co-processor did not drain the command FIFO in time.
    CoprocTimeout,
    /// The co-processor faulted on a malformed command (`REG_CMD_READ`
    /// stuck at `0xFFF`).
    CoprocFault,
}

impl From<spi::Error> for Error {
    fn from(e: spi::Error) -> Self {
        Self::Spi(e)
    }
}

/// FT813 on blocking SPI with manual CS and power-down pins.
///
/// All methods are synchronous except [`Ft81x::init`], which needs the
/// power-up delays. Single-owner: the LVGL flush callback and the touch poll
/// must run on the same executor task (see `oxivgl/display.rs`).
pub struct Ft81x {
    spi: Spi<'static, Blocking, spi::mode::Master>,
    cs: Output<'static>,
    pd: Output<'static>,
    config: spi::Config,
    /// Local mirror of `REG_CMD_WRITE`: byte offset of the next free slot in
    /// the `RAM_CMD` ring (always a multiple of 4, masked to the 4 KiB ring).
    cmd_offset: u16,
}

impl Ft81x {
    /// Wrap an already-configured blocking SPI bus (mode 0, ≤ 11 MHz for
    /// [`Ft81x::init`]; raise afterwards with [`Ft81x::set_spi_frequency`]).
    pub fn new(
        spi: Spi<'static, Blocking, spi::mode::Master>,
        cs: Output<'static>,
        pd: Output<'static>,
        config: spi::Config,
    ) -> Self {
        Self {
            spi,
            cs,
            pd,
            config,
            cmd_offset: 0,
        }
    }

    /// Change the SPI clock (call after [`Ft81x::init`]; FT81x max is 30 MHz).
    pub fn set_spi_frequency(&mut self, hz: u32) {
        self.config.frequency = embassy_stm32::time::Hertz(hz);
        // Only fails for unreachable divisors; keep the old clock in that case.
        if self.spi.set_config(&self.config).is_err() {
            defmt::warn!("ft81x: SPI frequency {} Hz not reachable, keeping previous", hz);
        }
    }

    // ── LVGL `lv_draw_eve` SPI bridge (raw bytes; CS/PD driven by op_cb) ────

    pub fn eve_cs_assert(&mut self) {
        self.cs.set_low();
    }

    pub fn eve_cs_deassert(&mut self) {
        self.cs.set_high();
    }

    pub fn eve_pd_set(&mut self, powered_down: bool) {
        if powered_down {
            self.pd.set_low();
        } else {
            self.pd.set_high();
        }
    }

    pub fn eve_spi_send(&mut self, data: &[u8]) -> Result<(), Error> {
        self.spi.blocking_write(data).map_err(Error::from)
    }

    pub fn eve_spi_receive(&mut self, data: &mut [u8]) -> Result<(), Error> {
        self.spi.blocking_read(data).map_err(Error::from)
    }

    fn host_command(&mut self, cmd: u8) -> Result<(), Error> {
        self.cs.set_low();
        let r = self.spi.blocking_write(&[cmd, 0x00, 0x00]);
        self.cs.set_high();
        r.map_err(Error::from)
    }

    /// Write `data` to `addr` (burst, auto-incrementing).
    pub fn wr_bytes(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        self.cs.set_low();
        let hdr = [(addr >> 16) as u8 & 0x3F | 0x80, (addr >> 8) as u8, addr as u8];
        let r = self
            .spi
            .blocking_write(&hdr)
            .and_then(|()| self.spi.blocking_write(data));
        self.cs.set_high();
        r.map_err(Error::from)
    }

    fn rd_bytes(&mut self, addr: u32, data: &mut [u8]) -> Result<(), Error> {
        self.cs.set_low();
        // 3 address bytes (top bits 00 = read) + 1 dummy byte.
        let hdr = [(addr >> 16) as u8 & 0x3F, (addr >> 8) as u8, addr as u8, 0x00];
        let r = self
            .spi
            .blocking_write(&hdr)
            .and_then(|()| self.spi.blocking_read(data));
        self.cs.set_high();
        r.map_err(Error::from)
    }

    fn wr8(&mut self, addr: u32, v: u8) -> Result<(), Error> {
        self.wr_bytes(addr, &[v])
    }

    fn wr16(&mut self, addr: u32, v: u16) -> Result<(), Error> {
        self.wr_bytes(addr, &v.to_le_bytes())
    }

    fn wr32(&mut self, addr: u32, v: u32) -> Result<(), Error> {
        self.wr_bytes(addr, &v.to_le_bytes())
    }

    fn rd8(&mut self, addr: u32) -> Result<u8, Error> {
        let mut b = [0u8; 1];
        self.rd_bytes(addr, &mut b)?;
        Ok(b[0])
    }

    fn rd32(&mut self, addr: u32) -> Result<u32, Error> {
        let mut b = [0u8; 4];
        self.rd_bytes(addr, &mut b)?;
        Ok(u32::from_le_bytes(b))
    }

    /// Power up and configure the FT813 for the 800×480 gen4 panel.
    ///
    /// Loads the panel timings and a blank display list but leaves the pixel
    /// clock and backlight **off**. After optionally raising the SPI speed
    /// ([`Ft81x::set_spi_frequency`]) and loading the first frame, call
    /// [`Ft81x::enable_display`] + [`Ft81x::set_backlight`] to light it up.
    pub async fn init(&mut self) -> Result<(), Error> {
        // The gen4-FT81x schematic (REV 1.3) fits a 12 MHz crystal on the
        // FT813, so prefer CLKEXT (measured 60.000 MHz core clock vs ~60.3 MHz
        // from the internal RC oscillator). Fall back to CLKINT once, for
        // module revisions without the crystal.
        for (attempt, clk) in [HCMD_CLKEXT, HCMD_CLKINT].into_iter().enumerate() {
            self.power_cycle().await;
            self.host_command(clk)?;
            self.host_command(HCMD_ACTIVE)?;
            Timer::after_millis(40).await;

            if self.poll_chip_id().await? {
                if attempt > 0 {
                    defmt::warn!("ft81x: no REG_ID with CLKEXT — running on the internal oscillator (CLKINT)");
                } else {
                    defmt::info!("ft81x: clock source CLKEXT (12 MHz crystal)");
                }
                return self.configure().await;
            }
            defmt::warn!("ft81x: no REG_ID with clock cmd {=u8:#x}, retrying", clk);
        }
        Err(Error::ChipIdTimeout)
    }

    async fn power_cycle(&mut self) {
        self.pd.set_low();
        Timer::after_millis(6).await;
        self.pd.set_high();
        Timer::after_millis(21).await;
        // Belt and braces: core reset via host command as well.
        let _ = self.host_command(HCMD_RST_PULSE);
        Timer::after_millis(21).await;
    }

    async fn poll_chip_id(&mut self) -> Result<bool, Error> {
        let deadline = Instant::now() + Duration::from_millis(400);
        while Instant::now() < deadline {
            if self.rd8(REG_ID)? == 0x7C {
                return Ok(true);
            }
            Timer::after_millis(5).await;
        }
        Ok(false)
    }

    async fn configure(&mut self) -> Result<(), Error> {
        // Wait for the boot ROM to release all engines from reset. This MUST
        // happen before anything is written to REG_CPURESET: right after
        // REG_ID goes 0x7C the touch engine is still booting, and writing the
        // register in that window rips it out of reset mid-initialisation —
        // capacitive touch then stays dead (REG_TOUCH_SCREEN_XY stuck at
        // 0x0000_0000, a permanent phantom press at the origin).
        let deadline = Instant::now() + Duration::from_millis(400);
        while self.rd8(REG_CPURESET)? != 0 {
            if Instant::now() >= deadline {
                return Err(Error::ResetTimeout);
            }
            Timer::after_millis(5).await;
        }

        // Reset the co-processor and clear its FIFO pointers. A previous run
        // that was halted mid-command (e.g. the debugger stopped the MCU during
        // an SPI burst) can leave the co-processor wedged so the first CMD_SWAP
        // never drains; the PDN power-cycle alone does not always clear it.
        // REG_CPURESET bit 0 = co-processor only (touch/audio untouched).
        self.wr8(REG_CPURESET, 1)?;
        self.wr32(REG_CMD_READ, 0)?;
        self.wr32(REG_CMD_WRITE, 0)?;
        self.wr8(REG_CPURESET, 0)?;
        Timer::after_millis(2).await;

        // Capacitive touch: compatibility mode (single touch), continuous.
        self.wr8(REG_CTOUCH_EXTENDED, 1)?;
        self.wr8(REG_TOUCH_MODE, 3)?;

        // Backlight PWM configured but dark for now.
        self.wr16(REG_PWM_HZ, 250)?;
        self.wr8(REG_PWM_DUTY, 0)?;

        // Sync our local FIFO write pointer with the co-processor's.
        self.cmd_offset = (self.rd32(REG_CMD_WRITE)? & 0x0FFF) as u16;

        // Blank co-processor frame so later timing writes stick. Apply the real
        // panel timings after the first visible frame ([`Ft81x::apply_panel_timings`])
        // while PCLK is still off.
        self.co_start()?;
        self.co_cmd(dl_clear_color_rgb(0, 0, 0))?;
        self.co_cmd(dl_clear())?;
        self.co_cmd(dl_display())?;
        self.co_swap()?;
        self.co_run().await?;

        // NB: DISP and the pixel clock are deliberately *not* enabled here.
        // Call [`Ft81x::enable_display`] as the last bring-up step, after the
        // final SPI speed and the first co-processor frame.

        defmt::info!("ft81x: init ok (REG_ID=0x7c, 800x480, display still off)");
        Ok(())
    }

    /// Enable the panel output: drive DISP high (`REG_GPIOX` bit 15) and start
    /// the pixel clock (`REG_PCLK`).
    ///
    /// Call this **last**, after [`Ft81x::set_spi_frequency`], the first
    /// co-processor frame, and [`Ft81x::apply_panel_timings`]. Writing
    /// `REG_PCLK` before the final SPI reconfiguration leaves it cleared again
    /// (observed on the H573I-DK: `set_config` toggles SPE and the FT813 loses
    /// the pixel clock).
    pub fn enable_display(&mut self) -> Result<(), Error> {
        self.enable_disp()?;
        self.wr8(REG_PCLK, PCLK_DIV)
    }

    /// Drive DISP high (`REG_GPIOX` bit 15) without starting the pixel clock.
    ///
    /// Both `REG_GPIOX_DIR` and `REG_GPIOX` are read-modify-written so only bit
    /// 15 (DISP) changes. A blind `REG_GPIOX_DIR = 0x8000` forces GPIO0..3 to
    /// inputs, which on the gen4 module drops the capacitive-touch controller's
    /// reset/INT line and kills touch permanently — so preserve the low bits.
    pub fn enable_disp(&mut self) -> Result<(), Error> {
        let dir = self.rd32(REG_GPIOX_DIR)? as u16;
        self.wr16(REG_GPIOX_DIR, dir | 0x8000)?;
        let gpiox = self.rd32(REG_GPIOX)? as u16;
        self.wr16(REG_GPIOX, gpiox | 0x8000)?;
        Ok(())
    }

    // ── Co-processor (RAM_CMD FIFO) ─────────────────────────────────────────
    //
    // The FT813 EVE co-processor renders high-level objects (text, buttons,
    // gradients, …) and full display lists from a 4 KiB command ring. Push
    // words with [`Ft81x::co_cmd`] (raw DL words) / [`Ft81x::co_text`], bracket
    // a frame with [`Ft81x::co_start`] (`CMD_DLSTART`) and [`Ft81x::co_swap`]
    // (`CMD_SWAP`), then submit and wait with [`Ft81x::co_run`].

    /// Append `CMD_DLSTART` — begin a fresh display list.
    pub fn co_start(&mut self) -> Result<(), Error> {
        self.co_cmd(CMD_DLSTART)
    }

    /// Append `CMD_SWAP` — make the just-built display list the active frame.
    pub fn co_swap(&mut self) -> Result<(), Error> {
        self.co_cmd(CMD_SWAP)
    }

    /// Append one 32-bit word (a co-processor opcode or a raw display-list
    /// command such as [`dl_color_rgb`]) to the command FIFO.
    pub fn co_cmd(&mut self, word: u32) -> Result<(), Error> {
        self.wr32(RAM_CMD + self.cmd_offset as u32, word)?;
        self.cmd_offset = (self.cmd_offset + 4) % RAM_CMD_SIZE;
        Ok(())
    }

    /// Append `CMD_TEXT`: draw `s` at (`x`, `y`) in the given built-in `font`
    /// (16..34), using `options` (e.g. [`OPT_CENTER`]).
    pub fn co_text(&mut self, x: i16, y: i16, font: i16, options: u16, s: &str) -> Result<(), Error> {
        self.co_cmd(CMD_TEXT)?;
        self.co_cmd((x as u16 as u32) | ((y as u16 as u32) << 16))?;
        self.co_cmd((font as u16 as u32) | ((options as u32) << 16))?;
        // The string is NUL-terminated and zero-padded so the FIFO pointer
        // stays 4-byte aligned.
        let mut word = [0u8; 4];
        let mut n = 0;
        for &byte in s.as_bytes() {
            word[n] = byte;
            n += 1;
            if n == 4 {
                self.co_cmd(u32::from_le_bytes(word))?;
                word = [0; 4];
                n = 0;
            }
        }
        // Always emit a final word: its trailing zero bytes carry the NUL
        // terminator plus padding (also covers an exact multiple-of-4 string).
        self.co_cmd(u32::from_le_bytes(word))
    }

    /// Submit the queued commands (`REG_CMD_WRITE`) and wait for the
    /// co-processor to drain the FIFO.
    pub async fn co_run(&mut self) -> Result<(), Error> {
        self.wr32(REG_CMD_WRITE, self.cmd_offset as u32)?;
        let deadline = Instant::now() + Duration::from_millis(500);
        loop {
            let rd = (self.rd32(REG_CMD_READ)? & 0x0FFF) as u16;
            if rd == CMD_FAULT {
                return Err(Error::CoprocFault);
            }
            if rd == self.cmd_offset {
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(Error::CoprocTimeout);
            }
            Timer::after_millis(2).await;
        }
    }

    /// Blocking variant of [`Ft81x::co_run`] for LVGL flush callbacks.
    pub fn co_run_blocking(&mut self) -> Result<(), Error> {
        self.wr32(REG_CMD_WRITE, self.cmd_offset as u32)?;
        let deadline = Instant::now() + Duration::from_millis(500);
        loop {
            let rd = (self.rd32(REG_CMD_READ)? & 0x0FFF) as u16;
            if rd == CMD_FAULT {
                return Err(Error::CoprocFault);
            }
            if rd == self.cmd_offset {
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(Error::CoprocTimeout);
            }
        }
    }

    /// Reset the co-processor command FIFO after `CMD_FAULT`.
    fn co_recover_from_fault(&mut self) -> Result<(), Error> {
        defmt::warn!("ft81x: co-processor fault, resetting command FIFO");
        self.wr8(REG_CPURESET, 1)?;
        self.wr32(REG_CMD_READ, 0)?;
        self.wr32(REG_CMD_WRITE, 0)?;
        self.wr8(REG_CPURESET, 0)?;
        let deadline = Instant::now() + Duration::from_millis(100);
        while self.rd8(REG_CPURESET)? != 0 {
            if Instant::now() >= deadline {
                break;
            }
        }
        self.cmd_offset = 0;
        Ok(())
    }

    /// Free space in the circular `RAM_CMD` ring (bytes).
    fn co_fifo_free(&mut self) -> Result<usize, Error> {
        let rd = (self.rd32(REG_CMD_READ)? & 0x0FFF) as usize;
        if rd == usize::from(CMD_FAULT) {
            return Err(Error::CoprocFault);
        }
        let wr = usize::from(self.cmd_offset);
        let free = if wr >= rd {
            usize::from(RAM_CMD_SIZE) - (wr - rd)
        } else {
            rd - wr
        };
        Ok(free.saturating_sub(4))
    }

    /// Wait until the FIFO is idle and sync the local write pointer.
    fn co_wait_idle(&mut self) -> Result<(), Error> {
        let rd = (self.rd32(REG_CMD_READ)? & 0x0FFF) as u16;
        if rd == CMD_FAULT {
            self.co_recover_from_fault()?;
        }
        let wr = (self.rd32(REG_CMD_WRITE)? & 0x0FFF) as u16;
        if rd != wr {
            self.cmd_offset = wr;
            return self.co_run_blocking();
        }
        self.cmd_offset = wr;
        Ok(())
    }

    fn co_ensure_fifo(&mut self, needed: usize) -> Result<(), Error> {
        loop {
            match self.co_fifo_free() {
                Ok(free) if free >= needed => return Ok(()),
                Ok(_) => {
                    if let Err(Error::CoprocFault) = self.co_run_blocking() {
                        self.co_recover_from_fault()?;
                    }
                }
                Err(Error::CoprocFault) => self.co_recover_from_fault()?,
                Err(e) => return Err(e),
            }
        }
    }

    /// Queue `CMD_MEMWRITE` + payload without submitting yet. Drains the FIFO
    /// when the ring does not have enough free space.
    ///
    /// Layout per the FT81x programmer guide: opcode, ptr, num, data (padded).
    fn co_queue_memwrite(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        let padded = data.len().div_ceil(4) * 4;
        let needed = 12 + padded;
        self.co_ensure_fifo(needed)?;
        self.co_cmd(CMD_MEMWRITE)?;
        self.co_cmd(addr)?;
        self.co_cmd(data.len() as u32)?;
        let mut word = [0u8; 4];
        let mut n = 0usize;
        for &byte in data {
            word[n] = byte;
            n += 1;
            if n == 4 {
                self.co_cmd(u32::from_le_bytes(word))?;
                word = [0; 4];
                n = 0;
            }
        }
        if n != 0 {
            self.co_cmd(u32::from_le_bytes(word))?;
        }
        Ok(())
    }

    fn co_mem_write(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        match self.co_mem_write_inner(addr, data) {
            Err(Error::CoprocFault) => {
                self.co_recover_from_fault()?;
                self.co_mem_write_inner(addr, data)
            }
            other => other,
        }
    }

    fn co_mem_write_inner(&mut self, addr: u32, data: &[u8]) -> Result<(), Error> {
        self.co_wait_idle()?;
        let mut off = 0usize;
        while off < data.len() {
            let end = (off + CO_MEMWRITE_CHUNK).min(data.len());
            self.co_queue_memwrite(addr + off as u32, &data[off..end])?;
            self.co_run_blocking()?;
            off = end;
        }
        Ok(())
    }

    /// Fill the whole `RAM_G` framebuffer with one RGB565 colour via the
    /// co-processor (`CMD_MEMZERO` / `CMD_MEMWRITE`).
    pub fn co_clear_framebuffer(&mut self, rgb565: u16) -> Result<(), Error> {
        self.co_wait_idle()?;
        if rgb565 == 0 {
            self.co_cmd(CMD_MEMZERO)?;
            self.co_cmd(RAM_G)?;
            self.co_cmd(FRAME_BYTES as u32)?;
            self.co_run_blocking()
        } else {
            let mut line = [0u8; LINE_STRIDE];
            for px in line.chunks_exact_mut(2) {
                px.copy_from_slice(&rgb565.to_le_bytes());
            }
            for y in 0..DISPLAY_HEIGHT {
                let addr = RAM_G + (y * LINE_STRIDE) as u32;
                self.co_mem_write(addr, &line)?;
            }
            Ok(())
        }
    }

    /// Build and swap in the display list that scans `RAM_G` out to the panel.
    pub async fn co_show_framebuffer(&mut self) -> Result<(), Error> {
        self.co_wait_idle()?;
        let w = DISPLAY_WIDTH as u32;
        let h = DISPLAY_HEIGHT as u32;
        let stride = LINE_STRIDE as u32;
        self.co_start()?;
        self.co_cmd(dl_clear_color_rgb(0, 0, 0))?;
        self.co_cmd(dl_clear())?;
        self.co_cmd(dl_bitmap_handle(0))?;
        self.co_cmd(dl_bitmap_source(RAM_G))?;
        self.co_cmd(dl_bitmap_layout(7, stride, h))?;
        self.co_cmd(dl_bitmap_layout_h(stride, h))?;
        self.co_cmd(dl_bitmap_size(w, h))?;
        self.co_cmd(dl_bitmap_size_h(w, h))?;
        self.co_cmd(dl_begin_bitmaps())?;
        self.co_cmd(dl_vertex2ii(0, 0))?;
        self.co_cmd(dl_end())?;
        self.co_cmd(dl_display())?;
        self.co_swap()?;
        self.co_run().await
    }

    /// Fill the whole `RAM_G` framebuffer with one RGB565 colour via host SPI.
    pub fn clear_framebuffer(&mut self, rgb565: u16) -> Result<(), Error> {
        let mut line = [0u8; LINE_STRIDE];
        for px in line.chunks_exact_mut(2) {
            px.copy_from_slice(&rgb565.to_le_bytes());
        }
        for y in 0..DISPLAY_HEIGHT {
            self.wr_bytes(RAM_G + (y * LINE_STRIDE) as u32, &line)?;
        }
        Ok(())
    }

    /// Copy a `w`×`h` RGB565 rectangle (`px`, row-major, little-endian) into
    /// `RAM_G` via host SPI. LVGL flush path — one burst per full-width stripe.
    pub fn blit(&mut self, x: usize, y: usize, w: usize, h: usize, px: &[u8]) -> Result<(), Error> {
        let row_bytes = w * 2;
        debug_assert!(px.len() >= row_bytes * h);
        debug_assert!(x + w <= DISPLAY_WIDTH && y + h <= DISPLAY_HEIGHT);

        if x == 0 && w == DISPLAY_WIDTH {
            return self.wr_bytes(RAM_G + (y * LINE_STRIDE) as u32, &px[..row_bytes * h]);
        }
        for row in 0..h {
            let addr = RAM_G + ((y + row) * LINE_STRIDE + x * 2) as u32;
            self.wr_bytes(addr, &px[row * row_bytes..][..row_bytes])?;
        }
        Ok(())
    }

    /// Backlight brightness, `0..=128`.
    pub fn set_backlight(&mut self, duty: u8) -> Result<(), Error> {
        self.wr8(REG_PWM_DUTY, duty.min(128))
    }

    /// Read back `REG_PCLK` and `REG_GPIOX` (for localising where the pixel
    /// clock / DISP enable gets lost during bring-up).
    pub fn read_pclk_gpiox(&mut self) -> Result<(u8, u16), Error> {
        Ok((self.rd8(REG_PCLK)?, self.rd32(REG_GPIOX)? as u16))
    }

    /// Write the panel timing + pixel-format registers. Must be applied *after*
    /// the co-processor has run its first display list — before that the FT81x
    /// ignores these writes (they read back 0 and the co-processor installs its
    /// 480×272 defaults). See [`Ft81x::configure`].
    pub fn apply_panel_timings(&mut self) -> Result<(), Error> {
        self.wr16(REG_HSIZE, DISPLAY_WIDTH as u16)?;
        self.wr16(REG_HCYCLE, H_CYCLE)?;
        self.wr16(REG_HOFFSET, H_OFFSET)?;
        self.wr16(REG_HSYNC0, H_SYNC0)?;
        self.wr16(REG_HSYNC1, H_SYNC1)?;
        self.wr16(REG_VSIZE, DISPLAY_HEIGHT as u16)?;
        self.wr16(REG_VCYCLE, V_CYCLE)?;
        self.wr16(REG_VOFFSET, V_OFFSET)?;
        self.wr16(REG_VSYNC0, V_SYNC0)?;
        self.wr16(REG_VSYNC1, V_SYNC1)?;
        self.wr8(REG_SWIZZLE, SWIZZLE)?;
        self.wr8(REG_PCLK_POL, PCLK_POL)?;
        self.wr8(REG_CSPREAD, CSPREAD)?;
        self.wr8(REG_DITHER, 1)?;
        Ok(())
    }

    /// Read back the panel-timing registers (to confirm the writes stuck and
    /// see the FT81x core clock) — for debugging display artefacts.
    pub fn log_timing(&mut self) -> Result<(), Error> {
        defmt::info!(
            "timing H: HCYCLE={} HSIZE={} HOFFSET={} HSYNC0={} HSYNC1={}",
            self.rd32(REG_HCYCLE)? & 0xFFFF,
            self.rd32(REG_HSIZE)? & 0xFFFF,
            self.rd32(REG_HOFFSET)? & 0xFFFF,
            self.rd32(REG_HSYNC0)? & 0xFFFF,
            self.rd32(REG_HSYNC1)? & 0xFFFF,
        );
        defmt::info!(
            "timing V: VCYCLE={} VSIZE={} VOFFSET={} VSYNC0={} VSYNC1={}",
            self.rd32(REG_VCYCLE)? & 0xFFFF,
            self.rd32(REG_VSIZE)? & 0xFFFF,
            self.rd32(REG_VOFFSET)? & 0xFFFF,
            self.rd32(REG_VSYNC0)? & 0xFFFF,
            self.rd32(REG_VSYNC1)? & 0xFFFF,
        );
        defmt::info!(
            "timing X: PCLK={} PCLK_POL={} SWIZZLE={} CSPREAD={} DITHER={} FREQUENCY={}",
            self.rd8(REG_PCLK)?,
            self.rd8(REG_PCLK_POL)?,
            self.rd8(REG_SWIZZLE)?,
            self.rd8(REG_CSPREAD)?,
            self.rd8(REG_DITHER)?,
            self.rd32(REG_FREQUENCY)?,
        );
        Ok(())
    }

    /// Log key registers for display bring-up debugging.
    ///
    /// The decisive one is `REG_FRAMES`: it increments once per scanned-out
    /// frame, so if it climbs (~60 in one second) the FT813 *is* driving the
    /// panel and any "black screen" is downstream — backlight power (the gen4
    /// needs 5 V for its boost) or the LCD flex — not the SPI link or the
    /// display-list. If it stays put, the pixel clock never locked (timings /
    /// `REG_PCLK`). `REG_CLOCK` always ticks while the core runs (a sanity
    /// check independent of `PCLK`).
    pub async fn log_display_status(&mut self) -> Result<(), Error> {
        let (frames0, clock0) = (self.rd32(REG_FRAMES)?, self.rd32(REG_CLOCK)?);
        Timer::after_millis(200).await;
        let (frames1, clock1) = (self.rd32(REG_FRAMES)?, self.rd32(REG_CLOCK)?);
        let pclk = self.rd8(REG_PCLK)?;
        let duty = self.rd8(REG_PWM_DUTY)?;
        let gpiox = self.rd32(REG_GPIOX)? as u16;
        defmt::info!(
            "ft81x diag over 200ms: REG_FRAMES {}->{} (+{}), REG_CLOCK +{}, REG_PCLK={}, REG_PWM_DUTY={}, REG_GPIOX={=u16:#06x} DISP={}",
            frames0,
            frames1,
            frames1.wrapping_sub(frames0),
            clock1.wrapping_sub(clock0),
            pclk,
            duty,
            gpiox,
            (gpiox & 0x8000) != 0,
        );
        Ok(())
    }

    /// Set `REG_PCLK` directly (0 = pixel clock off / display blank, the
    /// configured divisor = normal). For bring-up A/B tests.
    pub fn set_pclk(&mut self, div: u8) -> Result<(), Error> {
        self.wr8(REG_PCLK, div)
    }

    /// Read `(REG_GPIOX_DIR, REG_GPIOX)` for GPIO bring-up diagnostics.
    pub fn read_gpiox(&mut self) -> Result<(u16, u16), Error> {
        Ok((self.rd32(REG_GPIOX_DIR)? as u16, self.rd32(REG_GPIOX)? as u16))
    }

    /// Drive the DISP output (`REG_GPIOX` bit 15) high or low, preserving the
    /// other GPIOX bits. `true` enables the panel; on the gen4 module this also
    /// appears to disturb the capacitive touch, so it is worth testing whether
    /// the module drives its own DISP and this can stay off.
    pub fn set_disp(&mut self, on: bool) -> Result<(), Error> {
        let g = self.rd32(REG_GPIOX)? as u16;
        let g = if on { g | 0x8000 } else { g & !0x8000 };
        self.wr16(REG_GPIOX, g)
    }

    /// Read back the touch engine configuration: `(REG_TOUCH_MODE,
    /// REG_CTOUCH_EXTENDED)`. Expected after [`Ft81x::init`]: `(3, 1)`
    /// (continuous sampling, single-touch compatibility mode).
    pub fn touch_config(&mut self) -> Result<(u8, u8), Error> {
        Ok((self.rd8(REG_TOUCH_MODE)?, self.rd8(REG_CTOUCH_EXTENDED)?))
    }

    /// Re-assert the touch engine configuration (continuous, single-touch
    /// compatibility mode). Useful if a later SPI reconfiguration cleared it,
    /// the same way it clears `REG_PCLK` on the H573I-DK.
    pub fn reinit_touch(&mut self) -> Result<(), Error> {
        self.wr8(REG_CTOUCH_EXTENDED, 1)?;
        self.wr8(REG_TOUCH_MODE, 3)
    }

    /// Read one raw touch sample from the capacitive touch engine.
    ///
    /// This is a single, undebounced sample: capacitive noise (especially now
    /// that the panel is scanning) makes individual samples flicker between
    /// touched and released, so debounce in the caller (see the self-test
    /// loop / the LVGL indev).
    pub fn touch(&mut self) -> Result<TouchPoint, Error> {
        let xy = self.rd32(REG_TOUCH_SCREEN_XY)?;
        // This gen4 panel reports the axes transposed vs the FT81x default
        // (which is X in the high 16 bits, Y in the low): here X is the low
        // half and Y the high half. Read them accordingly.
        let rx = (xy & 0xFFFF) as i16; // X
        let ry = (xy >> 16) as i16; // Y
        // Not touched: both fields are the 0x8000 (i16::MIN) sentinel. A held
        // touch can briefly read the sentinel in only *one* field, so require
        // both — otherwise `pressed` flickers mid-touch. Debounce the release
        // in the caller for the remaining brief both-field dropouts.
        if rx == i16::MIN && ry == i16::MIN {
            return Ok(TouchPoint::default());
        }
        Ok(TouchPoint {
            x: (rx as i32).clamp(0, DISPLAY_WIDTH as i32 - 1),
            y: (ry as i32).clamp(0, DISPLAY_HEIGHT as i32 - 1),
            pressed: true,
        })
    }
}
