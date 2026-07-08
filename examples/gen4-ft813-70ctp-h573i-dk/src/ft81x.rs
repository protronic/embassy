//! Minimal FT81x (EVE2) host driver over blocking SPI.
//!
//! Targets the FT813 inside 4D Systems **gen4-FT813-xx** display modules
//! (here: gen4-FT813-70CTP-CLB, 800×480 with capacitive touch). The EVE chip
//! is used as a *dumb framebuffer*: the host renders RGB565 pixels (LVGL
//! partial stripes) and writes them into the FT813's 1 MiB graphics RAM
//! (`RAM_G`) over SPI; a static display list scans that bitmap out to the
//! panel every frame. Touch is read back from the FT813's built-in capacitive
//! touch engine (`REG_TOUCH_SCREEN_XY`) over the same SPI bus.
//!
//! This deliberately avoids the EVE co-processor (`RAM_CMD`) — no command
//! FIFO handling, no co-processor faults — at the cost of raw SPI bandwidth
//! for pixel data. With LVGL in PARTIAL render mode only dirty rectangles are
//! transferred, which is plenty for a widget UI.
//!
//! References: FT81x datasheet + BRT_AN_033 (EVE2 programming guide); panel
//! timings match the 4D Systems gen4-FT812/FT813 entries in well-known EVE
//! libraries (standard 800×480 WVGA set, no external crystal → `CLKINT`).

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
const RAM_DL: u32 = 0x30_0000;

const REG_ID: u32 = 0x30_2000;
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

// ── Host commands (3-byte frames) ───────────────────────────────────────────
const HCMD_ACTIVE: u8 = 0x00;
const HCMD_CLKEXT: u8 = 0x44;
const HCMD_CLKINT: u8 = 0x48;
const HCMD_RST_PULSE: u8 = 0x68;

// ── Panel timings: 4D Systems gen4-FT813-70 (standard 800×480 WVGA set) ─────
const H_CYCLE: u16 = 928;
const H_OFFSET: u16 = 88;
const H_SYNC0: u16 = 0;
const H_SYNC1: u16 = 48;
const V_CYCLE: u16 = 525;
const V_OFFSET: u16 = 32;
const V_SYNC0: u16 = 0;
const V_SYNC1: u16 = 3;
const PCLK_DIV: u8 = 2; // 60 MHz / 2 = 30 MHz pixel clock
const PCLK_POL: u8 = 1; // fetch on falling edge
const SWIZZLE: u8 = 0;
const CSPREAD: u8 = 0;

// ── Display-list encoding (subset) ──────────────────────────────────────────
const fn dl_clear_color_rgb(r: u8, g: u8, b: u8) -> u32 {
    0x0200_0000 | ((r as u32) << 16) | ((g as u32) << 8) | b as u32
}
const fn dl_clear() -> u32 {
    0x2600_0007 // clear colour + stencil + tag
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
const fn dl_end() -> u32 {
    0x2100_0000
}
const fn dl_display() -> u32 {
    0x0000_0000
}
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
        Self { spi, cs, pd, config }
    }

    /// Change the SPI clock (call after [`Ft81x::init`]; FT81x max is 30 MHz).
    pub fn set_spi_frequency(&mut self, hz: u32) {
        self.config.frequency = embassy_stm32::time::Hertz(hz);
        // Only fails for unreachable divisors; keep the old clock in that case.
        if self.spi.set_config(&self.config).is_err() {
            defmt::warn!("ft81x: SPI frequency {} Hz not reachable, keeping previous", hz);
        }
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
    /// Leaves the panel scanning a blank (black) display list with the
    /// backlight off; call [`Ft81x::show_framebuffer`] +
    /// [`Ft81x::set_backlight`] once the framebuffer has content.
    pub async fn init(&mut self) -> Result<(), Error> {
        // gen4-FT813 modules run from the FT813's internal oscillator. Try
        // CLKINT first and fall back to CLKEXT once, in case a module variant
        // ships a crystal after all.
        for (attempt, clk) in [HCMD_CLKINT, HCMD_CLKEXT].into_iter().enumerate() {
            self.power_cycle().await;
            self.host_command(clk)?;
            self.host_command(HCMD_ACTIVE)?;
            Timer::after_millis(40).await;

            if self.poll_chip_id().await? {
                if attempt > 0 {
                    defmt::info!("ft81x: came up with CLKEXT fallback");
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
        // Wait for the graphics engine to leave reset.
        let deadline = Instant::now() + Duration::from_millis(400);
        while self.rd8(REG_CPURESET)? != 0 {
            if Instant::now() >= deadline {
                return Err(Error::ResetTimeout);
            }
            Timer::after_millis(5).await;
        }

        // Panel timings (PCLK stays 0 = display off until the end).
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

        // Capacitive touch: compatibility mode (single touch), continuous.
        self.wr8(REG_CTOUCH_EXTENDED, 1)?;
        self.wr8(REG_TOUCH_MODE, 3)?;

        // Blank display list so the first visible frame is black.
        self.wr32(RAM_DL, dl_clear_color_rgb(0, 0, 0))?;
        self.wr32(RAM_DL + 4, dl_clear())?;
        self.wr32(RAM_DL + 8, dl_display())?;
        self.wr32(REG_DLSWAP, DLSWAP_FRAME)?;

        // DISP pin high (GPIOX bit 15), backlight PWM ready but dark.
        self.wr16(REG_GPIOX_DIR, 0x8000)?;
        let gpiox = self.rd32(REG_GPIOX)? as u16;
        self.wr16(REG_GPIOX, gpiox | 0x8000)?;
        self.wr16(REG_PWM_HZ, 250)?;
        self.wr8(REG_PWM_DUTY, 0)?;

        // Start the pixel clock — panel is now scanning.
        self.wr8(REG_PCLK, PCLK_DIV)?;

        defmt::info!("ft81x: init ok (REG_ID=0x7c, 800x480 @ 30 MHz PCLK)");
        Ok(())
    }

    /// Backlight brightness, `0..=128`.
    pub fn set_backlight(&mut self, duty: u8) -> Result<(), Error> {
        self.wr8(REG_PWM_DUTY, duty.min(128))
    }

    /// Fill the whole `RAM_G` framebuffer with one RGB565 colour.
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

    /// Swap in the static display list that scans the `RAM_G` framebuffer out
    /// to the panel. Needs to run only once; later `RAM_G` writes show up on
    /// the next panel refresh without another DL swap.
    pub fn show_framebuffer(&mut self) -> Result<(), Error> {
        let w = DISPLAY_WIDTH as u32;
        let h = DISPLAY_HEIGHT as u32;
        let stride = LINE_STRIDE as u32;
        let dl = [
            dl_clear_color_rgb(0, 0, 0),
            dl_clear(),
            dl_bitmap_handle(0),
            dl_bitmap_source(RAM_G),
            dl_bitmap_layout(7, stride, h), // 7 = RGB565
            dl_bitmap_layout_h(stride, h),
            dl_bitmap_size(w, h),
            dl_bitmap_size_h(w, h),
            dl_begin_bitmaps(),
            dl_vertex2ii(0, 0),
            dl_end(),
            dl_display(),
        ];
        for (i, cmd) in dl.iter().enumerate() {
            self.wr32(RAM_DL + 4 * i as u32, *cmd)?;
        }
        self.wr32(REG_DLSWAP, DLSWAP_FRAME)
    }

    /// Copy a `w`×`h` RGB565 rectangle (`px`, row-major, little-endian) into
    /// the framebuffer at (`x`, `y`). This is the LVGL flush path.
    pub fn blit(&mut self, x: usize, y: usize, w: usize, h: usize, px: &[u8]) -> Result<(), Error> {
        let row_bytes = w * 2;
        debug_assert!(px.len() >= row_bytes * h);
        debug_assert!(x + w <= DISPLAY_WIDTH && y + h <= DISPLAY_HEIGHT);

        if x == 0 && w == DISPLAY_WIDTH {
            // Full-width stripe → contiguous in RAM_G, single burst.
            return self.wr_bytes(RAM_G + (y * LINE_STRIDE) as u32, &px[..row_bytes * h]);
        }
        for row in 0..h {
            let addr = RAM_G + ((y + row) * LINE_STRIDE + x * 2) as u32;
            self.wr_bytes(addr, &px[row * row_bytes..][..row_bytes])?;
        }
        Ok(())
    }

    /// Read one touch sample from the capacitive touch engine.
    pub fn touch(&mut self) -> Result<TouchPoint, Error> {
        let xy = self.rd32(REG_TOUCH_SCREEN_XY)?;
        if xy == 0x8000_8000 {
            return Ok(TouchPoint::default());
        }
        let x = (xy >> 16) as i16 as i32;
        let y = (xy & 0xFFFF) as i16 as i32;
        Ok(TouchPoint {
            x: x.clamp(0, DISPLAY_WIDTH as i32 - 1),
            y: y.clamp(0, DISPLAY_HEIGHT as i32 - 1),
            pressed: true,
        })
    }
}
