#include "display.h"

#include "graphics4d_scanout.h"

// Workshop5 projects expose a global `gfx` object.
extern Graphics4D gfx;

void gen4_lcd_init(void) {
    gfx.Initialize();
    gfx.ScreenMode(LANDSCAPE);
    gfx.Contrast(8);
}

void gen4_lcd_backlight_enable(void) {
    // Backlight PWM is configured from Rust.
}

void gen4_lcd_present_rgb565(const uint16_t *pixels, uint16_t width, uint16_t height) {
    if (pixels == nullptr || width == 0 || height == 0) {
        return;
    }

    const uint32_t px_count = static_cast<uint32_t>(width) * static_cast<uint32_t>(height);

    // Same API as GFX4dESP32: copy external RGB565 pixels into the active
    // scan-out buffer, then refresh the PIO RGB panel.
    gfx.WriteToFrameBuffer(0, const_cast<uint16_t *>(pixels), px_count);
    gfx.Refresh();
}
