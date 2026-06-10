#include "display.h"

#ifdef GEN4_USE_GRAPHICS4D
#include "Graphics4D.h"
#endif

void gen4_lcd_init(void) {
#ifdef GEN4_USE_GRAPHICS4D
    gfx.Initialize();
    gfx.ScreenMode(LANDSCAPE);
    gfx.Contrast(8);
#endif
}

void gen4_lcd_backlight_enable(void) {
    /* Backlight PWM is handled from Rust; Graphics4D may also raise contrast. */
}

void gen4_lcd_present_rgb565(const uint16_t *pixels, uint16_t width, uint16_t height) {
    (void)pixels;
    (void)width;
    (void)height;
#ifdef GEN4_USE_GRAPHICS4D
    /* Workshop5 Graphics4D copies the active draw buffer to the RGB panel. */
    gfx.Refresh();
#endif
}
