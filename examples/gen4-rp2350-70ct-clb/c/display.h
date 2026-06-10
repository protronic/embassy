#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Initialise the on-module RGB panel (PIO scanout inside Graphics4D when linked).
void gen4_lcd_init(void);

/// Drive the backlight (PWM is configured in Rust; this enables the panel path).
void gen4_lcd_backlight_enable(void);

/// Present one full RGB565 frame (800×480 on the 70CT-CLB).
void gen4_lcd_present_rgb565(const uint16_t *pixels, uint16_t width, uint16_t height);

#ifdef __cplusplus
}
#endif
