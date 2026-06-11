#pragma once

#include "lcd_defs.h"

#define LCD_TOUCH_THRESHOLD         200

typedef struct lcd_init_command {
    uint8_t command;
    const uint8_t * data;
    uint len;
    uint delay_ms;
} lcd_init_command;

// gen4-RP2350-24 Series
// gen4-RP2350-28 Series
// gen4-RP2350-32 Series

#if defined(GEN4_RP2350_24) || defined(GEN4_RP2350_24T) || defined(GEN4_RP2350_24CT) || \
    defined(GEN4_RP2350_28) || defined(GEN4_RP2350_28T) || defined(GEN4_RP2350_28CT) || \
    defined(GEN4_RP2350_32) || defined(GEN4_RP2350_32T) || defined(GEN4_RP2350_32CT)
const lcd_init_command lcd_init_cmds[] = {
    { GFX4DST_SLPOUT,       NULL, 0, 120 },
    { GFX4DST_NORON,        NULL, 0, 0 },
    { 0xEF,                 (uint8_t[]) { 0x01, 0x01, 0x00 }, 3, 0 },
    { 0xCF,                 (uint8_t[]) { 0x00, 0xC1, 0x30 }, 3, 0 },
    { 0xED,                 (uint8_t[]) { 0x64, 0x03, 0x12, 0x81 }, 4, 0 },
    { 0xE8,                 (uint8_t[]) { 0x85, 0x00, 0x7a }, 3, 0 },
    { 0xCB,                 (uint8_t[]) { 0x39, 0x2C, 0x00, 0x34, 0x02 }, 5, 0 },
    { 0xF7,                 (uint8_t[]) { 0x20 }, 1, 0 },
    { 0xEA,                 (uint8_t[]) { 0x00, 0x00}, 2, 0 },
    { GFX4d_PWCTR1,         (uint8_t[]) { 0x26 }, 1, 0 },
    { GFX4d_PWCTR2,         (uint8_t[]) { 0x11 }, 1, 0 },
    { GFX4d_VMCTR1,         (uint8_t[]) { 0x39, 0x27}, 2, 0},
    { GFX4d_VMCTR2,         (uint8_t[]) { 0xa6 }, 1, 0 },
    { GFX4d_MADCTL,         (uint8_t[]) { 0x48 }, 1, 0 },
    { GFX4d_PIXFMT,         (uint8_t[]) { 0x55 }, 1, 0 },
    { GFX4d_FRMCTR1,        (uint8_t[]) { 0x00, 0x1b}, 2, 0},
    { GFX4d_DFUNCTR,        (uint8_t[]) { 0x08, 0x82, 0x27}, 3, 0},
    { 0xF2,                 (uint8_t[]) { 0x00 }, 1, 0 },
    { GFX4d_GAMMASET,       (uint8_t[]) { 0x01 }, 1, 0 },
    { GFX4d_GMCTRP1,        (uint8_t[]) { 0x0F, 0x2d, 0x0e, 0x08, 0x12, 0x0a, 0x3d, 0x95, 0x31, 0x04, 0x10, 0x09, 0x09, 0x0d, 0x00}, 0, 0 },
    { GFX4d_GMCTRN1,        (uint8_t[]) { 0x00, 0x12, 0x17, 0x03, 0x0d, 0x05, 0x2c, 0x44, 0x41, 0x05, 0x0F, 0x0a, 0x30, 0x32, 0x0F}, 15, 120 },
#if defined(GEN4_RP2350_32) || defined(GEN4_RP2350_32T) || defined(GEN4_RP2350_32CT)
    { GFX4DST_INVOFF,       NULL, 0, 0 },
#else
    { GFX4DST_INVON,        NULL, 0, 0 },
#endif
    { GFX4DST_DISPON,       NULL, 0, 120 },
};

#if defined(GEN4_RP2350_24T) || defined(GEN4_RP2350_28T) || defined(GEN4_RP2350_32T)
// TODO: Find correct values for 2.8 and 3.2
#define LCD_TOUCH_START_X           695
#define LCD_TOUCH_START_Y           3322
#define LCD_TOUCH_END_X             3012
#define LCD_TOUCH_END_Y             700
#endif

#endif

// gen4-RP2350-35 Series
#if defined(GEN4_RP2350_35) || defined(GEN4_RP2350_35T) || defined(GEN4_RP2350_35CT)

const lcd_init_command lcd_init_cmds[] = {
    { ILI9488_POSITIVE_GAMMA_CTL,       (uint8_t[]) { 0x00, 0x13, 0x18, 0x04, 0x0F, 0x06, 0x3A, 0x56, 0x4D, 0x03, 0x0A, 0x06, 0x30, 0x3E, 0x0F }, 15, 0 },
    { ILI9488_NEGATIVE_GAMMA_CTL,       (uint8_t[]) { 0x00, 0x13, 0x18, 0x01, 0x11, 0x06, 0x38, 0x34, 0x4D, 0x06, 0x0D, 0x0B, 0x31, 0x37, 0x0F }, 15, 0 },
    { ILI9488_POWER_CTL_ONE,            (uint8_t[]) { 0x18, 0x16 }, 2, 0 },
    { ILI9488_POWER_CTL_TWO,            (uint8_t[]) { 0x45 }, 1, 0 },
    { ILI9488_POWER_CTL_THREE,          (uint8_t[]) { 0x00, 0x63, 0x01 }, 3, 0 },
    { ILI9488_MADCTL,                   (uint8_t[]) { 0x48 }, 1, 0 },
    { ILI9488_PIXFMT,                   (uint8_t[]) { ILI9488_COLOR_MODE_16BIT }, 1, 0 },
    { ILI9488_INTRFC_MODE_CTL,          (uint8_t[]) { ILI9488_INTERFACE_MODE_IGNORE_SDO }, 1, 0 },
    { ILI9488_FRAME_RATE_NORMAL_CTL,    (uint8_t[]) { 0xB0 }, 1, 0 },
    { ILI9488_INVERSION_CTL,            (uint8_t[]) { 0x02 }, 1, 0 },
    { ILI9488_FUNCTION_CTL,             (uint8_t[]) { 0x02, 0x02 }, 2, 0 },
    { ILI9488_SET_IMAGE_FUNCTION,       (uint8_t[]) { 0x00 }, 1, 0 },
    { ILI9488_ADJUST_CTL_THREE,         (uint8_t[]) { 0xA9, 0x51, 0x2C, 0x82 }, 4, 0 },
    { ILI9488_CMD_SLEEP_OUT,            NULL, 0, 120 },
    { ILI9488_DISP_INVERT_ON,           NULL, 0, 0 },
    { ILI9488_CMD_DISPLAY_ON  ,         NULL, 0, 120 },
};

#ifdef GEN4_RP2350_35T
#define LCD_TOUCH_START_X           725
#define LCD_TOUCH_START_Y           3500
#define LCD_TOUCH_END_X             3122
#define LCD_TOUCH_END_Y             515
#endif

#endif

