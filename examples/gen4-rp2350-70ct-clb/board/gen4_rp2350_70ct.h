/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------

// pico_cmake_set PICO_PLATFORM=rp2350

#ifndef _BOARDS_GEN4_RP2350_70CT
#define _BOARDS_GEN4_RP2350_70CT

// For board detection
#define GEN4_RP2350_70CT	// CLB variants are exactly the same in operation

// --- RP2350 VARIANT ---
#define PICO_RP2350A 0

// --- UART ---
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 1
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 4
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 5
#endif

// --- LED ---
// no PICO_DEFAULT_LED_PIN
// no PICO_DEFAULT_WS2812_PIN

// --- I2C ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 8
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 9
#endif

// --- SPI ---
#ifndef PICO_DEFAULT_SPI
#define PICO_DEFAULT_SPI 1
#endif
#ifndef PICO_DEFAULT_SPI_SCK_PIN
#define PICO_DEFAULT_SPI_SCK_PIN 42
#endif
#ifndef PICO_DEFAULT_SPI_TX_PIN
#define PICO_DEFAULT_SPI_TX_PIN 43
#endif
#ifndef PICO_DEFAULT_SPI_RX_PIN
#define PICO_DEFAULT_SPI_RX_PIN 44
#endif
#ifndef PICO_DEFAULT_SPI_CSN_PIN
#define PICO_DEFAULT_SPI_CSN_PIN 45
#endif

// --- FLASH ---

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

// pico_cmake_set_default PICO_FLASH_SIZE_BYTES = (16 * 1024 * 1024)
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif

#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif

// 4DLCD Interface

#define GEN4_RP2350_RGB

#define PORTRAIT            0
#define LANDSCAPE           1
#define PORTRAIT_R          2
#define LANDSCAPE_R         3

#define LCD_ORIENTATION     LANDSCAPE
#define LCD_WIDTH           800
#define LCD_HEIGHT          480
#define LCD_BACKLIGHT       17
#define LCD_DE_PIN          18
#define LCD_VSYNC_PIN       19
#define LCD_HSYNC_PIN       20
#define LCD_PCLK_PIN        21
#define LCD_DATA0_PIN       22	// Blue - LSB
#define LCD_TOUCH_INT       38
#define LCD_TOUCH_SCL       39
#define LCD_TOUCH_SDA       46
#define LCD_TOUCH_RST       47
#define LCD_TOUCH_I2C       i2c1
#define LCD_TOUCH_FT5446
#define LCD_TOUCH_POINTS    5
#define LCD_TOUCH_SWAP_XY

#define LCD_SD_CLK          10
#define LCD_SD_CMD          11
#define LCD_SD_D0           12
#define LCD_SD_D1           13
#define LCD_SD_D2           14
#define LCD_SD_D3           15

#define LCD_CLK_FREQ        25000000

#endif
