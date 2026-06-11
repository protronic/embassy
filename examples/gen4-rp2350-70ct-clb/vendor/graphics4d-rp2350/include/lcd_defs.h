#pragma once

// TODO: Only keep those from actual driver chips used

#define ST7789_NOP          0x00
#define ST7789_SWRESET      0x01

#define ST7789_SLPIN        0x10  // sleep on
#define ST7789_SLPOUT       0x11  // sleep off
#define ST7789_PTLON        0x12  // partial on
#define ST7789_NORON        0x13  // partial off
#define ST7789_INVOFF       0x20  // invert off
#define ST7789_INVON        0x21  // invert on
#define ST7789_DISPOFF      0x28  // display off
#define ST7789_DISPON       0x29  // display on
#define ST7789_IDMOFF       0x38  // idle off
#define ST7789_IDMON        0x39  // idle on

#define ST7789_CASET        0x2A
#define ST7789_RASET        0x2B
#define ST7789_RAMWR        0x2C
#define ST7789_RAMRD        0x2E

#define ST7789_COLMOD       0x3A
#define ST7789_MADCTL       0x36

#define ST7789_PTLAR        0x30   // partial start/end
#define ST7789_VSCRDEF      0x33   // SETSCROLLAREA
#define ST7789_VSCRSADD     0x37

#define ST7789_WRDISBV      0x51
#define ST7789_WRCTRLD      0x53
#define ST7789_WRCACE       0x55
#define ST7789_WRCABCMB     0x5e

#define ST7789_POWSAVE      0xbc
#define ST7789_DLPOFFSAVE   0xbd

// bits in MADCTL
#define ST7789_MADCTL_MY    0x80
#define ST7789_MADCTL_MX    0x40
#define ST7789_MADCTL_MV    0x20
#define ST7789_MADCTL_ML    0x10
#define ST7789_MADCTL_RGB   0x00

#define GFX4DST_NOP			0x00
#define GFX4DST_SWRESET		0x01
#define GFX4DST_RDDID		0x04
#define GFX4DST_RDDST		0x09

#define GFX4DST_RDDPM		0x0A      // Read display power mode
#define GFX4DST_RDD_MADCTL	0x0B      // Read display MADCTL
#define GFX4DST_RDD_COLMOD	0x0C      // Read display pixel format
#define GFX4DST_RDDIM		0x0D      // Read display image mode
#define GFX4DST_RDDSM		0x0E      // Read display signal mode
#define GFX4DST_RDDSR		0x0F      // Read display self-diagnostic result (ST7789V)

#define GFX4DST_SLPIN		0x10
#define GFX4DST_SLPOUT		0x11
#define GFX4DST_PTLON		0x12
#define GFX4DST_NORON		0x13

#define GFX4DST_INVOFF		0x20
#define GFX4DST_INVON		0x21
#define GFX4DST_GAMSET		0x26      // Gamma set
#define GFX4DST_DISPOFF		0x28
#define GFX4DST_DISPON		0x29
#define GFX4DST_CASET		0x2A
#define GFX4DST_RASET		0x2B
#define GFX4DST_RAMWR		0x2C
#define GFX4DST_RGBSET		0x2D      // Color setting for 4096, 64K and 262K colors
#define GFX4DST_RAMRD		0x2E

#define GFX4DST_PTLAR		0x30
#define GFX4DST_VSCRDEF		0x33      // Vertical scrolling definition (ST7789V)
#define GFX4DST_TEOFF		0x34      // Tearing effect line off
#define GFX4DST_TEON		0x35      // Tearing effect line on
#define GFX4DST_MADCTL		0x36      // Memory data access control
#define GFX4DST_IDMOFF		0x38      // Idle mode off
#define GFX4DST_IDMON		0x39      // Idle mode on
#define GFX4DST_RAMWRC		0x3C      // Memory write continue (ST7789V)
#define GFX4DST_RAMRDC		0x3E      // Memory read continue (ST7789V)
#define GFX4DST_COLMOD		0x3A

#define GFX4DST_RAMCTRL		0xB0
#define GFX4DST_RGBCTRL		0xB1
#define GFX4DST_PORCTRL		0xB2
#define GFX4DST_FRCTRL1		0xB3
#define GFX4DST_PARCTRL		0xB5
#define GFX4DST_GCTRL		0xB7
#define GFX4DST_GTADJ		0xB8
#define GFX4DST_DGMEN		0xBA
#define GFX4DST_VCOMS		0xBB
#define GFX4DST_LCMCTRL		0xC0
#define GFX4DST_IDSET		0xC1
#define GFX4DST_VDVVRHEN	0xC2
#define GFX4DST_VRHS		0xC3
#define GFX4DST_VDVSET		0xC4
#define GFX4DST_VCMOFSET	0xC5
#define GFX4DST_FRCTR2		0xC6
#define GFX4DST_CABCCTRL	0xC7
#define GFX4DST_REGSEL1		0xC8
#define GFX4DST_REGSEL2		0xCA
#define GFX4DST_PWMFRSEL	0xCC
#define GFX4DST_PWCTRL1		0xD0
#define GFX4DST_VAPVANEN	0xD2
#define GFX4DST_CMD2EN		0xDF
#define GFX4DST_PVGAMCTRL	0xE0
#define GFX4DST_NVGAMCTRL	0xE1
#define GFX4DST_DGMLUTR		0xE2
#define GFX4DST_DGMLUTB		0xE3
#define GFX4DST_GATECTRL	0xE4
#define GFX4DST_SPI2EN		0xE7
#define GFX4DST_PWCTRL2		0xE8
#define GFX4DST_EQCTRL		0xE9
#define GFX4DST_PROMCTRL	0xEC
#define GFX4DST_PROMEN		0xFA
#define GFX4DST_NVMSET		0xFC
#define GFX4DST_PROMACT		0xFE

#define TFT_MAD_MY  	0x80
#define TFT_MAD_MX  	0x40
#define TFT_MAD_MV  	0x20
#define TFT_MAD_ML  	0x10
#define TFT_MAD_RGB 	0x00
#define TFT_MAD_BGR 	0x08
#define TFT_MAD_MH  	0x04
#define TFT_MAD_SS  	0x02
#define TFT_MAD_GS  	0x01
#define TFT_RGB_ORDER 	0x00
#define GFX4d_NOP         		0x00
#define GFX4d_SWRESET     		0x01
#define GFX4d_RDDID       		0x04
#define GFX4d_RDDST       		0x09

#define GFX4d_SLPIN       		0x10
#define GFX4d_SLPOUT      		0x11
#define GFX4d_PTLON       		0x12
#define GFX4d_NORON       		0x13

#define GFX4d_RDMODE      		0x0A
#define GFX4d_RDMADCTL    		0x0B
#define GFX4d_RDPIXFMT    		0x0C
#define GFX4d_RDIMGFMT    		0x0D
#define GFX4d_RDSELFDIAG  		0x0F

#define GFX4d_INVOFF      		0x20
#define GFX4d_INVON       		0x21
#define GFX4d_GAMMASET    		0x26
#define GFX4d_DISPOFF     		0x28
#define GFX4d_DISPON      		0x29

#define GFX4d_CASET       		0x2A
#define GFX4d_PASET       		0x2B
#define GFX4d_RAMWR       		0x2C
#define GFX4d_RAMRD       		0x2E

#define GFX4d_PTLAR       		0x30
#define GFX4d_VSCRDEF     		0x33
#define GFX4d_MADCTL      		0x36
#define GFX4d_VSCRSADD    		0x37
#define GFX4d_PIXFMT      		0x3A

#define GFX4d_FRMCTR1     		0xB1
#define GFX4d_FRMCTR2     		0xB2
#define GFX4d_FRMCTR3     		0xB3
#define GFX4d_INVCTR      		0xB4
#define GFX4d_DFUNCTR     		0xB6

#define GFX4d_PWCTR1      		0xC0
#define GFX4d_PWCTR2      		0xC1
#define GFX4d_PWCTR3      		0xC2
#define GFX4d_PWCTR4      		0xC3
#define GFX4d_PWCTR5      		0xC4
#define GFX4d_VMCTR1      		0xC5
#define GFX4d_VMCTR2      		0xC7

#define GFX4d_RDID1       		0xDA
#define GFX4d_RDID2       		0xDB
#define GFX4d_RDID3       		0xDC
#define GFX4d_RDID4       		0xDD

#define GFX4d_GMCTRP1     		0xE0
#define GFX4d_GMCTRN1     		0xE1

#define ILI9488_INTRFC_MODE_CTL                     0xB0
#define ILI9488_FRAME_RATE_NORMAL_CTL               0xB1
#define ILI9488_INVERSION_CTL                       0xB4
#define ILI9488_FUNCTION_CTL                        0xB6
#define ILI9488_ENTRY_MODE_CTL                      0xB7
#define ILI9488_POWER_CTL_ONE                       0xC0
#define ILI9488_POWER_CTL_TWO                       0xC1
#define ILI9488_POWER_CTL_THREE                     0xC5
#define ILI9488_POSITIVE_GAMMA_CTL                  0xE0
#define ILI9488_NEGATIVE_GAMMA_CTL                  0xE1
#define ILI9488_ADJUST_CTL_THREE                    0xF7
#define ILI9488_COLOR_MODE_16BIT                    0x55
#define ILI9488_COLOR_MODE_18BIT                    0x66
#define ILI9488_INTERFACE_MODE_USE_SDO              0x00
#define ILI9488_INTERFACE_MODE_IGNORE_SDO           0x80
#define ILI9488_IMAGE_FUNCTION_DISABLE_24BIT_DATA   0x00
#define ILI9488_WRITE_MODE_BCTRL_DD_ON              0x28
#define ILI9488_FRAME_RATE_60HZ                     0xA0
#define ILI9488_INIT_LENGTH_MASK                    0x1F
#define ILI9488_INIT_DONE_FLAG                      0xFF
#define ILI9488_MADCTL                              0x36
#define ILI9488_PIXFMT                              0x3A
#define ILI9488_CMD_SLEEP_OUT                       0x11
#define ILI9488_CMD_DISPLAY_OFF                     0x28
#define ILI9488_CMD_DISPLAY_ON                      0x29
#define ILI9488_SET_IMAGE_FUNCTION                  0xe9
#define ILI9488_DISP_INVERT_ON                      0x21

// #define GFX4d_35_PTLAR                              0x30
// #define GFX4d_35_VSCRDEF                            0x33
// #define GFX4d_35_MADCTL                             0x36
// #define GFX4d_35_VSCRSADD                           0x37
// #define GFX4d_35_PIXFMT                             0x3A
// #define GFX4d_35_FRMCTR1                            0xB1
// #define GFX4d_35_FRMCTR2                            0xB2
// #define GFX4d_35_FRMCTR3                            0xB3
// #define GFX4d_35_INVCTR                             0xB4
// #define GFX4d_35_DFUNCTR                            0xB6
// #define GFX4d_35_PWCTR1                             0xC0
// #define GFX4d_35_PWCTR2                             0xC1
// #define GFX4d_35_PWCTR3                             0xC2
// #define GFX4d_35_PWCTR4                             0xC3
// #define GFX4d_35_PWCTR5                             0xC4
// #define GFX4d_35_VMCTR1                             0xC5
// #define GFX4d_35_VMCTR2                             0xC7
// #define GFX4d_35_RDID1                              0xDA
// #define GFX4d_35_RDID2                              0xDB
// #define GFX4d_35_RDID3                              0xDC
// #define GFX4d_35_RDID4                              0xDD
// #define GFX4d_35_GMCTRP1                            0xE0
// #define GFX4d_35_GMCTRN1                            0xE1
// #define GFX4d_35_GAMMASET                           0x26