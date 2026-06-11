#pragma once

#include "pico/stdlib.h"
#ifdef LCD_TOUCH_4WIRE
#include "hardware/adc.h"
#endif
#include "hardware/i2c.h"
#include "hardware/pio.h"

#include <hardware/clocks.h>
#include "pico/stdio.h" // must also enable usb stdio in CMakeLists
#include "pico/stdio_usb.h"

#include "Colors4D.h"
#include "pico_4d_modules.h"

#include "fonts/font1.h" // default font, need at least one fallback font
#ifdef USE_4D_FONT2
#include "fonts/font2.h"
#endif
#ifdef USE_4D_FONT3
#include "fonts/font3.h"
#endif
#ifdef USE_4D_FONT4
#include "fonts/font4.h"
#endif

#include "hw_config.h"
#include "sd_card.h"
#include "f_util.h"
#include "ff.h"

#define PIXEL_COUNT             LCD_WIDTH * LCD_HEIGHT
#define FB_SIZE                 PIXEL_COUNT * 2

// Max Settings
#ifndef MAX_TXT_BUF_SIZE        // might be able to provide this via WS5
#define MAX_TXT_BUF_SIZE        8192 // used for printf functions
#endif

#ifndef MAX_FB_PXL_IN_SRAM      // might be able to provide this via WS5
#define MAX_FB_PXL_IN_SRAM      480 * 320
#endif
#define MAX_FB_SIZE_IN_SRAM     2 * MAX_FB_PXL_IN_SRAM

#ifndef MAX_PXL_IN_AUX          // might be able to provide this via WS5
#define MAX_PXL_IN_AUX          320 * 240
#endif
#define MAX_AUX_IN_SRAM         2 * MAX_PXL_IN_AUX

#define delay_us(us) busy_wait_us(us)
#define delay(ms) busy_wait_us((ms) * 1000)
#define millis() to_ms_since_boot(get_absolute_time())

class TextArea
{
private:
    TextArea(int x1, int y1, int x2, int y2, uint16_t fg_color, uint16_t bg_color);
    ~TextArea();

    bool wrap = true;
    bool scroll = false;
    bool bg_transparent = false;

    uint16_t fg_color;
    uint16_t bg_color;

    int x1, y1;
    int x2, y2;

    int cursor_x;
    int cursor_y;

public:
    friend class Graphics4D;
};

typedef TextArea *TextArea4D;

class Graphics4D
{
public:
    bool Initialize();
    void DrawWidget(int num, int f, int x, int y, const uint8_t * gciArray);
    void SetFramebuffer(uint16_t * buffer);

#ifdef GEN4_RP2350_RGB
    // Embassy / external LVGL scan-out helpers (GFX4dESP32-compatible API).
    void WriteToFrameBuffer(uint32_t offset, uint16_t *pixels, uint32_t count);
    void Refresh();
#endif

#ifndef GEN4_RP2350_RGB
    void Reset(); // does the display need to be reinitialized?
#endif

    void SetBacklightLevel(uint16_t level);
    void Contrast(uint8_t level);
    uint GetWidth();
    uint GetHeight();

    uint8_t ScreenMode(uint8_t orientation);

    void SetAddressWindow(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end);
#ifndef GEN4_RP2350_RGB
    void SendFrameBuffer(uint x1, uint y1, uint x2, uint y2);
#endif
    uint16_t *GetFrameBuffer();

    // utility functions

    uint16_t BlendColor(uint16_t base_color, uint16_t new_color, uint8_t alpha);

    // 4DGL-like functions
    // gfx_Cls
    void Cls(bool draw_fb = true);
    // gfx_RectangleFilled
    void RectangleFilled(int x1, int y1, int x2, int y2, uint16_t color, bool draw_fb = true);
    // gfx_WriteGRAMarea
    void RectangleFilled(int x1, int y1, int x2, int y2, const uint16_t *colors, bool draw_fb = true);
    void RectangleFilled(int x1, int y1, int x2, int y2, const uint8_t *buffer, bool draw_fb = true);
    void RectangleFilledWithAlpha(int x1, int y1, int x2, int y2, const uint8_t *buffer, bool draw_fb = true);
    // gfx_HLine
    void Hline(int y, int x1, int x2, uint16_t color, bool draw_fb = true);
    // gfx_VLine
    void Vline(int x, int y1, int y2, uint16_t color, bool draw_fb = true);
    // gfx_Rectangle
    void Rectangle(int x1, int y1, int x2, int y2, uint16_t color, bool draw_fb = true);
    // gfx_PutPixel
    void PutPixel(int x, int y, uint16_t color, bool draw_fb = true);
    void PutPixel(int x, int y, uint16_t color, uint8_t alpha, bool draw_fb = true);
    // gfx_Line
    void Line(int x1, int y1, int x2, int y2, uint16_t color, bool draw_fb = true);
    // gfx_Ellipse
    void Ellipse(int x, int y, uint x_rad, uint y_rad, uint16_t color, bool draw_fb = true);
    // gfx_EllipseFilled
    void EllipseFilled(int x, int y, uint x_rad, uint y_rad, uint16_t color, bool draw_fb = true);

    // gfx_Circle
    void Circle(int x, int y, uint radius, uint16_t color, bool draw_fb = true);
    // gfx4desp32::Arc
    void Arc(int x, int y, uint radius, int sa, uint16_t color, bool draw_fb = true);
    // gfx4desp32::ArcFilled
    void ArcFilled(int x, int y, uint radius, int sa, int ea, uint16_t color, bool draw_fb = true);
    // gfx_CircleFilled
    void CircleFilled(int x, int y, uint radius, uint16_t color, bool draw_fb = true);
    // gfx_Triangle
    void Triangle(int x1, int y1, int x2, int y2, int x3, int y3, uint16_t color, bool draw_fb = true);
    // gfx_TriangleFilled
    void TriangleFilled(int x1, int y1, int x2, int y2, int x3, int y3, uint16_t color, bool draw_fb = true);
    // gfx_Polyline
    void Polyline(uint len, const int *vx, const int *vy, uint16_t color, bool draw_fb = true);
    // gfx_Polygon
    void Polygon(uint len, const int *vx, const int *vy, uint16_t color, bool draw_fb = true);
    // gfx_PolygonFilled
    void PolygonFilled(uint len, const int *vx, const int *vy, uint16_t color, bool draw_fb = true);

    // gfx_Set functions
    uint16_t SetBackgroundColor(uint16_t color);

    // gfx_ClipWindow
    bool ClipWindow(int x1, int y1, int x2, int y2);

    const uint8_t *SetFont(const uint8_t *f);
    uint16_t SetFontForeground(TextArea4D area, uint16_t color);
    uint16_t SetFontBackground(TextArea4D area, uint16_t color, bool transparent = false);
    uint16_t SetFontForeground(uint16_t color);
    uint16_t SetFontBackground(uint16_t color, bool transparent = false);

    uint GetStringWidth(const char * str);
    uint GetFontHeight();

    // gfx_MoveTo
    bool MoveTo(int x, int y);
    // gfx_MoveRel
    bool MoveRel(int x_offset, int y_offset);

    // Prints to main text area
    size_t print(const char *str, bool draw_fb = true);
    size_t printf(const char *format, ...);

    TextArea4D CreateTextArea(int x1, int y1, int x2, int y2, uint16_t fg_color, uint16_t bg_color);
    size_t print(TextArea4D area, const char *str, bool draw_fb = true);
    size_t printf(TextArea4D area, const char *format, ...);

private:
    Graphics4D();
    ~Graphics4D();
    Graphics4D(const Graphics4D &);
    Graphics4D &operator=(const Graphics4D &);

#ifndef GEN4_RP2350_RGB
    void __write_command(uint8_t c);
    void __write_data(uint16_t c);
    uint16_t __read_data();
#endif
    uint16_t *__get_aux_buffer(uint width, uint height);
    uint16_t *__get_aux_buffer(size_t buffer_size);

    size_t __putch(TextArea4D area, uint16_t c, bool draw_fb = true);

    uint __bus_prog_offset;
    uint __bus_jmp_read;
    PIO __bus_pio;
    uint __bus_sm;

    uint16_t displayId[4];

    // main text area
    TextArea4D __textarea = NULL;

    // framebuffer

#if (FB_SIZE <= MAX_FB_IN_SRAM) && !defined(FB_IN_PSRAM)
    uint16_t *__framebuffer[PIXEL_COUNT];
#else
    uint16_t *__framebuffer = NULL;
#endif
    uint16_t *__aux_buffer = NULL;
    uint16_t __aux_buffer_sram[MAX_PXL_IN_AUX];
    uint16_t *__aux_buffer_psram = NULL;

    // display properties
    struct
    {
        uint8_t orientation;
        uint8_t rotation;
        int width;
        int height;
        uint pixel_count;
        uint16_t bg_color = 0;
        struct
        {
            int x1 = 0;
            int y1 = 0;
            int x2;
            int y2;
        } clip;
    } __display;

    // print settings
    char __print_buffer[MAX_TXT_BUF_SIZE + 1];
    char * __print_buffer_psram = NULL;
    size_t __print_buffer_psram_len = 0;

    struct
    {
        const uint8_t *ptr = NULL;
        const uint8_t *widths;
        const uint8_t *data;
        uint8_t height;          // fsh
        uint8_t max_width;       // fsw
        uint16_t char_count;     // fntCharCount
        uint16_t bytes_per_char; // fsb: number of bytes per character
        uint16_t last_char;      // fsc - 1: last character in font, todo: check if fsc is correct
        uint16_t first_char;     // fso: first character in font
    } __font;

public:
    static Graphics4D &GetInstance()
    {
        static Graphics4D GraphicsInstance;
        return GraphicsInstance;
    }

    friend class GraphicsMedia4D;
    friend class GraphicsTouch4D;
};

/*
 * uSD Media (Image/Video) Related functions
 */

// Look up table to check format mode (0xF8)
enum MediaType {
    MEDIA,
    FORM,
    BUTTON,
    DUAL_FRAME_GAUGE,
    LINEAR_GAUGE,
    SLIDER,
    KNOB,
    DIGITS,
    KEYBOARD
};

struct MediaInfo
{
    size_t offset;
    int16_t x;
    int16_t y;
    uint16_t width;
    uint16_t height;
    uint8_t mode;
    uint8_t type;
    uint8_t option;
    uint16_t frames;
    size_t bytes_per_frame;
    uint16_t value = 0;
    uint16_t last_value = -1;
    size_t formDataSize = 0;
    // Form
    MediaInfo * form;
    // Parts
    MediaInfo * parts;
    bool touch_enabled = false;
    const uint16_t * properties = NULL;
};

typedef MediaInfo *MediaInfo4D;

class ImageControl
{
private:
    ImageControl(const uint8_t *ptr, size_t gcxSize);
    ImageControl(FIL *fil);
    ~ImageControl();

    uint count;             // Number of widgets found in file or array
    uint formCount;         // Number of forms found in file or array
    size_t maxFormDataSize = 0; // Max number of bytes for each form and contained widgets in GCX
    uint currentForm = -1;  // Index of current form object (for touch testing)
    uint nextForm = -1;     // Index of next form object (for touch testing)
    FIL *fil = NULL;        // Opened file for widgets
    const uint8_t *ptr = NULL;  // Pointer to data array containing graphics data
    MediaInfo4D info;       // pointer to the first structured widget info
    MediaInfo4D * forms;    // pointer to an array containing the form MediaInfo4D pointers
    // TODO: Add more useful stuff here
public:
    friend class Graphics4D;
    friend class GraphicsMedia4D;
};

typedef ImageControl *ImageControl4D;

class GraphicsMedia4D
{
public:
    // file_LoadImageControl
    ImageControl4D LoadImageControl(const uint8_t *ptr, size_t gcxSize, uint formIndex = -1);
    ImageControl4D LoadImageControl(const char *filename, uint formIndex = -1);

    uint GetCount(ImageControl4D hndl);
    FIL * GetFile(ImageControl4D hndl);
    MediaInfo4D GetInfo(ImageControl4D hndl, uint index);

    void SetProperties(ImageControl4D hndl, uint index, const uint16_t * properties);
    // img_SetWord(IMAGE_INDEX)
    uint16_t SetValue(ImageControl4D hndl, uint index, uint16_t value);
    // img_GetWord(IMAGE_INDEX)
    uint16_t GetValue(ImageControl4D hndl, uint index);
    // img_SetWord(IMAGE_XPOS & IMAGE_YPOS)
    void SetPosition(ImageControl4D hndl, uint index, int16_t x, int16_t y);
    // img_GetWord(IMAGE_FRAMES)
    uint16_t GetFrames(ImageControl4D hndl, uint index);
    // img_Show
    void Show(ImageControl4D hndl, uint index, bool draw_fb = true);
    // void Show(ImageControl4D hndl, uint index, uint16_t color, bool draw_fb = true);
    void Clear(ImageControl4D hndl, uint index, uint16_t color, bool draw_fb = true);

    void ShowForm(ImageControl4D hndl, uint index);
#ifndef LCD_TOUCH_NONE
    uint Touched(ImageControl4D hndl, uint index = -1);
#endif

private:
    GraphicsMedia4D();
    ~GraphicsMedia4D();
    GraphicsMedia4D(const GraphicsMedia4D &);
    GraphicsMedia4D &operator=(const GraphicsMedia4D &);

    Graphics4D &gfx = Graphics4D::GetInstance();

    FATFS fs;

    bool redrawForm = true;

    uint8_t * __form_data = NULL;
    size_t __max_form_data = 0;

    struct BufferRegion {
        int x1;
        int y1;
        int x2;
        int y2;
    };

    void __redraw_form_region(ImageControl4D hndl, MediaInfo4D form, int x1, int y1, int x2, int y2);
    uint16_t *__copy_form_to_aux(ImageControl4D hndl, MediaInfo4D form, int x1, int y1, int x2, int y2);
    void __draw_to_buffer(MediaInfo4D info, const uint8_t * ptr, uint16_t * dest, int width, int height, int x, int y, int value, BufferRegion * drawArea = nullptr);
    void __draw_to_buffer(ImageControl4D hndl, MediaInfo4D info, uint16_t * dest, int width, int height, int x, int y, int value, BufferRegion * drawArea = nullptr);
    void __show_digits(ImageControl4D hndl, MediaInfo4D digits, bool draw_fb = true);
    void __show_2_frame_gauge(ImageControl4D hndl, MediaInfo4D gauge, bool draw_fb = true);
    void __show_linear_gauge(ImageControl4D hndl, MediaInfo4D gauge, bool draw_fb = true); // should also work for slider
    void __show_knob(ImageControl4D hndl, MediaInfo4D knob, bool draw_fb = true);


public:
    static GraphicsMedia4D &GetInstance()
    {
        static GraphicsMedia4D GraphicsInstance;
        return GraphicsInstance;
    }
};

#ifndef LCD_TOUCH_NONE

/*
 * Touch Related functions
 */

#define TOUCH_PRESSED 3
#define TOUCH_MOVING 2   // held for CTP
#define TOUCH_RELEASED 1 // released for CTP
#define NO_TOUCH 0

#define PRESS_THRESHOLD 1000
#define RELEASE_THRESHOLD 500
#define DEBOUNCE_DELAY 20

struct TouchInfo
{
    int8_t status;
    int16_t id;
    int16_t xraw;
    int16_t yraw;
    int16_t xpos;
    int16_t ypos;
    uint8_t weight;
    uint8_t area;
};


class GraphicsTouch4D
{
public:
    bool Initialize();

#if defined(LCD_TOUCH_4WIRE) || defined(LCD_TOUCH_NS2009)
    bool Calibrate();
#endif

    uint8_t GetPoints();
    int8_t GetStatus(uint8_t point = 0);
    int16_t GetID(uint8_t point = 0);
    int16_t GetX(uint8_t point = 0);
    int16_t GetY(uint8_t point = 0);
    int16_t GetWeight(uint8_t point = 0);
    int16_t GetArea(uint8_t point = 0);

private:
    GraphicsTouch4D();
    ~GraphicsTouch4D();
    GraphicsTouch4D(const GraphicsTouch4D &);
    GraphicsTouch4D &operator=(const GraphicsTouch4D &);

    static Graphics4D &gfx;

    static uint8_t __points;
    static struct TouchInfo __touch[LCD_TOUCH_POINTS];

#if defined(LCD_TOUCH_4WIRE) || defined(LCD_TOUCH_NS2009)
    bool __calibrated = false;
    static int __calib_start_x;
    static int __calib_start_y;
    static int __calib_end_x;
    static int __calib_end_y;
#endif

    int16_t __get_raw_x(uint8_t point);
    int16_t __get_raw_y(uint8_t point);

#if defined(LCD_TOUCH_4WIRE) || defined(LCD_TOUCH_NS2009)
    void __set_calibration();
    bool __get_calibration();
#endif
public:
    static GraphicsTouch4D &GetInstance()
    {
        static GraphicsTouch4D GraphicsInstance;
        return GraphicsInstance;
    }
};

#endif

extern Graphics4D &gfx;
extern GraphicsMedia4D &img;
#ifndef LCD_TOUCH_NONE
extern GraphicsTouch4D &touch;
#endif