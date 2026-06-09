#include "hall_ui.h"

#include "lvgl.h"

#include <stdio.h>
#include <string.h>

static uint16_t *g_framebuffer;
static uint16_t g_fb_width;
static uint16_t g_fb_height;

static lv_disp_draw_buf_t g_draw_buf;
static lv_color_t g_draw_buf_pixels[800 * 10];
static lv_disp_drv_t g_disp_drv;
static lv_indev_drv_t g_indev_drv;
static lv_indev_data_t g_touch_data;

static hall_ui_press_cb g_on_press;
static hall_ui_release_cb g_on_release;

static lv_obj_t *g_summary_label;
static lv_obj_t *g_button_widgets[HALL_UI_MAX_BUTTONS];
static uint8_t g_button_count;

static const lv_color_t COLOR_BG = LV_COLOR_MAKE(0x10, 0x18, 0x28);
static const lv_color_t COLOR_HEADER = LV_COLOR_MAKE(0x1A, 0x4A, 0x7A);
static const lv_color_t COLOR_CARD = LV_COLOR_MAKE(0x1E, 0x2A, 0x3A);
static const lv_color_t COLOR_BTN = LV_COLOR_MAKE(0x2A, 0x3A, 0x50);
static const lv_color_t COLOR_BTN_ACTIVE = LV_COLOR_MAKE(0x2E, 0x7D, 0x32);
static const lv_color_t COLOR_BTN_BORDER = LV_COLOR_MAKE(0x3A, 0x5A, 0x8A);
static const lv_color_t COLOR_TEXT = LV_COLOR_MAKE(0xE8, 0xEE, 0xF4);
static const lv_color_t COLOR_MUTED = LV_COLOR_MAKE(0x90, 0xA0, 0xB0);

static void disp_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
    if(g_framebuffer != NULL) {
        int32_t x;
        int32_t y;
        for(y = area->y1; y <= area->y2; y++) {
            for(x = area->x1; x <= area->x2; x++) {
                if(x >= 0 && y >= 0 && (uint32_t)x < g_fb_width && (uint32_t)y < g_fb_height) {
                    g_framebuffer[(uint32_t)y * g_fb_width + (uint32_t)x] = color_p->full;
                }
                color_p++;
            }
        }
    }

    lv_disp_flush_ready(drv);
}

static void touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    (void)drv;
    *data = g_touch_data;
}

static void setup_display(uint16_t *framebuffer, uint16_t width, uint16_t height)
{
    g_framebuffer = framebuffer;
    g_fb_width = width;
    g_fb_height = height;

    lv_init();

    lv_disp_draw_buf_init(&g_draw_buf, g_draw_buf_pixels, NULL, width * 10);
    lv_disp_drv_init(&g_disp_drv);
    g_disp_drv.hor_res = width;
    g_disp_drv.ver_res = height;
    g_disp_drv.flush_cb = disp_flush_cb;
    g_disp_drv.draw_buf = &g_draw_buf;
    lv_disp_drv_register(&g_disp_drv);

    lv_indev_drv_init(&g_indev_drv);
    g_indev_drv.type = LV_INDEV_TYPE_POINTER;
    g_indev_drv.read_cb = touch_read_cb;
    lv_indev_drv_register(&g_indev_drv);

    g_touch_data.state = LV_INDEV_STATE_RELEASED;
    g_touch_data.point.x = 0;
    g_touch_data.point.y = 0;
}

static void set_button_active_style(lv_obj_t *btn, bool active)
{
    if(active) {
        lv_obj_set_style_bg_color(btn, COLOR_BTN_ACTIVE, LV_PART_MAIN);
        lv_obj_set_style_border_color(btn, lv_color_white(), LV_PART_MAIN);
        lv_obj_set_style_border_width(btn, 2, LV_PART_MAIN);
    }
    else {
        lv_obj_set_style_bg_color(btn, COLOR_BTN, LV_PART_MAIN);
        lv_obj_set_style_border_color(btn, COLOR_BTN_BORDER, LV_PART_MAIN);
        lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN);
    }
}

static void button_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    uint8_t button_index = (uint8_t)(uintptr_t)lv_event_get_user_data(e);

    if(code == LV_EVENT_PRESSED) {
        if(g_on_press != NULL) {
            g_on_press(button_index);
        }
    }
    else if(code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        if(g_on_release != NULL) {
            g_on_release();
        }
    }
}

static lv_obj_t *create_lux_button(lv_obj_t *parent, const char *text, uint8_t button_index,
                                   lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, w, h);
    lv_obj_set_style_bg_color(btn, COLOR_BTN, LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, COLOR_BTN_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(btn, 6, LV_PART_MAIN);
    lv_obj_add_event_cb(btn, button_event_cb, LV_EVENT_ALL, (void *)(uintptr_t)button_index);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, COLOR_TEXT, LV_PART_MAIN);
    lv_obj_center(label);

    if(button_index < HALL_UI_MAX_BUTTONS) {
        g_button_widgets[button_index] = btn;
    }

    return btn;
}

static void create_field_card(lv_obj_t *parent, const hall_ui_field_t *field, const hall_ui_layout_t *layout,
                              const char *off_label)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_pos(card, field->x, field->y);
    lv_obj_set_size(card, field->w, field->h);
    lv_obj_set_style_bg_color(card, COLOR_CARD, LV_PART_MAIN);
    lv_obj_set_style_border_color(card, COLOR_BTN_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(card, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(card, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(card, 6, LV_PART_MAIN);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *eyebrow = lv_label_create(card);
    lv_label_set_text(eyebrow, field->eyebrow);
    lv_obj_set_style_text_color(eyebrow, COLOR_MUTED, LV_PART_MAIN);
    lv_obj_align(eyebrow, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *title = lv_label_create(card);
    lv_label_set_text(title, field->label);
    lv_obj_set_style_text_color(title, COLOR_TEXT, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 14);

    lv_coord_t btn_h = 32;
    if(field->h > 80) {
        btn_h = 36;
    }
    lv_coord_t btn_w = (field->w - 16) / 3;
    lv_coord_t btn_y = field->h - btn_h - 12;

    char buf[32];
    snprintf(buf, sizeof(buf), "500 %s", layout->lux_suffix);
    create_lux_button(card, buf, field->button_base + 0, 4, btn_y, btn_w, btn_h);

    snprintf(buf, sizeof(buf), "300 %s", layout->lux_suffix);
    create_lux_button(card, buf, field->button_base + 1, 8 + btn_w, btn_y, btn_w, btn_h);

    create_lux_button(card, off_label, field->button_base + 2, 12 + 2 * btn_w, btn_y, btn_w, btn_h);
}

static void create_group_card(lv_obj_t *parent, const hall_ui_layout_t *layout)
{
    lv_coord_t margin = 8;
    lv_coord_t card_h = layout->height / 10;
    if(card_h < 56) {
        card_h = 56;
    }
    lv_coord_t card_y = layout->height - card_h - margin;
    lv_coord_t card_w = layout->width - 2 * margin;

    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_pos(card, margin, card_y);
    lv_obj_set_size(card, card_w, card_h);
    lv_obj_set_style_bg_color(card, COLOR_CARD, LV_PART_MAIN);
    lv_obj_set_style_border_color(card, COLOR_BTN_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(card, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(card, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(card, 6, LV_PART_MAIN);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *eyebrow = lv_label_create(card);
    lv_label_set_text(eyebrow, layout->group_eyebrow);
    lv_obj_set_style_text_color(eyebrow, COLOR_MUTED, LV_PART_MAIN);
    lv_obj_align(eyebrow, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_coord_t btn_h = card_h - 28;
    lv_coord_t btn_w = (card_w - 24) / 3;
    lv_coord_t btn_y = card_h - btn_h - 4;

    char buf[48];
    snprintf(buf, sizeof(buf), "%s 500 %s", layout->all_prefix, layout->lux_suffix);
    create_lux_button(card, buf, layout->group_button_base + 0, 4, btn_y, btn_w, btn_h);

    snprintf(buf, sizeof(buf), "%s 300 %s", layout->all_prefix, layout->lux_suffix);
    create_lux_button(card, buf, layout->group_button_base + 1, 8 + btn_w, btn_y, btn_w, btn_h);

    create_lux_button(card, layout->central_off_label, layout->group_button_base + 2, 12 + 2 * btn_w, btn_y,
                      btn_w, btn_h);
}

void hall_ui_init(uint16_t *framebuffer, uint16_t width, uint16_t height, const hall_ui_layout_t *layout)
{
    memset(g_button_widgets, 0, sizeof(g_button_widgets));
    g_button_count = layout->button_count;

    setup_display(framebuffer, width, height);

    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, COLOR_BG, LV_PART_MAIN);

    lv_obj_t *header = lv_obj_create(screen);
    lv_obj_set_size(header, width, 44);
    lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(header, COLOR_HEADER, LV_PART_MAIN);
    lv_obj_set_style_border_width(header, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(header, 0, LV_PART_MAIN);
    lv_obj_clear_flag(header, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *hall_label = lv_label_create(header);
    lv_label_set_text(hall_label, layout->hall_name);
    lv_obj_set_style_text_color(hall_label, COLOR_TEXT, LV_PART_MAIN);
    lv_obj_align(hall_label, LV_ALIGN_LEFT_MID, 12, 0);

    g_summary_label = lv_label_create(screen);
    lv_label_set_text(g_summary_label, "Lichtsteuerung bereit");
    lv_obj_set_style_text_color(g_summary_label, COLOR_MUTED, LV_PART_MAIN);
    lv_obj_align(g_summary_label, LV_ALIGN_TOP_MID, 0, 50);

    for(uint8_t i = 0; i < layout->field_count; i++) {
        create_field_card(screen, &layout->fields[i], layout, layout->off_label);
    }

    create_group_card(screen, layout);
}

void hall_ui_set_touch(uint16_t x, uint16_t y, bool pressed)
{
    g_touch_data.point.x = x;
    g_touch_data.point.y = y;
    g_touch_data.state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

void hall_ui_tick(uint32_t ms)
{
    lv_tick_inc(ms);
}

void hall_ui_handler(void)
{
    lv_timer_handler();
}

void hall_ui_set_button_active(uint8_t button_index, bool active)
{
    if(button_index < g_button_count && g_button_widgets[button_index] != NULL) {
        set_button_active_style(g_button_widgets[button_index], active);
    }
}

void hall_ui_set_callbacks(hall_ui_press_cb on_press, hall_ui_release_cb on_release)
{
    g_on_press = on_press;
    g_on_release = on_release;
}
