#pragma once

#include <stdbool.h>
#include <stdint.h>

#define HALL_UI_MAX_FIELDS 16
#define HALL_UI_MAX_BUTTONS 64
#define HALL_UI_MAX_STR 48

typedef struct {
    char label[HALL_UI_MAX_STR];
    char eyebrow[HALL_UI_MAX_STR];
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    uint8_t button_base;
} hall_ui_field_t;

typedef struct {
    char hall_name[HALL_UI_MAX_STR];
    char off_label[24];
    char lux_suffix[16];
    char all_prefix[24];
    char central_off_label[32];
    char group_eyebrow[32];
    uint16_t width;
    uint16_t height;
    uint8_t field_count;
    uint8_t button_count;
    uint8_t group_button_base;
    hall_ui_field_t fields[HALL_UI_MAX_FIELDS];
} hall_ui_layout_t;

typedef void (*hall_ui_press_cb)(uint8_t button_index);
typedef void (*hall_ui_release_cb)(void);

void hall_ui_init(uint16_t *framebuffer, uint16_t width, uint16_t height, const hall_ui_layout_t *layout);
void hall_ui_set_touch(uint16_t x, uint16_t y, bool pressed);
void hall_ui_tick(uint32_t ms);
void hall_ui_handler(void);
void hall_ui_set_button_active(uint8_t button_index, bool active);
void hall_ui_set_callbacks(hall_ui_press_cb on_press, hall_ui_release_cb on_release);

extern const hall_ui_layout_t g_hall_ui_layout;
