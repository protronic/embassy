//! Hall lighting touch UI built with the `lvgl` Rust bindings.

extern crate alloc;

use alloc::boxed::Box;
use alloc::format;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};

use cstr_core::{CStr, CString};
use heapless::Vec;
use lvgl::input_device::pointer::Pointer;
use lvgl::input_device::InputDriver;
use lvgl_sys;
use lvgl::style::Style;
use lvgl::widgets::{Btn, Label};
use lvgl::{
    init, Align, Color, Display, DrawBuffer, Event, LvError, LvResult, NativeObject, Obj, Part,
    Widget,
};

use crate::touch_config::{
    self, ALL_PREFIX, CENTRAL_OFF_LABEL, FIELDS, GROUP_BUTTON_BASE, GROUP_EYEBROW, HALL_NAME,
    LUX_SUFFIX, OFF_LABEL,
};

const MAX_BUTTONS: usize = 64;
const DRAW_BUF_PIXELS: usize = touch_config::DISPLAY_WIDTH as usize * 10;

static TOUCH_X: AtomicU16 = AtomicU16::new(0);
static TOUCH_Y: AtomicU16 = AtomicU16::new(0);
static TOUCH_PRESSED: AtomicBool = AtomicBool::new(false);

const COLOR_BG: (u8, u8, u8) = (0x10, 0x18, 0x28);
const COLOR_HEADER: (u8, u8, u8) = (0x1A, 0x4A, 0x7A);
const COLOR_CARD: (u8, u8, u8) = (0x1E, 0x2A, 0x3A);
const COLOR_BTN: (u8, u8, u8) = (0x2A, 0x3A, 0x50);
const COLOR_BTN_ACTIVE: (u8, u8, u8) = (0x2E, 0x7D, 0x32);
const COLOR_BTN_BORDER: (u8, u8, u8) = (0x3A, 0x5A, 0x8A);
const COLOR_TEXT: (u8, u8, u8) = (0xE8, 0xEE, 0xF4);
const COLOR_MUTED: (u8, u8, u8) = (0x90, 0xA0, 0xB0);

pub type PressHandler = fn(u8);
pub type ReleaseHandler = fn();

pub struct HallUi {
    _display: Display,
    _pointer: Pointer,
    buttons: Vec<Btn, MAX_BUTTONS>,
    style_btn: Style,
    style_btn_active: Style,
}

/// Update touch coordinates read from the panel I2C driver.
pub fn set_touch(x: u16, y: u16, pressed: bool) {
    TOUCH_X.store(x, Ordering::Relaxed);
    TOUCH_Y.store(y, Ordering::Relaxed);
    TOUCH_PRESSED.store(pressed, Ordering::Relaxed);
}

/// Initialize LVGL, register the framebuffer display, and build the hall UI.
pub fn setup(
    framebuffer: *mut u16,
    on_press: PressHandler,
    on_release: ReleaseHandler,
) -> LvResult<HallUi> {
    init();

    let width = touch_config::DISPLAY_WIDTH;
    let height = touch_config::DISPLAY_HEIGHT;
    let fb_ptr = framebuffer;

    let buffer = DrawBuffer::<DRAW_BUF_PIXELS>::default();
    let display = Display::register(buffer, width.into(), height.into(), move |refresh| {
        flush_area(fb_ptr, width, refresh);
    })?;

    let pointer = unsafe {
        Pointer::new_raw(Some(touch_read_cb), None, &display)?
    };

    let mut screen = display.get_scr_act()?;
    let mut screen_style = Style::default();
    screen_style.set_bg_color(Color::from_rgb(COLOR_BG));
    screen.add_style(Part::Main, &mut screen_style)?;

    build_header(&mut screen)?;
    build_summary(&mut screen)?;

    let mut style_btn = Style::default();
    style_btn.set_bg_color(Color::from_rgb(COLOR_BTN));
    style_btn.set_border_color(Color::from_rgb(COLOR_BTN_BORDER));
    style_btn.set_border_width(1);
    style_btn.set_radius(6);

    let mut style_btn_active = Style::default();
    style_btn_active.set_bg_color(Color::from_rgb(COLOR_BTN_ACTIVE));
    style_btn_active.set_border_color(Color::from_rgb((0xFF, 0xFF, 0xFF)));
    style_btn_active.set_border_width(2);
    style_btn_active.set_radius(6);

    let mut buttons = Vec::new();

    for field in FIELDS.iter() {
        let mut card = obj_create(&mut screen)?;
        style_card(&mut card)?;
        card.set_pos(field.x as i16, field.y as i16)?;
        card.set_size(field.w as i16, field.h as i16)?;

        let mut eyebrow = Label::create(&mut card)?;
        style_muted_text(&mut eyebrow)?;
        eyebrow.set_text(leaked_cstr(field.eyebrow)?)?;

        let mut title = Label::create(&mut card)?;
        style_body_text(&mut title)?;
        title.set_text(leaked_cstr(field.label)?)?;
        title.set_pos(0, 14)?;

        let btn_h = if field.h > 80 { 36 } else { 32 };
        let btn_w = ((field.w.saturating_sub(16)) / 3) as i16;
        let btn_y = (field.h.saturating_sub(btn_h as u16 + 12)) as i16;

        create_lux_button(
            &mut card,
            &format!("500 {LUX_SUFFIX}"),
            field.button_base,
            4,
            btn_y,
            btn_w,
            btn_h,
            on_press,
            on_release,
            &mut style_btn,
            &mut buttons,
        )?;
        create_lux_button(
            &mut card,
            &format!("300 {LUX_SUFFIX}"),
            field.button_base + 1,
            8 + btn_w,
            btn_y,
            btn_w,
            btn_h,
            on_press,
            on_release,
            &mut style_btn,
            &mut buttons,
        )?;
        create_lux_button(
            &mut card,
            OFF_LABEL,
            field.button_base + 2,
            12 + 2 * btn_w,
            btn_y,
            btn_w,
            btn_h,
            on_press,
            on_release,
            &mut style_btn,
            &mut buttons,
        )?;
    }

    build_group_card(
        &mut screen,
        on_press,
        on_release,
        &mut style_btn,
        &mut buttons,
    )?;

    Ok(HallUi {
        _display: display,
        _pointer: pointer,
        buttons,
        style_btn,
        style_btn_active,
    })
}

impl HallUi {
    pub fn set_button_active(&mut self, index: usize, active: bool) {
        if let Some(btn) = self.buttons.get_mut(index) {
            if active {
                btn.add_style(Part::Main, &mut self.style_btn_active).ok();
            } else {
                btn.add_style(Part::Main, &mut self.style_btn).ok();
            }
        }
    }
}

fn rgb565_from_color(color: Color) -> u16 {
    let r = color.r() as u16;
    let g = color.g() as u16;
    let b = color.b() as u16;
    ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
}

fn flush_area(fb: *mut u16, fb_width: u16, refresh: &lvgl::DisplayRefresh<DRAW_BUF_PIXELS>) {
    let area = &refresh.area;
    let width = (area.x2 - area.x1 + 1) as usize;
    let height = (area.y2 - area.y1 + 1) as usize;
    let count = width * height;

    for i in 0..count {
        let x = area.x1 as usize + i % width;
        let y = area.y1 as usize + i / width;
        let idx = y * fb_width as usize + x;
        let color = refresh.colors[i];
        let pixel = rgb565_from_color(color);
        unsafe {
            *fb.add(idx) = pixel;
        }
    }
}

unsafe extern "C" fn touch_read_cb(
    _drv: *mut lvgl_sys::lv_indev_drv_t,
    data: *mut lvgl_sys::lv_indev_data_t,
) {
    unsafe {
        (*data).point.x = TOUCH_X.load(Ordering::Relaxed) as lvgl_sys::lv_coord_t;
        (*data).point.y = TOUCH_Y.load(Ordering::Relaxed) as lvgl_sys::lv_coord_t;
        (*data).state = if TOUCH_PRESSED.load(Ordering::Relaxed) {
            lvgl_sys::lv_indev_state_t_LV_INDEV_STATE_PRESSED
        } else {
            lvgl_sys::lv_indev_state_t_LV_INDEV_STATE_RELEASED
        };
    }
}

fn obj_create(parent: &mut impl NativeObject) -> LvResult<Obj> {
    unsafe {
        let ptr = lvgl_sys::lv_obj_create(parent.raw()?.as_mut());
        NonNull::new(ptr)
            .map(Obj::from_raw)
            .ok_or(LvError::InvalidReference)
    }
}

fn cstr(text: &str) -> LvResult<CString> {
    CString::new(text).map_err(|_| LvError::InvalidReference)
}

/// Store label text for the lifetime of the firmware (UI is built once).
fn leaked_cstr(text: &str) -> LvResult<&'static CStr> {
    let owned = cstr(text)?;
    Ok(Box::leak(Box::new(owned)).as_c_str())
}

fn style_card(obj: &mut Obj) -> LvResult<()> {
    let mut style = Style::default();
    style.set_bg_color(Color::from_rgb(COLOR_CARD));
    style.set_border_color(Color::from_rgb(COLOR_BTN_BORDER));
    style.set_border_width(1);
    style.set_radius(8);
    obj.add_style(Part::Main, &mut style)
}

fn style_muted_text(label: &mut Label) -> LvResult<()> {
    let mut style = Style::default();
    style.set_text_color(Color::from_rgb(COLOR_MUTED));
    label.add_style(Part::Main, &mut style)
}

fn style_body_text(label: &mut Label) -> LvResult<()> {
    let mut style = Style::default();
    style.set_text_color(Color::from_rgb(COLOR_TEXT));
    label.add_style(Part::Main, &mut style)
}

fn build_header(screen: &mut Obj) -> LvResult<()> {
    let mut header = obj_create(screen)?;
    header.set_size(touch_config::DISPLAY_WIDTH as i16, 44)?;
    header.set_pos(0, 0)?;
    let mut header_style = Style::default();
    header_style.set_bg_color(Color::from_rgb(COLOR_HEADER));
    header_style.set_border_width(0);
    header_style.set_radius(0);
    header.add_style(Part::Main, &mut header_style)?;

    let mut label = Label::create(&mut header)?;
    style_body_text(&mut label)?;
    label.set_text(leaked_cstr(HALL_NAME)?)?;
    label.set_pos(12, 12)?;
    Ok(())
}

fn build_summary(screen: &mut Obj) -> LvResult<()> {
    let mut label = Label::create(screen)?;
    style_muted_text(&mut label)?;
    label.set_text(leaked_cstr("Lichtsteuerung bereit")?)?;
    label.set_align(Align::TopMid, 0, 50)?;
    Ok(())
}

fn build_group_card(
    screen: &mut Obj,
    on_press: PressHandler,
    on_release: ReleaseHandler,
    style_btn: &mut Style,
    buttons: &mut Vec<Btn, MAX_BUTTONS>,
) -> LvResult<()> {
    let margin = 8i16;
    let card_h = (touch_config::DISPLAY_HEIGHT / 10).max(56) as i16;
    let card_y = touch_config::DISPLAY_HEIGHT as i16 - card_h - margin;
    let card_w = touch_config::DISPLAY_WIDTH as i16 - 2 * margin;

    let mut card = obj_create(screen)?;
    style_card(&mut card)?;
    card.set_pos(margin, card_y)?;
    card.set_size(card_w, card_h)?;

    let mut eyebrow = Label::create(&mut card)?;
    style_muted_text(&mut eyebrow)?;
    eyebrow.set_text(leaked_cstr(GROUP_EYEBROW)?)?;

    let btn_h = card_h - 28;
    let btn_w = (card_w - 24) / 3;
    let btn_y = card_h - btn_h - 4;

    create_lux_button(
        &mut card,
        &format!("{ALL_PREFIX} 500 {LUX_SUFFIX}"),
        GROUP_BUTTON_BASE,
        4,
        btn_y,
        btn_w,
        btn_h,
        on_press,
        on_release,
        style_btn,
        buttons,
    )?;
    create_lux_button(
        &mut card,
        &format!("{ALL_PREFIX} 300 {LUX_SUFFIX}"),
        GROUP_BUTTON_BASE + 1,
        8 + btn_w,
        btn_y,
        btn_w,
        btn_h,
        on_press,
        on_release,
        style_btn,
        buttons,
    )?;
    create_lux_button(
        &mut card,
        CENTRAL_OFF_LABEL,
        GROUP_BUTTON_BASE + 2,
        12 + 2 * btn_w,
        btn_y,
        btn_w,
        btn_h,
        on_press,
        on_release,
        style_btn,
        buttons,
    )?;

    Ok(())
}

fn create_lux_button(
    parent: &mut Obj,
    text: &str,
    button_index: u8,
    x: i16,
    y: i16,
    w: i16,
    h: i16,
    on_press: PressHandler,
    on_release: ReleaseHandler,
    style_btn: &mut Style,
    buttons: &mut Vec<Btn, MAX_BUTTONS>,
) -> LvResult<()> {
    let mut button = Btn::create(parent)?;
    button.set_pos(x, y)?;
    button.set_size(w, h)?;
    button.add_style(Part::Main, style_btn)?;

    let mut label = Label::create(&mut button)?;
    style_body_text(&mut label)?;
    label.set_text(leaked_cstr(text)?)?;
    label.set_align(Align::Center, 0, 0)?;

    wire_button_events(&mut button, button_index, on_press, on_release)?;
    buttons.push(button).map_err(|_| LvError::LvOOMemory)?;
    Ok(())
}

fn wire_button_events(
    button: &mut Btn,
    button_index: u8,
    press_handler: PressHandler,
    release_handler: ReleaseHandler,
) -> LvResult<()> {
    button.on_event(move |_btn, event| {
        match event {
            Event::Pressed => press_handler(button_index),
            Event::Released | Event::PressLost => release_handler(),
            _ => {}
        }
    })
}
