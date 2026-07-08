//! OxivGL view for the JSON-driven CANopen node (`od_node`).
//!
//! CANbossTouch-style node screen: one row per OD datapoint (name,
//! index.subindex, type/access, live value) with − / + / toggle controls for
//! bus-writable entries, plus a process-data panel with virtual LEDs (from
//! `leds(mask)` in Rhai), a momentary push button (`button()` in Rhai), the
//! last script `print` line and CAN counters.

extern crate alloc;

use alloc::vec::Vec;

use log::info;
use oxivgl::enums::{EventCode, ObjFlag};
use oxivgl::event::Event;
use oxivgl::fonts::{MONTSERRAT_14, MONTSERRAT_16};
use oxivgl::style::{GradDir, Selector};
use oxivgl::view::{NavAction, View, register_event_on};
use oxivgl::widgets::{AsLvHandle, Button, Label, Obj, RADIUS_MAX, Screen, TextAlign, WidgetError};

use crate::od_node::{self, DpType};

const EVENT_RELEASED: EventCode = EventCode(oxivgl_sys::lv_event_code_t_LV_EVENT_RELEASED);
const EVENT_PRESS_LOST: EventCode = EventCode(oxivgl_sys::lv_event_code_t_LV_EVENT_PRESS_LOST);

const SCREEN_BG: u32 = 0xE7DCC8;
const SURFACE: u32 = 0xFFFDF8;
const CARD_BG: u32 = 0xFFFDF7;
const BUTTON_BG: u32 = 0xFCF7EC;
const BUTTON_BG_PRESSED: u32 = 0xE8DDC8;
const BORDER: u32 = 0xE4D8C3;
const TEXT: u32 = 0x151515;
const MUTED: u32 = 0x665F54;
const ACCENT: u32 = 0xA37418;
const LOGO: u32 = 0x6F6A62;
const LED_ON: u32 = 0xA37418;
const LED_OFF: u32 = 0xE4D8C3;

const SHELL_X: i32 = 16;
const SHELL_Y: i32 = 12;
const SHELL_W: i32 = 768;
const SHELL_H: i32 = 456;

const LIST_X: i32 = 10;
const LIST_Y: i32 = 56;
const LIST_W: i32 = 470;
const LIST_H: i32 = 390;
const ROW_H: i32 = 52;
const ROW_PAD: i32 = 8;

const PANEL_X: i32 = LIST_X + LIST_W + 8;
const PANEL_Y: i32 = LIST_Y;
const PANEL_W: i32 = SHELL_W - PANEL_X - 10;
const PANEL_H: i32 = LIST_H;

const LED_COUNT: usize = 4;

/// What a clicked button does.
#[derive(Clone, Copy)]
enum Action {
    Dec { index: u16, sub: u8, step: i32 },
    Inc { index: u16, sub: u8, step: i32 },
    Toggle { index: u16, sub: u8 },
    /// Momentary hardware button (PRESSED/RELEASED → `button()` in Rhai).
    Taster,
}

pub struct OdView {
    labels: Vec<Label<'static>>,
    objects: Vec<Obj<'static>>,
    buttons: Vec<(Button<'static>, Action)>,
    /// Value label per OD row, parallel to `od_node::od_rows()` order.
    value_labels: Vec<(Label<'static>, String)>,
    led_dots: Vec<Obj<'static>>,
    status_label: Option<Label<'static>>,
    can_label: Option<Label<'static>>,
    counter_label: Option<Label<'static>>,
    last_gen: u32,
    last_led_mask: i32,
}

impl Default for OdView {
    fn default() -> Self {
        Self {
            labels: Vec::new(),
            objects: Vec::new(),
            buttons: Vec::new(),
            value_labels: Vec::new(),
            led_dots: Vec::new(),
            status_label: None,
            can_label: None,
            counter_label: None,
            last_gen: u32::MAX,
            last_led_mask: -1,
        }
    }
}

impl View for OdView {
    fn create(&mut self, container: &Obj<'static>) -> Result<(), WidgetError> {
        container
            .bg_color(SCREEN_BG)
            .bg_opa(255)
            .style_bg_grad_dir(GradDir::None, Selector::DEFAULT)
            .remove_scrollable()
            .pad(0);

        let shell = Obj::new(container)?;
        shell
            .size(SHELL_W, SHELL_H)
            .pos(SHELL_X, SHELL_Y)
            .bg_color(SURFACE)
            .bg_opa(255)
            .style_bg_grad_dir(GradDir::None, Selector::DEFAULT)
            .border_width(0)
            .radius(18, Selector::DEFAULT)
            .remove_scrollable()
            .pad(0);

        // Header: node identity + CAN status + logo.
        self.labels
            .push(make_label(&shell, "CANOPEN NODE", 10, 8, 250, ACCENT, LabelKind::Eyebrow)?);
        let title = alloc::format!("{} · Node {}", od_node::node_name(), od_node::node_id());
        self.labels
            .push(make_label(&shell, &title, 10, 26, 420, TEXT, LabelKind::Title)?);
        let can_label = make_label(&shell, &od_node::can_status(), 200, 8, 400, MUTED, LabelKind::Body)?;
        self.can_label = Some(can_label);

        self.labels
            .push(make_label(&shell, "protronic", SHELL_W - 128, 16, 105, LOGO, LabelKind::Logo)?);
        let logo_dot = Obj::new(&shell)?;
        logo_dot
            .size(9, 9)
            .pos(SHELL_W - 24, 12)
            .bg_color(LOGO)
            .bg_opa(255)
            .border_width(0)
            .radius(RADIUS_MAX, Selector::DEFAULT)
            .remove_scrollable();
        self.objects.push(logo_dot);

        self.create_od_list(&shell)?;
        self.create_process_panel(&shell)?;

        self.objects.push(shell);
        container.update_layout();
        Ok(())
    }

    fn register_events(&mut self) {
        if let Some(screen) = Screen::active() {
            register_event_on(self, screen.handle());
        }
        for idx in 0..self.buttons.len() {
            register_event_on(self, self.buttons[idx].0.handle());
        }
    }

    fn on_event(&mut self, event: &Event) -> NavAction {
        let code = event.code();
        let action = self
            .buttons
            .iter()
            .find(|(btn, _)| btn.handle() == event.target_handle())
            .map(|(_, action)| *action);
        let Some(action) = action else {
            return NavAction::None;
        };

        if let Action::Taster = action {
            // Momentary button: held state feeds `button()` in Rhai.
            if code == EventCode::PRESSED {
                od_node::BUTTON_HELD.store(true, core::sync::atomic::Ordering::Relaxed);
            } else if code == EVENT_RELEASED || code == EVENT_PRESS_LOST {
                od_node::BUTTON_HELD.store(false, core::sync::atomic::Ordering::Relaxed);
            }
            return NavAction::None;
        }
        if code == EventCode::CLICKED {
            match action {
                Action::Dec { index, sub, step } => od_node::ui_adjust(index, sub, -step),
                Action::Inc { index, sub, step } => od_node::ui_adjust(index, sub, step),
                Action::Toggle { index, sub } => od_node::ui_toggle(index, sub),
                Action::Taster => {}
            }
        }
        NavAction::None
    }

    fn update(&mut self) -> Result<NavAction, WidgetError> {
        let generation = od_node::ui_generation();
        if generation == self.last_gen {
            return Ok(NavAction::None);
        }
        self.last_gen = generation;

        // OD values.
        let rows = od_node::od_rows();
        for (slot, row) in self.value_labels.iter_mut().zip(rows.iter()) {
            if slot.1 != row.value {
                slot.0.text(&row.value);
                slot.1 = row.value.clone();
            }
        }

        // Virtual LEDs.
        let mask = od_node::LED_MASK.load(core::sync::atomic::Ordering::Relaxed);
        if mask != self.last_led_mask {
            self.last_led_mask = mask;
            for (i, dot) in self.led_dots.iter().enumerate() {
                let on = (mask >> i) & 1 != 0;
                dot.bg_color(if on { LED_ON } else { LED_OFF });
            }
        }

        // Status lines.
        if let Some(label) = &self.status_label {
            label.text(&od_node::last_print());
        }
        if let Some(label) = &self.can_label {
            label.text(&od_node::can_status());
        }
        if let Some(label) = &self.counter_label {
            let tx = od_node::CAN_TX_COUNT.load(core::sync::atomic::Ordering::Relaxed);
            let rx = od_node::CAN_RX_COUNT.load(core::sync::atomic::Ordering::Relaxed);
            label.text(&alloc::format!("CAN TX {tx} · RX {rx}"));
        }

        Ok(NavAction::None)
    }
}

impl OdView {
    /// Scrollable list card: one row per OD datapoint.
    fn create_od_list(&mut self, shell: &Obj<'static>) -> Result<(), WidgetError> {
        let list = Obj::new(shell)?;
        list.size(LIST_W, LIST_H)
            .pos(LIST_X, LIST_Y)
            .bg_color(CARD_BG)
            .bg_opa(255)
            .style_bg_grad_dir(GradDir::None, Selector::DEFAULT)
            .border_width(1)
            .radius(14, Selector::DEFAULT)
            .pad(0);
        set_border_color(&list, BORDER, 255);
        // NOTE: keep scrollable — imported ODs can exceed the card height.

        let rows = od_node::od_rows();
        for (i, row) in rows.iter().enumerate() {
            let y = ROW_PAD + i as i32 * ROW_H;

            self.labels
                .push(make_label(&list, &row.name, ROW_PAD, y + 6, 220, TEXT, LabelKind::Title)?);
            let meta = alloc::format!(
                "0x{:04x}.{:02x} · {} · {}",
                row.index,
                row.sub,
                row.ty.name(),
                row.access.name()
            );
            self.labels
                .push(make_label(&list, &meta, ROW_PAD, y + 26, 220, MUTED, LabelKind::Body)?);

            let value_label = make_label(&list, &row.value, 234, y + 14, 120, TEXT, LabelKind::Title)?;
            self.value_labels.push((value_label, row.value.clone()));

            // Controls for bus/UI-writable entries.
            if row.access.bus_writable() {
                match row.ty {
                    DpType::Bool => {
                        let btn = make_small_button(&list, "wechseln", 362, y + 6, 94, &mut self.labels)?;
                        self.buttons.push((
                            btn,
                            Action::Toggle {
                                index: row.index,
                                sub: row.sub,
                            },
                        ));
                    }
                    DpType::Str => {} // strings: display only
                    _ => {
                        let dec = make_small_button(&list, "-", 362, y + 6, 44, &mut self.labels)?;
                        self.buttons.push((
                            dec,
                            Action::Dec {
                                index: row.index,
                                sub: row.sub,
                                step: row.step,
                            },
                        ));
                        let inc = make_small_button(&list, "+", 412, y + 6, 44, &mut self.labels)?;
                        self.buttons.push((
                            inc,
                            Action::Inc {
                                index: row.index,
                                sub: row.sub,
                                step: row.step,
                            },
                        ));
                    }
                }
            }

            if i + 1 < rows.len() {
                let sep = Obj::new(&list)?;
                sep.size(LIST_W - 2 * ROW_PAD, 1)
                    .pos(ROW_PAD, y + ROW_H - 6)
                    .bg_color(BORDER)
                    .bg_opa(255)
                    .border_width(0)
                    .remove_scrollable();
                self.objects.push(sep);
            }
        }

        self.objects.push(list);
        Ok(())
    }

    /// Right panel: virtual LEDs, momentary button, script/CAN status.
    fn create_process_panel(&mut self, shell: &Obj<'static>) -> Result<(), WidgetError> {
        let panel = Obj::new(shell)?;
        panel
            .size(PANEL_W, PANEL_H)
            .pos(PANEL_X, PANEL_Y)
            .bg_color(CARD_BG)
            .bg_opa(255)
            .style_bg_grad_dir(GradDir::None, Selector::DEFAULT)
            .border_width(1)
            .radius(14, Selector::DEFAULT)
            .remove_scrollable()
            .pad(0);
        set_border_color(&panel, BORDER, 255);

        self.labels
            .push(make_label(&panel, "PROZESSDATEN", 10, 12, PANEL_W - 20, ACCENT, LabelKind::Eyebrow)?);

        // Virtual LEDs driven by leds(mask) / led(n, on) in Rhai.
        self.labels
            .push(make_label(&panel, "LEDs (0x6200)", 10, 40, PANEL_W - 20, MUTED, LabelKind::Body)?);
        for i in 0..LED_COUNT {
            let dot = Obj::new(&panel)?;
            dot.size(28, 28)
                .pos(12 + i as i32 * 40, 62)
                .bg_color(LED_OFF)
                .bg_opa(255)
                .border_width(1)
                .radius(RADIUS_MAX, Selector::DEFAULT)
                .remove_scrollable();
            set_border_color(&dot, BORDER, 255);
            self.led_dots.push(dot);
        }

        // Momentary push button: the node's digital input (0x6000).
        self.labels
            .push(make_label(&panel, "Taster (0x6000)", 10, 108, PANEL_W - 20, MUTED, LabelKind::Body)?);
        let taster = Button::new(&panel)?;
        taster
            .remove_style_all()
            .size(PANEL_W - 24, 84)
            .pos(12, 130)
            .bg_color(BUTTON_BG)
            .bg_opa(255)
            .style_bg_grad_dir(GradDir::None, Selector::DEFAULT)
            .border_width(1)
            .radius(10, Selector::DEFAULT)
            .style_bg_color(
                unsafe { oxivgl_sys::lv_color_hex(BUTTON_BG_PRESSED) },
                oxivgl::enums::ObjState::PRESSED,
            )
            .remove_scrollable()
            .add_flag(ObjFlag::CLICKABLE)
            .bubble_events()
            .pad(0);
        set_border_color(&taster, BORDER, 255);
        let taster_label = Label::new(&taster)?;
        taster_label.remove_style_all();
        taster_label
            .text("Taster\n(halten)")
            .width(PANEL_W - 40)
            .text_color(TEXT)
            .text_font(MONTSERRAT_16)
            .text_align(TextAlign::Center)
            .center()
            .remove_scrollable();
        set_text_opa(&taster_label, 255);
        self.labels.push(taster_label);
        self.buttons.push((taster, Action::Taster));

        // Last script print + CAN counters.
        self.labels
            .push(make_label(&panel, "Skript", 10, 236, PANEL_W - 20, MUTED, LabelKind::Body)?);
        let status = make_label(&panel, "", 10, 256, PANEL_W - 20, TEXT, LabelKind::Body)?;
        self.status_label = Some(status);

        let counters = make_label(&panel, "CAN TX 0 · RX 0", 10, PANEL_H - 30, PANEL_W - 20, MUTED, LabelKind::Body)?;
        self.counter_label = Some(counters);

        self.objects.push(panel);
        Ok(())
    }

    pub fn log_layout(&self) {
        info!(
            "od view: {} rows, {} buttons, {} leds",
            self.value_labels.len(),
            self.buttons.len(),
            self.led_dots.len()
        );
    }
}

#[derive(Clone, Copy)]
enum LabelKind {
    Eyebrow,
    Title,
    Body,
    Logo,
}

fn make_label(
    parent: &impl AsLvHandle,
    text: &str,
    x: i32,
    y: i32,
    w: i32,
    color: u32,
    kind: LabelKind,
) -> Result<Label<'static>, WidgetError> {
    let label = Label::new(parent)?;
    label.remove_style_all();
    label
        .text(text)
        .pos(x, y)
        .width(w)
        .text_color(color)
        .remove_scrollable();
    set_text_opa(&label, 255);

    match kind {
        LabelKind::Eyebrow => {
            label
                .text_font(MONTSERRAT_14)
                .style_text_letter_space(3, Selector::DEFAULT)
                .text_align(TextAlign::Left);
        }
        LabelKind::Title => {
            label.text_font(MONTSERRAT_16).text_align(TextAlign::Left);
        }
        LabelKind::Body => {
            label.text_font(MONTSERRAT_14).text_align(TextAlign::Left);
        }
        LabelKind::Logo => {
            label
                .text_font(MONTSERRAT_16)
                .style_text_letter_space(1, Selector::DEFAULT)
                .text_align(TextAlign::Right);
        }
    }

    Ok(label)
}

fn make_small_button(
    parent: &impl AsLvHandle,
    text: &str,
    x: i32,
    y: i32,
    w: i32,
    labels: &mut Vec<Label<'static>>,
) -> Result<Button<'static>, WidgetError> {
    let button = Button::new(parent)?;
    button
        .remove_style_all()
        .size(w, 36)
        .pos(x, y)
        .bg_color(BUTTON_BG)
        .bg_opa(255)
        .style_bg_grad_dir(GradDir::None, Selector::DEFAULT)
        .border_width(1)
        .radius(8, Selector::DEFAULT)
        .style_bg_color(
            unsafe { oxivgl_sys::lv_color_hex(BUTTON_BG_PRESSED) },
            oxivgl::enums::ObjState::PRESSED,
        )
        .remove_scrollable()
        .add_flag(ObjFlag::CLICKABLE)
        .bubble_events()
        .pad(0);
    set_border_color(&button, BORDER, 255);

    let label = Label::new(&button)?;
    label.remove_style_all();
    label
        .text(text)
        .width(w - 6)
        .text_color(TEXT)
        .text_font(MONTSERRAT_14)
        .text_align(TextAlign::Center)
        .center()
        .remove_scrollable();
    set_text_opa(&label, 255);
    labels.push(label);

    Ok(button)
}

fn set_text_opa(obj: &impl AsLvHandle, opa: u8) {
    unsafe {
        oxivgl_sys::lv_obj_set_style_text_opa(obj.lv_handle(), opa as oxivgl_sys::lv_opa_t, 0);
    }
}

fn set_border_color(obj: &impl AsLvHandle, color: u32, opa: u8) {
    unsafe {
        oxivgl_sys::lv_obj_set_style_border_color(obj.lv_handle(), oxivgl_sys::lv_color_hex(color), 0);
        oxivgl_sys::lv_obj_set_style_border_opa(obj.lv_handle(), opa as oxivgl_sys::lv_opa_t, 0);
    }
}
