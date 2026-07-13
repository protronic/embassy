//! JSON-driven hall lighting UI for OxivGL on the gen4-FT813-70CTP-CLB.
//!
//! Uses the fixed 5-column shell layout from [`super::widget_view`]; JSON config
//! (`examples/touch-projects/<project>/hall_config.json`) supplies strings and
//! field metadata only. Button presses feed [`crate::touch_can`], and the
//! active-state highlight follows the CAN/PLC state.
//!
//! Keep in sync with `examples/rvt50hqsnwc00-b/src/oxivgl/hall_view.rs` and
//! `examples/oxivgl-host/src/hall_view.rs` (this copy is ported to the
//! oxivgl 0.5.0 `View` trait: `register_events_on` receives the container).

extern crate alloc;

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

use defmt::info;
use oxivgl::enums::{EventCode, ObjFlag, ObjState};
use oxivgl::event::Event;
use oxivgl::style::{GradDir, Selector};
use oxivgl::view::{NavAction, View, register_event_on};
use oxivgl::widgets::{AsLvHandle, Button, Label, Obj, RADIUS_MAX, TextAlign, WidgetError};
use touch_hall_common::{
    ALL_FIELDS_HEADING, ALL_PREFIX, CENTRAL_OFF_LABEL, FIELDS, GROUP_BUTTON_BASE, GROUP_EYEBROW, HALL_NAME, LUX_SUFFIX,
    OFF_LABEL, PAGE_TITLE_PREFIX, TRIBUNE_EYEBROW, touch_hold,
};

use crate::board::{DISPLAY_HEIGHT, DISPLAY_WIDTH};
use crate::oxivgl::fonts::{MONTSERRAT_14, MONTSERRAT_16};
use crate::touch_can::{self, on_button_press, on_button_release};

/// `LV_EVENT_PRESS_LOST` — fehlt in `oxivgl::enums`, daher wie in
/// `canboss::views::node` direkt aus den Bindings konstruiert. Kommt, wenn der
/// Finger (oder ein Touch-Aussetzer) den Button verlaesst: das spaetere
/// `RELEASED` geht dann an das Objekt unter dem Finger, nicht mehr an den
/// Button — ohne diese Behandlung bliebe der CAN-Hold-Loop haengen.
const EVENT_PRESS_LOST: EventCode = EventCode(oxivgl_sys::lv_event_code_t_LV_EVENT_PRESS_LOST);

const SCREEN_BG: u32 = 0xE7DCC8;
const SURFACE: u32 = 0xFFFDF8;
const CARD_BG: u32 = 0xFFFDF7;
const CARD_BG_HIGHLIGHT: u32 = 0xFFF7E7;
const BUTTON_BG: u32 = 0xFCF7EC;
const BUTTON_BG_ACTIVE: u32 = 0xEEE8DB;
const BUTTON_BG_PRESSED: u32 = 0xE8DDC8;
const BORDER: u32 = 0xE4D8C3;
const BORDER_ACTIVE: u32 = 0xC5BAA8;
const TEXT: u32 = 0x151515;
const MUTED: u32 = 0x665F54;
const ACCENT: u32 = 0xA37418;
const LOGO: u32 = 0x6F6A62;

/// Full-bleed shell on the 800×480 panel — no outer margin that can show through
/// as bright strips (especially bottom-right with EVE partial redraw).
const SHELL_X: i32 = 0;
const SHELL_Y: i32 = 0;
const SHELL_W: i32 = DISPLAY_WIDTH as i32;
const SHELL_H: i32 = DISPLAY_HEIGHT as i32;

const CARD_X0: i32 = 10;
/// Five [`CARD_W`] columns with [`CARD_X0`] side padding (pitch 160 at 800 px).
const CARD_COL_PITCH: i32 = CARD_W + (SHELL_W - 2 * CARD_X0 - 5 * CARD_W) / 4;
const CARD_W: i32 = 140;
const CARD_Y: i32 = 56;
const CARD_H: i32 = SHELL_H - CARD_Y;
const CARD_PAD_X: i32 = 10;
const CARD_LABEL_W: i32 = CARD_W - CARD_PAD_X * 2;

const BUTTON_TRIM: i32 = 2;
const BUTTON_W: i32 = 120 - BUTTON_TRIM;
const BUTTON_Y0: i32 = 58;
const BUTTON_GAP: i32 = 10;
const BUTTON_ZONE_H: i32 = CARD_H - BUTTON_Y0;
const BUTTON_H: i32 = (BUTTON_ZONE_H - 2 * BUTTON_GAP) / 3 - BUTTON_TRIM;
const BUTTON_Y_STEP: i32 = BUTTON_H + BUTTON_GAP;
const BUTTON_LABEL_W: i32 = BUTTON_W - 8;

struct ColumnSpec {
    eyebrow: String,
    title: String,
    buttons: [String; 3],
    button_indices: [u8; 3],
    highlight: bool,
}

/// JSON-driven hall lighting control view (5-column shell layout).
#[derive(Default)]
pub struct HallView {
    labels: Vec<Label<'static>>,
    buttons: Vec<Button<'static>>,
    button_indices: Vec<u8>,
    objects: Vec<Obj<'static>>,
    button_active: Vec<bool>,
    pressed_can_button: Option<u8>,
}

impl View for HallView {
    fn create(&mut self, container: &Obj<'static>) -> Result<(), WidgetError> {
        self.labels.clear();
        self.buttons.clear();
        self.button_indices.clear();
        self.objects.clear();
        self.button_active.clear();

        container
            .size(DISPLAY_WIDTH as i32, DISPLAY_HEIGHT as i32)
            .pos(0, 0)
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
            .radius(0, Selector::DEFAULT)
            .remove_scrollable()
            .pad(0);

        let header = uppercase_ascii(PAGE_TITLE_PREFIX);
        self.labels
            .push(make_label(&shell, &header, 10, 8, 250, ACCENT, LabelKind::Eyebrow)?);

        let badge = Obj::new(&shell)?;
        badge
            .size(110, 28)
            .pos(SHELL_W / 2 - 55, 16)
            .bg_color(SURFACE)
            .bg_opa(255)
            .border_width(1)
            .radius(RADIUS_MAX, Selector::DEFAULT)
            .remove_scrollable()
            .pad(0);
        set_border_color(&badge, BORDER, 255);
        self.labels
            .push(make_label(&badge, HALL_NAME, 0, 6, 110, MUTED, LabelKind::Body)?);
        self.objects.push(badge);

        self.labels.push(make_label(
            &shell,
            "protronic",
            SHELL_W - 128,
            16,
            105,
            LOGO,
            LabelKind::Logo,
        )?);
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

        for (idx, column) in build_columns().iter().enumerate() {
            let x = CARD_X0 + idx as i32 * CARD_COL_PITCH;
            self.create_column(&shell, column, x)?;
        }

        self.objects.push(shell);
        container.update_layout();
        container.invalidate();
        Ok(())
    }

    fn register_events_on(&mut self, container: &Obj<'static>) {
        // Container-level handler catches bubbled events from children.
        register_event_on(self, container.lv_handle());

        for idx in 0..self.buttons.len() {
            register_event_on(self, self.buttons[idx].handle());
        }
    }

    fn on_event(&mut self, event: &Event) -> NavAction {
        let btn_idx = self.button_index_for_handle(event.target_handle());
        match event.code() {
            EventCode::PRESSED => {
                if let Some(ui_idx) = btn_idx {
                    if let Some(index) = self.button_indices.get(ui_idx) {
                        self.pressed_can_button = Some(*index);
                        on_button_press(*index);
                        info!("hall button press {}", index);
                    }
                }
            }
            EventCode::RELEASED | EVENT_PRESS_LOST => {
                if let Some(ui_idx) = btn_idx {
                    if let Some(index) = self.button_indices.get(ui_idx) {
                        if self.pressed_can_button == Some(*index) {
                            on_button_release();
                            if !touch_hold::is_latched() {
                                self.pressed_can_button = None;
                            }
                        }
                    }
                }
            }
            _ => {}
        }
        NavAction::None
    }

    fn update(&mut self) -> Result<NavAction, WidgetError> {
        let updates: Vec<_> = self
            .button_indices
            .iter()
            .enumerate()
            .filter_map(|(ui_idx, protocol_idx)| {
                let active = touch_can::button_status(*protocol_idx as usize);
                if self.button_active.get(ui_idx).copied().unwrap_or(false) != active {
                    Some((ui_idx, active))
                } else {
                    None
                }
            })
            .collect();
        for (ui_idx, active) in updates {
            self.set_button_active(ui_idx, active);
        }
        Ok(NavAction::None)
    }
}

impl HallView {
    fn create_column(&mut self, parent: &impl AsLvHandle, column: &ColumnSpec, x: i32) -> Result<(), WidgetError> {
        let card = Obj::new(parent)?;
        card.size(CARD_W, CARD_H)
            .pos(x, CARD_Y)
            .bg_color(if column.highlight { CARD_BG_HIGHLIGHT } else { CARD_BG })
            .bg_opa(255)
            .style_bg_grad_dir(GradDir::None, Selector::DEFAULT)
            .border_width(1)
            .radius(14, Selector::DEFAULT)
            .remove_scrollable()
            .pad(0);
        set_border_color(&card, BORDER, 255);

        self.labels.push(make_label(
            &card,
            &column.eyebrow,
            CARD_PAD_X,
            12,
            CARD_LABEL_W,
            ACCENT,
            LabelKind::Eyebrow,
        )?);
        self.labels.push(make_label(
            &card,
            &column.title,
            CARD_PAD_X,
            28,
            CARD_LABEL_W,
            TEXT,
            LabelKind::Title,
        )?);

        for (idx, text) in column.buttons.iter().enumerate() {
            let button = make_scene_button(
                &card,
                text,
                CARD_PAD_X,
                BUTTON_Y0 + idx as i32 * BUTTON_Y_STEP,
                false,
                &mut self.labels,
            )?;
            self.buttons.push(button);
            self.button_indices.push(column.button_indices[idx]);
            self.button_active.push(false);
        }

        self.objects.push(card);
        Ok(())
    }

    fn set_button_active(&mut self, index: usize, active: bool) {
        let Some(button) = self.buttons.get(index) else {
            return;
        };
        self.button_active.resize(self.buttons.len(), false);
        if let Some(slot) = self.button_active.get_mut(index) {
            *slot = active;
        }
        button
            .bg_color(if active { BUTTON_BG_ACTIVE } else { BUTTON_BG })
            .style_bg_color(
                unsafe { oxivgl_sys::lv_color_hex(BUTTON_BG_PRESSED) },
                ObjState::PRESSED,
            );
        set_border_color(button, if active { BORDER_ACTIVE } else { BORDER }, 255);
    }

    fn button_index_for_handle(&self, handle: *mut oxivgl_sys::lv_obj_t) -> Option<usize> {
        self.buttons.iter().position(|btn| btn.handle() == handle)
    }
}

fn build_columns() -> Vec<ColumnSpec> {
    let mut columns = Vec::with_capacity(5);

    for (i, field) in FIELDS.iter().enumerate() {
        let eyebrow = if i == 0 && !TRIBUNE_EYEBROW.is_empty() {
            TRIBUNE_EYEBROW
        } else {
            field.eyebrow
        };
        columns.push(ColumnSpec {
            eyebrow: eyebrow.into(),
            title: field.label.into(),
            buttons: [
                format!("500 {LUX_SUFFIX}"),
                format!("300 {LUX_SUFFIX}"),
                OFF_LABEL.into(),
            ],
            button_indices: [field.button_base, field.button_base + 1, field.button_base + 2],
            highlight: false,
        });
    }

    columns.push(ColumnSpec {
        eyebrow: GROUP_EYEBROW.into(),
        title: ALL_FIELDS_HEADING.into(),
        buttons: [
            format!("{ALL_PREFIX}\n500 {LUX_SUFFIX}"),
            format!("{ALL_PREFIX}\n300 {LUX_SUFFIX}"),
            CENTRAL_OFF_LABEL.into(),
        ],
        button_indices: [GROUP_BUTTON_BASE, GROUP_BUTTON_BASE + 1, GROUP_BUTTON_BASE + 2],
        highlight: true,
    });

    columns
}

fn uppercase_ascii(s: &str) -> String {
    s.chars()
        .map(|c| {
            if c.is_ascii_lowercase() {
                char::from_u32(c as u32 - 32).unwrap_or(c)
            } else {
                c
            }
        })
        .collect()
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
            label.text_font(MONTSERRAT_14).text_align(TextAlign::Center);
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

fn make_scene_button(
    parent: &impl AsLvHandle,
    text: &str,
    x: i32,
    y: i32,
    active: bool,
    labels: &mut Vec<Label<'static>>,
) -> Result<Button<'static>, WidgetError> {
    let button = Button::new(parent)?;
    button
        .remove_style_all()
        .size(BUTTON_W, BUTTON_H)
        .pos(x, y)
        .bg_color(if active { BUTTON_BG_ACTIVE } else { BUTTON_BG })
        .bg_opa(255)
        .style_bg_grad_dir(GradDir::None, Selector::DEFAULT)
        .border_width(1)
        .radius(10, Selector::DEFAULT)
        .style_bg_color(
            unsafe { oxivgl_sys::lv_color_hex(BUTTON_BG_PRESSED) },
            ObjState::PRESSED,
        )
        .remove_scrollable()
        .add_flag(ObjFlag::CLICKABLE)
        .bubble_events()
        .pad(0);
    set_border_color(&button, if active { BORDER_ACTIVE } else { BORDER }, 255);

    let label = Label::new(&button)?;
    label.remove_style_all();
    label
        .text(text)
        .width(BUTTON_LABEL_W)
        .text_color(TEXT)
        .text_font(MONTSERRAT_16)
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
