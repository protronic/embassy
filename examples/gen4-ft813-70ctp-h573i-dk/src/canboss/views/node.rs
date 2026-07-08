//! Generierter Knoten-Screen: eine Widget-Zeile je EDS-Datenpunkt.
//!
//! Rust-Gegenstueck zu `App/canboss_ui.c` im C-Projekt: Werte werden
//! zyklisch per SDO-Upload gelesen (max. [`READS_PER_SCAN`] neue Reads je
//! Scan, [`READ_INTERVAL_MS`] je Zeile) und aus den Widgets per
//! SDO-Download geschrieben. Widget-Zuordnung siehe [`WidgetKind`]
//! (Wert/Balken/Switch/Slider/Spinbox/Festkomma-F32/Textfeld).

extern crate alloc;

use alloc::vec::Vec;
use core::fmt::Write as _;

use embassy_time::{Duration, Instant};
use heapless::String;
use oxivgl::enums::{EventCode, ObjFlag, ObjState};
use oxivgl::event::Event;
use oxivgl::style::Selector;
use oxivgl::view::{NavAction, View, register_event_on};
use oxivgl::widgets::{
    AsLvHandle, Bar, Button, Keyboard, Label, Obj, Slider, Spinbox, Switch, TextAlign, Textarea, WidgetError,
};

use super::{ACCENT, BORDER, CARD_BG, ERR_RED, MUTED, OK_GREEN, SCREEN_BG, SURFACE, TEXT};
use crate::canboss::sdo::{self, SDO_DATA_MAX, SdoState};
use crate::canboss::{Access, DType, Dp, F32_SCALE, NodeDesc, WidgetKind, decode_int, encode_int};
use crate::oxivgl::fonts::{MONTSERRAT_14, MONTSERRAT_16};

const EVENT_RELEASED: EventCode = EventCode(oxivgl_sys::lv_event_code_t_LV_EVENT_RELEASED);
const EVENT_CANCEL: EventCode = EventCode(oxivgl_sys::lv_event_code_t_LV_EVENT_CANCEL);

/// Zykluszeit je Datenpunkt (wie CB_READ_INTERVAL_MS im C-Port).
const READ_INTERVAL_MS: u64 = 1000;
/// Neue Leseauftraege je update()-Scan (wie CB_READS_PER_TICK).
const READS_PER_SCAN: usize = 4;
/// Nach Nutzereingabe: Refresh aussetzen (wie CB_EDIT_HOLD_MS).
const EDIT_HOLD_MS: u64 = 2000;

const HEADER_H: i32 = 48;
const ROW_W: i32 = 760;
const ROW_H: i32 = 66;
const ROW_STEP: i32 = 74;
const ROW_PAD: i32 = 12;
const WIDGET_RIGHT: i32 = ROW_W - 16;

enum RowWidget {
    Value,
    Bar(Bar<'static>),
    Switch(Switch<'static>),
    Slider(Slider<'static>),
    Spinbox {
        sb: Spinbox<'static>,
        dec: Button<'static>,
        inc: Button<'static>,
        f32_fixed: bool,
    },
    Text(Textarea<'static>),
}

struct Row {
    dp: &'static Dp,
    slot: usize,
    widget: RowWidget,
    value_label: Option<Label<'static>>,
    next_read: Instant,
    req_is_write: bool,
    pending_write: Option<([u8; SDO_DATA_MAX], usize)>,
    errors: u8,
    had_ok: bool,
}

/// Screen eines CANopen-Knotens (aus den generierten EDS-Tabellen).
pub struct NodeView {
    node: &'static NodeDesc,
    rows: Vec<Row>,
    labels: Vec<Label<'static>>,
    objects: Vec<Obj<'static>>,
    back: Option<Button<'static>>,
    status_led: Option<Obj<'static>>,
    keyboard: Option<Keyboard<'static>>,
    led_ok: Option<bool>,
}

impl NodeView {
    pub fn new(node: &'static NodeDesc) -> Self {
        Self {
            node,
            rows: Vec::new(),
            labels: Vec::new(),
            objects: Vec::new(),
            back: None,
            status_led: None,
            keyboard: None,
            led_ok: None,
        }
    }
}

impl View for NodeView {
    fn create(&mut self, container: &Obj<'static>) -> Result<(), WidgetError> {
        self.rows.clear();
        self.labels.clear();
        self.objects.clear();
        self.led_ok = None;

        container.bg_color(SCREEN_BG).bg_opa(255).remove_scrollable().pad(0);

        // ── Header: Zurueck + Titel + Status-LED ──────────────────────────
        let header = Obj::new(container)?;
        header
            .size(800, HEADER_H)
            .pos(0, 0)
            .bg_color(SURFACE)
            .bg_opa(255)
            .border_width(0)
            .radius(0, Selector::DEFAULT)
            .remove_scrollable()
            .pad(0);

        let back = Button::new(&header)?;
        back.remove_style_all()
            .size(110, 32)
            .pos(8, 8)
            .bg_color(CARD_BG)
            .bg_opa(255)
            .border_width(1)
            .radius(8, Selector::DEFAULT)
            .remove_scrollable()
            .pad(0);
        set_border(&back, BORDER);
        let back_label = Label::new(&back)?;
        back_label
            .text("< Netzwerk")
            .width(100)
            .text_color(TEXT)
            .text_font(MONTSERRAT_14)
            .center();
        self.labels.push(back_label);
        self.back = Some(back);

        let title = Label::new(&header)?;
        let mut title_text: String<80> = String::new();
        let _ = write!(title_text, "Node {}  {}", self.node.node_id, self.node.name);
        title
            .text(title_text.as_str())
            .pos(132, 14)
            .width(600)
            .text_color(TEXT)
            .text_font(MONTSERRAT_16)
            .text_align(TextAlign::Left);
        self.labels.push(title);

        let led = Obj::new(&header)?;
        led.size(16, 16)
            .pos(800 - 28, 16)
            .bg_color(MUTED)
            .bg_opa(255)
            .border_width(0)
            .radius(8, Selector::DEFAULT)
            .remove_scrollable();
        self.status_led = Some(led);
        self.objects.push(header);

        // ── Scrollbare Datenpunktliste ────────────────────────────────────
        let list = Obj::new(container)?;
        list.size(800, 480 - HEADER_H)
            .pos(0, HEADER_H)
            .bg_color(SCREEN_BG)
            .bg_opa(255)
            .border_width(0)
            .radius(0, Selector::DEFAULT)
            .pad(0);

        let count = self.node.dps.len().min(sdo::SLOT_COUNT);
        for (i, dp) in self.node.dps.iter().take(count).enumerate() {
            self.create_row(&list, i, dp)?;
        }
        // Hoehe des Inhalts vorgeben, damit LVGL scrollen kann
        let spacer = Obj::new(&list)?;
        spacer
            .size(1, 1)
            .pos(2, 8 + count as i32 * ROW_STEP)
            .bg_opa(0)
            .border_width(0);
        self.objects.push(spacer);
        self.objects.push(list);

        // ── Geteilte Bildschirmtastatur (fuer Textzeilen) ─────────────────
        let keyboard = Keyboard::new(container)?;
        let kb = keyboard.lv_handle();
        unsafe {
            oxivgl_sys::lv_obj_set_size(kb, 800, 200);
            oxivgl_sys::lv_obj_set_pos(kb, 0, 480 - 200);
        }
        keyboard.add_flag(ObjFlag::HIDDEN);
        self.keyboard = Some(keyboard);

        container.update_layout();
        Ok(())
    }

    fn register_events_on(&mut self, container: &Obj<'static>) {
        register_event_on(self, container.lv_handle());
        if let Some(back) = &self.back {
            register_event_on(self, back.lv_handle());
        }
        let mut handles: alloc::vec::Vec<*mut oxivgl_sys::lv_obj_t> = alloc::vec::Vec::new();
        for row in &self.rows {
            match &row.widget {
                RowWidget::Value | RowWidget::Bar(_) => {}
                RowWidget::Switch(w) => handles.push(w.lv_handle()),
                RowWidget::Slider(w) => handles.push(w.lv_handle()),
                RowWidget::Spinbox { sb, dec, inc, .. } => {
                    handles.push(sb.lv_handle());
                    handles.push(dec.lv_handle());
                    handles.push(inc.lv_handle());
                }
                RowWidget::Text(w) => handles.push(w.lv_handle()),
            }
        }
        for handle in handles {
            register_event_on(self, handle);
        }
    }

    fn on_event(&mut self, event: &Event) -> NavAction {
        let code = event.code();
        let handle = event.target_handle();

        if code == EventCode::CLICKED {
            if let Some(back) = &self.back {
                if back.lv_handle() == handle {
                    return NavAction::Pop(None);
                }
            }
        }

        let Some(row_idx) = self.row_for_handle(handle) else {
            return NavAction::None;
        };
        self.handle_row_event(row_idx, handle, code);
        NavAction::None
    }

    fn update(&mut self) -> Result<NavAction, WidgetError> {
        let now = Instant::now();
        let mut reads_started = 0usize;
        let mut any_ok = false;
        let mut any_row = false;

        for row in self.rows.iter_mut() {
            any_row = true;

            match sdo::state(row.slot) {
                SdoState::Done => {
                    let mut data = [0u8; SDO_DATA_MAX];
                    let (len, _) = sdo::read_result(row.slot, &mut data);
                    sdo::release(row.slot);
                    row.errors = 0;
                    row.had_ok = true;
                    if !row.req_is_write {
                        apply_read_result(row, &data[..len]);
                    }
                }
                SdoState::Error => {
                    sdo::release(row.slot);
                    row.errors = row.errors.saturating_add(1);
                    if !row.req_is_write && row.errors >= 2 {
                        if let (Some(label), RowWidget::Value | RowWidget::Bar(_)) = (&row.value_label, &row.widget) {
                            label.text("--");
                        }
                    }
                }
                SdoState::Pending => continue,
                SdoState::Idle => {}
            }
            if row.had_ok && row.errors == 0 {
                any_ok = true;
            }

            if sdo::state(row.slot) != SdoState::Idle {
                continue;
            }

            // Wartende Schreibauftraege zuerst
            if let Some((data, len)) = row.pending_write.take() {
                if sdo::submit_write(row.slot, self.node.node_id, row.dp.index, row.dp.sub, &data[..len]) {
                    row.req_is_write = true;
                } else {
                    row.pending_write = Some((data, len));
                }
                continue;
            }

            // Zyklisches Lesen (wo-Objekte lassen sich nicht lesen)
            if row.dp.access == Access::Wo {
                continue;
            }
            if reads_started < READS_PER_SCAN && now >= row.next_read {
                if sdo::submit_read(row.slot, self.node.node_id, row.dp.index, row.dp.sub) {
                    row.req_is_write = false;
                    reads_started += 1;
                }
                row.next_read = now + Duration::from_millis(READ_INTERVAL_MS);
            }
        }

        // Status-LED: gruen sobald mindestens ein Datenpunkt gelesen wurde
        if any_row {
            let ok = any_ok;
            if self.led_ok != Some(ok) {
                self.led_ok = Some(ok);
                if let Some(led) = &self.status_led {
                    led.bg_color(if ok { OK_GREEN } else { ERR_RED });
                }
            }
        }
        Ok(NavAction::None)
    }
}

impl NodeView {
    fn row_for_handle(&self, handle: *mut oxivgl_sys::lv_obj_t) -> Option<usize> {
        self.rows.iter().position(|row| match &row.widget {
            RowWidget::Value | RowWidget::Bar(_) => false,
            RowWidget::Switch(w) => w.lv_handle() == handle,
            RowWidget::Slider(w) => w.lv_handle() == handle,
            RowWidget::Spinbox { sb, dec, inc, .. } => {
                sb.lv_handle() == handle || dec.lv_handle() == handle || inc.lv_handle() == handle
            }
            RowWidget::Text(w) => w.lv_handle() == handle,
        })
    }

    fn queue_write(&mut self, row_idx: usize, payload: &[u8]) {
        let row = &mut self.rows[row_idx];
        let mut data = [0u8; SDO_DATA_MAX];
        let len = payload.len().min(SDO_DATA_MAX);
        data[..len].copy_from_slice(&payload[..len]);
        row.pending_write = Some((data, len));
        row.next_read = Instant::now() + Duration::from_millis(EDIT_HOLD_MS);
    }

    fn handle_row_event(&mut self, row_idx: usize, handle: *mut oxivgl_sys::lv_obj_t, code: EventCode) {
        enum Act {
            None,
            WriteInt(i64, usize),
            WriteF32(f32),
            WriteText,
            SliderLabel(i32),
        }
        let mut act = Act::None;

        {
            let row = &self.rows[row_idx];
            let dsize = row.dp.dtype.size().max(1);
            match &row.widget {
                RowWidget::Switch(sw) => {
                    if code == EventCode::VALUE_CHANGED {
                        let on = Obj::from_raw_non_owning(sw.lv_handle()).has_state(ObjState::CHECKED);
                        act = Act::WriteInt(on as i64, dsize);
                    }
                }
                RowWidget::Slider(slider) => {
                    if code == EventCode::VALUE_CHANGED {
                        act = Act::SliderLabel(slider.get_value());
                    } else if code == EVENT_RELEASED {
                        act = Act::WriteInt(slider.get_value() as i64, dsize);
                    }
                }
                RowWidget::Spinbox {
                    sb,
                    dec,
                    inc,
                    f32_fixed,
                } => {
                    if code == EventCode::CLICKED && handle == dec.lv_handle() {
                        sb.decrement();
                    } else if code == EventCode::CLICKED && handle == inc.lv_handle() {
                        sb.increment();
                    } else if code != EventCode::VALUE_CHANGED || handle != sb.lv_handle() {
                        return;
                    }
                    let v = sb.get_value();
                    act = if *f32_fixed {
                        Act::WriteF32(v as f32 / F32_SCALE)
                    } else {
                        Act::WriteInt(v as i64, dsize)
                    };
                }
                RowWidget::Text(ta) => {
                    if code == EventCode::FOCUSED {
                        if let Some(kb) = &self.keyboard {
                            kb.set_textarea(ta);
                            kb.remove_flag(ObjFlag::HIDDEN);
                        }
                        return;
                    }
                    if code == EventCode::DEFOCUSED || code == EVENT_CANCEL {
                        if let Some(kb) = &self.keyboard {
                            kb.add_flag(ObjFlag::HIDDEN);
                        }
                        return;
                    }
                    if code == EventCode::READY {
                        act = Act::WriteText;
                    }
                }
                RowWidget::Value | RowWidget::Bar(_) => {}
            }
        }

        match act {
            Act::None => {}
            Act::WriteInt(v, len) => {
                let mut data = [0u8; 8];
                encode_int(&mut data, len, v);
                self.queue_write(row_idx, &data[..len]);
            }
            Act::WriteF32(f) => {
                self.queue_write(row_idx, &f.to_le_bytes());
            }
            Act::WriteText => {
                let mut buf = [0u8; SDO_DATA_MAX];
                let mut len = 0usize;
                if let RowWidget::Text(ta) = &self.rows[row_idx].widget {
                    if let Some(text) = ta.get_text() {
                        len = text.len().min(SDO_DATA_MAX);
                        buf[..len].copy_from_slice(&text.as_bytes()[..len]);
                    }
                }
                self.queue_write(row_idx, &buf[..len]);
                if let Some(kb) = &self.keyboard {
                    kb.add_flag(ObjFlag::HIDDEN);
                }
            }
            Act::SliderLabel(v) => {
                let row = &mut self.rows[row_idx];
                if let Some(label) = &row.value_label {
                    let mut s: String<16> = String::new();
                    let _ = write!(s, "{}", v);
                    label.text(s.as_str());
                }
                row.next_read = Instant::now() + Duration::from_millis(EDIT_HOLD_MS);
            }
        }
    }

    fn create_row(&mut self, parent: &Obj<'static>, i: usize, dp: &'static Dp) -> Result<(), WidgetError> {
        let row_obj = Obj::new(parent)?;
        row_obj
            .size(ROW_W, ROW_H)
            .pos(20, 8 + i as i32 * ROW_STEP)
            .bg_color(CARD_BG)
            .bg_opa(255)
            .border_width(1)
            .radius(10, Selector::DEFAULT)
            .remove_scrollable()
            .pad(0);
        set_border(&row_obj, BORDER);

        let name = Label::new(&row_obj)?;
        name.text(dp.name)
            .pos(ROW_PAD, 10)
            .width(420)
            .text_color(TEXT)
            .text_font(MONTSERRAT_16)
            .text_align(TextAlign::Left);
        self.labels.push(name);

        let addr = Label::new(&row_obj)?;
        let mut addr_text: String<16> = String::new();
        let _ = write!(addr_text, "0x{:04X}.{:02X}", dp.index, dp.sub);
        addr.text(addr_text.as_str())
            .pos(ROW_PAD, 38)
            .width(200)
            .text_color(MUTED)
            .text_font(MONTSERRAT_14)
            .text_align(TextAlign::Left);
        self.labels.push(addr);

        let mut value_label = None;
        let widget = match dp.widget {
            WidgetKind::Value => {
                value_label = Some(self.make_value_label(&row_obj, WIDGET_RIGHT - 220, 20, 220)?);
                RowWidget::Value
            }
            WidgetKind::Bar => {
                value_label = Some(self.make_value_label(&row_obj, WIDGET_RIGHT - 220, 8, 220)?);
                let bar = Bar::new(&row_obj)?;
                bar.set_range_raw(dp.min, dp.max);
                let h = bar.lv_handle();
                unsafe {
                    oxivgl_sys::lv_obj_set_size(h, 160, 12);
                    oxivgl_sys::lv_obj_set_pos(h, WIDGET_RIGHT - 160, 42);
                }
                RowWidget::Bar(bar)
            }
            WidgetKind::Switch => {
                let sw = Switch::new(&row_obj)?;
                let h = sw.lv_handle();
                unsafe {
                    oxivgl_sys::lv_obj_set_size(h, 64, 32);
                    oxivgl_sys::lv_obj_set_pos(h, WIDGET_RIGHT - 64, 17);
                }
                RowWidget::Switch(sw)
            }
            WidgetKind::Slider => {
                value_label = Some(self.make_value_label(&row_obj, WIDGET_RIGHT - 220, 6, 220)?);
                let slider = Slider::new(&row_obj)?;
                slider.set_range(dp.min, dp.max);
                let h = slider.lv_handle();
                unsafe {
                    oxivgl_sys::lv_obj_set_size(h, 210, 14);
                    oxivgl_sys::lv_obj_set_pos(h, WIDGET_RIGHT - 220, 40);
                }
                RowWidget::Slider(slider)
            }
            WidgetKind::Spinbox | WidgetKind::SpinboxF32 => {
                let f32_fixed = dp.widget == WidgetKind::SpinboxF32;
                let dec = self.make_step_button(&row_obj, WIDGET_RIGHT - 236, "-")?;
                let sb = Spinbox::new(&row_obj)?;
                if dp.has_limits {
                    sb.set_range(dp.min, dp.max);
                }
                if f32_fixed {
                    sb.set_digit_format(8, 5); // Festkomma x1000
                } else {
                    sb.set_digit_format(8, 0);
                }
                let h = sb.lv_handle();
                unsafe {
                    oxivgl_sys::lv_obj_set_size(h, 140, 40);
                    oxivgl_sys::lv_obj_set_pos(h, WIDGET_RIGHT - 190, 13);
                }
                let inc = self.make_step_button(&row_obj, WIDGET_RIGHT - 44, "+")?;
                RowWidget::Spinbox {
                    sb,
                    dec,
                    inc,
                    f32_fixed,
                }
            }
            WidgetKind::Text => {
                let ta = Textarea::new(&row_obj)?;
                ta.set_one_line(true).set_max_length((SDO_DATA_MAX - 1) as u32);
                let h = ta.lv_handle();
                unsafe {
                    oxivgl_sys::lv_obj_set_size(h, 260, 40);
                    oxivgl_sys::lv_obj_set_pos(h, WIDGET_RIGHT - 260, 13);
                }
                RowWidget::Text(ta)
            }
        };

        self.objects.push(row_obj);
        self.rows.push(Row {
            dp,
            slot: i,
            widget,
            value_label,
            next_read: Instant::now(),
            req_is_write: false,
            pending_write: None,
            errors: 0,
            had_ok: false,
        });
        Ok(())
    }

    fn make_value_label(
        &mut self,
        parent: &Obj<'static>,
        x: i32,
        y: i32,
        w: i32,
    ) -> Result<Label<'static>, WidgetError> {
        let label = Label::new(parent)?;
        label
            .text("--")
            .pos(x, y)
            .width(w)
            .text_color(TEXT)
            .text_font(MONTSERRAT_16)
            .text_align(TextAlign::Right);
        Ok(label)
    }

    fn make_step_button(&mut self, parent: &Obj<'static>, x: i32, text: &str) -> Result<Button<'static>, WidgetError> {
        let button = Button::new(parent)?;
        button
            .remove_style_all()
            .size(40, 40)
            .pos(x, 13)
            .bg_color(ACCENT)
            .bg_opa(255)
            .border_width(0)
            .radius(8, Selector::DEFAULT)
            .remove_scrollable()
            .add_flag(ObjFlag::CLICKABLE)
            .pad(0);
        let label = Label::new(&button)?;
        label
            .text(text)
            .width(30)
            .text_color(SURFACE)
            .text_font(MONTSERRAT_16)
            .center();
        self.labels.push(label);
        Ok(button)
    }
}

/// SDO-Leseergebnis in das Zeilen-Widget uebernehmen (Pendant zu
/// `cb_apply_read_result` im C-Port, inkl. Edit-Guard).
fn apply_read_result(row: &mut Row, data: &[u8]) {
    // Widget nicht ueberschreiben, solange der Nutzer editiert/drueckt
    const STATE_EDITED: ObjState = ObjState(oxivgl_sys::lv_state_t_LV_STATE_EDITED);
    let editing = |h: *mut oxivgl_sys::lv_obj_t| {
        let o = Obj::from_raw_non_owning(h);
        o.has_state(ObjState::PRESSED) || o.has_state(ObjState::FOCUSED) || o.has_state(STATE_EDITED)
    };

    match &row.widget {
        RowWidget::Value => {
            if let Some(label) = &row.value_label {
                let mut text: String<64> = String::new();
                format_value(row.dp, data, &mut text);
                label.text(text.as_str());
            }
        }
        RowWidget::Bar(bar) => {
            let v = decode_int(data, row.dp.dtype.is_signed()) as i32;
            bar.set_value_raw(v, false);
            if let Some(label) = &row.value_label {
                let mut text: String<64> = String::new();
                format_value(row.dp, data, &mut text);
                label.text(text.as_str());
            }
        }
        RowWidget::Switch(sw) => {
            if editing(sw.lv_handle()) {
                return;
            }
            let on = !data.is_empty() && data[0] != 0;
            let obj = Obj::from_raw_non_owning(sw.lv_handle());
            if on != obj.has_state(ObjState::CHECKED) {
                if on {
                    obj.add_state(ObjState::CHECKED);
                } else {
                    obj.remove_state(ObjState::CHECKED);
                }
            }
        }
        RowWidget::Slider(slider) => {
            if editing(slider.lv_handle()) {
                return;
            }
            let v = decode_int(data, row.dp.dtype.is_signed()) as i32;
            if v != slider.get_value() {
                slider.set_value(v);
            }
            if let Some(label) = &row.value_label {
                let mut text: String<16> = String::new();
                let _ = write!(text, "{}", v);
                label.text(text.as_str());
            }
        }
        RowWidget::Spinbox { sb, f32_fixed, .. } => {
            if editing(sb.lv_handle()) {
                return;
            }
            let v = if *f32_fixed {
                let mut b = [0u8; 4];
                let n = data.len().min(4);
                b[..n].copy_from_slice(&data[..n]);
                let f = f32::from_le_bytes(b) * F32_SCALE;
                (if f >= 0.0 { f + 0.5 } else { f - 0.5 }) as i32
            } else {
                decode_int(data, row.dp.dtype.is_signed()) as i32
            };
            if v != sb.get_value() {
                sb.set_value(v);
            }
        }
        RowWidget::Text(ta) => {
            if editing(ta.lv_handle()) {
                return;
            }
            let mut text: String<64> = String::new();
            format_value(row.dp, data, &mut text);
            if ta.get_text() != Some(text.as_str()) {
                ta.set_text(text.as_str());
            }
        }
    }
}

/// Wert datentyp-gerecht formatieren (Pendant zu `cb_format_value`).
fn format_value(dp: &Dp, data: &[u8], out: &mut String<64>) {
    out.clear();
    match dp.dtype {
        DType::Bool => {
            let _ = out.push_str(if !data.is_empty() && data[0] != 0 { "EIN" } else { "AUS" });
        }
        DType::F32 => {
            let mut b = [0u8; 4];
            let n = data.len().min(4);
            b[..n].copy_from_slice(&data[..n]);
            let _ = write!(out, "{:.3}", f32::from_le_bytes(b));
        }
        DType::Str => {
            for &c in data.iter().take(out.capacity() - 1) {
                if c == 0 {
                    break;
                }
                let ch = if (0x20..0x7F).contains(&c) { c as char } else { '?' };
                let _ = out.push(ch);
            }
        }
        DType::Octet => {
            for &b in data.iter().take(12) {
                let _ = write!(out, "{:02X} ", b);
            }
        }
        _ => {
            let _ = write!(out, "{}", decode_int(data, dp.dtype.is_signed()));
        }
    }
}

fn set_border(obj: &impl AsLvHandle, color: u32) {
    unsafe {
        oxivgl_sys::lv_obj_set_style_border_color(obj.lv_handle(), oxivgl_sys::lv_color_hex(color), 0);
        oxivgl_sys::lv_obj_set_style_border_opa(obj.lv_handle(), 255, 0);
    }
}
