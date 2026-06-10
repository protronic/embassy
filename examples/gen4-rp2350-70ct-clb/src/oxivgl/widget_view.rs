//! OxivGL widget showcase view for the gen4-RP2350-70CT-CLB (LVGL v9.5).
//!
//! 800×480 layout with three cards: buttons with a click counter, a slider
//! mirrored by a bar and a value label, and toggle widgets with a spinner.

extern crate alloc;

use alloc::format;
use alloc::vec::Vec;

use defmt::info;
use oxivgl::enums::{EventCode, ObjFlag, ObjState};
use oxivgl::event::Event;
use oxivgl::fonts::{MONTSERRAT_14, MONTSERRAT_16};
use oxivgl::style::Selector;
use oxivgl::view::{NavAction, View, register_event_on};
use oxivgl::widgets::{
    AsLvHandle, Bar, Button, Checkbox, Label, Obj, Screen, Slider, Spinner, Switch, TextAlign, WidgetError,
};

const SCREEN_BG: u32 = 0x10222E;
const CARD_BG: u32 = 0x18313F;
const CARD_BORDER: u32 = 0x2A4A5C;
const TEXT: u32 = 0xEAF2F6;
const MUTED: u32 = 0x9AB2BE;
const ACCENT: u32 = 0x37B6E9;
const BUTTON_BG: u32 = 0x205066;
const BUTTON_BG_PRESSED: u32 = 0x2D6F8C;

const SLIDER_DEFAULT: i32 = 40;

/// Multi-widget OxivGL demo: buttons, slider + bar, switch, checkbox, spinner.
#[derive(Default)]
pub struct WidgetView {
    labels: Vec<Label<'static>>,
    objects: Vec<Obj<'static>>,
    buttons: Vec<Button<'static>>,
    counter_label: Option<Label<'static>>,
    slider: Option<Slider<'static>>,
    bar: Option<Bar<'static>>,
    value_label: Option<Label<'static>>,
    switch: Option<Switch<'static>>,
    checkbox: Option<Checkbox<'static>>,
    spinner: Option<Spinner<'static>>,
    clicks: u32,
}

impl View for WidgetView {
    fn create(&mut self, container: &Obj<'static>) -> Result<(), WidgetError> {
        container.bg_color(SCREEN_BG).bg_opa(255).remove_scrollable().pad(0);

        // --- Header ---
        self.labels.push(make_label(
            container,
            "gen4-RP2350-70CT-CLB  /  OxivGL widget demo",
            24,
            18,
            752,
            TEXT,
            MONTSERRAT_16,
            TextAlign::Left,
        )?);
        self.labels.push(make_label(
            container,
            "LVGL v9.5 on Embassy — PIO DPI scan-out from PSRAM",
            24,
            44,
            752,
            MUTED,
            MONTSERRAT_14,
            TextAlign::Left,
        )?);

        // --- Cards ---
        let card_y = 80;
        let card_h = 370;
        let buttons_card = self.make_card(container, "BUTTONS", 24, card_y, 240, card_h)?;
        let slider_card = self.make_card(container, "SLIDER + BAR", 280, card_y, 280, card_h)?;
        let toggles_card = self.make_card(container, "TOGGLES", 576, card_y, 200, card_h)?;

        // Buttons card: a tappable button and a click counter.
        let button = make_button(&buttons_card, "Tap me", 20, 60, 200, 64)?;
        button.on(EventCode::CLICKED, on_demo_button_click);
        self.buttons.push(button);
        self.buttons
            .push(make_button(&buttons_card, "Me too", 20, 140, 200, 64)?);

        let counter = make_label(
            &buttons_card,
            "0 clicks",
            20,
            230,
            200,
            ACCENT,
            MONTSERRAT_16,
            TextAlign::Left,
        )?;
        self.counter_label = Some(counter);

        // Slider card: slider drives the bar and the value label.
        let slider = Slider::new(&slider_card)?;
        slider.set_range(0, 100).set_value(SLIDER_DEFAULT);
        slider.size(220, 16).pos(20, 70);
        self.slider = Some(slider);

        let bar = Bar::new(&slider_card)?;
        bar.set_range_raw(0, 100).set_value_raw(SLIDER_DEFAULT, false);
        bar.size(220, 16).pos(20, 130);
        self.bar = Some(bar);

        let value = make_label(
            &slider_card,
            &format!("{}%", SLIDER_DEFAULT),
            20,
            180,
            220,
            ACCENT,
            MONTSERRAT_16,
            TextAlign::Left,
        )?;
        self.value_label = Some(value);

        // Toggles card: switch, checkbox, spinner.
        let switch = Switch::new(&toggles_card)?;
        switch.size(64, 32).pos(20, 60);
        switch.add_state(ObjState::CHECKED);
        self.switch = Some(switch);

        let checkbox = Checkbox::new(&toggles_card)?;
        checkbox.text("Backlight");
        checkbox.pos(20, 120);
        checkbox.add_state(ObjState::CHECKED);
        self.checkbox = Some(checkbox);

        let spinner = Spinner::new(&toggles_card)?;
        spinner.set_anim_params(1200, 200);
        spinner.size(72, 72).pos(20, 180);
        self.spinner = Some(spinner);

        self.objects.push(buttons_card);
        self.objects.push(slider_card);
        self.objects.push(toggles_card);

        container.update_layout();
        Ok(())
    }

    fn register_events(&mut self) {
        // Screen-level handler catches bubbled events from children.
        if let Some(screen) = Screen::active() {
            register_event_on(self, screen.handle());
        }
        for idx in 0..self.buttons.len() {
            register_event_on(self, self.buttons[idx].handle());
        }
        if let Some(slider) = &self.slider {
            register_event_on(self, slider.handle());
        }
        if let Some(switch) = &self.switch {
            register_event_on(self, switch.handle());
        }
        if let Some(checkbox) = &self.checkbox {
            register_event_on(self, checkbox.handle());
        }
    }

    fn on_event(&mut self, event: &Event) -> NavAction {
        let code = event.code();

        if let Some(slider) = &self.slider {
            if event.matches(slider, EventCode::VALUE_CHANGED) {
                let v = slider.get_value();
                if let Some(bar) = &self.bar {
                    bar.set_value_raw(v, false);
                }
                if let Some(label) = &self.value_label {
                    label.text(&format!("{}%", v));
                }
                info!("oxivgl slider value {}", v);
                return NavAction::None;
            }
        }

        if let Some(switch) = &self.switch {
            if event.matches(switch, EventCode::VALUE_CHANGED) {
                info!("oxivgl switch toggled on={}", switch.has_state(ObjState::CHECKED));
                return NavAction::None;
            }
        }

        if let Some(checkbox) = &self.checkbox {
            if event.matches(checkbox, EventCode::VALUE_CHANGED) {
                info!("oxivgl checkbox toggled on={}", checkbox.has_state(ObjState::CHECKED));
                return NavAction::None;
            }
        }

        match code {
            EventCode::PRESSED => info!("oxivgl widget pressed ({:?})", code.0),
            EventCode::CLICKED | EventCode::SHORT_CLICKED | EventCode::SINGLE_CLICKED => {
                let mut is_button = false;
                for button in &self.buttons {
                    if event.matches(button, code) {
                        is_button = true;
                        break;
                    }
                }
                if is_button {
                    self.clicks += 1;
                    if let Some(label) = &self.counter_label {
                        label.text(&format!("{} clicks", self.clicks));
                    }
                    info!("oxivgl button click #{}", self.clicks);
                }
            }
            _ => {}
        }
        NavAction::None
    }

    fn update(&mut self) -> Result<NavAction, WidgetError> {
        Ok(NavAction::None)
    }
}

impl WidgetView {
    fn make_card(
        &mut self,
        parent: &impl AsLvHandle,
        title: &str,
        x: i32,
        y: i32,
        w: i32,
        h: i32,
    ) -> Result<Obj<'static>, WidgetError> {
        let card = Obj::new(parent)?;
        card.size(w, h)
            .pos(x, y)
            .bg_color(CARD_BG)
            .bg_opa(255)
            .border_width(1)
            .radius(12, Selector::DEFAULT)
            .remove_scrollable()
            .pad(0);
        set_border_color(&card, CARD_BORDER, 255);

        self.labels.push(make_label(
            &card,
            title,
            20,
            18,
            w - 40,
            MUTED,
            MONTSERRAT_14,
            TextAlign::Left,
        )?);
        Ok(card)
    }

    /// Log widget bounds once (RTT) to verify touch hit targets.
    pub fn log_layout(&self) {
        if let Some(btn) = self.buttons.first() {
            let area = btn.get_coords();
            info!(
                "oxivgl first button area x1={} y1={} x2={} y2={}",
                area.x1, area.y1, area.x2, area.y2
            );
        }
        if let Some(slider) = &self.slider {
            let area = slider.get_coords();
            info!(
                "oxivgl slider area x1={} y1={} x2={} y2={}",
                area.x1, area.y1, area.x2, area.y2
            );
        }
    }
}

fn on_demo_button_click(_event: &Event) {
    info!("oxivgl direct button CLICKED");
}

#[allow(clippy::too_many_arguments)]
fn make_label(
    parent: &impl AsLvHandle,
    text: &str,
    x: i32,
    y: i32,
    w: i32,
    color: u32,
    font: oxivgl::fonts::Font,
    align: TextAlign,
) -> Result<Label<'static>, WidgetError> {
    let label = Label::new(parent)?;
    label
        .text(text)
        .pos(x, y)
        .width(w)
        .text_color(color)
        .text_font(font)
        .text_align(align)
        .remove_scrollable();
    Ok(label)
}

fn make_button(
    parent: &impl AsLvHandle,
    text: &str,
    x: i32,
    y: i32,
    w: i32,
    h: i32,
) -> Result<Button<'static>, WidgetError> {
    let button = Button::new(parent)?;
    button
        .size(w, h)
        .pos(x, y)
        .bg_color(BUTTON_BG)
        .bg_opa(255)
        .border_width(0)
        .radius(10, Selector::DEFAULT)
        .style_bg_color(
            unsafe { oxivgl_sys::lv_color_hex(BUTTON_BG_PRESSED) },
            ObjState::PRESSED,
        )
        .remove_scrollable()
        .add_flag(ObjFlag::CLICKABLE)
        .bubble_events()
        .pad(0);

    let label = Label::new(&button)?;
    label
        .text(text)
        .text_color(TEXT)
        .text_font(MONTSERRAT_16)
        .text_align(TextAlign::Center)
        .center()
        .remove_scrollable();
    // Keep the label alive for the static UI lifetime (parent owns the C object).
    core::mem::forget(label);

    Ok(button)
}

fn set_border_color(obj: &impl AsLvHandle, color: u32, opa: u8) {
    unsafe {
        oxivgl_sys::lv_obj_set_style_border_color(obj.lv_handle(), oxivgl_sys::lv_color_hex(color), 0);
        oxivgl_sys::lv_obj_set_style_border_opa(obj.lv_handle(), opa as oxivgl_sys::lv_opa_t, 0);
    }
}
