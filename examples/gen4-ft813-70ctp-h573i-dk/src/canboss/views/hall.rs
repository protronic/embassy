//! PoC-Hallenlicht als Navigator-Screen: [`HallView`] + Zurueck-Button.
//!
//! Der eigentliche Hallenscreen kommt unveraendert aus
//! [`crate::oxivgl::hall_view`]; dieser Wrapper legt lediglich einen
//! Zurueck-Button oben links darueber (ueberdeckt den Anfang der
//! Eyebrow-Zeile) und meldet dessen Klick als [`NavAction::Pop`].
//! Die CAN-Seite (Press/Hold/Repeat, minp-Highlights) laeuft in den bei
//! App-Start gespawnten [`crate::touch_can`]-Tasks weiter.

extern crate alloc;

use oxivgl::enums::EventCode;
use oxivgl::event::Event;
use oxivgl::style::Selector;
use oxivgl::view::{NavAction, View, register_event_on};
use oxivgl::widgets::{AsLvHandle, Button, Label, Obj, WidgetError};

use super::{BORDER, SURFACE, TEXT};
use crate::oxivgl::fonts::MONTSERRAT_14;
use crate::oxivgl::hall_view::HallView;

/// Hallenlicht-Screen mit Navigator-Rueckweg.
#[derive(Default)]
pub struct HallScreenView {
    hall: HallView,
    back: Option<Button<'static>>,
    back_label: Option<Label<'static>>,
}

impl View for HallScreenView {
    fn create(&mut self, container: &Obj<'static>) -> Result<(), WidgetError> {
        self.hall.create(container)?;

        // Zurueck-Button als Overlay oben links
        let back = Button::new(container)?;
        back.remove_style_all()
            .size(96, 32)
            .pos(12, 8)
            .bg_color(SURFACE)
            .bg_opa(255)
            .border_width(1)
            .radius(8, Selector::DEFAULT)
            .remove_scrollable()
            .pad(0);
        unsafe {
            oxivgl_sys::lv_obj_set_style_border_color(back.lv_handle(), oxivgl_sys::lv_color_hex(BORDER), 0);
            oxivgl_sys::lv_obj_set_style_border_opa(back.lv_handle(), 255, 0);
        }

        let label = Label::new(&back)?;
        label
            .text("< Menue")
            .width(88)
            .text_color(TEXT)
            .text_font(MONTSERRAT_14)
            .center();

        self.back_label = Some(label);
        self.back = Some(back);
        container.update_layout();
        Ok(())
    }

    fn register_events_on(&mut self, container: &Obj<'static>) {
        self.hall.register_events_on(container);
        if let Some(back) = &self.back {
            let handle = back.lv_handle();
            register_event_on(self, handle);
        }
    }

    fn on_event(&mut self, event: &Event) -> NavAction {
        if event.code() == EventCode::CLICKED {
            if let Some(back) = &self.back {
                if event.target_handle() == back.lv_handle() {
                    return NavAction::Pop(None);
                }
            }
        }
        // Hallen-Buttons erreichen HallView direkt (eigene Registrierung).
        NavAction::None
    }

    fn update(&mut self) -> Result<NavAction, WidgetError> {
        self.hall.update()
    }
}
