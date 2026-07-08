//! Hauptmenue: Knotenliste aus eds/network.json + PoC-Hallenlicht.

extern crate alloc;

use alloc::format;
use alloc::vec::Vec;

use oxivgl::enums::{EventCode, ObjFlag};
use oxivgl::event::Event;
use oxivgl::style::Selector;
use oxivgl::view::{NavAction, View, register_event_on};
use oxivgl::widgets::{AsLvHandle, Button, Label, Obj, TextAlign, WidgetError};

use super::hall::HallScreenView;
use super::node::NodeView;
use super::{ACCENT, BORDER, CARD_BG, MUTED, SCREEN_BG, SURFACE, TEXT};
use crate::canboss::{NODES, NodeDesc};
use crate::oxivgl::fonts::MONTSERRAT_16;

const ROW_W: i32 = 760;
const ROW_H: i32 = 64;
const ROW_X: i32 = 20;
const ROW_Y0: i32 = 76;
const ROW_STEP: i32 = 74;

enum Target {
    Node(&'static NodeDesc),
    Hall,
}

/// Root-View: Liste aller Knoten + Eintrag "Hallenlicht (PoC)".
#[derive(Default)]
pub struct MenuView {
    buttons: Vec<Button<'static>>,
    labels: Vec<Label<'static>>,
    objects: Vec<Obj<'static>>,
    targets: Vec<Target>,
}

impl View for MenuView {
    fn create(&mut self, container: &Obj<'static>) -> Result<(), WidgetError> {
        self.buttons.clear();
        self.labels.clear();
        self.objects.clear();
        self.targets.clear();

        container.bg_color(SCREEN_BG).bg_opa(255).remove_scrollable().pad(0);

        // Header
        let header = Obj::new(container)?;
        header
            .size(800, 56)
            .pos(0, 0)
            .bg_color(SURFACE)
            .bg_opa(255)
            .border_width(0)
            .radius(0, Selector::DEFAULT)
            .remove_scrollable()
            .pad(0);
        let title = Label::new(&header)?;
        title
            .text("CANbossTouch — CANopen Netzwerk")
            .pos(20, 16)
            .width(760)
            .text_color(TEXT)
            .text_font(MONTSERRAT_16)
            .text_align(TextAlign::Center);
        self.labels.push(title);
        self.objects.push(header);

        // Ein Eintrag je Knoten + PoC-Hallenlicht
        let mut row = 0;
        for node in NODES {
            let text = format!("Node {}  {}  ({} Datenpunkte)", node.node_id, node.name, node.dps.len());
            self.add_row(container, row, &text, Target::Node(node))?;
            row += 1;
        }
        self.add_row(container, row, "Hallenlicht (PoC)", Target::Hall)?;

        container.update_layout();
        Ok(())
    }

    fn register_events_on(&mut self, container: &Obj<'static>) {
        register_event_on(self, container.lv_handle());
        for idx in 0..self.buttons.len() {
            register_event_on(self, self.buttons[idx].lv_handle());
        }
    }

    fn on_event(&mut self, event: &Event) -> NavAction {
        if event.code() != EventCode::CLICKED {
            return NavAction::None;
        }
        let handle = event.target_handle();
        let Some(idx) = self.buttons.iter().position(|b| b.lv_handle() == handle) else {
            return NavAction::None;
        };
        match self.targets.get(idx) {
            Some(Target::Node(node)) => NavAction::push(NodeView::new(node), None),
            Some(Target::Hall) => NavAction::push(HallScreenView::default(), None),
            None => NavAction::None,
        }
    }
}

impl MenuView {
    fn add_row(&mut self, parent: &Obj<'static>, row: i32, text: &str, target: Target) -> Result<(), WidgetError> {
        let button = Button::new(parent)?;
        button
            .remove_style_all()
            .size(ROW_W, ROW_H)
            .pos(ROW_X, ROW_Y0 + row * ROW_STEP)
            .bg_color(CARD_BG)
            .bg_opa(255)
            .border_width(1)
            .radius(12, Selector::DEFAULT)
            .remove_scrollable()
            .add_flag(ObjFlag::CLICKABLE)
            .pad(0);
        set_border_color(&button, BORDER);

        let label = Label::new(&button)?;
        label
            .text(text)
            .pos(20, 20)
            .width(ROW_W - 60)
            .text_color(TEXT)
            .text_font(MONTSERRAT_16)
            .text_align(TextAlign::Left);
        self.labels.push(label);

        let chevron = Label::new(&button)?;
        chevron
            .text(">")
            .pos(ROW_W - 32, 20)
            .width(20)
            .text_color(MUTED)
            .text_font(MONTSERRAT_16)
            .text_align(TextAlign::Right);
        self.labels.push(chevron);

        // Eyebrow-Akzent links
        let accent = Obj::new(&button)?;
        accent
            .size(4, ROW_H - 20)
            .pos(4, 10)
            .bg_color(ACCENT)
            .bg_opa(255)
            .border_width(0)
            .radius(2, Selector::DEFAULT)
            .remove_scrollable();
        self.objects.push(accent);

        self.buttons.push(button);
        self.targets.push(target);
        Ok(())
    }
}

fn set_border_color(obj: &impl AsLvHandle, color: u32) {
    unsafe {
        oxivgl_sys::lv_obj_set_style_border_color(obj.lv_handle(), oxivgl_sys::lv_color_hex(color), 0);
        oxivgl_sys::lv_obj_set_style_border_opa(obj.lv_handle(), 255, 0);
    }
}
