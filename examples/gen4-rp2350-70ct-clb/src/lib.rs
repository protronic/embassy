#![no_std]
#![allow(static_mut_refs)]

pub mod gen4_board;
pub mod usb_log;

#[cfg(feature = "oxivgl")]
pub mod oxivgl;
