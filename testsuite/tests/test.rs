#![no_std]
#![no_main]

use cortex_m_rt::entry;
use nucleo_usb as _; // memory layout + panic handler

#[entry]
fn main() -> ! {
    assert!(false, "TODO: Write actual tests");

    nucleo_usb::exit();
}
