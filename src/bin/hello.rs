#![no_main]
#![no_std]

use cortex_m::peripheral::DWT;
use embedded_hal::digital::ToggleableOutputPin;
use nucleo_usb as _;
use rtic::cyccnt::{Instant, U32Ext as _};
use stm32f7xx_hal::delay::Delay;
use stm32f7xx_hal::gpio::gpiob::PB;
use stm32f7xx_hal::gpio::{Output, PushPull};
use stm32f7xx_hal::otg_fs::{UsbBus, UsbBusType, USB};
use stm32f7xx_hal::pac;
use stm32f7xx_hal::prelude::*;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

const PERIOD: u32 = 16_000_000;

#[rtic::app(device = stm32f7xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        green_led: PB<Output<PushPull>>,
        blue_led: PB<Output<PushPull>>,
        // red_led: PB<Output<PushPull>>
        serial: SerialPort<'static, UsbBusType>,
        usb_dev: UsbDevice<'static, UsbBusType>,
    }

    #[init(schedule = [blink, blink_blue])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

        let mut core: rtic::Peripherals = cx.core;
        let device: stm32f7xx_hal::pac::Peripherals = cx.device;

        // enable CYCCNT
        core.DCB.enable_trace();
        DWT::unlock();
        core.DWT.enable_cycle_counter();

        let mut rcc = device
            .RCC
            .constrain()
            .cfgr
            .sysclk(216.mhz())
            .use_pll()
            .use_pll48clk();
        let clocks = rcc.freeze();

        let gpiob = device.GPIOB.split();
        let green_led = gpiob.pb0.into_push_pull_output().downgrade();
        let blue_led = gpiob.pb7.into_push_pull_output().downgrade();
        let red_led = gpiob.pb14.into_push_pull_output().downgrade();

        let now = cx.start;
        cx.schedule.blink(now + PERIOD.cycles());
        cx.schedule.blink_blue(now + PERIOD.cycles());

        let gpioa = device.GPIOA.split();
        let usb = USB::new(
            device.OTG_FS_GLOBAL,
            device.OTG_FS_DEVICE,
            device.OTG_FS_PWRCLK,
            (
                gpioa.pa11.into_alternate_af10(),
                gpioa.pa12.into_alternate_af10(),
            ),
            clocks,
        );

        unsafe {
            *USB_BUS = Some(UsbBus::new(usb, &mut EP_MEMORY[..]));
        }

        let mut serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

        let mut usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        init::LateResources {
            green_led,
            blue_led,
            serial,
            usb_dev,
        }
    }

    #[idle(resources = [green_led])]
    fn idle(cx: idle::Context) -> ! {
        // let led: &mut PB<Output<PushPull>> = cx.resources.led;
        loop {
            cortex_m::asm::nop();
            // delay.delay_ms(100u8);
            // // led.set_high();
            // delay.delay_ms(100u8);
            // led.set_low();
            // defmt::warn!("Kurdebele, jo se na to vyserim.");
        }
    }

    #[task(binds = OTG_FS, resources = [serial, usb_dev])]
    fn usb_handler(cx: usb_handler::Context) {
        let serial: &mut SerialPort<UsbBusType> = cx.resources.serial;
        let usb_dev: &mut UsbDevice<UsbBusType> = cx.resources.usb_dev;
        if !usb_dev.poll(&mut [serial]) {
            return;
        }
        let mut buf = [0u8; 64];

        match serial.read(&mut buf[..]) {
            Ok(count) => {
                defmt::debug!("data: start");
                for c in buf[..count].iter() {
                    serial.write(&[*c]).unwrap();
                    defmt::debug!("data {:u8}", c);
                }
                defmt::debug!("data: end");
                // count bytes were read to &buf[..count]
            }
            Err(UsbError::WouldBlock) => {
                // defmt::debug!("would block.");
            } // No data received
            Err(err) => {
                defmt::debug!("err.");
            } // An error occurred
        };
    }

    #[task(resources = [green_led], schedule = [blink])]
    fn blink(cx: blink::Context) {
        let led: &mut PB<Output<PushPull>> = cx.resources.green_led;
        if led.is_low().unwrap() {
            led.set_high();
        } else {
            led.set_low();
        }

        cx.schedule.blink(cx.scheduled + PERIOD.cycles()).unwrap();
    }

    #[task(resources = [blue_led], schedule = [blink_blue])]
    fn blink_blue(cx: blink_blue::Context) {
        let led: &mut PB<Output<PushPull>> = cx.resources.blue_led;
        if led.is_low().unwrap() {
            led.set_high();
        } else {
            led.set_low();
        }

        cx.schedule
            .blink_blue(cx.scheduled + (2 * PERIOD).cycles())
            .unwrap();
    }

    extern "C" {
        fn EXTI0();
    }
};
