#![no_main]
#![no_std]

use cortex_m::peripheral::DWT;
use nucleo_usb as _;
use rtic::cyccnt::{U32Ext as _};
use smoltcp::socket::{SocketHandle, SocketSetItem};
use stm32_eth::smoltcp::iface::{EthernetInterfaceBuilder, NeighborCache};
use stm32_eth::smoltcp::socket::{SocketSet, TcpSocket, TcpSocketBuffer};
use stm32_eth::smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address};
use stm32_eth::{Eth, EthPins, PhyAddress, RingEntry};
use stm32f7xx_hal::gpio::gpiob::PB;
use stm32f7xx_hal::gpio::{Output, PushPull};
use stm32f7xx_hal::otg_fs::{UsbBus, UsbBusType, USB};
use stm32f7xx_hal::pac;
use stm32f7xx_hal::prelude::*;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

const PERIOD: u32 = 16_000_000;
const MS_PERIOD: u32 = 216_000;

const SRC_MAC: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

#[rtic::app(device = stm32f7xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        green_led: PB<Output<PushPull>>,
        blue_led: PB<Output<PushPull>>,
        // red_led: PB<Output<PushPull>>
        serial: SerialPort<'static, UsbBusType>,
        usb_dev: UsbDevice<'static, UsbBusType>,
        #[init(0)]
        time: u64,
        #[init(false)]
        eth_pending: bool,
        server_handle: SocketHandle,
        sockets: SocketSet<'static, 'static, 'static>,
    }

    #[init(schedule = [blink, blink_blue, ms_tick])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

        static mut SOCKETS_STORAGE: [Option<SocketSetItem<'static, 'static>>; 2] = [None, None];
        static mut SERVER_RX_BUFFER: [u8; 2048] = [0; 2048];
        static mut SERVER_TX_BUFFER: [u8; 2048] = [0; 2048];

        let mut core: rtic::Peripherals = cx.core;
        let device: stm32f7xx_hal::pac::Peripherals = cx.device;

        // enable CYCCNT
        core.DCB.enable_trace();
        DWT::unlock();
        core.DWT.enable_cycle_counter();

        let rcc = device
            .RCC
            .constrain()
            .cfgr
            .sysclk(216.mhz())
            .use_pll()
            .use_pll48clk()
            .hclk(50.mhz());
        let clocks = rcc.freeze();

        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();
        let gpioc = device.GPIOC.split();
        let gpiog = device.GPIOG.split();

        let green_led = gpiob.pb0.into_push_pull_output().downgrade();
        let blue_led = gpiob.pb7.into_push_pull_output().downgrade();
        let red_led = gpiob.pb14.into_push_pull_output().downgrade();

        let eth_pins = EthPins {
            ref_clk: gpioa.pa1,
            md_io: gpioa.pa2,
            md_clk: gpioc.pc1,
            crs: gpioa.pa7,
            tx_en: gpiog.pg11,
            tx_d0: gpiog.pg13,
            tx_d1: gpiob.pb13,
            rx_d0: gpioc.pc4,
            rx_d1: gpioc.pc5,
        };

        let mut rx_ring: [RingEntry<_>; 8] = Default::default();
        let mut tx_ring: [RingEntry<_>; 2] = Default::default();
        let mut eth = Eth::new(
            device.ETHERNET_MAC,
            device.ETHERNET_DMA,
            &mut rx_ring[..],
            &mut tx_ring[..],
            PhyAddress::_0,
            clocks,
            eth_pins,
        )
        .unwrap();
        eth.enable_interrupt();

        let local_addr = Ipv4Address::new(10, 0, 0, 1);
        let ip_addr = IpCidr::new(IpAddress::from(local_addr), 24);
        let mut ip_addrs = [ip_addr];
        let mut neighbor_storage = [None; 16];
        let neighbor_cache = NeighborCache::new(&mut neighbor_storage[..]);
        let ethernet_addr = EthernetAddress(SRC_MAC);
        let mut iface = EthernetInterfaceBuilder::new(&mut eth)
            .ethernet_addr(ethernet_addr)
            .ip_addrs(&mut ip_addrs[..])
            .neighbor_cache(neighbor_cache)
            .finalize();

        let server_socket = TcpSocket::new(
            TcpSocketBuffer::new(unsafe { SERVER_RX_BUFFER.as_mut() }),
            TcpSocketBuffer::new(unsafe { SERVER_TX_BUFFER.as_mut() }),
        );

        let mut sockets = SocketSet::new(unsafe { SOCKETS_STORAGE.as_mut() });
        let server_handle = sockets.add(server_socket);

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

        let serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });

        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        let now = cx.start;
        cx.schedule.blink(now + PERIOD.cycles()).unwrap();
        cx.schedule.blink_blue(now + PERIOD.cycles()).unwrap();
        cx.schedule.ms_tick(now + MS_PERIOD.cycles()).unwrap();

        init::LateResources {
            green_led,
            blue_led,
            serial,
            usb_dev,
            server_handle,
            sockets,
        }
    }

    #[idle(resources = [eth_pending, time])]
    fn idle(mut cx: idle::Context) -> ! {
        let pending: &mut bool = cx.resources.eth_pending;
        loop {
            let time: u64 = cx.resources.time.lock(|time| *time);
            *pending = false;
            cortex_m::asm::nop();
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
            Err(_) => {
                defmt::debug!("err.");
            } // An error occurred
        };
    }

    #[task(binds = ETH, resources = [])]
    fn eth_handler(cx: eth_handler::Context) {
        // Clear interrupt flags
        let p = unsafe { pac::Peripherals::steal() };
        stm32_eth::eth_interrupt_handler(&p.ETHERNET_DMA);
    }

    #[task(resources = [green_led], schedule = [blink])]
    fn blink(cx: blink::Context) {
        let led: &mut PB<Output<PushPull>> = cx.resources.green_led;
        if led.is_low().unwrap() {
            led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }

        cx.schedule.blink(cx.scheduled + PERIOD.cycles()).unwrap();
    }

    #[task(resources = [blue_led], schedule = [blink_blue])]
    fn blink_blue(cx: blink_blue::Context) {
        let led: &mut PB<Output<PushPull>> = cx.resources.blue_led;
        if led.is_low().unwrap() {
            led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }

        cx.schedule
            .blink_blue(cx.scheduled + (2 * PERIOD).cycles())
            .unwrap();
    }

    #[task(resources = [time], schedule = [ms_tick])]
    fn ms_tick(cx: ms_tick::Context) {
        *cx.resources.time += 1;

        cx.schedule
            .ms_tick(cx.scheduled + (MS_PERIOD).cycles())
            .unwrap();
    }

    extern "C" {
        fn EXTI0();
    }
};
