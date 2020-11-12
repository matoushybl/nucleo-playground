#![no_main]
#![no_std]

use core::fmt::Write;
use cortex_m::peripheral::DWT;
use nucleo_usb as _;
use rtic::cyccnt::U32Ext as _;
use smoltcp::iface::{EthernetInterface, Neighbor};
use smoltcp::socket::{SocketHandle, SocketSetItem};
use stm32_eth::smoltcp::iface::{EthernetInterfaceBuilder, NeighborCache};
use stm32_eth::smoltcp::socket::{SocketSet, TcpSocket, TcpSocketBuffer};
use stm32_eth::smoltcp::time::Instant;
use stm32_eth::smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address};
use stm32_eth::{Eth, EthPins, PhyAddress, RingEntry, RxDescriptor, TxDescriptor};
use stm32f7xx_hal::gpio::gpiob::PB;
use stm32f7xx_hal::gpio::{Output, PushPull};
use stm32f7xx_hal::otg_fs::{UsbBus, UsbBusType, USB};
use stm32f7xx_hal::pac;
use stm32f7xx_hal::prelude::*;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

const PERIOD: u32 = 16_000_000;
const MS_PERIOD: u32 = 108_000;

const SRC_MAC: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

type Ethernet = EthernetInterface<'static, 'static, 'static, &'static mut Eth<'static, 'static>>;
type Sockets = SocketSet<'static, 'static, 'static>;

#[rtic::app(device = stm32f7xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        green_led: PB<Output<PushPull>>,
        blue_led: PB<Output<PushPull>>,
        red_led: PB<Output<PushPull>>,
        serial: SerialPort<'static, UsbBusType>,
        usb_dev: UsbDevice<'static, UsbBusType>,
        #[init(0)]
        time: u64,
        #[init(false)]
        eth_pending: bool,
        server_handle: SocketHandle,
        sockets: Sockets,
        interface: Ethernet,
    }

    #[init(schedule = [blink, blink_blue, ms_tick])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

        static mut RX_RING: Option<[RingEntry<RxDescriptor>; 8]> = None;
        static mut TX_RING: Option<[RingEntry<TxDescriptor>; 2]> = None;

        static mut IP_ADDRS: Option<[IpCidr; 1]> = None;

        static mut NEIGHBOR_STORAGE: [Option<(IpAddress, Neighbor)>; 16] = [None; 16];

        static mut ETH: Option<Eth<'static, 'static>> = None;

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
            .hclk(216.mhz());
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

        *RX_RING = Some(Default::default());
        *TX_RING = Some(Default::default());

        let eth = Eth::new(
            device.ETHERNET_MAC,
            device.ETHERNET_DMA,
            RX_RING.as_mut().unwrap(),
            TX_RING.as_mut().unwrap(),
            PhyAddress::_0,
            clocks,
            eth_pins,
        )
        .unwrap();
        eth.enable_interrupt();

        *IP_ADDRS = Some([IpCidr::new(
            IpAddress::from(Ipv4Address::new(10, 15, 0, 124)),
            24,
        )]);
        *ETH = Some(eth);

        let interface = EthernetInterfaceBuilder::new(ETH.as_mut().unwrap())
            .ethernet_addr(EthernetAddress(SRC_MAC))
            .ip_addrs(IP_ADDRS.as_mut().unwrap().as_mut())
            .neighbor_cache(NeighborCache::new(NEIGHBOR_STORAGE.as_mut()))
            .finalize();

        let server_socket = TcpSocket::new(
            TcpSocketBuffer::new(SERVER_RX_BUFFER.as_mut()),
            TcpSocketBuffer::new(SERVER_TX_BUFFER.as_mut()),
        );

        let mut sockets = SocketSet::new(SOCKETS_STORAGE.as_mut());
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

        *USB_BUS = Some(UsbBus::new(usb, &mut EP_MEMORY[..]));

        let serial = usbd_serial::SerialPort::new(USB_BUS.as_ref().unwrap());

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
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
            red_led,
            serial,
            usb_dev,
            server_handle,
            sockets,
            interface,
        }
    }

    #[idle(resources = [eth_pending, time, interface, sockets, server_handle])]
    fn idle(mut cx: idle::Context) -> ! {
        let pending: &mut bool = cx.resources.eth_pending;
        let interface: &mut Ethernet = cx.resources.interface;
        let sockets: &mut Sockets = cx.resources.sockets;
        let handle: &mut SocketHandle = cx.resources.server_handle;

        loop {
            let time: u64 = cx.resources.time.lock(|time| *time);
            *pending = false;
            match interface.poll(sockets, Instant::from_millis(time as i64)) {
                Ok(true) => {
                    let mut socket = sockets.get::<TcpSocket>(*handle);
                    if !socket.is_open() {
                        if socket.listen(80).is_err() {
                            defmt::error!("TCP listen error");
                        }
                    }

                    if socket.can_send() {
                        if write!(socket, "Hello, ethernet!\n")
                            .map(|_| {
                                socket.close();
                            })
                            .is_err()
                        {
                            defmt::error!("TCP send error.");
                        }
                    }
                }
                Ok(false) => {}
                Err(e) =>
                // Ignore malformed packets
                {
                    // defmt::error!("Malformed packet.");
                }
            }
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
                for c in buf[..count].iter() {
                    serial.write(&[*c]).unwrap();
                    defmt::warn!("data {:u8}", c);
                }
                // count bytes were read to &buf[..count]
            }
            Err(UsbError::WouldBlock) => {
                // defmt::debug!("would block.");
            } // No data received
            Err(_) => {
                defmt::warn!("err.");
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

    #[task(resources = [time, red_led], schedule = [ms_tick])]
    fn ms_tick(cx: ms_tick::Context) {
        let led: &mut PB<Output<PushPull>> = cx.resources.red_led;
        *cx.resources.time += 1;

        if led.is_low().unwrap() {
            led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }

        cx.schedule
            .ms_tick(cx.scheduled + (MS_PERIOD).cycles())
            .unwrap();
    }

    extern "C" {
        fn EXTI0();
    }
};
