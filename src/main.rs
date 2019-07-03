#![no_std]
#![no_main]
#![feature(lang_items)]

extern crate panic_halt;
use stm32l0xx_hal as hal;
use sx1276;

use sx1276::LongFi;
use sx1276::{ClientEvent, QualityOfService, RfConfig, RfEvent};

use core::fmt::Write;
use hal::{exti, exti::TriggerEdge, gpio::*, pac, prelude::*, rcc::Config, serial, spi};
use stm32l0;

#[macro_use]
extern crate nb;
use nb::block;

#[macro_use]
extern crate enum_primitive;

mod bma400;
mod ubx;

use crate::ubx::Message;
use embedded_hal::digital::v2::OutputPin;

pub struct AntennaSwitches<RX, TX_RFO, TX_BOOST> {
    rx: RX,
    tx_rfo: TX_RFO,
    tx_boost: TX_BOOST,
}

impl<RX, TX_RFO, TX_BOOST> AntennaSwitches<RX, TX_RFO, TX_BOOST>
where
    RX: embedded_hal::digital::v2::OutputPin,
    TX_RFO: embedded_hal::digital::v2::OutputPin,
    TX_BOOST: embedded_hal::digital::v2::OutputPin,
{
    pub fn set_tx(&mut self) {
        self.rx.set_low();
        self.tx_rfo.set_low();
        self.tx_boost.set_high();
    }

    pub fn set_rx(&mut self) {
        self.rx.set_high();
        self.tx_rfo.set_low();
        self.tx_boost.set_low();
    }
}

#[rtfm::app(device = stm32l0xx_hal::pac)]
const APP: () = {
    static mut EXTI: pac::EXTI = ();
    static mut SX1276_DIO0: gpiob::PB4<Input<PullUp>> = ();
    static mut BMA400_INT1: gpioa::PA0<Input<Floating>> = ();
    static mut DEBUG_UART: serial::Tx<stm32l0::stm32l0x2::USART2> = ();
    static mut BUFFER: [u8; 512] = [0; 512];
    static mut GPS_EN: gpiob::PB2<Output<PushPull>> = ();
    static mut GPS_TX: serial::Tx<stm32l0::stm32l0x2::USART1> = ();
    static mut GPS_RX: serial::Rx<stm32l0::stm32l0x2::USART1> = ();
    static mut I2C: stm32l0xx_hal::i2c::I2c<
        stm32l0::stm32l0x2::I2C1,
        stm32l0xx_hal::gpio::gpiob::PB9<
            stm32l0xx_hal::gpio::Output<stm32l0xx_hal::gpio::OpenDrain>,
        >,
        stm32l0xx_hal::gpio::gpiob::PB8<
            stm32l0xx_hal::gpio::Output<stm32l0xx_hal::gpio::OpenDrain>,
        >,
    > = ();
    static mut ACCEL: bma400::Bma400 = ();
    static mut UBX: ubx::Ubx = ();
    static mut ANT_SW: AntennaSwitches<
        stm32l0xx_hal::gpio::gpioa::PA1<stm32l0xx_hal::gpio::Output<stm32l0xx_hal::gpio::PushPull>>,
        stm32l0xx_hal::gpio::gpioc::PC2<stm32l0xx_hal::gpio::Output<stm32l0xx_hal::gpio::PushPull>>,
        stm32l0xx_hal::gpio::gpioc::PC1<stm32l0xx_hal::gpio::Output<stm32l0xx_hal::gpio::PushPull>>,
    > = ();
    static mut WD: stm32l0xx_hal::watchdog::IndependedWatchdog = ();

    #[init(resources = [BUFFER])]
    fn init() -> init::LateResources {
        // Configure the clock.
        let mut rcc = device.RCC.freeze(Config::hsi16());

        // Acquire the GPIOB peripheral. This also enables the clock for GPIOB in
        // the RCC register.
        let gpioa = device.GPIOA.split(&mut rcc);
        let gpiob = device.GPIOB.split(&mut rcc);
        let gpioc = device.GPIOC.split(&mut rcc);

        let tx_pin = gpioa.pa2;
        let rx_pin = gpioa.pa3;

        // Configure the serial peripheral.
        let serial = device
            .USART2
            .usart((tx_pin, rx_pin), serial::Config::default(), &mut rcc)
            .unwrap();

        let (mut tx, mut rx) = serial.split();

        write!(tx, "Tracker Demo\r\n").unwrap();

        // Configure PB2 as input.
        let mut gps_ldo_en = gpiob.pb2.into_push_pull_output();

        gps_ldo_en.set_high();

        let mut gps_enable = gpioa.pa5.into_push_pull_output();

        gps_enable.set_low();

        let exti = device.EXTI;

        // Configure PB4 as input for rising interrupt
        let sx1276_dio0 = gpiob.pb4.into_pull_up_input();
        exti.listen(
            &mut rcc,
            &mut device.SYSCFG_COMP,
            sx1276_dio0.port,
            sx1276_dio0.i,
            TriggerEdge::Rising,
        );

        // Configure PB5 as input for rising interrupt
        let accel_int1 = gpioa.pa0.into_floating_input();
        exti.listen(
            &mut rcc,
            &mut device.SYSCFG_COMP,
            accel_int1.port,
            accel_int1.i,
            TriggerEdge::All,
        );

        let sck = gpiob.pb3;
        let miso = gpioa.pa6;
        let mosi = gpioa.pa7;
        let nss = gpioa.pa15.into_push_pull_output();

        // Initialise the SPI peripheral.
        let spi = device
            .SPI1
            .spi((sck, miso, mosi), spi::MODE_0, 2_000_000.hz(), &mut rcc);

        // Get the delay provider.
        let mut delay = core.SYST.delay(rcc.clocks);
        let mut reset = gpioc.pc0.into_push_pull_output();
        let mut en_tcxo = gpiob.pb14.into_push_pull_output();

        let mut ant_sw = AntennaSwitches {
            rx: gpioa.pa1.into_push_pull_output(),
            tx_rfo: gpioc.pc2.into_push_pull_output(),
            tx_boost: gpioc.pc1.into_push_pull_output(),
        };

        ant_sw.rx.set_low();
        ant_sw.tx_rfo.set_low();
        ant_sw.tx_boost.set_low();

        en_tcxo.set_high();

        reset.set_low();

        delay.delay_ms(1_u16);

        reset.set_high();
        delay.delay_ms(6_u16);
        LongFi::enable_tcxo();

        reset.set_low();

        delay.delay_ms(1_u16);

        reset.set_high();


        let raw_device_id = unsafe {::core::ptr::read(0x1FF8_0064 as *const u32)};
        let device_id: u16 = (raw_device_id as u16) | ((raw_device_id & 0xFF0000) >> 8) as u16;
        write!(tx, "Device ID = {:x}\r\n", device_id).unwrap();

        LongFi::initialize(RfConfig {
            always_on: true,
            qos: QualityOfService::QOS_0,
            network_poll: 0,
            device_id,
            oui: 0x365aabe7
        });
        LongFi::set_buffer(resources.BUFFER);

        let gps_tx_pin = gpioa.pa9;
        let gps_rx_pin = gpioa.pa10;

        // Configure the serial peripheral.
        let mut serial = device
            .USART1
            .usart(
                (gps_tx_pin, gps_rx_pin),
                serial::Config::default(),
                &mut rcc,
            )
            .unwrap();

        serial.listen(serial::Event::Rxne);
        gps_enable.set_high();
        let (mut gps_tx, mut gps_rx) = serial.split();

        delay.delay_ms(1000_u16);

        let sda = gpiob.pb9.into_open_drain_output();
        let scl = gpiob.pb8.into_open_drain_output();

        let mut i2c = device.I2C1.i2c(sda, scl, 100.khz(), &mut rcc);

        let accel = bma400::Bma400::new(false);

        accel.wake(&mut i2c);
        accel.configure_inactivity_interupt(&mut i2c, 0x003F);

        let ubx = ubx::Ubx::new();
        ubx.enable_ubx_protocol(&mut gps_tx);
        delay.delay_ms(50_u16);
        ubx.enable_ubx_protocol(&mut gps_tx);
        delay.delay_ms(50_u16);
        ubx.enable_nav_pvt(&mut gps_tx);
        delay.delay_ms(50_u16);
        ubx.enable_ext_ant(&mut gps_tx);

        // Configure the independent watchdog.
        let mut watchdog = device.IWDG.watchdog();

        // Start a watchdog with a 1000ms period.
        watchdog.start(1000.ms());
        
        // Return the initialised resources.
        init::LateResources {
            EXTI: exti,
            SX1276_DIO0: sx1276_dio0,
            BMA400_INT1: accel_int1,
            DEBUG_UART: tx,
            GPS_TX: gps_tx,
            GPS_RX: gps_rx,
            I2C: i2c,
            ACCEL: accel,
            GPS_EN: gps_ldo_en,
            UBX: ubx,
            ANT_SW: ant_sw,
            WD: watchdog
        }
    }

    #[task(capacity = 4, priority = 2, resources = [DEBUG_UART, I2C, ACCEL, GPS_EN])]
    fn accel_activity() {
        let int_status = resources.ACCEL.get_int_status(resources.I2C).unwrap();

        if int_status == 0 {
            write!(resources.DEBUG_UART, "Moving\r\n");
        //resources.GPS_EN.set_high();
        } else {
            write!(resources.DEBUG_UART, "Idle\r\n");
            //resources.GPS_EN.set_low();
        }
    }

    #[task(capacity = 4, priority = 2, resources = [DEBUG_UART, BUFFER, ANT_SW, WD])]
    fn radio_event(event: RfEvent) {
        resources.WD.feed();
        let client_event = LongFi::handle_event(event);

        match client_event {
            ClientEvent::ClientEvent_TxDone => {
                write!(resources.DEBUG_UART, "=>Transmit Done!\r\n\r\n").unwrap();
                // resources.ANT_SW.set_rx();
                // LongFi::set_rx();
            }
            ClientEvent::ClientEvent_Rx => {
                // get receive buffer
                let rx_packet = LongFi::get_rx();
                write!(resources.DEBUG_UART, "Received packet\r\n").unwrap();
                write!(resources.DEBUG_UART, "  Length =  {}\r\n", rx_packet.len).unwrap();
                write!(resources.DEBUG_UART, "  Rssi   = {}\r\n", rx_packet.rssi).unwrap();
                write!(resources.DEBUG_UART, "  Snr    =  {}\r\n", rx_packet.snr).unwrap();
                unsafe {
                    for i in 0..rx_packet.len {
                        write!(
                            resources.DEBUG_UART,
                            "{:X} ",
                            *rx_packet.buf.offset(i as isize)
                        )
                        .unwrap();
                    }
                    write!(resources.DEBUG_UART, "\r\n").unwrap();
                }
                // give buffer back to library
                LongFi::set_buffer(resources.BUFFER);
            }
            _ => {
                write!(resources.DEBUG_UART, "Unhandled Client Event\r\n").unwrap();
            }
        }
    }

    #[task(capacity = 16, priority = 2, resources = [DEBUG_UART, UBX, ANT_SW, WD])]
    fn ubx_parse(byte: u8) {
        static mut COUNT: u16 = 0;
        resources.WD.feed();
        
        if let Some(msg) = resources.UBX.push(byte) {
            match msg {
                Message::NP(navpvt) => {
                    write!(resources.DEBUG_UART, "{}\t{}", *COUNT, navpvt);

                    let packet: [u8; 14] = [
                        (*COUNT >> 8) as u8,
                        *COUNT as u8,
                        (navpvt.lat >> 24) as u8,
                        (navpvt.lat >> 16) as u8,
                        (navpvt.lat >> 8) as u8,
                        navpvt.lat as u8,
                        (navpvt.lon >> 24) as u8,
                        (navpvt.lon >> 16) as u8,
                        (navpvt.lon >> 8) as u8,
                        navpvt.lon as u8,
                        (navpvt.alt >> 8) as u8,
                        navpvt.alt as u8,
                        (navpvt.speed >> 8) as u8,
                        navpvt.speed as u8,
                    ];
                    resources.ANT_SW.set_tx();
                    LongFi::send(&packet, packet.len());
                    *COUNT = COUNT.wrapping_add(1);
                }
            }
        }
    }

    #[interrupt(priority = 3, resources = [GPS_RX], spawn = [ubx_parse])]
    fn USART1() {
        if let Ok(byte) = resources.GPS_RX.read() {
            spawn.ubx_parse(byte);
        }
    }

    #[interrupt(priority = 3, resources = [SX1276_DIO0, EXTI], spawn = [radio_event])]
    fn EXTI4_15() {
        let reg = resources.EXTI.get_pending_irq();

        if exti::line_is_triggered(reg, resources.SX1276_DIO0.i) {
            resources.EXTI.clear_irq(resources.SX1276_DIO0.i);
            spawn.radio_event(RfEvent::DIO0);
        }
    }

    #[interrupt(priority = 3, resources = [BMA400_INT1, EXTI], spawn = [accel_activity])]
    fn EXTI0_1() {
        let reg = resources.EXTI.get_pending_irq();

        if exti::line_is_triggered(reg, resources.BMA400_INT1.i) {
            resources.EXTI.clear_irq(resources.BMA400_INT1.i);
            spawn.accel_activity();
        }
    }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn USART4_USART5();
    }
};

use stm32l0xx_hal::gpio::gpioa::*;
use stm32l0xx_hal::gpio::{Floating, Input, PushPull};

use core::ffi;
use embedded_hal::spi::FullDuplex;

use stm32l0xx_hal::pac::SPI1;

#[repr(C, align(4))]
pub struct SpiInstance {
    Instance: *mut ffi::c_void,
}

#[repr(C, align(4))]
pub struct Spi_s {
    Spi: SpiInstance,
    Nss: Gpio_t,
}

pub type Spi_t = Spi_s;

#[no_mangle]
pub extern "C" fn SpiInOut(s: &mut Spi_t, outData: u16) -> u16 {
    let spi: &mut hal::spi::Spi<
        SPI1,
        (
            PA3<Input<Floating>>,
            PA6<Input<Floating>>,
            PA7<Input<Floating>>,
        ),
    > = unsafe {
        &mut *(s.Spi.Instance
            as *mut hal::spi::Spi<
                SPI1,
                (
                    PA3<Input<Floating>>,
                    PA6<Input<Floating>>,
                    PA7<Input<Floating>>,
                ),
            >)
    };

    spi.send(outData as u8).unwrap();
    let inData = block!(spi.read()).unwrap();

    inData as u16
}

type Gpio_t = *mut ffi::c_void;

#[repr(C)]
pub enum PinNames {
    MCU_PINS,
    IOE_PINS,
    RADIO_RESET,
}

#[repr(C)]
pub enum PinModes {
    PIN_INPUT = 0,
    PIN_OUTPUT,
    PIN_ALTERNATE_FCT,
    PIN_ANALOGIC,
}

#[repr(C)]
pub enum PinTypes {
    PIN_NO_PULL = 0,
    PIN_PULL_UP,
    PIN_PULL_DOWN,
}

#[repr(C)]
pub enum PinConfigs {
    PIN_PUSH_PULL = 0,
    PIN_OPEN_DRAIN,
}

#[no_mangle]
pub extern "C" fn GpioInit(
    obj: Gpio_t,
    pin: PinNames,
    mode: PinModes,
    config: PinConfigs,
    pin_type: PinTypes,
    val: u32,
) {
    let mut gpio: &mut stm32l0xx_hal::gpio::gpioc::PC0<Output<PushPull>> =
        unsafe { &mut *(obj as *mut stm32l0xx_hal::gpio::gpioc::PC0<Output<PushPull>>) };

    if (val == 0) {
        gpio.set_low();
    } else {
        gpio.set_high();
    }
}

#[no_mangle]
pub extern "C" fn GpioWrite(obj: Gpio_t, val: u8) {
    let gpio: &mut stm32l0xx_hal::gpio::gpioa::PA15<Output<PushPull>> =
        unsafe { &mut *(obj as *mut stm32l0xx_hal::gpio::gpioa::PA15<Output<PushPull>>) };

    if (val == 0) {
        gpio.set_low().unwrap();
    } else {
        gpio.set_high().unwrap();
    }
}

#[repr(C, align(4))]
pub struct TimerEvent_s {
    IsRunning: bool,
}

type TimerEvent_t = TimerEvent_s;

#[no_mangle]
pub extern "C" fn TimerInit(obj: &TimerEvent_t, cb: Option<extern "C" fn()>) {}

#[no_mangle]
pub extern "C" fn TimerStart(obj: &TimerEvent_t) {}

#[no_mangle]
pub extern "C" fn TimerStop(obj: &TimerEvent_t) {}

#[no_mangle]
pub extern "C" fn TimerReset(obj: &TimerEvent_t) {}

#[no_mangle]
pub extern "C" fn TimerSetValue(obj: &TimerEvent_t, value: u32) {}

#[no_mangle]
pub extern "C" fn TimerGetCurrentTime() {}

#[no_mangle]
pub extern "C" fn TimerGetElapsedTime(saved_time: &TimerEvent_t) {}

#[no_mangle]
pub extern "C" fn TimerGetFutureTime(event_in_future: &TimerEvent_t) {}

#[no_mangle]
pub extern "C" fn TimerLowPowerHandler() {}

type irq_ptr = extern "C" fn();

#[no_mangle]
pub extern "C" fn SX1276GetPaSelect(channel: u32) -> u8 {
    0
}

use cortex_m::asm;

#[no_mangle]
pub extern "C" fn DelayMs(ms: u32) {
    //asm::delay(ms);
}

#[no_mangle]
pub extern "C" fn SX1276SetAntSwLowPower(status: bool) {}

#[no_mangle]
pub extern "C" fn SX1276SetAntSw(rxTx: u8) {}

#[no_mangle]
pub extern "C" fn assert_param(expr: bool) {}
