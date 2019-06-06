#![no_std]
#![no_main]
#![feature(lang_items)]

extern crate panic_halt;
use stm32l0xx_hal as hal;
use sx1276;

use sx1276::{RfConfig, QualityOfService, ClientEvent, RfEvent};
use sx1276::LongFi;

use core::fmt::Write;
use hal::{exti::TriggerEdge, gpio::*, pac, prelude::*, rcc::Config, spi, serial};
use stm32l0;


use embedded_hal::digital::v2::OutputPin;

#[rtfm::app(device = stm32l0xx_hal::pac)]
const APP: () = {
    static mut LED: gpiob::PB5<Output<PushPull>> = ();
    static mut INT: pac::EXTI = ();
    static mut BUTTON: gpiob::PB2<Input<PullUp>> = ();
    static mut SX1276_DIO0: gpiob::PB4<Input<PullUp>> = ();
    static mut DEBUG_UART: serial::Tx<stm32l0::stm32l0x2::USART2> = ();
    static mut BUFFER: [u8; 512] = [0; 512];
    static mut COUNT: u8 = 0;
    static mut GPS_TX: serial::Tx<stm32l0::stm32l0x2::USART1> = ();
    static mut GPS_RX: serial::Rx<stm32l0::stm32l0x2::USART1> = ();

    #[init(resources = [BUFFER])]
    fn init() -> init::LateResources {
        // Configure the clock.
        let mut rcc = device.RCC.freeze(Config::hsi16());

        // Acquire the GPIOB peripheral. This also enables the clock for GPIOB in
        // the RCC register.
        let gpioa = device.GPIOA.split(&mut rcc);
        let gpiob = device.GPIOB.split(&mut rcc);
        let gpioc = device.GPIOC.split(&mut rcc);

        let mut gps_enable = gpioa.pa5.into_push_pull_output();

        gps_enable.set_low();

        let tx_pin = gpioa.pa2;
        let rx_pin = gpioa.pa3;

        // Configure the serial peripheral.
        let serial = device
            .USART2
            .usart((tx_pin, rx_pin), serial::Config::default(), &mut rcc)
            .unwrap();

        let (mut tx, mut rx) = serial.split();

        write!(tx, "SX1276 test\r\n").unwrap();

        // Configure PB5 as output.
        let led = gpiob.pb5.into_push_pull_output();

        let exti = device.EXTI;

        // Configure PB2 as input.
        let button = gpiob.pb2.into_pull_up_input();
        // Configure the external interrupt on the falling edge for the pin 2.
        exti.listen(
            &mut rcc,
            &mut device.SYSCFG_COMP,
            button.port,
            button.i,
            TriggerEdge::Falling,
        );

        // // Configure PB4 as input.
        let sx1276_dio0 = gpiob.pb4.into_pull_up_input();
        // Configure the external interrupt on the falling edge for the pin 2.
        exti.listen(
            &mut rcc,
            &mut device.SYSCFG_COMP,
            sx1276_dio0.port,
            sx1276_dio0.i,
            TriggerEdge::Rising,
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

        reset.set_low();

        delay.delay_ms(100_u16);

        reset.set_high();

        LongFi::initialize(
            RfConfig {
                always_on: true,
                qos: QualityOfService::QOS_0,
                network_poll: 0,
            }
        );
        LongFi::set_buffer(resources.BUFFER);
        LongFi::set_rx();

        let gps_tx_pin = gpioa.pa9;
        let gps_rx_pin = gpioa.pa10;

        // Configure the serial peripheral.
        let mut serial = device
            .USART1
            .usart((gps_tx_pin, gps_rx_pin), serial::Config::default(), &mut rcc)
            .unwrap();

        serial.listen(serial::Event::Rxne);

        gps_enable.set_high();

        let (mut gps_tx, mut gps_rx) = serial.split();
        
        write!(gps_tx, "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n").unwrap();
        write!(gps_tx, "$PMTK220,1000*1F\r\n").unwrap();

        // Return the initialised resources.
        init::LateResources {
            LED: led,
            INT: exti,
            BUTTON: button,
            SX1276_DIO0: sx1276_dio0,
            DEBUG_UART: tx,
            GPS_TX: gps_tx,
            GPS_RX: gps_rx,
        }
    }

    #[task(capacity = 4, priority = 1, resources = [DEBUG_UART, BUFFER])]
    fn radio_event(event: RfEvent){
        let client_event = LongFi::handle_event(event);
        
        match client_event {
            ClientEvent::ClientEvent_TxDone => {
                write!(resources.DEBUG_UART, "Transmit Done!\r\n").unwrap();
                LongFi::set_rx();
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
                        
                            write!(resources.DEBUG_UART, "{:X} ", *rx_packet.buf.offset(i as isize)).unwrap();
                        
                    }
                    write!(resources.DEBUG_UART, "\r\n").unwrap();
                }
                // give buffer back to library
                LongFi::set_buffer(resources.BUFFER);
            }
            _ => {
                write!(resources.DEBUG_UART, "Unhandled Client Event\r\n").unwrap();
            },
        }
    }

    #[task(capacity = 4, priority = 1, resources = [DEBUG_UART, COUNT])]
    fn send_ping(){
        write!(resources.DEBUG_UART, "Sending Ping\r\n").unwrap();
        let packet: [u8; 5] = [0xDE, 0xAD, 0xBE, 0xEF, *resources.COUNT];
        *resources.COUNT+=1;
        LongFi::send(&packet, packet.len());
    }

    #[task(capacity = 8, priority = 1, resources = [DEBUG_UART])]
    fn echo(byte: u8) {
        resources.DEBUG_UART.write(byte);
    }

    #[interrupt(priority = 2, resources = [GPS_RX], spawn = [echo])]
    fn USART1() {
        if let Ok(byte) = resources.GPS_RX.read() {
            spawn.echo(byte);
        }
    }

    #[interrupt(priority = 2, resources = [LED, INT, BUTTON], spawn = [send_ping])]
    fn EXTI2_3() {
        static mut STATE: bool = false;
        // Clear the interrupt flag.
        resources.INT.clear_irq(resources.BUTTON.i);
        if *STATE {
           resources.LED.set_low().unwrap();
           *STATE = false;
        } else {
            resources.LED.set_high().unwrap();
           *STATE = true;
        }
        spawn.send_ping();
    }

    #[interrupt(priority = 2, resources = [SX1276_DIO0, INT], spawn = [radio_event])]
    fn EXTI4_15() {
        resources.INT.clear_irq(resources.SX1276_DIO0.i);
        spawn.radio_event(RfEvent::DIO0); 
    }

    // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn USART4_USART5();
    }
};


use stm32l0xx_hal::gpio::gpioa::*;
use stm32l0xx_hal::gpio::{Floating, Input, PushPull};

use embedded_hal::spi::FullDuplex;
use core::ffi; 
use nb::block;

use stm32l0xx_hal::pac::SPI1;



#[repr(C, align(4))]
pub struct SpiInstance {
    Instance:*mut ffi::c_void,
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
        &mut *(s.Spi.Instance as *mut hal::spi::Spi<
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
    }
    else {
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
pub extern "C" fn SX1276GetPaSelect(channel: u32) -> u8 {0}

use cortex_m::asm;

#[no_mangle]
pub extern "C" fn DelayMs(ms: u32){
    //asm::delay(ms);
}

#[no_mangle]
pub extern "C" fn SX1276SetAntSwLowPower(status: bool){}

#[no_mangle]
pub extern "C" fn SX1276SetAntSw(rxTx: u8){}

#[no_mangle]
pub extern "C" fn assert_param(expr: bool){}
