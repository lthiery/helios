#![no_std]
#![no_main]
#![feature(lang_items)]

//use nb::block;

/*
#define RADIO_RESET                          STM32L0_GPIO_PIN_PC0

#define RADIO_MOSI                           STM32L0_GPIO_PIN_PA7_SPI1_MOSI
#define RADIO_MISO                           STM32L0_GPIO_PIN_PA6_SPI1_MISO
#define RADIO_SCLK                           STM32L0_GPIO_PIN_PB3_SPI1_SCK
#define RADIO_NSS                            STM32L0_GPIO_PIN_PA15_SPI1_NSS

#define RADIO_DIO_0                          STM32L0_GPIO_PIN_PB4
#define RADIO_DIO_1                          STM32L0_GPIO_PIN_PB1_TIM3_CH4
#define RADIO_DIO_2                          STM32L0_GPIO_PIN_PB0_TIM3_CH3
//#define RADIO_DIO_3                          STM32L0_GPIO_PIN_PC13

//#define RADIO_TCXO_VCC                       STM32L0_GPIO_PIN_PH1

#define RADIO_ANT_SWITCH_RX                  STM32L0_GPIO_PIN_PA1
#define RADIO_ANT_SWITCH_TX_RFO              STM32L0_GPIO_PIN_PC2
#define RADIO_ANT_SWITCH_TX_BOOST            STM32L0_GPIO_PIN_PC1

#define BOARD_TCXO_WAKEUP_TIME               5
*/

// panic handler
extern crate panic_semihosting;

use core::fmt::Write;
use cortex_m_rt::entry;
use sx1276;
use stm32l0xx_hal as hal;
use hal::{exti::TriggerEdge, gpio::*, pac, prelude::*, rcc::Config, serial, spi};
use embedded_hal::digital::v2::OutputPin;

#[rtfm::app(device = stm32l0xx_hal::pac)]
const APP: () = {
    static mut LED: gpiob::PB5<Output<PushPull>> = ();
    static mut INT: pac::EXTI = ();
    static mut BUTTON: gpiob::PB2<Input<PullUp>> = ();

    #[init]
    fn init() -> init::LateResources {

        let dp = pac::Peripherals::take().unwrap();

        // Configure the clock.
        let mut rcc = dp.RCC.freeze(Config::hsi16());

        let gpioa = dp.GPIOA.split(&mut rcc);

        let tx_pin = gpioa.pa2;
        let rx_pin = gpioa.pa3;

        let serial = dp
            .USART2
            .usart((tx_pin, rx_pin), serial::Config::default(), &mut rcc)
            .unwrap();

        let gpiob = dp.GPIOB.split(&mut rcc);
        let sck = gpiob.pb3;
        let miso = gpioa.pa6;
        let mosi = gpioa.pa7;
        let nss = gpioa.pa15.into_push_pull_output();

        // Initialise the SPI peripheral.
        let spi = dp
            .SPI1
            .spi((sck, miso, mosi), spi::MODE_0, 100_000.hz(), &mut rcc);

        // Acquire the GPIOB peripheral. This also enables the clock for GPIOB in
        // the RCC register.
        let gpiob = device.GPIOB.split(&mut rcc);

        // Configure PB5 as output.
        let led = gpiob.pb5.into_push_pull_output();

        // Configure PB2 as input.
        let button = gpiob.pb2.into_pull_up_input();

        let exti = device.EXTI;
        // Configure the external interrupt on the falling edge for the pin 2.
        exti.listen(
            &mut rcc,
            &mut device.SYSCFG_COMP,
            button.port,
            button.i,
            TriggerEdge::Falling,
        );

        // Return the initialised resources.
        init::LateResources {
            LED: led,
            INT: exti,
            BUTTON: button,
        }

    }

    #[idle]
    fn idle() -> !{
        loop {}
    }

    #[interrupt(resources = [LED, INT, BUTTON])]
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
        
    }

};

use stm32l0xx_hal::gpio::gpioa::*;
use stm32l0xx_hal::gpio::gpiob::*;
use stm32l0xx_hal::gpio::{Floating, Input, PushPull};
use stm32l0xx_hal::pac::SPI1;

use embedded_hal::spi::FullDuplex;
use core::ffi; 
use nb::block;


#[repr(C, align(4))]
pub struct SpiInstance {
    Instance:*mut ffi::c_void,
    //Reset: &Gpio_t,
}

#[repr(C, align(4))]
pub struct Spi_s {
    Spi: SpiInstance,
    Nss: Gpio_t,
}

pub type Spi_t = Spi_s;


//find more elegant way to make cbindgen export Spi_t
#[no_mangle]
pub extern "C" fn foo(s: Spi_t) {
}

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

    /*
    let spi: &mut hal::spi::Spi<
        SPI1,
        (
            PB5<Input<Floating>>,
            PA11<Input<Floating>>,
            PA12<Input<Floating>>,
        ),
    > = unsafe {
        &mut *(s.Spi.Instance as *mut hal::spi::Spi<
            SPI1,
            (
                PB5<Input<Floating>>,
                PA11<Input<Floating>>,
                PA12<Input<Floating>>,
            ),
        >)
    };
    */


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
    obj: &Gpio_t,
    pin: PinNames,
    mode: PinModes,
    config: PinConfigs,
    pin_type: PinTypes,
    val: u32,
) {

}

#[no_mangle]
pub extern "C" fn GpioWrite(obj: Gpio_t, val: u8) {
    //let gpio: &mut stm32l0xx_hal::gpio::gpioa::PA2<Output<PushPull>> =
    //    unsafe { &mut *(obj as *mut stm32l0xx_hal::gpio::gpioa::PA2<Output<PushPull>>) };
    let gpio: &mut stm32l0xx_hal::gpio::gpioa::PA15<Output<PushPull>> =
        unsafe { &mut *(obj as *mut stm32l0xx_hal::gpio::gpioa::PA15<Output<PushPull>>) };


    if (val == 0) {
        embedded_hal::digital::v2::OutputPin::set_low(gpio).unwrap();
    } else {
        embedded_hal::digital::v2::OutputPin::set_high(gpio).unwrap();
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
pub extern "C" fn SX1276IoIrqInit(irq_handlers: [irq_ptr; 6]) {}

#[no_mangle]
pub extern "C" fn SX1276GetPaSelect(channel: u32) -> u8 {0}

#[no_mangle]
pub extern "C" fn DelayMs(ms: u32){

    
}

#[no_mangle]
pub extern "C" fn memcpy1(dst: &u8, src: &u8, size: u16){}

#[no_mangle]
pub extern "C" fn SX1276SetAntSwLowPower(status: bool){}

#[no_mangle]
pub extern "C" fn SX1276SetAntSw(rxTx: u8){}

#[no_mangle]
pub extern "C" fn assert_param(expr: bool){}