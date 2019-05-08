#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(lang_items)]

// panic handler
extern crate panic_semihosting;

use core::fmt::Write;
use cortex_m::asm;
use cortex_m_rt::entry;
use stm32l0xx_hal::{pac, prelude::*, rcc::Config, serial, spi};

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

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Configure the clock.
    let mut rcc = dp.RCC.freeze(Config::hsi16());

    // Acquire the GPIOA peripheral. This also enables the clock for GPIOA in
    // the RCC register.
    let gpioa = dp.GPIOA.split(&mut rcc);

    let tx_pin = gpioa.pa9;
    let rx_pin = gpioa.pa10;

    // Configure the serial peripheral.
    let serial = dp
        .USART1
        .usart((tx_pin, rx_pin), serial::Config::default(), &mut rcc)
        .unwrap();

    let (mut tx, _rx) = serial.split();

    write!(tx, "Hello, world!\r\n").unwrap();

    let gpiob = dp.GPIOB.split(&mut rcc);
    let sck = gpiob.pb3;
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7;

    // Initialise the SPI peripheral.
    let mut spi = dp
        .SPI1
        .spi((sck, miso, mosi), spi::MODE_0, 100_000.hz(), &mut rcc);
    spi.write(&[0, 1]).unwrap();
    loop {    
    	asm::nop();
	    write!(tx, ".").unwrap();

    }
}

/*
uint8_t data;

SX1276Acquire( );

stm32l0_gpio_pin_write(RADIO_NSS, 0);

stm32l0_spi_data(&RADIO_SPI, addr & ~0x80);
data = stm32l0_spi_data(&RADIO_SPI, 0xff);

stm32l0_gpio_pin_write(RADIO_NSS, 1);
*/