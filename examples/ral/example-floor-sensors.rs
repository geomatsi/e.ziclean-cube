#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
use rt::entry;
use rt::exception;
use rt::ExceptionFrame;

extern crate cortex_m as cm;
extern crate panic_semihosting;

extern crate cortex_m_semihosting;
use cortex_m_semihosting::hprintln;

extern crate stm32ral;
use stm32ral::gpio;
use stm32ral::modify_reg;
use stm32ral::rcc;
use stm32ral::read_reg;

#[entry]
fn main() -> ! {
    let rcc = rcc::RCC::take().unwrap();

    hprintln!("Ready to test pins\n").unwrap();

    modify_reg!(rcc, rcc, APB2ENR, IOPAEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPBEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPCEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPDEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPEEN: 1);

    let gpioa = gpio::GPIOA::take().unwrap();
    let gpiob = gpio::GPIOB::take().unwrap();
    let gpioc = gpio::GPIOC::take().unwrap();
    let gpiod = gpio::GPIOD::take().unwrap();

    // PA7, PB1, PC1: floating input
    // FIXME: probably should be analog input and ADC readings
    modify_reg!(gpio, gpioa, CRL, MODE7: 0b00, CNF7: 0b01);
    modify_reg!(gpio, gpiob, CRL, MODE1: 0b00, CNF1: 0b01);
    modify_reg!(gpio, gpioc, CRL, MODE1: 0b00, CNF1: 0b01);

    // PD9: GPIO push-pull output: enable IR LEDs of all the 3 floor sensors
    modify_reg!(gpio, gpiod, CRH, MODE9: 0b11, CNF9: 0b00);
    modify_reg!(gpio, gpiod, ODR, ODR9: 1);

    loop {
        let v1 = read_reg!(gpio, gpioa, IDR, IDR7);
        let v2 = read_reg!(gpio, gpiob, IDR, IDR1);
        let v3 = read_reg!(gpio, gpioc, IDR, IDR1);
        hprintln!("{}-{}-{}", v1, v2, v3).unwrap();
    }
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
