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

    modify_reg!(rcc, rcc, APB2ENR, IOPCEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPDEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPEEN: 1);

    let gpioc = gpio::GPIOC::take().unwrap();
    let gpiod = gpio::GPIOD::take().unwrap();
    let gpioe = gpio::GPIOE::take().unwrap();

    // PD13: GPIO open-drain: active low: IR LEDs on encoders
    modify_reg!(gpio, gpiod, CRH, MODE13: 0b11, CNF13: 0b01);
    modify_reg!(gpio, gpiod, ODR, ODR13: 0);

    // PC12, PE8: GPIO floating input
    modify_reg!(gpio, gpioc, CRH, MODE12: 0b00, CNF12: 0b01);
    modify_reg!(gpio, gpioe, CRH, MODE8: 0b00, CNF8: 0b01);

    loop {
        let v1 = read_reg!(gpio, gpioc, IDR, IDR12);
        let v2 = read_reg!(gpio, gpioe, IDR, IDR8);
        hprintln!("{}-{}", v1, v2).unwrap();
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
