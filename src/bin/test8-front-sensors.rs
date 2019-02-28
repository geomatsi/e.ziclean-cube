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
    let gpioc = gpio::GPIOC::take().unwrap();
    let gpiod = gpio::GPIOD::take().unwrap();
    let gpioe = gpio::GPIOE::take().unwrap();

    // FIXME: probably should be analog input and ADC readings
    modify_reg!(gpio, gpioa, CRL, MODE4: 0b00, CNF4: 0b01);
    modify_reg!(gpio, gpioa, CRL, MODE6: 0b00, CNF6: 0b01);

    modify_reg!(gpio, gpioc, CRL, MODE5: 0b00, CNF5: 0b01);
    modify_reg!(gpio, gpioc, CRH, MODE11: 0b00, CNF11: 0b01);

    modify_reg!(gpio, gpiod, CRH, MODE15: 0b00, CNF15: 0b01);

    modify_reg!(gpio, gpioe, CRH, MODE10: 0b00, CNF10: 0b01);

    // PC7: GPIO push-pull output: enable IR LEDs of all the front sensors
    modify_reg!(gpio, gpioc, CRL, MODE7: 0b11, CNF7: 0b00);
    modify_reg!(gpio, gpioc, ODR, ODR7: 1);

    loop {
        let (a4, a6) = read_reg!(gpio, gpioa, IDR, IDR4, IDR6);
        let (c5, c11) = read_reg!(gpio, gpioc, IDR, IDR5, IDR11);
        let d15 = read_reg!(gpio, gpiod, IDR, IDR15);
        let e10 = read_reg!(gpio, gpioe, IDR, IDR10);

        hprintln!("{}-{}-{}-{}-{} {}", c11, a4, a6, c5, e10, d15).unwrap();
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
