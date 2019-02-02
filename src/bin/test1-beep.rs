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

//
// Example: use STM32RAL (Register Access Layer) to toggle PE0 (beeper) pin
//

#[entry]
fn main() -> ! {
    let rcc = rcc::RCC::take().unwrap();
    let gpioe = gpio::GPIOE::take().unwrap();

    hprintln!("Ready to beep...").unwrap();

    // Enable GPIOE
    modify_reg!(rcc, rcc, APB2ENR, IOPEEN: 1);

    // PE0 GPIO 50MHz push-pull
    modify_reg!(gpio, gpioe, CRL, MODE0: 0b11, CNF0: 0b00);
    modify_reg!(gpio, gpioe, ODR, ODR0: 0);

    loop {
        modify_reg!(gpio, gpioe, ODR, ODR0: 0);
        delay(2000);
        modify_reg!(gpio, gpioe, ODR, ODR0: 1);
        delay(500);
    }
}

fn delay(count: u32) {
    for _ in 0..count {
        cm::asm::nop();
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
