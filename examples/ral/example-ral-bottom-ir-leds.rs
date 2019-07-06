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

#[entry]
fn main() -> ! {
    let rcc = rcc::RCC::take().unwrap();
    let gpiox = gpio::GPIOD::take().unwrap();

    hprintln!("Ready to test pins\n").unwrap();

    modify_reg!(rcc, rcc, APB2ENR, IOPDEN: 1);
    modify_reg!(gpio, gpiox, CRH, MODE9: 0b11, CNF9: 0b00);

    loop {
        modify_reg!(gpio, gpiox, ODR, ODR9: 0);
        hprintln!("PD9 -> 0").unwrap();
        delay(20000);
        modify_reg!(gpio, gpiox, ODR, ODR9: 1);
        hprintln!("PD9 -> 1").unwrap();
        delay(20000);
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
