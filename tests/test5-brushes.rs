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
    let gpiob = gpio::GPIOB::take().unwrap();

    modify_reg!(rcc, rcc, APB2ENR, IOPBEN: 1);

    // TODO: all brushes: use TIM3_CH2 to control their speed
    modify_reg!(gpio, gpiob, CRL, MODE5: 0b11, CNF5: 0b00);

    loop {
        hprintln!("brushes: on").unwrap();
        modify_reg!(gpio, gpiob, ODR, ODR5: 1);
        delay(30000);
        hprintln!("brushes: off").unwrap();
        modify_reg!(gpio, gpiob, ODR, ODR5: 0);
        delay(30000);
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
