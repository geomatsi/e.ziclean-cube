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
// Example: use STM32RAL (Register Access Layer) to toggle PC7 pin
// to control all 5 IR LEDs of front IR obstacle sensors
//

#[entry]
fn main() -> ! {
    let gpiox = gpio::GPIOC::take().unwrap();
    let rcc = rcc::RCC::take().unwrap();

    hprintln!("Ready to test pins\n").unwrap();

    modify_reg!(rcc, rcc, APB2ENR, IOPCEN: 1);
    modify_reg!(gpio, gpiox, CRL, MODE7: 0b11, CNF7: 0b00);
    modify_reg!(gpio, gpiox, ODR, ODR7: 0);

    loop {
        hprintln!("PC7 -> 0").unwrap();
        delay(20000);
        modify_reg!(gpio, gpiox, ODR, ODR7: 1);
        hprintln!("PC7 -> 1").unwrap();
        delay(20000);
        modify_reg!(gpio, gpiox, ODR, ODR7: 0);
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
