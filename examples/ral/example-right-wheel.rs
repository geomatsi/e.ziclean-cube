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

    hprintln!("Ready to test motors\n").unwrap();

    modify_reg!(rcc, rcc, APB2ENR, IOPBEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPEEN: 1);

    let gpiob = gpio::GPIOB::take().unwrap();
    let gpioe = gpio::GPIOE::take().unwrap();

    // PB8: right wheel forward
    // - TODO configure PB8 as TIM4_CH3/PWM to control speed
    modify_reg!(gpio, gpiob, CRH, MODE8: 0b11, CNF8: 0b00);
    modify_reg!(gpio, gpiob, ODR, ODR8: 0);

    // PB9: right wheel reverse
    // - TODO configure PB9 as TIM4_CH4/PWM to control speed
    modify_reg!(gpio, gpiob, CRH, MODE9: 0b11, CNF9: 0b00);
    modify_reg!(gpio, gpiob, ODR, ODR9: 0);

    // PE14: right wheel forward enable/disable
    modify_reg!(gpio, gpioe, CRH, MODE14: 0b11, CNF14: 0b01);
    modify_reg!(gpio, gpioe, ODR, ODR14: 1);

    // PB11: right wheel reverse enable/disable
    modify_reg!(gpio, gpiob, CRH, MODE11: 0b11, CNF11: 0b01);
    modify_reg!(gpio, gpiob, ODR, ODR11: 1);

    // NOTE: it looks like PE14 and PB11 are control signal of
    // SR-latch circuitry protecting H-bridge for right motor
    //  _________________________
    // | PE14 | PB11 | FWD | REV |
    //  _________________________
    // |  0   |   0  |  -  |  -  |
    //  _________________________
    // |  1   |   0  |  -  |  +  |
    //  _________________________
    // |  0   |   1  |  +  |  -  |
    //  _________________________
    // |  1   |   1  |  -  |  -  |
    //  _________________________

    loop {
        hprintln!("---> PE14(1) PB11(1): stop").unwrap();
        modify_reg!(gpio, gpioe, ODR, ODR14: 1);
        modify_reg!(gpio, gpiob, ODR, ODR11: 1);
        delay(30000);

        hprintln!("---> PE14(0) PB11(1): forward").unwrap();
        modify_reg!(gpio, gpioe, ODR, ODR14: 0);
        modify_reg!(gpio, gpiob, ODR, ODR11: 1);
        delay(30000);

        hprintln!("---> PE14(1) PB11(0): reverse").unwrap();
        modify_reg!(gpio, gpioe, ODR, ODR14: 1);
        modify_reg!(gpio, gpiob, ODR, ODR11: 0);
        delay(30000);

        hprintln!("---> PE14(0) PB11(0): stop").unwrap();
        modify_reg!(gpio, gpioe, ODR, ODR14: 0);
        modify_reg!(gpio, gpiob, ODR, ODR11: 0);
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
