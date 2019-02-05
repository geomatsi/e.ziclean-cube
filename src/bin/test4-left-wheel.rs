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

    hprintln!("Ready to test pins\n").unwrap();

    modify_reg!(rcc, rcc, APB2ENR, IOPBEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPDEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPEEN: 1);

    let gpiob = gpio::GPIOB::take().unwrap();
    let gpiod = gpio::GPIOD::take().unwrap();

    // PB7: left wheel forward
    // - TODO configure PB7 as TIM4_CH2/PWM to control speed
    modify_reg!(gpio, gpiob, CRL, MODE7: 0b11, CNF7: 0b00);
    modify_reg!(gpio, gpiob, ODR, ODR7: 0);

    // PB6: left wheel reverse
    // - TODO configure PB6 as TIM4_CH1/PWM to control speed
    modify_reg!(gpio, gpiob, CRL, MODE6: 0b11, CNF6: 0b00);
    modify_reg!(gpio, gpiob, ODR, ODR6: 0);

    // PD2: left wheel forward enable/disable
    modify_reg!(gpio, gpiod, CRL, MODE2: 0b11, CNF2: 0b01);
    modify_reg!(gpio, gpiod, ODR, ODR2: 1);

    // PD5: left wheel reverse enable/disable
    modify_reg!(gpio, gpiod, CRL, MODE5: 0b11, CNF2: 0b01);
    modify_reg!(gpio, gpiod, ODR, ODR5: 1);

    // NOTE: it looks like PD2 and PD5 are control signal of
    // SR-latch circuitry protecting H-bridge for right motor
    //  _________________________
    // | PD2  | PD5  | FWD | REV |
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
        hprintln!("---> PD2(1) PD5(1): stop").unwrap();
        modify_reg!(gpio, gpiod, ODR, ODR2: 1);
        modify_reg!(gpio, gpiod, ODR, ODR5: 1);
        delay(30000);

        hprintln!("---> PD2(0) PD5(1): forward").unwrap();
        modify_reg!(gpio, gpiod, ODR, ODR2: 0);
        modify_reg!(gpio, gpiod, ODR, ODR5: 1);
        delay(30000);

        hprintln!("---> PD2(1) PD5(0): reverse").unwrap();
        modify_reg!(gpio, gpiod, ODR, ODR2: 1);
        modify_reg!(gpio, gpiod, ODR, ODR5: 0);
        delay(30000);

        hprintln!("---> PD2(0) PD5(0): stop").unwrap();
        modify_reg!(gpio, gpiod, ODR, ODR2: 0);
        modify_reg!(gpio, gpiod, ODR, ODR5: 0);
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
