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
use stm32ral::afio;
use stm32ral::gpio;
use stm32ral::modify_reg;
use stm32ral::rcc;
use stm32ral::tim4;

#[entry]
fn main() -> ! {
    // configure RCC

    let rcc = rcc::RCC::take().unwrap();

    modify_reg!(rcc, rcc, APB1ENR, TIM4EN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPBEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPDEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPEEN: 1);

    hprintln!("Clock configuration completed...").unwrap();

    // configure GPIO

    let afio = afio::AFIO::take().unwrap();
    let gpiob = gpio::GPIOB::take().unwrap();
    let gpiod = gpio::GPIOD::take().unwrap();
    let gpioe = gpio::GPIOE::take().unwrap();

    // TIM4 remapping: PB[6789]
    modify_reg!(afio, afio, MAPR, TIM4_REMAP: 0);

    // PB6 (left wheel rev): alternative function TIM4_CH1 (PWM)
    modify_reg!(gpio, gpiob, CRL, MODE6: 0b11, CNF6: 0b10);
    modify_reg!(gpio, gpiob, ODR, ODR6: 0);

    // PB7 (left wheel fwd): alternative function TIM4_CH2 (PWM)
    modify_reg!(gpio, gpiob, CRL, MODE7: 0b11, CNF7: 0b10);
    modify_reg!(gpio, gpiob, ODR, ODR7: 0);

    // PD2: left wheel forward enable/disable
    modify_reg!(gpio, gpiod, CRL, MODE2: 0b11, CNF2: 0b01);
    modify_reg!(gpio, gpiod, ODR, ODR2: 1);

    // PD5: left wheel reverse enable/disable
    modify_reg!(gpio, gpiod, CRL, MODE5: 0b11, CNF2: 0b01);
    modify_reg!(gpio, gpiod, ODR, ODR5: 1);

    // NOTE: it looks like PD2 and PD5 are control signal of
    // SR-latch circuitry protecting H-bridge for left motor
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

    // PB8 (right wheel fwd): alternative function TIM4_CH3 (PWM)
    modify_reg!(gpio, gpiob, CRH, MODE8: 0b11, CNF8: 0b10);
    modify_reg!(gpio, gpiob, ODR, ODR8: 0);

    // PB9 (right wheel rev): alternative function TIM4_CH4 (PWM)
    modify_reg!(gpio, gpiob, CRH, MODE9: 0b11, CNF9: 0b10);
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

    hprintln!("GPIO configuration completed...").unwrap();

    // configure TIM4

    let tim4 = tim4::TIM4::take().unwrap();

    // TIM4 reset
    modify_reg!(rcc, rcc, APB1RSTR, TIM4RST: 1);
    delay(100);
    modify_reg!(rcc, rcc, APB1RSTR, TIM4RST: 0);

    // OC1 output
    modify_reg!(tim4, tim4, CCMR1, OC1M: 0b110, OC1PE: 1);
    modify_reg!(tim4, tim4, CCER, CC1P: 0, CC1E: 1);

    // OC2 output
    modify_reg!(tim4, tim4, CCMR1, OC2M: 0b110, OC2PE: 1);
    modify_reg!(tim4, tim4, CCER, CC2P: 0, CC2E: 1);

    // OC3 output
    modify_reg!(tim4, tim4, CCMR2, OC3M: 0b110, OC3PE: 1);
    modify_reg!(tim4, tim4, CCER, CC3P: 0, CC3E: 1);

    // OC4 output
    modify_reg!(tim4, tim4, CCMR2, OC4M: 0b110, OC4PE: 1);
    modify_reg!(tim4, tim4, CCER, CC4P: 0, CC4E: 1);

    // mode
    modify_reg!(tim4, tim4, PSC, PSC: 0);
    modify_reg!(tim4, tim4, ARR, ARR: 0xffff);

    modify_reg!(tim4, tim4, CR1, CMS: 0);
    modify_reg!(tim4, tim4, CR1, DIR: 0);
    modify_reg!(tim4, tim4, CR1, OPM: 0);
    modify_reg!(tim4, tim4, CR1, CEN: 1);

    hprintln!("TIM4 configuration completed...").unwrap();

    // enable left wheel reverse
    hprintln!("enable left wheel reverse").unwrap();
    modify_reg!(gpio, gpiod, ODR, ODR2: 1);
    modify_reg!(gpio, gpiod, ODR, ODR5: 0);

    // enable right wheel fwd
    hprintln!("enable right wheel forward").unwrap();
    modify_reg!(gpio, gpioe, ODR, ODR14: 0);
    modify_reg!(gpio, gpiob, ODR, ODR11: 1);

    hprintln!("TIM4 started...").unwrap();

    let duty: [u32; 12] = [
        10, 1000, 5000, 10000, 15000, 20000, 25000, 30000, 35000, 40000, 50000, 60000,
    ];
    loop {
        for s in 0..duty.len() {
            hprintln!("duty {}", duty[s]).unwrap();
            modify_reg!(tim4, tim4, CCR1, CCR1: duty[s]);
            modify_reg!(tim4, tim4, CCR3, CCR3: duty[s]);
            delay(20000);
        }
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
