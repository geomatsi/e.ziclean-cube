#![no_main]
#![no_std]

use embedded_hal::digital::v2::OutputPin;

use cortex_m_rt::entry;

use cortex_m as cm;

use panic_semihosting as _;

use cortex_m_semihosting::hprintln;

use stm32f1xx_hal::{prelude::*, stm32};

#[entry]
fn main() -> ! {
    let p = stm32::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);

    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    let mut gpiod = p.GPIOD.split(&mut rcc.apb2);
    let mut gpioe = p.GPIOE.split(&mut rcc.apb2);

    // GPIO

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

    let mut left_fwd = gpiod.pd2.into_open_drain_output(&mut gpiod.crl);
    left_fwd.set_low().unwrap();

    let mut left_rev = gpiod.pd5.into_open_drain_output(&mut gpiod.crl);
    left_rev.set_low().unwrap();

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

    let mut right_fwd = gpioe.pe14.into_open_drain_output(&mut gpioe.crh);
    right_fwd.set_low().unwrap();

    let mut right_rev = gpiob.pb11.into_open_drain_output(&mut gpiob.crh);
    right_rev.set_low().unwrap();

    // TIM4

    let c1 = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    let c2 = gpiob.pb7.into_alternate_push_pull(&mut gpiob.crl);
    let c3 = gpiob.pb8.into_alternate_push_pull(&mut gpiob.crh);
    let c4 = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);

    let mut pwm = p.TIM4.pwm(
        (c1, c2, c3, c4),
        &mut afio.mapr,
        500.hz(),
        clocks,
        &mut rcc.apb1,
    );

    let max = pwm.0.get_max_duty();

    hprintln!("pwm max duty: {}...", max).unwrap();

    pwm.0.enable();
    pwm.1.enable();
    pwm.2.enable();
    pwm.3.enable();

    //

    let duty: [u32; 5] = [16, 8, 4, 2, 1];

    hprintln!("lets rock...").unwrap();

    left_rev.set_high().unwrap();
    right_fwd.set_high().unwrap();

    loop {
        for s in 0..duty.len() {
            let d = max / duty[s] as u16;
            hprintln!("duty: {}", d).unwrap();
            pwm.0.set_duty(d);
            pwm.1.set_duty(d);
            pwm.2.set_duty(d);
            pwm.3.set_duty(d);
            delay(20000);
        }

        left_rev.toggle().unwrap();
        left_fwd.toggle().unwrap();

        right_rev.toggle().unwrap();
        right_fwd.toggle().unwrap();
    }
}

fn delay(count: u32) {
    for _ in 0..count {
        cm::asm::nop();
    }
}
