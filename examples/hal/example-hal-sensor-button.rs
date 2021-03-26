#![no_main]
#![no_std]

use cm::interrupt::Mutex;
use core::cell::RefCell;
use cortex_m as cm;
use cortex_m_rt::entry;
use hal::prelude::*;
use hal::stm32;
use hal::stm32::interrupt;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal as hal;

type ExtIntr = stm32::EXTI;

static G_EXTI: Mutex<RefCell<Option<ExtIntr>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let (Some(mut cp), Some(dp)) = (cm::Peripherals::take(), stm32::Peripherals::take()) {
        cm::interrupt::free(|cs| {
            let mut flash = dp.FLASH.constrain();
            let mut rcc = dp.RCC.constrain();

            rtt_init_print!();

            let _clocks = rcc
                .cfgr
                .sysclk(8.mhz())
                .pclk1(8.mhz())
                .freeze(&mut flash.acr);

            setup_interrupts(&mut cp);

            let mut gpiod = dp.GPIOD.split(&mut rcc.apb2);

            // PD1: GPIO floating input: one-channel touch sensor
            gpiod.pd1.into_floating_input(&mut gpiod.crl);

            let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

            // select PD1 as source input for line EXTI1
            afio.exticr1
                .exticr1()
                .modify(|_, w| unsafe { w.exti1().bits(0b0011) });

            // enable EXTI1 line and configure interrupt on rising edge
            dp.EXTI.imr.modify(|_, w| w.mr1().set_bit());
            // Note on levels:
            //   - use falling edge to detect pressed button
            //   - use rising edge to detect released button
            dp.EXTI.ftsr.modify(|_, w| w.tr1().set_bit());

            G_EXTI.borrow(cs).replace(Some(dp.EXTI));
        });
    }

    loop {
        rprintln!("idle loop");
        delay(1000000);
    }
}

fn delay(count: u32) {
    for _ in 0..count {
        cm::asm::nop();
    }
}

fn setup_interrupts(cp: &mut cm::peripheral::Peripherals) {
    let nvic = &mut cp.NVIC;

    // Enable EXTI1, set prio 1, clear any pending IRQs
    unsafe {
        cm::peripheral::NVIC::unmask(stm32::Interrupt::EXTI1);
        nvic.set_priority(stm32::Interrupt::EXTI1, 1);
    }

    cm::peripheral::NVIC::unpend(stm32::Interrupt::EXTI1);
}

#[interrupt]
fn EXTI1() {
    cm::interrupt::free(|cs| {
        rprintln!("Touch !");

        if let Some(exti) = G_EXTI.borrow(cs).borrow().as_ref() {
            exti.pr.modify(|_, w| w.pr1().set_bit());
        }
    });
}
