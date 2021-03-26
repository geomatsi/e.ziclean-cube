#![no_main]
#![no_std]

use cm::interrupt::Mutex;
use core::cell::RefCell;
use cortex_m as cm;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
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
            // configure RTT logger
            rtt_init_print!();

            // configure NVIC interrupts
            setup_interrupts(&mut cp);

            let mut flash = dp.FLASH.constrain();
            let mut rcc = dp.RCC.constrain();
            let _clocks = rcc.cfgr.freeze(&mut flash.acr);

            let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
            let mut gpiod = dp.GPIOD.split(&mut rcc.apb2);
            let mut gpioe = dp.GPIOE.split(&mut rcc.apb2);

            // PD13: GPIO open-drain output: IR LEDs for both main motors encodersd, active low
            let mut pd13 = gpiod.pd13.into_open_drain_output(&mut gpiod.crh);
            pd13.set_low().unwrap();

            // PC12, PE8: GPIO floating input: IR diodes for both main motors encoders
            gpioc.pc12.into_floating_input(&mut gpioc.crh);
            gpioe.pe8.into_floating_input(&mut gpioe.crh);

            let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

            // select PE8 as source input for line EXTI8
            afio.exticr3
                .exticr3()
                .modify(|_, w| unsafe { w.exti8().bits(0b0100) });

            // select PC12 as source input for line EXTI12
            afio.exticr4
                .exticr4()
                .modify(|_, w| unsafe { w.exti12().bits(0b0010) });

            // enable EXTI8 and EXTI12 lines and configure interrupt on rising edge
            dp.EXTI.imr.modify(|_, w| w.mr8().set_bit());
            dp.EXTI.rtsr.modify(|_, w| w.tr8().set_bit());

            dp.EXTI.imr.modify(|_, w| w.mr12().set_bit());
            dp.EXTI.rtsr.modify(|_, w| w.tr12().set_bit());

            G_EXTI.borrow(cs).replace(Some(dp.EXTI));
        });
    }

    loop {
        cm::interrupt::free(|_cs| {
            rprintln!("idle loop");
        });

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

    // Enable EXTI9_5 and EXTI15_10, set prio 1, clear any pending IRQs
    unsafe {
        cm::peripheral::NVIC::unmask(stm32::Interrupt::EXTI9_5);
        cm::peripheral::NVIC::unmask(stm32::Interrupt::EXTI15_10);

        nvic.set_priority(stm32::Interrupt::EXTI9_5, 1);
        nvic.set_priority(stm32::Interrupt::EXTI15_10, 1);
    }

    cm::peripheral::NVIC::unpend(stm32::Interrupt::EXTI9_5);
    cm::peripheral::NVIC::unpend(stm32::Interrupt::EXTI15_10);
}

#[interrupt]
fn EXTI9_5() {
    cm::interrupt::free(|cs| {
        rprintln!("EXTI9_5");

        if let Some(exti) = G_EXTI.borrow(cs).borrow().as_ref() {
            exti.pr.modify(|_, w| w.pr8().set_bit());
        }
    });
}

#[interrupt]
fn EXTI15_10() {
    cm::interrupt::free(|cs| {
        rprintln!("EXTI15_10");

        if let Some(exti) = G_EXTI.borrow(cs).borrow().as_ref() {
            exti.pr.modify(|_, w| w.pr12().set_bit());
        }
    });
}
