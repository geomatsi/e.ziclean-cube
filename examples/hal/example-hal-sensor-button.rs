#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
use rt::entry;
use rt::exception;
use rt::ExceptionFrame;

extern crate cortex_m as cm;
use cm::interrupt::Mutex;
use cm::iprintln;

extern crate panic_itm;

extern crate lazy_static;
use lazy_static::lazy_static;

extern crate stm32f1xx_hal as hal;
use hal::prelude::*;
use hal::stm32;
use hal::stm32::interrupt;

use core::cell::RefCell;
use core::ops::DerefMut;

type ExtIntr = stm32::EXTI;
type DbgPort = stm32::ITM;

lazy_static! {
    static ref G_EXTI: Mutex<RefCell<Option<ExtIntr>>> = Mutex::new(RefCell::new(None));
    static ref G_ITM: Mutex<RefCell<Option<DbgPort>>> = Mutex::new(RefCell::new(None));
}

#[entry]
fn main() -> ! {
    if let (Some(mut cp), Some(dp)) = (cm::Peripherals::take(), stm32::Peripherals::take()) {
        cm::interrupt::free(|cs| {
            let mut rcc = dp.RCC.constrain();
            let mut flash = dp.FLASH.constrain();

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
            G_ITM.borrow(cs).replace(Some(cp.ITM));
        });
    }

    loop {
        cm::interrupt::free(|cs| {
            if let Some(ref mut itm) = G_ITM.borrow(cs).borrow_mut().deref_mut() {
                let d = &mut itm.stim[0];
                iprintln!(d, "idle loop");
            }
        });

        delay(10000);
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
    nvic.enable(stm32::Interrupt::EXTI1);

    unsafe {
        nvic.set_priority(stm32::Interrupt::EXTI1, 1);
    }

    cm::peripheral::NVIC::unpend(stm32::Interrupt::EXTI1);
}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}

#[interrupt]
fn EXTI1() {
    cm::interrupt::free(|cs| {
        if let Some(ref mut itm) = G_ITM.borrow(cs).borrow_mut().deref_mut() {
            let d = &mut itm.stim[0];
            iprintln!(d, "Touch !");
        }

        if let Some(exti) = G_EXTI.borrow(cs).borrow().as_ref() {
            exti.pr.modify(|_, w| w.pr1().set_bit());
        }
    });
}
