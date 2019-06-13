#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
use rt::entry;
use rt::exception;
use rt::ExceptionFrame;

extern crate cortex_m as cm;
use cm::iprintln;

extern crate panic_itm;

use stm32f1xx_hal::{prelude::*, stm32};

#[entry]
fn main() -> ! {
    let mut core = cm::Peripherals::take().unwrap();
    let dbg = &mut core.ITM.stim[0];

    let p = stm32::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = p.GPIOC.split(&mut rcc.apb2);
    let mut gpiod = p.GPIOD.split(&mut rcc.apb2);
    let mut gpioe = p.GPIOE.split(&mut rcc.apb2);

    // GPIO

    //let mut test = gpioe.pe12.into_open_drain_output(&mut gpioe.crh);
    //let mut test = gpioe.pe15.into_alternate_push_pull(&mut gpioe.crh);
    //let mut test = gpiod.pd4.into_alternate_push_pull(&mut gpiod.crl);
    //let mut test = gpiod.pd4.into_alternate_push_pull(&mut gpiod.crl);
    //let mut test = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    //let mut test = gpiob.pb4.into_alternate_push_pull(&mut gpiob.crl);
    //let mut test = gpioa.pa11.into_alternate_push_pull(&mut gpioa.crh);
    let mut test = gpioa.pa15.into_alternate_push_pull(&mut gpioa.crh);

    loop {
        iprintln!(dbg, "LOW");
        test.set_low();
        delay(30000);
        iprintln!(dbg, "HIGH");
        test.set_high();
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
