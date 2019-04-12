#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
use rt::entry;
use rt::exception;
use rt::ExceptionFrame;

extern crate cortex_m as cm;
use cm::iprintln;

extern crate panic_itm;

extern crate stm32f1xx_hal as hal;
use hal::prelude::*;
use hal::stm32;
use hal::stm32::interrupt;

type ExtIntr = stm32::EXTI;
type DbgPort = stm32::ITM;

static mut G_EXTI: Option<ExtIntr> = None;
static mut G_STIM: Option<DbgPort> = None;

#[entry]
fn main() -> ! {
    let mut core = cm::Peripherals::take().unwrap();

    // configure NVIC interrupts
    setup_interrupts(&mut core);

    let p = stm32::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let _clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioc = p.GPIOC.split(&mut rcc.apb2);
    let mut gpiod = p.GPIOD.split(&mut rcc.apb2);
    let mut gpioe = p.GPIOE.split(&mut rcc.apb2);

    // PD13: GPIO open-drain output: IR LEDs for both main motors encoders, active low
    let mut pd13 = gpiod.pd13.into_open_drain_output(&mut gpiod.crh);
    pd13.set_low();

    // PC12, PE8: GPIO floating input: IR diodes for both main motors encoders
    gpioc.pc12.into_floating_input(&mut gpioc.crh);
    gpioe.pe8.into_floating_input(&mut gpioe.crh);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);

    // select PE8 as source input for line EXTI8
    afio.exticr3
        .exticr3()
        .modify(|_, w| unsafe { w.exti8().bits(0b0100) });

    // select PC12 as source input for line EXTI12
    afio.exticr4
        .exticr4()
        .modify(|_, w| unsafe { w.exti12().bits(0b0010) });

    // enable EXTI8 and EXTI12 lines and configure interrupt on rising edge
    p.EXTI.imr.modify(|_, w| w.mr8().set_bit());
    p.EXTI.rtsr.modify(|_, w| w.tr8().set_bit());

    p.EXTI.imr.modify(|_, w| w.mr12().set_bit());
    p.EXTI.rtsr.modify(|_, w| w.tr12().set_bit());

    unsafe {
        G_EXTI = Some(p.EXTI);
        G_STIM = Some(core.ITM);
    }

    loop {
        cm::asm::wfi();
    }
}

fn setup_interrupts(cp: &mut cm::peripheral::Peripherals) {
    let nvic = &mut cp.NVIC;

    // Enable EXTI9_5 and EXTI15_10, set prio 1, clear any pending IRQs
    nvic.enable(stm32::Interrupt::EXTI9_5);
    nvic.enable(stm32::Interrupt::EXTI15_10);

    unsafe {
        nvic.set_priority(stm32::Interrupt::EXTI9_5, 1);
        nvic.set_priority(stm32::Interrupt::EXTI15_10, 1);
    }

    cm::peripheral::NVIC::unpend(stm32::Interrupt::EXTI9_5);
    cm::peripheral::NVIC::unpend(stm32::Interrupt::EXTI15_10);
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
fn EXTI9_5() {
    let exti = unsafe { G_EXTI.as_mut().unwrap() };
    let itm = unsafe { G_STIM.as_mut().unwrap() };
    let d = &mut itm.stim[0];

    iprintln!(d, "EXTI9_5");

    exti.pr.modify(|_, w| w.pr8().set_bit());
}

#[interrupt]
fn EXTI15_10() {
    let exti = unsafe { G_EXTI.as_mut().unwrap() };
    let itm = unsafe { G_STIM.as_mut().unwrap() };
    let d = &mut itm.stim[0];

    iprintln!(d, "EXTI15_10");

    exti.pr.modify(|_, w| w.pr12().set_bit());
}
