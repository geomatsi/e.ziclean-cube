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
use core::cell::RefCell;
use core::ops::DerefMut;
use hal::prelude::*;
use hal::stm32;
use hal::stm32::interrupt;
use hal::timer::Event;
use hal::timer::Timer;

lazy_static! {
    static ref G_ITM: Mutex<RefCell<Option<stm32::ITM>>> = Mutex::new(RefCell::new(None));
    static ref G_TIM2: Mutex<RefCell<Option<Timer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));
    static ref G_TIM3: Mutex<RefCell<Option<Timer<stm32::TIM3>>>> = Mutex::new(RefCell::new(None));
    static ref G_TIM4: Mutex<RefCell<Option<Timer<stm32::TIM4>>>> = Mutex::new(RefCell::new(None));
}

#[entry]
fn main() -> ! {
    if let (Some(mut cp), Some(dp)) = (cm::Peripherals::take(), stm32::Peripherals::take()) {
        cm::interrupt::free(|cs| {
            // configure NVIC interrupts
            setup_interrupts(&mut cp);

            // configure clocks
            let mut rcc = dp.RCC.constrain();
            let mut flash = dp.FLASH.constrain();
            let clocks = rcc
                .cfgr
                .sysclk(8.mhz())
                .pclk1(8.mhz())
                .freeze(&mut flash.acr);

            // configure and start periodic timers
            let mut tim2 = Timer::tim2(dp.TIM2, 1.hz(), clocks, &mut rcc.apb1);
            let mut tim3 = Timer::tim3(dp.TIM3, 1.hz(), clocks, &mut rcc.apb1);
            let mut tim4 = Timer::tim4(dp.TIM4, 1.hz(), clocks, &mut rcc.apb1);

            tim2.listen(Event::Update);
            tim3.listen(Event::Update);
            tim4.listen(Event::Update);

            G_ITM.borrow(cs).replace(Some(cp.ITM));
            G_TIM2.borrow(cs).replace(Some(tim2));
            G_TIM3.borrow(cs).replace(Some(tim3));
            G_TIM4.borrow(cs).replace(Some(tim4));
        });
    }

    loop {
        cm::asm::wfi();
    }
}

fn setup_interrupts(cp: &mut cm::peripheral::Peripherals) {
    let nvic = &mut cp.NVIC;

    // Enable TIM3 IRQ, set prio 1 and clear any pending IRQs
    nvic.enable(stm32::Interrupt::TIM2);
    nvic.enable(stm32::Interrupt::TIM3);
    nvic.enable(stm32::Interrupt::TIM4);

    cm::peripheral::NVIC::unpend(stm32::Interrupt::TIM2);
    cm::peripheral::NVIC::unpend(stm32::Interrupt::TIM3);
    cm::peripheral::NVIC::unpend(stm32::Interrupt::TIM4);

    unsafe {
        nvic.set_priority(stm32::Interrupt::TIM2, 1);
        nvic.set_priority(stm32::Interrupt::TIM3, 1);
        nvic.set_priority(stm32::Interrupt::TIM4, 1);
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

#[interrupt]
fn TIM2() {
    cm::interrupt::free(|cs| {
        if let (Some(ref mut itm), Some(ref mut tim)) = (
            G_ITM.borrow(cs).borrow_mut().deref_mut(),
            G_TIM2.borrow(cs).borrow_mut().deref_mut(),
        ) {
            let d = &mut itm.stim[0];

            iprintln!(d, "TIM2");
            tim.start(1.hz());
        }
    });
}

#[interrupt]
fn TIM3() {
    cm::interrupt::free(|cs| {
        if let (Some(ref mut itm), Some(ref mut tim)) = (
            G_ITM.borrow(cs).borrow_mut().deref_mut(),
            G_TIM3.borrow(cs).borrow_mut().deref_mut(),
        ) {
            let d = &mut itm.stim[0];

            iprintln!(d, "TIM3");
            tim.start(1.hz());
        }
    });
}

#[interrupt]
fn TIM4() {
    cm::interrupt::free(|cs| {
        if let (Some(ref mut itm), Some(ref mut tim)) = (
            G_ITM.borrow(cs).borrow_mut().deref_mut(),
            G_TIM4.borrow(cs).borrow_mut().deref_mut(),
        ) {
            let d = &mut itm.stim[0];

            iprintln!(d, "TIM4");
            tim.start(1.hz());
        }
    });
}
