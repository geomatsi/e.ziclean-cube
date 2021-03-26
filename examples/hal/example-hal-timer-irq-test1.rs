#![no_main]
#![no_std]

use cm::interrupt::Mutex;
use cm::peripheral::syst::SystClkSource;
use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m as cm;
use cortex_m_rt::entry;
use cortex_m_rt::exception;
use hal::prelude::*;
use hal::stm32;
use hal::stm32::interrupt;
use hal::timer::CountDownTimer;
use hal::timer::Event;
use hal::timer::Timer;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal as hal;

static G_TIM2: Mutex<RefCell<Option<CountDownTimer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));
static G_TIM3: Mutex<RefCell<Option<CountDownTimer<stm32::TIM3>>>> = Mutex::new(RefCell::new(None));
static G_TIM4: Mutex<RefCell<Option<CountDownTimer<stm32::TIM4>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let (Some(mut cp), Some(dp)) = (cm::Peripherals::take(), stm32::Peripherals::take()) {
        cm::interrupt::free(|cs| {
            // configure RTT logging
            rtt_init_print!();

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

            // configure SysTick timer
            let mut syst = cp.SYST;
            syst.set_clock_source(SystClkSource::Core);
            syst.set_reload(16_000_000 - 1);
            syst.enable_counter();
            syst.enable_interrupt();

            // configure and start periodic GP timers
            let mut tim2 = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(1.hz());
            let mut tim3 = Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1).start_count_down(1.hz());
            let mut tim4 = Timer::tim4(dp.TIM4, &clocks, &mut rcc.apb1).start_count_down(1.hz());

            tim2.listen(Event::Update);
            tim3.listen(Event::Update);
            tim4.listen(Event::Update);

            G_TIM2.borrow(cs).replace(Some(tim2));
            G_TIM3.borrow(cs).replace(Some(tim3));
            G_TIM4.borrow(cs).replace(Some(tim4));
        });
    }

    loop {
        cm::asm::nop();
    }
}

fn setup_interrupts(cp: &mut cm::peripheral::Peripherals) {
    let nvic = &mut cp.NVIC;

    // Enable TIM3 IRQ, set prio 1 and clear any pending IRQs
    unsafe {
        cm::peripheral::NVIC::unmask(stm32::Interrupt::TIM2);
        cm::peripheral::NVIC::unmask(stm32::Interrupt::TIM3);
        cm::peripheral::NVIC::unmask(stm32::Interrupt::TIM4);

        nvic.set_priority(stm32::Interrupt::TIM2, 1);
        nvic.set_priority(stm32::Interrupt::TIM3, 1);
        nvic.set_priority(stm32::Interrupt::TIM4, 1);
    }

    cm::peripheral::NVIC::unpend(stm32::Interrupt::TIM2);
    cm::peripheral::NVIC::unpend(stm32::Interrupt::TIM3);
    cm::peripheral::NVIC::unpend(stm32::Interrupt::TIM4);
}

#[interrupt]
fn TIM2() {
    cm::interrupt::free(|cs| {
        if let Some(ref mut tim) = G_TIM2.borrow(cs).borrow_mut().deref_mut() {
            rprintln!("TIM2");
            tim.clear_update_interrupt_flag();
        }
    });
}

#[interrupt]
fn TIM3() {
    cm::interrupt::free(|cs| {
        if let Some(ref mut tim) = G_TIM3.borrow(cs).borrow_mut().deref_mut() {
            rprintln!("TIM3");
            tim.clear_update_interrupt_flag();
        }
    });
}

#[interrupt]
fn TIM4() {
    cm::interrupt::free(|cs| {
        if let Some(ref mut tim) = G_TIM4.borrow(cs).borrow_mut().deref_mut() {
            rprintln!("TIM4");
            tim.clear_update_interrupt_flag();
        }
    });
}

#[exception]
fn SysTick() {
    cm::interrupt::free(|_cs| {
        rprintln!("SYSTICK");
    });
}
