#![no_main]
#![no_std]

use cm::interrupt::Mutex;
use cm::singleton;
use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m as cm;
use cortex_m_rt::entry;
use hal::prelude::*;
use hal::stm32;
use hal::stm32::interrupt;
use hal::timer::CountDownTimer;
use hal::timer::Event;
use hal::timer::Timer;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal as hal;

type ReadBuf = &'static mut [u32; 10];

const FREQ: u32 = 40_000;
const TEST_FREQ: u32 = 1_000;

static G_TIM2: Mutex<RefCell<Option<CountDownTimer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));
static G_TIM3: Mutex<RefCell<Option<CountDownTimer<stm32::TIM3>>>> = Mutex::new(RefCell::new(None));
static G_CNT: Mutex<RefCell<Option<u32>>> = Mutex::new(RefCell::new(None));
static G_BUF: Mutex<RefCell<Option<ReadBuf>>> = Mutex::new(RefCell::new(None));
static G_IDX: Mutex<RefCell<Option<usize>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let (Some(mut cp), Some(dp)) = (cm::Peripherals::take(), stm32::Peripherals::take()) {
        cm::interrupt::free(|cs| {
            let mut rcc = dp.RCC.constrain();
            let mut flash = dp.FLASH.constrain();

            rtt_init_print!();

            let clocks = rcc
                .cfgr
                .sysclk(8.mhz())
                .pclk1(8.mhz())
                .freeze(&mut flash.acr);

            let buf = singleton!(: [u32; 10] = [0; 10]).unwrap();

            let mut tim2 = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(FREQ.hz());
            tim2.listen(Event::Update);

            let mut tim3 =
                Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1).start_count_down(TEST_FREQ.hz());
            tim3.listen(Event::Update);

            setup_interrupts(&mut cp);

            G_TIM2.borrow(cs).replace(Some(tim2));
            G_TIM3.borrow(cs).replace(Some(tim3));
            G_BUF.borrow(cs).replace(Some(buf));
            G_CNT.borrow(cs).replace(Some(0));
            G_IDX.borrow(cs).replace(Some(0));
        });
    }

    loop {
        cm::asm::nop();
    }
}

fn setup_interrupts(cp: &mut cm::peripheral::Peripherals) {
    let nvic = &mut cp.NVIC;

    unsafe {
        cm::peripheral::NVIC::unmask(stm32::Interrupt::TIM2);
        cm::peripheral::NVIC::unmask(stm32::Interrupt::TIM3);

        nvic.set_priority(stm32::Interrupt::TIM2, 1);
        nvic.set_priority(stm32::Interrupt::TIM3, 1);
    }

    cm::peripheral::NVIC::unpend(stm32::Interrupt::TIM2);
    cm::peripheral::NVIC::unpend(stm32::Interrupt::TIM3);
}

#[interrupt]
fn TIM2() {
    cm::interrupt::free(|cs| {
        if let (Some(ref mut tim), Some(ref mut cnt)) = (
            G_TIM2.borrow(cs).borrow_mut().deref_mut(),
            G_CNT.borrow(cs).borrow_mut().deref_mut(),
        ) {
            *cnt = cnt.wrapping_add(1);
            tim.clear_update_interrupt_flag();
        }
    });
}

#[interrupt]
fn TIM3() {
    cm::interrupt::free(|cs| {
        if let (Some(ref mut tim), Some(ref mut cnt), Some(ref mut idx), Some(ref mut buf)) = (
            G_TIM3.borrow(cs).borrow_mut().deref_mut(),
            G_CNT.borrow(cs).borrow_mut().deref_mut(),
            G_IDX.borrow(cs).borrow_mut().deref_mut(),
            G_BUF.borrow(cs).borrow_mut().deref_mut(),
        ) {
            if *idx < buf.len() {
                buf[*idx] = *cnt;
                *idx = *idx + 1;
            } else {
                rprintln!("ts: {:?}", buf);
                *idx = 0;
            }

            tim.clear_update_interrupt_flag();
        }
    });
}
