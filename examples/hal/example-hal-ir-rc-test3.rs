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

type ExtIntr = stm32::EXTI;
type ReadBuf = &'static mut [u32; 10];

const FREQ: u32 = 100_000;

static G_TIM: Mutex<RefCell<Option<CountDownTimer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));
static G_EXTI: Mutex<RefCell<Option<ExtIntr>>> = Mutex::new(RefCell::new(None));
static G_BUF: Mutex<RefCell<Option<ReadBuf>>> = Mutex::new(RefCell::new(None));
static G_IDX: Mutex<RefCell<Option<usize>>> = Mutex::new(RefCell::new(None));
static G_CNT: Mutex<RefCell<Option<u32>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let (Some(mut cp), Some(dp)) = (cm::Peripherals::take(), stm32::Peripherals::take()) {
        cm::interrupt::free(|cs| {
            rtt_init_print!();

            let mut rcc = dp.RCC.constrain();
            let mut flash = dp.FLASH.constrain();
            let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

            let clocks = rcc
                .cfgr
                .sysclk(8.mhz())
                .pclk1(8.mhz())
                .freeze(&mut flash.acr);

            // top IR RC diode
            let mut gpiod = dp.GPIOD.split(&mut rcc.apb2);
            gpiod.pd15.into_floating_input(&mut gpiod.crh);

            // PD15 as input
            afio.exticr4
                .exticr4()
                .modify(|_, w| unsafe { w.exti15().bits(0b0011) });

            // unmask EXTI15 for PD15
            dp.EXTI.imr.modify(|_, w| w.mr15().set_bit());

            // enable configure edge triggers for PD15
            dp.EXTI.ftsr.modify(|_, w| w.tr15().set_bit());
            dp.EXTI.rtsr.modify(|_, w| w.tr15().clear_bit());

            // timestamps buffer

            let buf = singleton!(: [u32; 10] = [0; 10]).unwrap();

            // timers

            let mut tim2 = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(FREQ.hz());
            tim2.listen(Event::Update);

            setup_interrupts(&mut cp);

            G_EXTI.borrow(cs).replace(Some(dp.EXTI));
            G_TIM.borrow(cs).replace(Some(tim2));
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
        cm::peripheral::NVIC::unmask(stm32::Interrupt::EXTI15_10);
        cm::peripheral::NVIC::unmask(stm32::Interrupt::TIM2);

        nvic.set_priority(stm32::Interrupt::EXTI15_10, 1);
        nvic.set_priority(stm32::Interrupt::TIM2, 1);
    }

    cm::peripheral::NVIC::unpend(stm32::Interrupt::EXTI15_10);
    cm::peripheral::NVIC::unpend(stm32::Interrupt::TIM2);
}

#[interrupt]
fn EXTI15_10() {
    cm::interrupt::free(|cs| {
        if let (Some(exti), Some(cnt), Some(ref mut idx), Some(ref mut buf)) = (
            G_EXTI.borrow(cs).borrow().as_ref(),
            G_CNT.borrow(cs).borrow().as_ref(),
            G_IDX.borrow(cs).borrow_mut().deref_mut(),
            G_BUF.borrow(cs).borrow_mut().deref_mut(),
        ) {
            let reg = exti.pr.read();

            if reg.pr15().bit_is_set() {
                exti.pr.modify(|_, w| w.pr10().set_bit());

                if *idx < buf.len() {
                    buf[*idx] = *cnt;
                    *idx = *idx + 1;
                } else {
                    rprintln!("ts: {:?}", buf);
                    *idx = 0;
                }
            }
        }
    });
}

#[interrupt]
fn TIM2() {
    cm::interrupt::free(|cs| {
        if let (Some(ref mut tim), Some(ref mut cnt)) = (
            G_TIM.borrow(cs).borrow_mut().deref_mut(),
            G_CNT.borrow(cs).borrow_mut().deref_mut(),
        ) {
            *cnt = cnt.wrapping_add(1);
            tim.clear_update_interrupt_flag();
        }
    });
}
