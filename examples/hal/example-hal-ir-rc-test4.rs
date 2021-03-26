#![no_main]
#![no_std]

use cm::interrupt::Mutex;
use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m as cm;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::InputPin;
use eziclean::hw::ir_rc::IRReceiver;
use eziclean::hw::ir_rc::ReceiverResult;
use hal::gpio::gpiod::PD15;
use hal::gpio::Floating;
use hal::gpio::Input;
use hal::prelude::*;
use hal::stm32;
use hal::stm32::interrupt;
use hal::timer::CountDownTimer;
use hal::timer::Event;
use hal::timer::Timer;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal as hal;

const FREQ: u32 = 20000;

static G_TIM: Mutex<RefCell<Option<CountDownTimer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));
static G_DEC: Mutex<RefCell<Option<IRReceiver>>> = Mutex::new(RefCell::new(None));
static G_IRR: Mutex<RefCell<Option<PD15<Input<Floating>>>>> = Mutex::new(RefCell::new(None));
static G_CNT: Mutex<RefCell<Option<u32>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let (Some(mut cp), Some(dp)) = (cm::Peripherals::take(), stm32::Peripherals::take()) {
        cm::interrupt::free(|cs| {
            rtt_init_print!();

            let mut rcc = dp.RCC.constrain();
            let mut flash = dp.FLASH.constrain();
            let clocks = rcc
                .cfgr
                .sysclk(8.mhz())
                .pclk1(8.mhz())
                .freeze(&mut flash.acr);

            rprintln!("SCLK: {} Hz ...", clocks.sysclk().0);
            rprintln!("PCLK: {} Hz ...", clocks.pclk1().0);

            // top IR diode: signal connected to PD15
            let mut gpiod = dp.GPIOD.split(&mut rcc.apb2);
            let irr = gpiod.pd15.into_floating_input(&mut gpiod.crh);

            setup_interrupts(&mut cp);
            let mut tmr = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(FREQ.hz());
            tmr.listen(Event::Update);

            let dec = IRReceiver::new(FREQ);

            G_DEC.borrow(cs).replace(Some(dec));
            G_TIM.borrow(cs).replace(Some(tmr));
            G_IRR.borrow(cs).replace(Some(irr));
            G_CNT.borrow(cs).replace(Some(0));
        });
    }

    loop {
        cm::asm::nop();
    }
}

fn setup_interrupts(cp: &mut cm::peripheral::Peripherals) {
    let nvic = &mut cp.NVIC;

    unsafe {
        nvic.set_priority(stm32::Interrupt::TIM2, 1);
        cm::peripheral::NVIC::unmask(stm32::Interrupt::TIM2);
    }

    cm::peripheral::NVIC::unpend(stm32::Interrupt::TIM2);
}

#[interrupt]
fn TIM2() {
    cm::interrupt::free(|cs| {
        if let (Some(ref mut tim), Some(ref mut pin), Some(ref mut dec), Some(ref mut cnt)) = (
            G_TIM.borrow(cs).borrow_mut().deref_mut(),
            G_IRR.borrow(cs).borrow_mut().deref_mut(),
            G_DEC.borrow(cs).borrow_mut().deref_mut(),
            G_CNT.borrow(cs).borrow_mut().deref_mut(),
        ) {
            let val = pin.is_high().unwrap();

            tim.clear_update_interrupt_flag();

            match dec.sample(val, *cnt) {
                ReceiverResult::Done(v) => {
                    rprintln!("result 0x{:x}", v);
                    dec.reset();
                }
                ReceiverResult::Fail(e) => {
                    rprintln!("error {}", e);
                    dec.reset();
                }
                ReceiverResult::Proc => {}
            }

            *cnt = cnt.wrapping_add(1);
        }
    });
}
