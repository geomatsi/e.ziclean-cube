#![no_main]
#![no_std]

use embedded_hal::digital::v2::InputPin;

use cm::interrupt::Mutex;
use cm::iprintln;
use cortex_m as cm;

use panic_itm as _;

use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m_rt::entry;

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
use stm32f1xx_hal as hal;

const FREQ: u32 = 20000;

type DbgPort = stm32::ITM;

static G_TIM: Mutex<RefCell<Option<CountDownTimer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));
static G_DEC: Mutex<RefCell<Option<IRReceiver>>> = Mutex::new(RefCell::new(None));
static G_IRR: Mutex<RefCell<Option<PD15<Input<Floating>>>>> = Mutex::new(RefCell::new(None));
static G_CNT: Mutex<RefCell<Option<u32>>> = Mutex::new(RefCell::new(None));
static G_ITM: Mutex<RefCell<Option<DbgPort>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let (Some(mut cp), Some(dp)) = (cm::Peripherals::take(), stm32::Peripherals::take()) {
        cm::interrupt::free(|cs| {
            let dbg = &mut cp.ITM.stim[0];
            let mut rcc = dp.RCC.constrain();
            let mut flash = dp.FLASH.constrain();
            let clocks = rcc
                .cfgr
                .sysclk(8.mhz())
                .pclk1(8.mhz())
                .freeze(&mut flash.acr);

            iprintln!(dbg, "SCLK: {} Hz ...", clocks.sysclk().0);
            iprintln!(dbg, "PCLK: {} Hz ...", clocks.pclk1().0);

            // top IR diode: signal connected to PD15
            let mut gpiod = dp.GPIOD.split(&mut rcc.apb2);
            let irr = gpiod.pd15.into_floating_input(&mut gpiod.crh);

            setup_interrupts(&mut cp);
            let mut tmr = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(FREQ.hz());
            tmr.listen(Event::Update);

            let dec = IRReceiver::new(FREQ);

            G_ITM.borrow(cs).replace(Some(cp.ITM));
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
        if let (
            Some(ref mut itm),
            Some(ref mut tim),
            Some(ref mut pin),
            Some(ref mut dec),
            Some(ref mut cnt),
        ) = (
            G_ITM.borrow(cs).borrow_mut().deref_mut(),
            G_TIM.borrow(cs).borrow_mut().deref_mut(),
            G_IRR.borrow(cs).borrow_mut().deref_mut(),
            G_DEC.borrow(cs).borrow_mut().deref_mut(),
            G_CNT.borrow(cs).borrow_mut().deref_mut(),
        ) {
            let dbg = &mut itm.stim[0];
            let val = pin.is_high().unwrap();

            tim.clear_update_interrupt_flag();

            match dec.sample(val, *cnt) {
                ReceiverResult::Done(v) => {
                    iprintln!(dbg, "result 0x{:x}", v);
                    dec.reset();
                }
                ReceiverResult::Fail(e) => {
                    iprintln!(dbg, "error {}", e);
                    dec.reset();
                }
                ReceiverResult::Proc => {}
            }

            *cnt = cnt.wrapping_add(1);
        }
    });
}
