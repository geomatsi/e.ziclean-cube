#![no_main]
#![no_std]

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

use cm::interrupt::Mutex;
use cm::iprintln;
use cm::peripheral::itm::Stim;
use cortex_m as cm;

use panic_itm as _;

use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m_rt::entry;

use hal::gpio::gpioc::PC11;
use hal::gpio::gpiod::PD11;
use hal::gpio::gpiod::PD15;
use hal::gpio::gpioe::PE10;
use hal::gpio::Floating;
use hal::gpio::Input;

use hal::prelude::*;
use hal::stm32;
use hal::stm32::interrupt;
use hal::timer::CountDownTimer;
use hal::timer::Event;
use hal::timer::Timer;
use stm32f1xx_hal as hal;

use infrared::nec::*;
use infrared::rc5::*;
use infrared::rc6::*;
use infrared::Receiver;
use infrared::ReceiverState;

const FREQ: u32 = 40_000;

static G_TIM: Mutex<RefCell<Option<CountDownTimer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));
static G_ITM: Mutex<RefCell<Option<stm32::ITM>>> = Mutex::new(RefCell::new(None));

static G_NES: Mutex<RefCell<Option<NecSamsungReceiver>>> = Mutex::new(RefCell::new(None));
static G_NEC: Mutex<RefCell<Option<NecReceiver>>> = Mutex::new(RefCell::new(None));
static G_RC5: Mutex<RefCell<Option<Rc5Receiver>>> = Mutex::new(RefCell::new(None));
static G_RC6: Mutex<RefCell<Option<Rc6Receiver>>> = Mutex::new(RefCell::new(None));

static G_IRT: Mutex<RefCell<Option<PD15<Input<Floating>>>>> = Mutex::new(RefCell::new(None));
static G_IRL: Mutex<RefCell<Option<PC11<Input<Floating>>>>> = Mutex::new(RefCell::new(None));
static G_IRF: Mutex<RefCell<Option<PD11<Input<Floating>>>>> = Mutex::new(RefCell::new(None));
static G_IRR: Mutex<RefCell<Option<PE10<Input<Floating>>>>> = Mutex::new(RefCell::new(None));

static G_PIN: Mutex<RefCell<Option<bool>>> = Mutex::new(RefCell::new(None));
static G_CNT: Mutex<RefCell<Option<u32>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let (Some(mut cp), Some(dp)) = (cm::Peripherals::take(), stm32::Peripherals::take()) {
        cm::interrupt::free(|cs| {
            let mut rcc = dp.RCC.constrain();
            let mut flash = dp.FLASH.constrain();

            let clocks = rcc
                .cfgr
                .sysclk(8.mhz())
                .pclk1(8.mhz())
                .freeze(&mut flash.acr);

            let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
            let mut gpiod = dp.GPIOD.split(&mut rcc.apb2);
            let mut gpioe = dp.GPIOE.split(&mut rcc.apb2);

            // top IR RC diode
            let irt = gpiod.pd15.into_floating_input(&mut gpiod.crh);

            // left IR RC diode
            let irl = gpioc.pc11.into_floating_input(&mut gpioc.crh);

            // front IR RC diode
            let irf = gpiod.pd11.into_floating_input(&mut gpiod.crh);

            // right IR RC diode
            let irr = gpioe.pe10.into_floating_input(&mut gpioe.crh);

            // PWR for left/front/right IR RC diodes: PE12 is connected to Base of Q7 via R22
            // ?????
            // In fact diodes are powered regardless of PE12 level.
            // Reconstructed schematics looks reasonable, so not yet sure what is wrong...
            gpioe
                .pe12
                .into_open_drain_output(&mut gpioe.crh)
                .set_high()
                .unwrap();

            let mut tim2 = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(FREQ.hz());
            tim2.listen(Event::Update);

            setup_interrupts(&mut cp);

            let nes = NecSamsungReceiver::new(FREQ);
            let nec = NecReceiver::new(FREQ);
            let rc5 = Rc5Receiver::new(FREQ);
            let rc6 = Rc6Receiver::new(FREQ);

            G_ITM.borrow(cs).replace(Some(cp.ITM));
            G_TIM.borrow(cs).replace(Some(tim2));

            G_NES.borrow(cs).replace(Some(nes));
            G_NEC.borrow(cs).replace(Some(nec));
            G_RC5.borrow(cs).replace(Some(rc5));
            G_RC6.borrow(cs).replace(Some(rc6));

            G_IRT.borrow(cs).replace(Some(irt));
            G_IRL.borrow(cs).replace(Some(irl));
            G_IRF.borrow(cs).replace(Some(irf));
            G_IRR.borrow(cs).replace(Some(irr));

            G_PIN.borrow(cs).replace(Some(false));
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
            Some(ref mut nes),
            Some(ref mut nec),
            Some(ref mut rc5),
            Some(ref mut rc6),
            Some(ref mut val),
            Some(ref mut cnt),
        ) = (
            G_ITM.borrow(cs).borrow_mut().deref_mut(),
            G_TIM.borrow(cs).borrow_mut().deref_mut(),
            G_IRR.borrow(cs).borrow_mut().deref_mut(),
            G_NES.borrow(cs).borrow_mut().deref_mut(),
            G_NEC.borrow(cs).borrow_mut().deref_mut(),
            G_RC5.borrow(cs).borrow_mut().deref_mut(),
            G_RC6.borrow(cs).borrow_mut().deref_mut(),
            G_PIN.borrow(cs).borrow_mut().deref_mut(),
            G_CNT.borrow(cs).borrow_mut().deref_mut(),
        ) {
            let d = &mut itm.stim[0];
            let new_val = pin.is_low().unwrap();

            if *val != new_val {
                let rising = new_val;

                if let Some(cmd) = sample_on_edge(nec, rising, *cnt, d) {
                    iprintln!(d, "{:?}", cmd);
                    nec.reset();
                }

                if let Some(cmd) = sample_on_edge(nes, rising, *cnt, d) {
                    iprintln!(d, "{:?}", cmd);
                    nes.reset();
                }

                if let Some(cmd) = sample_on_edge(rc5, rising, *cnt, d) {
                    iprintln!(d, "{:?}", cmd);
                    rc5.reset();
                }

                if let Some(cmd) = sample_on_edge(rc6, rising, *cnt, d) {
                    iprintln!(d, "{:?}", cmd);
                    rc6.reset();
                }
            }

            *cnt = cnt.wrapping_add(1);
            *val = new_val;

            tim.clear_update_interrupt_flag();
        }
    });
}

fn sample_on_edge<CMD, ERR>(
    recv: &mut dyn Receiver<Cmd = CMD, Err = ERR>,
    edge: bool,
    t: u32,
    _d: &mut Stim,
) -> Option<CMD> {
    match recv.sample_edge(edge, t) {
        ReceiverState::Idle => {
            return None;
        }
        ReceiverState::Receiving => {
            return None;
        }
        ReceiverState::Disabled => {
            return None;
        }
        ReceiverState::Done(c) => {
            return Some(c);
        }
        ReceiverState::Error(_err) => {
            recv.reset();
            return None;
        }
    }
}
