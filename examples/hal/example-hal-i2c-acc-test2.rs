#![no_std]
#![no_main]

use cortex_m_rt::entry;

use cm::interrupt::Mutex;
use cm::iprintln;
use cortex_m as cm;

use panic_itm as _;

use hal::device::{TIM2, TIM3};
use hal::gpio::gpiob::PB2;
use hal::gpio::gpioe::PE7;
use hal::gpio::{OpenDrain, Output};
use hal::prelude::*;
use hal::stm32;
use hal::stm32::interrupt;
use hal::timer::CountDownTimer;
use hal::timer::Timer;
use stm32f1xx_hal as hal;

use bitbang_hal;
use bitbang_hal::i2c::I2cBB;

use kxcj9;
use kxcj9::ic::G8Device;
use kxcj9::{GScale8, Kxcj9, Resolution, SlaveAddr};
use kxcj9::{InterruptPinLatching, InterruptPinPolarity};
use kxcj9::{WakeUpInterruptConfig, WakeUpOutputDataRate, WakeUpTriggerMotion};

use nb::block;

use core::cell::RefCell;
use core::ops::DerefMut;

/* */

type GAccType = Kxcj9<I2cBB<PE7<Output<OpenDrain>>, PB2<Output<OpenDrain>>, CountDownTimer<TIM2>>, G8Device>;

static G_EXTI: Mutex<RefCell<Option<stm32::EXTI>>> = Mutex::new(RefCell::new(None));
static G_ITM: Mutex<RefCell<Option<stm32::ITM>>> = Mutex::new(RefCell::new(None));
static G_TMR: Mutex<RefCell<Option<CountDownTimer<TIM3>>>> = Mutex::new(RefCell::new(None));
static G_ACC: Mutex<RefCell<Option<GAccType>>> = Mutex::new(RefCell::new(None));

/* */

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

            setup_interrupts(&mut cp);

            let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
            let mut gpioe = dp.GPIOE.split(&mut rcc.apb2);

            // NOTE: in real firmware TIM2 is used for charging, TIM3 is used for brushes
            let tmr = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).start_count_down(100.khz());
            let delay = Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1).start_count_down(1.hz());

            let scl = gpioe.pe7.into_open_drain_output(&mut gpioe.crl);
            let sda = gpiob.pb2.into_open_drain_output(&mut gpiob.crl);
            let _irq = gpioe.pe9.into_floating_input(&mut gpioe.crh);

            let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

            // select PE9 as source input for line EXTI9
            afio.exticr3
                .exticr3()
                .modify(|_, w| unsafe { w.exti9().bits(0b0100) });

            // enable EXTI9 and configure interrupt on rising edge
            dp.EXTI.imr.modify(|_, w| w.mr9().set_bit());
            dp.EXTI.rtsr.modify(|_, w| w.tr9().set_bit());

            let i2c = bitbang_hal::i2c::I2cBB::new(scl, sda, tmr);

            let address = SlaveAddr::Alternative(true);
            let mut acc = Kxcj9::new_kxcj9_1008(i2c, address);
            block!(acc.reset()).ok();

            acc.enable().unwrap();
            acc.set_scale(GScale8::G2).unwrap();
            acc.set_resolution(Resolution::Low).unwrap();

            acc.set_interrupt_pin_polarity(InterruptPinPolarity::ActiveHigh)
                .unwrap();
            acc.set_interrupt_pin_latching(InterruptPinLatching::Latching)
                .unwrap();
            acc.enable_interrupt_pin().unwrap();

            let config = WakeUpInterruptConfig {
                trigger_motion: WakeUpTriggerMotion {
                    x_negative: true,
                    x_positive: true,
                    y_negative: true,
                    y_positive: true,
                    z_negative: true,
                    z_positive: true,
                },
                data_rate: WakeUpOutputDataRate::Hz25,
                fault_count: 1,
                threshold: 0.5,
            };

            acc.enable_wake_up_interrupt(config).unwrap();

            G_EXTI.borrow(cs).replace(Some(dp.EXTI));
            G_ITM.borrow(cs).replace(Some(cp.ITM));
            G_TMR.borrow(cs).replace(Some(delay));
            G_ACC.borrow(cs).replace(Some(acc));
        });
    }

    loop {
        cm::interrupt::free(|cs| {
            if let Some(ref mut itm) = G_ITM.borrow(cs).borrow_mut().deref_mut() {
                let d = &mut itm.stim[0];
                iprintln!(d, "idle loop");
            }

            if let Some(ref mut delay) = G_TMR.borrow(cs).borrow_mut().deref_mut() {
                block!(delay.wait()).ok();
            }
        });
    }
}

fn setup_interrupts(cp: &mut cm::peripheral::Peripherals) {
    let nvic = &mut cp.NVIC;

    // Enable EXTI9_5, set prio 1, clear any pending IRQs
    nvic.enable(stm32::Interrupt::EXTI9_5);

    unsafe {
        nvic.set_priority(stm32::Interrupt::EXTI9_5, 1);
    }

    cm::peripheral::NVIC::unpend(stm32::Interrupt::EXTI9_5);
}

#[interrupt]
fn EXTI9_5() {
    cm::interrupt::free(|cs| {
        if let (Some(ref mut acc), Some(ref mut itm), Some(exti)) = (
            G_ACC.borrow(cs).borrow_mut().deref_mut(),
            G_ITM.borrow(cs).borrow_mut().deref_mut(),
            G_EXTI.borrow(cs).borrow().as_ref(),
        ) {
            let d = &mut itm.stim[0];
            let info = acc.read_interrupt_info().unwrap();

            iprintln!(d, "MOTION: {:?}", info);

            acc.clear_interrupts().unwrap();
            exti.pr.modify(|_, w| w.pr9().set_bit());
        }
    });
}
