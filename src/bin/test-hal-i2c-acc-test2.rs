#![no_std]
#![no_main]

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
use hal::device::{TIM2, TIM3};
use hal::gpio::gpiob::PB2;
use hal::gpio::gpioe::PE7;
use hal::gpio::{OpenDrain, Output};
use hal::prelude::*;
use hal::stm32;
use hal::stm32::interrupt;
use hal::timer::Timer;

extern crate bitbang_hal;
use bitbang_hal::i2c::I2cBB;

extern crate kxcj9;
use kxcj9::ic::G8Device;
use kxcj9::{GScale8, Kxcj9, Resolution, SlaveAddr};
use kxcj9::{InterruptPinLatching, InterruptPinPolarity};
use kxcj9::{WakeUpInterruptConfig, WakeUpOutputDataRate, WakeUpTriggerMotion};

extern crate nb;
use nb::block;

use core::cell::RefCell;
use core::ops::DerefMut;

/* */

type GAccType = Kxcj9<I2cBB<PE7<Output<OpenDrain>>, PB2<Output<OpenDrain>>, Timer<TIM2>>, G8Device>;

lazy_static! {
    static ref G_EXTI: Mutex<RefCell<Option<stm32::EXTI>>> = Mutex::new(RefCell::new(None));
    static ref G_ITM: Mutex<RefCell<Option<stm32::ITM>>> = Mutex::new(RefCell::new(None));
    static ref G_TMR: Mutex<RefCell<Option<Timer<TIM3>>>> = Mutex::new(RefCell::new(None));
    static ref G_ACC: Mutex<RefCell<Option<GAccType>>> = Mutex::new(RefCell::new(None));
}

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

            // FIXME: TIM2 is used for charging, TIM3 is used for brushes
            // FIXME: need more timers in stm32f1xx_hal
            let tmr = Timer::tim2(dp.TIM2, 100.khz(), clocks, &mut rcc.apb1);
            let delay = Timer::tim3(dp.TIM3, 1.hz(), clocks, &mut rcc.apb1);

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
            dp.EXTI.ftsr.modify(|_, w| w.tr9().set_bit());

            let i2c = bitbang_hal::i2c::I2cBB::new(scl, sda, tmr);

            let address = SlaveAddr::Alternative(true);
            let mut acc = Kxcj9::new_kxcj9_1008(i2c, address);
            block!(acc.reset()).ok();

            acc.enable().unwrap();
            acc.set_scale(GScale8::G2).unwrap();
            acc.set_resolution(Resolution::Low).unwrap();

            acc.set_interrupt_pin_polarity(InterruptPinPolarity::ActiveHigh)
                .unwrap();
            acc.set_interrupt_pin_latching(InterruptPinLatching::NonLatching)
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
    cm::interrupt::free(|cs| {
        if let Some(ref mut itm) = G_ITM.borrow(cs).borrow_mut().deref_mut() {
            let d = &mut itm.stim[0];
            iprintln!(d, "Motion !");
        }

        if let Some(ref mut acc) = G_ACC.borrow(cs).borrow_mut().deref_mut() {
            let info = acc.read_interrupt_info().unwrap();
            acc.clear_interrupts().unwrap();
        }

        if let Some(exti) = G_EXTI.borrow(cs).borrow().as_ref() {
            exti.pr.modify(|_, w| w.pr9().set_bit());
        }
    });
}
