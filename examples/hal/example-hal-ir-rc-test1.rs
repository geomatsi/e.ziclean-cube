#![no_main]
#![no_std]

use embedded_hal::digital::v2::OutputPin;

use cortex_m_rt::entry;

use cm::interrupt::Mutex;
use cm::iprintln;
use cortex_m as cm;

use panic_itm as _;

use hal::prelude::*;
use hal::stm32;
use hal::stm32::interrupt;
use stm32f1xx_hal as hal;

use core::cell::RefCell;
use core::ops::DerefMut;

type ExtIntr = stm32::EXTI;
type DbgPort = stm32::ITM;

static G_EXTI: Mutex<RefCell<Option<ExtIntr>>> = Mutex::new(RefCell::new(None));
static G_ITM: Mutex<RefCell<Option<DbgPort>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let (Some(mut cp), Some(dp)) = (cm::Peripherals::take(), stm32::Peripherals::take()) {
        cm::interrupt::free(|cs| {
            let mut rcc = dp.RCC.constrain();
            let mut flash = dp.FLASH.constrain();

            let _clocks = rcc
                .cfgr
                .sysclk(8.mhz())
                .pclk1(8.mhz())
                .freeze(&mut flash.acr);

            setup_interrupts(&mut cp);

            let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
            let mut gpiod = dp.GPIOD.split(&mut rcc.apb2);
            let mut gpioe = dp.GPIOE.split(&mut rcc.apb2);

            // top IR RC diode
            gpiod.pd15.into_floating_input(&mut gpiod.crh);

            // left IR RC diode
            gpioc.pc11.into_floating_input(&mut gpioc.crh);

            // front IR RC diode
            gpiod.pd11.into_floating_input(&mut gpiod.crh);

            // right IR RC diode
            gpioe.pe10.into_floating_input(&mut gpioe.crh);

            // PWR for left/front/right IR RC diodes: PE12 is connected to Base of Q7 via R22
            // ?????
            // In fact diodes are powered regardless of PE12 level.
            // Reconstructed schematics looks reasonable, so not yet sure what is wrong...
            gpioe
                .pe12
                .into_open_drain_output(&mut gpioe.crh)
                .set_high()
                .unwrap();

            let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

            // PE10 as input
            afio.exticr3
                .exticr3()
                .modify(|_, w| unsafe { w.exti10().bits(0b0100) });

            // ???????
            // The choice of h/w lines for IR RC diodes  does not look right:
            // it is not possible to use both PC11 and PD11 as a sources for EXTI11 at the same time
            // In this test PC11 is selected as a source...
            afio.exticr3
                .exticr3()
                .modify(|_, w| unsafe { w.exti11().bits(0b0010) });

            // PD15 as input
            afio.exticr4
                .exticr4()
                .modify(|_, w| unsafe { w.exti15().bits(0b0011) });

            // unmask EXIT10, EXTI11, EXTI15
            dp.EXTI.imr.modify(|_, w| w.mr10().set_bit());
            dp.EXTI.imr.modify(|_, w| w.mr11().set_bit());
            dp.EXTI.imr.modify(|_, w| w.mr15().set_bit());

            // enable both rising and falling edge triggers for EXIT10, EXTI11, EXTI15
            dp.EXTI.rtsr.modify(|_, w| w.tr10().set_bit());
            //dp.EXTI.ftsr.modify(|_, w| w.tr10().set_bit());
            dp.EXTI.rtsr.modify(|_, w| w.tr11().set_bit());
            //dp.EXTI.ftsr.modify(|_, w| w.tr11().set_bit());
            dp.EXTI.rtsr.modify(|_, w| w.tr15().set_bit());
            //dp.EXTI.ftsr.modify(|_, w| w.tr15().set_bit());

            G_EXTI.borrow(cs).replace(Some(dp.EXTI));
            G_ITM.borrow(cs).replace(Some(cp.ITM));
        });
    }

    loop {
        cm::interrupt::free(|cs| {
            if let Some(ref mut itm) = G_ITM.borrow(cs).borrow_mut().deref_mut() {
                let d = &mut itm.stim[0];
                iprintln!(d, "idle loop");
            }
        });

        delay(10000);
    }
}

fn delay(count: u32) {
    for _ in 0..count {
        cm::asm::nop();
    }
}

fn setup_interrupts(cp: &mut cm::peripheral::Peripherals) {
    let nvic = &mut cp.NVIC;

    nvic.enable(stm32::Interrupt::EXTI15_10);

    unsafe {
        nvic.set_priority(stm32::Interrupt::EXTI15_10, 1);
    }

    cm::peripheral::NVIC::unpend(stm32::Interrupt::EXTI15_10);
}

#[interrupt]
fn EXTI15_10() {
    cm::interrupt::free(|cs| {
        if let (Some(ref mut itm), Some(exti)) = (
            G_ITM.borrow(cs).borrow_mut().deref_mut(),
            G_EXTI.borrow(cs).borrow().as_ref(),
        ) {
            let d = &mut itm.stim[0];
            let mut t = 0;
            let mut l = 0;
            let mut r = 0;
            let f = 0;

            let reg = exti.pr.read();

            if reg.pr10().bit_is_set() {
                exti.pr.modify(|_, w| w.pr10().set_bit());
                r = 1;
            }

            if reg.pr11().bit_is_set() {
                exti.pr.modify(|_, w| w.pr11().set_bit());
                //f = 1
                l = 1;
            }

            if reg.pr15().bit_is_set() {
                exti.pr.modify(|_, w| w.pr15().set_bit());
                t = 1;
            }

            iprintln!(d, "IR: top[{}] left[{}] front[{}] right[{}]", t, l, f, r);
        }
    });
}
