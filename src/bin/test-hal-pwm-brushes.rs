#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
use rt::entry;
use rt::exception;
use rt::ExceptionFrame;

extern crate cortex_m as cm;
use cm::iprintln;

extern crate panic_itm;

use stm32f1xx_hal::{prelude::*, stm32};

use stm32f1xx_hal::gpio::gpiob::{PB4, PB5};
use stm32f1xx_hal::gpio::{Alternate, PushPull};
use stm32f1xx_hal::pwm::{Pins, Pwm, C1, C2};
use stm32f1xx_hal::stm32::TIM3;

struct Brushes(PB5<Alternate<PushPull>>);

impl Pins<TIM3> for Brushes {
    const REMAP: u8 = 0b10;
    const C1: bool = false;
    const C2: bool = true;
    const C3: bool = false;
    const C4: bool = false;
    type Channels = Pwm<TIM3, C2>;
}

#[entry]
fn main() -> ! {
    let mut core = cm::Peripherals::take().unwrap();
    let dbg = &mut core.ITM.stim[0];

    let p = stm32::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);

    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

    // TIM3: CH1 (not used), CH2 (all 3 brushes)

    let pin = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);

    let mut pwm = p.TIM3.pwm(
        Brushes(pin),
        &mut afio.mapr,
        10.khz(),
        clocks,
        &mut rcc.apb1,
    );

    let max = pwm.get_max_duty();

    iprintln!(dbg, "pwm max duty: {}...", max);

    pwm.enable();

    let duty: [u16; 7] = [
        0,
        max * 1 / 2,
        max * 2 / 3,
        max * 3 / 4,
        max * 4 / 5,
        max * 5 / 6,
        max,
    ];

    iprintln!(dbg, "lets rock...");

    loop {
        for &s in duty.iter() {
            iprintln!(dbg, "duty: {}", s);
            pwm.set_duty(s);
            delay(20000);
        }
    }
}

fn delay(count: u32) {
    for _ in 0..count {
        cm::asm::nop();
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
