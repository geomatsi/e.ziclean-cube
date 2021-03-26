#![no_main]
#![no_std]

use cortex_m as cm;
use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::timer::Timer;
use stm32f1xx_hal::{prelude::*, stm32};

#[entry]
fn main() -> ! {
    let core = cm::Peripherals::take().unwrap();
    let p = stm32::Peripherals::take().unwrap();

    rtt_init_print!();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut delay = Delay::new(core.SYST, clocks);
    let mut afio = p.AFIO.constrain(&mut rcc.apb2);
    let gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);

    // Use this to configure NJTRST as PB4
    let (_pa15, _pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    // TIM3: CH1 (pump), CH2 (all 3 brushes)

    let p1 = pb4.into_alternate_push_pull(&mut gpiob.crl);
    let p2 = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);

    let (mut pump, mut brushes) =
        Timer::tim3(p.TIM3, &clocks, &mut rcc.apb1).pwm((p1, p2), &mut afio.mapr, 10.khz());

    let max = pump.get_max_duty();

    rprintln!("pwm max duty: {}...", max);

    pump.enable();
    brushes.enable();

    let duty: [u16; 9] = [
        0,
        max * 1 / 2,
        max * 2 / 3,
        max * 3 / 4,
        max * 4 / 5,
        max * 5 / 6,
        max * 6 / 7,
        max * 7 / 8,
        max,
    ];

    rprintln!("lets rock...");

    loop {
        for i in 0..duty.len() {
            rprintln!("duty: {}", duty[i]);
            pump.set_duty(duty[i]);
            brushes.set_duty(duty[i]);
            delay.delay_ms(5_000_u16)
        }
    }
}
