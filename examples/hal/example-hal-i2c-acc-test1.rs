#![no_std]
#![no_main]

use cortex_m_rt::entry;

extern crate cortex_m as cm;
use cm::iprintln;

use panic_itm as _;

use bitbang_hal;

use stm32f1xx_hal::timer::Timer;
use stm32f1xx_hal::{prelude::*, stm32};

extern crate kxcj9;
use kxcj9::{GScale8, Kxcj9, Resolution, SlaveAddr};

#[entry]
fn main() -> ! {
    let mut core = cm::Peripherals::take().unwrap();
    let dbg = &mut core.ITM.stim[0];

    let p = stm32::Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let mut flash = p.FLASH.constrain();

    let clocks = rcc
        .cfgr
        .sysclk(8.mhz())
        .pclk1(8.mhz())
        .freeze(&mut flash.acr);

    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    let mut gpioe = p.GPIOE.split(&mut rcc.apb2);

    let scl = gpioe.pe7.into_open_drain_output(&mut gpioe.crl);
    let sda = gpiob.pb2.into_open_drain_output(&mut gpiob.crl);
    gpioe.pe9.into_floating_input(&mut gpioe.crh);

    // TIM2 is used for charging, TIM3 for brushes, TIM4 for wheels
    let tmr = Timer::tim2(p.TIM2, &clocks, &mut rcc.apb1).start_count_down(100.hz());

    let i2c = bitbang_hal::i2c::I2cBB::new(scl, sda, tmr);

    let address = SlaveAddr::Alternative(true);
    let mut acc = Kxcj9::new_kxcj9_1008(i2c, address);

    let model = match acc.who_am_i().unwrap() {
        0x0a => "kxcj9-1008",
        0x1d => "kxcj9-1018",
        0x21 => "kxcj9-1041",
        _ => "unkonwn",
    };

    iprintln!(dbg, "KXCJ9 model: {}", model);

    acc.enable().unwrap();
    acc.set_scale(GScale8::G2).unwrap();
    acc.set_resolution(Resolution::High).unwrap();

    loop {
        let mut xa: f32 = 0.0;
        let mut ya: f32 = 0.0;
        let mut za: f32 = 0.0;

        for _ in 0..10 {
            let data = acc.read().unwrap();
            xa += data.x;
            ya += data.y;
            za += data.z;
        }

        iprintln!(
            dbg,
            " X {:>8.3}   Y {:>8.3}   Z{:>8.3}",
            xa / 10.0,
            ya / 10.0,
            za / 10.0
        );
    }
}
