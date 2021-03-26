#![no_std]
#![no_main]

use bitbang_hal as bb;
use cortex_m_rt as rt;
use eziclean::hw::display;
use nb::block;
use panic_rtt_target as _;
use rt::entry;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::timer::Timer;
use stm32f1xx_hal::{prelude::*, stm32};

#[entry]
fn main() -> ! {
    let p = stm32::Peripherals::take().unwrap();

    rtt_init_print!();

    let mut rcc = p.RCC.constrain();
    let mut flash = p.FLASH.constrain();

    let clocks = rcc
        .cfgr
        .sysclk(8.mhz())
        .pclk1(8.mhz())
        .freeze(&mut flash.acr);

    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpioc = p.GPIOC.split(&mut rcc.apb2);
    let mut gpiod = p.GPIOD.split(&mut rcc.apb2);
    let mut gpioe = p.GPIOE.split(&mut rcc.apb2);

    let tmp = gpioe.pe15.into_floating_input(&mut gpioe.crh);
    let clk = gpioc.pc8.into_push_pull_output(&mut gpioc.crh);
    let dio = gpiod.pd14.into_push_pull_output(&mut gpiod.crh);
    let stb = gpioa.pa11.into_push_pull_output(&mut gpioa.crh);

    let timer = Timer::tim2(p.TIM2, &clocks, &mut rcc.apb1).start_count_down(200.khz());
    let mut delay = Timer::tim3(p.TIM3, &clocks, &mut rcc.apb1).start_count_down(2.hz());

    let mut spi = bb::spi::SPI::new(bb::spi::MODE_3, tmp, dio, clk, timer);
    spi.set_bit_order(bb::spi::BitOrder::LSBFirst);

    let mut screen = display::Display::new(spi, stb);
    screen.enable().unwrap();

    loop {
        rprintln!("clean");
        screen.clear().unwrap();
        block!(delay.wait()).ok();

        rprintln!("test #1");

        for i in 0..10 {
            screen.print4([i, i, i, i]).unwrap();
            block!(delay.wait()).ok();
        }

        rprintln!("test #2");

        for i in 0..10 {
            for p in 0..4 {
                screen.print(p, i).unwrap();
                block!(delay.wait()).ok();
            }
        }

        rprintln!("test #3");

        for i in 10..20 {
            for k in [true, false, true, false].iter() {
                screen.print_time([i, 30 - i], *k).unwrap();
                block!(delay.wait()).ok();
            }
        }
    }
}
