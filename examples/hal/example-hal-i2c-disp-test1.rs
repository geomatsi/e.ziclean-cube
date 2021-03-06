#![no_std]
#![no_main]

use bitbang_hal;
use bitbang_hal::spi::BitOrder;
use bitbang_hal::spi::MODE_3;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use nb::block;
use panic_rtt_target as _;
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

    let tmp = gpiod.pd15.into_floating_input(&mut gpiod.crh);
    let clk = gpioc.pc8.into_push_pull_output(&mut gpioc.crh);
    let dio = gpiod.pd14.into_push_pull_output(&mut gpiod.crh);
    let mut stb = gpioa.pa11.into_push_pull_output(&mut gpioa.crh);

    // Note: TIM2 is used for charging, TIM3 for brushes, TIM4 for wheels
    let timer = Timer::tim2(p.TIM2, &clocks, &mut rcc.apb1).start_count_down(200.khz());
    let mut delay = Timer::tim3(p.TIM3, &clocks, &mut rcc.apb1).start_count_down(1.hz());

    let mut spi = bitbang_hal::spi::SPI::new(MODE_3, tmp, dio, clk, timer);
    spi.set_bit_order(BitOrder::LSBFirst);

    rprintln!("start... wait 1 sec...");
    stb.set_high().unwrap();
    block!(delay.wait()).ok();

    // display mode setting: 7 grids, 11 segments
    stb.set_low().unwrap();
    block!(spi.send(0b0000_0011)).unwrap();
    stb.set_high().unwrap();

    // data setting: normal mode, fixed addr, write data to display
    stb.set_low().unwrap();
    block!(spi.send(0b0100_0100)).unwrap();
    stb.set_high().unwrap();

    // display control: display ON, PWM 13/16
    stb.set_low().unwrap();
    block!(spi.send(0b1000_1010)).unwrap();
    stb.set_high().unwrap();

    // e.ziclean display connected to TM1668 so that address byte varies
    // while data byte is fixes for each grid

    let addr = [0x0, 0x2, 0x4, 0x6, 0x8, 0xa, 0xc];
    let grid1 = 0x08;
    let grid2 = 0x80;
    let grid3 = 0x40;
    let grid4 = 0x20;

    // TODO: (addr = 6, data = 2) -> ':' symbol on display

    // segment 'bitmaps' for digits in all 4 grids

    let sym1 = [
        [1, 1, 1, 1, 1, 1, 0],
        [1, 0, 0, 0, 0, 1, 0],
        [1, 1, 0, 1, 1, 0, 1],
        [1, 1, 0, 0, 1, 1, 1],
        [1, 0, 1, 0, 0, 1, 1],
        [0, 1, 1, 0, 1, 1, 1],
        [0, 1, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 1, 0],
        [1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 0, 1, 1, 1],
    ];

    let sym2 = [
        [1, 1, 1, 1, 0, 1, 1],
        [0, 1, 0, 0, 0, 1, 0],
        [0, 1, 1, 1, 1, 0, 1],
        [0, 1, 1, 1, 1, 1, 0],
        [1, 1, 0, 0, 1, 1, 0],
        [1, 0, 1, 1, 1, 1, 0],
        [1, 0, 1, 1, 1, 1, 1],
        [0, 1, 1, 0, 0, 1, 0],
        [1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 0],
    ];

    let sym3 = [
        [1, 1, 1, 1, 1, 1, 0],
        [1, 0, 0, 0, 0, 1, 0],
        [1, 1, 0, 1, 1, 0, 1],
        [1, 1, 0, 0, 1, 1, 1],
        [1, 0, 1, 0, 0, 1, 1],
        [0, 1, 1, 0, 1, 1, 1],
        [0, 1, 1, 1, 1, 1, 1],
        [1, 1, 0, 0, 0, 1, 0],
        [1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 0, 1, 1, 1],
    ];

    let sym4 = [
        [1, 1, 1, 1, 1, 0, 1],
        [0, 1, 0, 0, 1, 0, 0],
        [0, 1, 1, 1, 0, 1, 1],
        [0, 1, 1, 1, 1, 1, 0],
        [1, 1, 0, 0, 1, 1, 0],
        [1, 0, 1, 1, 1, 1, 0],
        [1, 0, 1, 1, 1, 1, 1],
        [0, 1, 1, 0, 1, 0, 0],
        [1, 1, 1, 1, 1, 1, 1],
        [1, 1, 1, 1, 1, 1, 0],
    ];

    loop {
        for i in 0..7 {
            stb.set_low().unwrap();
            block!(spi.send(0b1100_0000 | addr[i])).unwrap();
            block!(spi.send(0b0000_0000)).unwrap();
            stb.set_high().unwrap();
        }

        rprintln!("clean");
        block!(delay.wait()).ok();

        for n in 0..10 {
            for i in 0..7 {
                stb.set_low().unwrap();
                block!(spi.send(0b1100_0000 | addr[i])).unwrap();
                let data = (grid1 * sym1[n][i])
                    | (grid2 * sym2[9 - n][i])
                    | (grid3 * sym3[9 - n][i])
                    | (grid4 * sym4[n][i]);
                block!(spi.send(data)).unwrap();
                stb.set_high().unwrap();
            }

            rprintln!("sym: {}", n);
            block!(delay.wait()).ok();
        }

        rprintln!("done");
        block!(delay.wait()).ok();
    }
}
