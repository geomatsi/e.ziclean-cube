#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
use rt::entry;
use rt::exception;
use rt::ExceptionFrame;

extern crate cortex_m as cm;
use cm::iprintln;

extern crate panic_itm;

use stm32f1xx_hal::{adc::Adc, prelude::*, stm32};

#[entry]
fn main() -> ! {
    let mut core = cm::Peripherals::take().unwrap();
    let d = &mut core.ITM.stim[0];

    let p = stm32::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);

    iprintln!(d, "SYSCLK: {} Hz ...", clocks.sysclk().0);
    iprintln!(d, "PCLK2: {} Hz ...", clocks.pclk2().0);
    iprintln!(d, "ADCCLK: {} Hz ...", clocks.adcclk().0);

    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = p.GPIOC.split(&mut rcc.apb2);
    let mut gpiod = p.GPIOD.split(&mut rcc.apb2);

    // PA0-PA7 analog input: ADC_IN0 .. ADC_IN7
    let _ch0 = gpioa.pa0.into_analog(&mut gpioa.crl);
    let _ch1 = gpioa.pa1.into_analog(&mut gpioa.crl);
    let _ch2 = gpioa.pa2.into_analog(&mut gpioa.crl);
    let _ch3 = gpioa.pa3.into_analog(&mut gpioa.crl);
    let ch4 = gpioa.pa4.into_analog(&mut gpioa.crl);
    let _ch5 = gpioa.pa5.into_analog(&mut gpioa.crl);
    let ch6 = gpioa.pa6.into_analog(&mut gpioa.crl);
    let ch7 = gpioa.pa7.into_analog(&mut gpioa.crl);

    // PB0-PB1 analog input: ADC_IN8 .. ADC_IN9
    let ch8 = gpiob.pb0.into_analog(&mut gpiob.crl);
    let ch9 = gpiob.pb1.into_analog(&mut gpiob.crl);

    // PC0-PC5 analog input: ADC_IN10 .. ADC_IN15
    let ch10 = gpioc.pc0.into_analog(&mut gpioc.crl);
    let ch11 = gpioc.pc1.into_analog(&mut gpioc.crl);
    let _ch12 = gpioc.pc2.into_analog(&mut gpioc.crl);
    let _ch13 = gpioc.pc3.into_analog(&mut gpioc.crl);
    let _ch14 = gpioc.pc4.into_analog(&mut gpioc.crl);
    let ch15 = gpioc.pc5.into_analog(&mut gpioc.crl);

    // PC7: GPIO push-pull output: enable IR LEDs of all the front sensors
    let mut front_leds = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
    front_leds.set_high();

    // PD9: push-pull output: enable IR LEDs of all 3 floor sensors
    let mut bottom_leds = gpiod.pd9.into_push_pull_output(&mut gpiod.crh);
    bottom_leds.set_high();

    // ADC setup
    let mut adc = Adc::adc1(p.ADC1, &mut rcc.apb2, clocks.adcclk());
    let max_range: u16 = adc.max_sample();

    let mut front_left_90 = ch10;
    let mut front_left_45 = ch4;
    let mut front_center = ch6;
    let mut front_right_45 = ch15;
    let mut front_right_90 = ch8;

    let mut bottom_left = ch11;
    let mut bottom_center = ch7;
    let mut bottom_right = ch9;

    iprintln!(d, "Start ADC readings...");

    loop {
        //let data: u16 = adc.read(&mut ch8).unwrap();

        let sll: u16 = adc.read(&mut front_left_90).unwrap();
        let slc: u16 = adc.read(&mut front_left_45).unwrap();
        let scc: u16 = adc.read(&mut front_center).unwrap();
        let src: u16 = adc.read(&mut front_right_45).unwrap();
        let srr: u16 = adc.read(&mut front_right_90).unwrap();

        let sl: u16 = adc.read(&mut bottom_left).unwrap();
        let sc: u16 = adc.read(&mut bottom_center).unwrap();
        let sr: u16 = adc.read(&mut bottom_right).unwrap();

        iprintln!(
            d,
            "front: ({},{},{},{},{})     bottom: ({},{},{})",
            max_range - sll,
            max_range - slc,
            max_range - scc,
            max_range - src,
            max_range - srr,
            max_range - sl,
            max_range - sc,
            max_range - sr
        );

        delay(100);
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
