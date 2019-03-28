#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
use rt::entry;
use rt::exception;
use rt::ExceptionFrame;

extern crate cortex_m as cm;
extern crate panic_semihosting;

extern crate cortex_m_semihosting;
use cortex_m_semihosting::hprintln;

use stm32f1xx_hal::{adc::Adc, prelude::*, stm32};

#[entry]
fn main() -> ! {
    let p = stm32::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);

    hprintln!("SYSCLK: {} Hz ...", clocks.sysclk().0).unwrap();
    hprintln!("PCLK2: {} Hz ...", clocks.pclk2().0).unwrap();
    hprintln!("ADCCLK: {} Hz ...", clocks.adcclk().0).unwrap();

    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = p.GPIOC.split(&mut rcc.apb2);
    let mut gpiod = p.GPIOD.split(&mut rcc.apb2);

    // PA0-PA7 analog input: ADC_IN0 .. ADC_IN7
    let mut _ch0 = gpioa.pa0.into_analog(&mut gpioa.crl);
    let mut _ch1 = gpioa.pa1.into_analog(&mut gpioa.crl);
    let mut _ch2 = gpioa.pa2.into_analog(&mut gpioa.crl);
    let mut _ch3 = gpioa.pa3.into_analog(&mut gpioa.crl);
    let mut _ch4 = gpioa.pa4.into_analog(&mut gpioa.crl);
    let mut _ch5 = gpioa.pa5.into_analog(&mut gpioa.crl);
    let mut _ch6 = gpioa.pa6.into_analog(&mut gpioa.crl);
    let mut _ch7 = gpioa.pa7.into_analog(&mut gpioa.crl);

    // PB0-PB1 analog input: ADC_IN8 .. ADC_IN9
    let mut ch8 = gpiob.pb0.into_analog(&mut gpiob.crl);
    let mut _ch9 = gpiob.pb1.into_analog(&mut gpiob.crl);

    // PC0-PC5 analog input: ADC_IN10 .. ADC_IN15
    let mut _ch10 = gpioc.pc0.into_analog(&mut gpioc.crl);
    let mut _ch11 = gpioc.pc1.into_analog(&mut gpioc.crl);
    let mut _ch12 = gpioc.pc2.into_analog(&mut gpioc.crl);
    let mut _ch13 = gpioc.pc3.into_analog(&mut gpioc.crl);
    let mut _ch14 = gpioc.pc4.into_analog(&mut gpioc.crl);
    let mut _ch15 = gpioc.pc5.into_analog(&mut gpioc.crl);

    // PC7: GPIO push-pull output: enable IR LEDs of all the front sensors
    let mut front_leds = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
    front_leds.set_high();

    // PD9: push-pull output: enable IR LEDs of all 3 floor sensors
    let mut bottom_leds = gpiod.pd9.into_push_pull_output(&mut gpiod.crh);
    bottom_leds.set_high();

    // ADC setup
    let mut adc = Adc::adc1(p.ADC1, &mut rcc.apb2);
    let vmax: u16 = adc.max_sample();

    hprintln!("Start ADC readings...").unwrap();

    loop {
        let data: u16 = adc.read(&mut ch8).unwrap();

        if data < vmax - 200 {
            hprintln!("data: {}", data).unwrap();
        }

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
