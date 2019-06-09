#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
use rt::entry;
use rt::exception;
use rt::ExceptionFrame;

extern crate cortex_m as cm;
use cm::iprintln;

extern crate panic_itm;

use stm32f1xx_hal::{adc::Adc, prelude::*, stm32, time::Hertz};

#[entry]
fn main() -> ! {
    let mut core = cm::Peripherals::take().unwrap();
    let d = &mut core.ITM.stim[0];

    let p = stm32::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);

    iprintln!(d, "SYSCLK: {} Hz ...", clocks.sysclk().0);
    iprintln!(d, "ADCCLK: {} Hz ...", clocks.adcclk().0);

    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpioe = p.GPIOE.split(&mut rcc.apb2);

    // PA1, PA2: battery voltage and current control ADC channels
    let mut ch1 = gpioa.pa1.into_analog(&mut gpioa.crl);
    let mut ch2 = gpioa.pa2.into_analog(&mut gpioa.crl);

    // ADC setup
    let mut adc = Adc::adc1(p.ADC1, &mut rcc.apb2, clocks.adcclk());

    // pwr source indicators
    let plug = gpioe.pe4.into_floating_input(&mut gpioe.crl);
    let base = gpioe.pe5.into_floating_input(&mut gpioe.crl);
    let batt = gpioe.pe6.into_floating_input(&mut gpioe.crl);

    loop {
        // Ambient temperature
        let temp = adc.read_temp();

        iprintln!(d, "Temp: {} C", temp);

        // Battery voltage
        let v_ref: u32 = adc.read_vref().into();
        let v_ch1: u32 = adc.read(&mut ch1).unwrap();
        // As per PCB investigation, battery voltage divider:
        // v_ch1 = v_bat * 20k / (200k + 20k) */
        let v_bat: u32 = 11 * v_ch1 * 1200 / v_ref;

        iprintln!(d, "V_bat: {} mV", v_bat);

        // Voltage drop on shunt resistor R75

        let v_ch2: u32 = adc.read(&mut ch2).unwrap();
        // As per PCB investigation, OpAmp-1 (voltage subtractor) + OpAmp-2 (voltage follower):
        // v_ch2 = v_shunt * (200k / 10k)
        let v_shunt: u32 = v_ch2 * 1200 * 10 / v_ref / 200;

        iprintln!(d, "V_shunt: {} mV", v_shunt);

        // Battery or charge connector availability

        iprintln!(
            d,
            "PLUG: {}, BASE: {}, BATTERY: {}",
            plug.is_high(),
            base.is_high(),
            batt.is_high()
        );

        delay(10000);
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
