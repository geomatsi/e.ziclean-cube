#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
use rt::entry;
use rt::exception;
use rt::ExceptionFrame;

extern crate cortex_m as cm;
use cm::iprintln;

extern crate panic_itm;

use stm32f1xx_hal::gpio::gpiob::PB10;
use stm32f1xx_hal::gpio::{Alternate, PushPull};
use stm32f1xx_hal::pwm::{Pins, Pwm, C3};
use stm32f1xx_hal::stm32::TIM2;
use stm32f1xx_hal::{adc::Adc, prelude::*, stm32};

struct Charger(PB10<Alternate<PushPull>>);

impl Pins<TIM2> for Charger {
    const REMAP: u8 = 0b10;
    const C1: bool = false;
    const C2: bool = false;
    const C3: bool = true;
    const C4: bool = false;
    type Channels = Pwm<TIM2, C3>;
}

#[entry]
fn main() -> ! {
    let mut core = cm::Peripherals::take().unwrap();
    let d = &mut core.ITM.stim[0];

    let p = stm32::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.adcclk(2.mhz()).freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain(&mut rcc.apb2);

    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    //let mut gpioc = p.GPIOC.split(&mut rcc.apb2);
    //let mut gpiod = p.GPIOD.split(&mut rcc.apb2);
    let mut gpioe = p.GPIOE.split(&mut rcc.apb2);

    // PA1, PA2: battery voltage and current control ADC channels
    let mut ch1 = gpioa.pa1.into_analog(&mut gpioa.crl);
    let mut ch2 = gpioa.pa2.into_analog(&mut gpioa.crl);

    // PWM output to control charge current
    let pin = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);

    // power source indicators
    let plug = gpioe.pe4.into_floating_input(&mut gpioe.crl);
    let base = gpioe.pe5.into_floating_input(&mut gpioe.crl);
    let batt = gpioe.pe6.into_floating_input(&mut gpioe.crl);

    // ADC1 setup
    let mut adc = Adc::adc1(p.ADC1, &mut rcc.apb2, clocks);
    let adc_max_range: u16 = adc.max_sample();

    // TIM2 PWM setup
    let mut charger = p.TIM2.pwm(
        Charger(pin),
        &mut afio.mapr,
        10.khz(),
        clocks,
        &mut rcc.apb1,
    );

    let pwm_max_duty = charger.get_max_duty();

    charger.disable();

    // make sure that battery is connected

    while !batt.is_high() {
        iprintln!(d, "Battery not connected...");
        delay(10000);
    }

    iprintln!(d, "Battery connected !");

    // make sure charger is connected

    loop {
        while !plug.is_high() && !base.is_high() {
            iprintln!(d, "Charger not connected...");
        }

        iprintln!(d, "Charger connected !");

        charger.set_duty(pwm_max_duty / 2 as u16);
        charger.enable();

        loop {
            // Battery voltage
            let v_ref: u32 = adc.read_vref().into();
            let v_ch1: u32 = adc.read(&mut ch1).unwrap();
            // As per PCB investigation, battery voltage divider:
            // v_ch1 = v_bat * 20k / (200k + 20k) */
            let v_bat: u32 = 11 * v_ch1 * 1200 / v_ref;

            // Voltage drop on shunt resistor R75

            let v_ch2: u32 = adc.read(&mut ch2).unwrap();
            // As per PCB investigation, OpAmp-1 (voltage subtractor) + OpAmp-2 (voltage follower):
            // v_ch2 = v_shunt * (200k / 10k)
            let v_shunt: u32 = v_ch2 * 1200 * 10 / v_ref / 200;

            iprintln!(d, "V_bat: {} mV; V_shunt {} mV", v_bat, v_shunt);
            delay(1000);

            if !plug.is_high() && !base.is_high() {
                iprintln!(d, "Charger disconnected: stop charging!");
                charger.disable();
                break;
            }
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
