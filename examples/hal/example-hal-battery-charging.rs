#![no_main]
#![no_std]

use cortex_m as cm;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::InputPin;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::timer;
use stm32f1xx_hal::timer::Tim2PartialRemap2;
use stm32f1xx_hal::{adc::Adc, adc::SampleTime, prelude::*, stm32};

#[entry]
fn main() -> ! {
    let p = stm32::Peripherals::take().unwrap();

    rtt_init_print!();

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
    adc.set_sample_time(SampleTime::T_71);
    adc.read_vref();
    adc.read_vref();
    let v_ref: u32 = adc.read_vref().into();

    // TIM2 PWM setup
    let mut charger = timer::Timer::tim2(p.TIM2, &clocks, &mut rcc.apb1)
        .pwm::<Tim2PartialRemap2, _, _, _>(pin, &mut afio.mapr, 30.khz());
    let pwm_max_duty = charger.get_max_duty();
    charger.disable();

    loop {
        // make sure charger is connected
        while !batt.is_high().unwrap() {
            rprintln!("Battery not connected...");
            delay(1000000);
        }

        rprintln!("Battery connected !");

        // make sure that battery is connected
        while !plug.is_high().unwrap() && !base.is_high().unwrap() {
            rprintln!("Charger not connected...");
            delay(1000000);
        }

        rprintln!("Charger connected !");

        charger.set_duty(pwm_max_duty * 50 / 100 as u16);
        charger.enable();

        loop {
            // Battery voltage
            let v_ch1: u32 = adc.read(&mut ch1).unwrap();
            // As per PCB investigation, battery voltage divider:
            // v_ch1 = v_bat * 20k / (200k + 20k) */
            let v_bat: u32 = 11 * v_ch1 * 1200 / v_ref;

            // Voltage drop on shunt resistor R75
            let v_ch2: u32 = adc.read(&mut ch2).unwrap();
            // As per PCB investigation, OpAmp-1 (voltage subtractor) + OpAmp-2 (voltage follower):
            // v_ch2 = v_shunt * (200k / 10k)
            let v_shunt: u32 = v_ch2 * 1200 * 10 / v_ref / 200;

            rprintln!(
                "V_bat: {} mV; V_shunt {} mV; V_ref: {}",
                v_bat,
                v_shunt,
                v_ref
            );

            delay(1000000);

            if !plug.is_high().unwrap() && !base.is_high().unwrap() {
                rprintln!("Charger disconnected: stop charging!");
                charger.disable();
                break;
            }

            if !batt.is_high().unwrap() {
                rprintln!("Battery disconnected: stop charging...");
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
