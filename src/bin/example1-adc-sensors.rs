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

use core::fmt;

extern crate stm32ral;
use stm32ral::adc1;
use stm32ral::gpio;
use stm32ral::modify_reg;
use stm32ral::rcc;
use stm32ral::read_reg;

/* */

pub enum IRsensor {
    FLL,
    FL,
    FC,
    FR,
    FRR,
    BL,
    BC,
    BR,
}

use IRsensor::*;

impl fmt::Display for IRsensor {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            IRsensor::FLL => write!(f, "FLL"),
            IRsensor::FL => write!(f, "FL"),
            IRsensor::FC => write!(f, "FC"),
            IRsensor::FR => write!(f, "FR"),
            IRsensor::FRR => write!(f, "FRR"),
            IRsensor::BL => write!(f, "BL"),
            IRsensor::BC => write!(f, "BC"),
            IRsensor::BR => write!(f, "BR"),
        }
    }
}

// device specific mapping: sensors <-> ADC channels
fn adc_chan(sensor: &IRsensor) -> u32 {
    let chan = match sensor {
        IRsensor::FLL => 10,
        IRsensor::FL => 4,
        IRsensor::FC => 6,
        IRsensor::FR => 15,
        IRsensor::FRR => 8,
        IRsensor::BL => 11,
        IRsensor::BC => 7,
        IRsensor::BR => 9,
    };

    chan as u32
}

/* */

#[entry]
fn main() -> ! {
    // RCC clock setup: GPIO[ABCD], ADC1

    let rcc = rcc::RCC::take().unwrap();
    hprintln!("Configure RCC").unwrap();

    modify_reg!(rcc, rcc, APB2ENR, IOPAEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPBEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPCEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, IOPDEN: 1);
    modify_reg!(rcc, rcc, APB2ENR, ADC1EN: 1);

    // GPIO setup

    let gpioa = gpio::GPIOA::take().unwrap();
    let gpiob = gpio::GPIOB::take().unwrap();
    let gpioc = gpio::GPIOC::take().unwrap();
    let gpiod = gpio::GPIOD::take().unwrap();
    hprintln!("Configure GPIO").unwrap();

    // PA0-PA7 analog input: ADC_IN0 .. ADC_IN7
    modify_reg!(gpio, gpioa, CRL, MODE0: 0b00, CNF0: 0b00);
    modify_reg!(gpio, gpioa, CRL, MODE1: 0b00, CNF1: 0b00);
    modify_reg!(gpio, gpioa, CRL, MODE2: 0b00, CNF2: 0b00);
    modify_reg!(gpio, gpioa, CRL, MODE3: 0b00, CNF3: 0b00);
    modify_reg!(gpio, gpioa, CRL, MODE4: 0b00, CNF4: 0b00);
    modify_reg!(gpio, gpioa, CRL, MODE5: 0b00, CNF5: 0b00);
    modify_reg!(gpio, gpioa, CRL, MODE6: 0b00, CNF6: 0b00);
    modify_reg!(gpio, gpioa, CRL, MODE7: 0b00, CNF7: 0b00);

    // PB0-PB1 analog input: ADC_IN8 .. ADC_IN9
    modify_reg!(gpio, gpiob, CRL, MODE0: 0b00, CNF0: 0b00);
    modify_reg!(gpio, gpiob, CRL, MODE1: 0b00, CNF1: 0b00);

    // PC0-PC5 analog input: ADC_IN10 .. ADC_IN15
    modify_reg!(gpio, gpioc, CRL, MODE0: 0b00, CNF0: 0b00);
    modify_reg!(gpio, gpioc, CRL, MODE1: 0b00, CNF1: 0b00);
    modify_reg!(gpio, gpioc, CRL, MODE2: 0b00, CNF2: 0b00);
    modify_reg!(gpio, gpioc, CRL, MODE3: 0b00, CNF3: 0b00);
    modify_reg!(gpio, gpioc, CRL, MODE4: 0b00, CNF4: 0b00);
    modify_reg!(gpio, gpioc, CRL, MODE5: 0b00, CNF5: 0b00);

    // PC7: GPIO push-pull output: enable IR LEDs of all the front sensors
    modify_reg!(gpio, gpioc, CRL, MODE7: 0b11, CNF7: 0b00);
    modify_reg!(gpio, gpioc, ODR, ODR7: 1);

    // PD9: push-pull output: enable IR LEDs of all 3 floor sensors
    modify_reg!(gpio, gpiod, CRH, MODE9: 0b11, CNF9: 0b00);
    modify_reg!(gpio, gpiod, ODR, ODR9: 1);

    // ADC setup

    let adc1 = adc1::ADC1::take().unwrap();
    hprintln!("Configure ADC1").unwrap();

    // ADC power off
    modify_reg!(adc1, adc1, CR2, ADON: 0);
    // ADC reset
    modify_reg!(rcc, rcc, APB2RSTR, ADC1RST: 1);
    delay(100);
    modify_reg!(rcc, rcc, APB2RSTR, ADC1RST: 0);

    // ADC clock prescaler: APB2 / 8
    modify_reg!(rcc, rcc, CFGR, ADCPRE: 0b11);
    // ADC disable scan mode
    modify_reg!(adc1, adc1, CR1, SCAN: 0);
    // ADC set single conversion mode
    modify_reg!(adc1, adc1, CR2, CONT: 0);

    // ADC set sample time 0b011(28.5 cycles) for channels
    modify_reg!(adc1, adc1, SMPR2, SMP0: 0b011);
    modify_reg!(adc1, adc1, SMPR2, SMP1: 0b011);
    modify_reg!(adc1, adc1, SMPR2, SMP2: 0b011);
    modify_reg!(adc1, adc1, SMPR2, SMP3: 0b011);
    modify_reg!(adc1, adc1, SMPR2, SMP4: 0b011);
    modify_reg!(adc1, adc1, SMPR2, SMP5: 0b011);
    modify_reg!(adc1, adc1, SMPR2, SMP6: 0b011);
    modify_reg!(adc1, adc1, SMPR2, SMP7: 0b011);
    modify_reg!(adc1, adc1, SMPR2, SMP8: 0b011);
    modify_reg!(adc1, adc1, SMPR2, SMP9: 0b011);

    modify_reg!(adc1, adc1, SMPR1, SMP10: 0b011);
    modify_reg!(adc1, adc1, SMPR1, SMP11: 0b011);
    modify_reg!(adc1, adc1, SMPR1, SMP12: 0b011);
    modify_reg!(adc1, adc1, SMPR1, SMP13: 0b011);
    modify_reg!(adc1, adc1, SMPR1, SMP14: 0b011);
    modify_reg!(adc1, adc1, SMPR1, SMP15: 0b011);

    // ADC enable software trigger
    modify_reg!(adc1, adc1, CR2, EXTSEL: 0b111);
    modify_reg!(adc1, adc1, CR2, EXTTRIG: 1);

    // ADC power on
    modify_reg!(adc1, adc1, CR2, ADON: 1);
    delay(10000);

    // ADC reset calibration
    modify_reg!(adc1, adc1, CR2, RSTCAL: 1);
    while read_reg!(adc1, adc1, CR2, RSTCAL == 1) {}

    // ADC calibrate
    modify_reg!(adc1, adc1, CR2, CAL: 1);
    while read_reg!(adc1, adc1, CR2, CAL == 1) {}

    hprintln!("Start ADC conversions").unwrap();

    // Set ADC discontinous mode for regular sequence of IR sensor channels
    let sensors: [IRsensor; 8] = [FLL, FL, FC, FR, FRR, BL, BC, BR];
    let max_12bit = 0b111111111111;

    modify_reg!(adc1, adc1, CR1, DISCEN: 1);
    modify_reg!(adc1, adc1, SQR1, L: (sensors.len() - 1) as u32);

    modify_reg!(adc1, adc1, SQR3,
                SQ1: adc_chan(&sensors[0]),
                SQ2: adc_chan(&sensors[1]),
                SQ3: adc_chan(&sensors[2]),
                SQ4: adc_chan(&sensors[3]),
                SQ5: adc_chan(&sensors[4]),
                SQ6: adc_chan(&sensors[5]));

    modify_reg!(adc1, adc1, SQR2,
                SQ7: adc_chan(&sensors[6]),
                SQ8: adc_chan(&sensors[7]));

    loop {
        for i in 0..sensors.len() {
            // ADC start conversion of regular sequence
            modify_reg!(adc1, adc1, CR2, SWSTART: 1);
            while read_reg!(adc1, adc1, CR2, SWSTART == 1) {}

            // ADC wait for conversion results
            while read_reg!(adc1, adc1, SR, EOC == 0) {}
            // ADC read results
            let data = max_12bit - read_reg!(adc1, adc1, DR, DATA);

            if data > 100 {
                hprintln!("{}: {}", sensors[i], data).unwrap();
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
