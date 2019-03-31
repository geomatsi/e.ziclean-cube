#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m as cm;
extern crate cortex_m_rt as rt;
extern crate panic_semihosting;

extern crate cortex_m_semihosting;
use cortex_m_semihosting::hprintln;

extern crate rtfm;
use rtfm::app;

extern crate stm32f1xx_hal as hal;
use hal::adc;
use hal::gpio;
use hal::prelude::*;
use hal::stm32;
use hal::timer;

/* */

#[app(device = hal::stm32)]
const APP: () = {
    // basic mcu resources
    static mut tmr2: timer::Timer<stm32::TIM2> = ();
    static mut adc1: adc::Adc<stm32::ADC1> = ();

    // front IR sensors
    static mut fll: gpio::gpioc::PC0<gpio::Analog> = ();
    static mut fl: gpio::gpioa::PA4<gpio::Analog> = ();
    static mut fc: gpio::gpioa::PA6<gpio::Analog> = ();
    static mut fr: gpio::gpioc::PC5<gpio::Analog> = ();
    static mut frr: gpio::gpiob::PB0<gpio::Analog> = ();

    // bottom IR sensors
    static mut bl: gpio::gpioc::PC1<gpio::Analog> = ();
    static mut bc: gpio::gpioa::PA7<gpio::Analog> = ();
    static mut br: gpio::gpiob::PB1<gpio::Analog> = ();

    // left wheel controls
    static mut lwf: gpio::gpiod::PD5<gpio::Output<gpio::OpenDrain>> = ();
    static mut lwr: gpio::gpiod::PD2<gpio::Output<gpio::OpenDrain>> = ();

    // right wheel controls
    static mut rwf: gpio::gpiob::PB11<gpio::Output<gpio::OpenDrain>> = ();
    static mut rwr: gpio::gpioe::PE14<gpio::Output<gpio::OpenDrain>> = ();

    #[init]
    fn init() {
        let mut rcc = device.RCC.constrain();

        // configure clocks
        let mut flash = device.FLASH.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(8.mhz())
            .pclk1(8.mhz())
            .adcclk(4.mhz())
            .freeze(&mut flash.acr);

        hprintln!("SYSCLK: {} Hz ...", clocks.sysclk().0).unwrap();
        hprintln!("PCLK2: {} Hz ...", clocks.pclk2().0).unwrap();
        hprintln!("ADCCLK: {} Hz ...", clocks.adcclk().0).unwrap();

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);
        let mut gpiod = device.GPIOD.split(&mut rcc.apb2);
        let mut gpioe = device.GPIOE.split(&mut rcc.apb2);

        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

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

        // NOTE: it looks like PD2 and PD5 are control signal of
        // SR-latch circuitry protecting H-bridge for left motor
        //  _________________________
        // | PD2  | PD5  | FWD | REV |
        //  _________________________
        // |  0   |   0  |  -  |  -  |
        //  _________________________
        // |  1   |   0  |  -  |  +  |
        //  _________________________
        // |  0   |   1  |  +  |  -  |
        //  _________________________
        // |  1   |   1  |  -  |  -  |
        //  _________________________

        let mut left_rev = gpiod.pd2.into_open_drain_output(&mut gpiod.crl);
        left_rev.set_low();

        let mut left_fwd = gpiod.pd5.into_open_drain_output(&mut gpiod.crl);
        left_fwd.set_low();

        // NOTE: it looks like PE14 and PB11 are control signal of
        // SR-latch circuitry protecting H-bridge for right motor
        //  _________________________
        // | PE14 | PB11 | FWD | REV |
        //  _________________________
        // |  0   |   0  |  -  |  -  |
        //  _________________________
        // |  1   |   0  |  -  |  +  |
        //  _________________________
        // |  0   |   1  |  +  |  -  |
        //  _________________________
        // |  1   |   1  |  -  |  -  |
        //  _________________________

        let mut right_rev = gpioe.pe14.into_open_drain_output(&mut gpioe.crh);
        right_rev.set_low();

        let mut right_fwd = gpiob.pb11.into_open_drain_output(&mut gpiob.crh);
        right_fwd.set_low();

        // TIM4

        let c1 = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
        let c2 = gpiob.pb7.into_alternate_push_pull(&mut gpiob.crl);
        let c3 = gpiob.pb8.into_alternate_push_pull(&mut gpiob.crh);
        let c4 = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);

        let mut pwm = device.TIM4.pwm(
            (c1, c2, c3, c4),
            &mut afio.mapr,
            500.hz(),
            clocks,
            &mut rcc.apb1,
        );

        pwm.0.enable();
        pwm.1.enable();
        pwm.2.enable();
        pwm.3.enable();

        // ADC setup
        let mut a1 = adc::Adc::adc1(device.ADC1, &mut rcc.apb2);
        a1.set_sample_time(adc::AdcSampleTime::T_13);

        // configure and start TIM2 periodic timer
        let mut t2 = timer::Timer::tim2(device.TIM2, 50.hz(), clocks, &mut rcc.apb1);
        t2.listen(timer::Event::Update);

        // start moving forward !
        let max_duty = pwm.0.get_max_duty();
        let duty = max_duty / 2 as u16;

        hprintln!("max_duty[{}] duty[{}]", max_duty, duty).unwrap();
        pwm.0.set_duty(duty);
        pwm.1.set_duty(duty);
        pwm.2.set_duty(duty);
        pwm.3.set_duty(duty);

        // init late resources

        tmr2 = t2;
        adc1 = a1;

        fll = ch10;
        fl = ch4;
        fc = ch6;
        fr = ch15;
        frr = ch8;

        bl = ch11;
        bc = ch7;
        br = ch9;

        lwf = left_fwd;
        lwr = left_rev;
        rwf = right_fwd;
        rwr = right_rev;
    }

    #[idle()]
    fn idle() -> ! {
        loop {
            cm::asm::wfi();
        }
    }

    #[interrupt(resources = [tmr2, adc1, fll, fl, fc, fr, frr, bl, bc, br, lwf, lwr, rwf, rwr])]
    fn TIM2() {
        let vmax: u16 = resources.adc1.max_sample();
        let mut r_obstacle = false;
        let mut l_obstacle = false;

        let data: u16 = resources.adc1.read(resources.fll).unwrap();
        if is_obstacle(data, vmax) {
            l_obstacle = true;
        }

        let data: u16 = resources.adc1.read(resources.fl).unwrap();
        if is_obstacle(data, vmax) {
            l_obstacle = true;
        }

        let data: u16 = resources.adc1.read(resources.fc).unwrap();
        if is_obstacle(data, vmax) {
            l_obstacle = true;
        }

        let data: u16 = resources.adc1.read(resources.fr).unwrap();
        if is_obstacle(data, vmax) {
            r_obstacle = true;
        }

        let data: u16 = resources.adc1.read(resources.frr).unwrap();
        if is_obstacle(data, vmax) {
            r_obstacle = true;
        }

        hprintln!("sensors: ({},{})", l_obstacle, r_obstacle).unwrap();

        match (l_obstacle, r_obstacle) {
            (true, true) => {
                // both obstacles, rotate counter-clockwise
                resources.rwf.set_high();
                resources.rwr.set_low();
                resources.lwf.set_low();
                resources.lwr.set_high();
            }
            (false, true) => {
                // right obstacle, rotate counter-clockwise
                resources.rwf.set_high();
                resources.rwr.set_low();
                resources.lwf.set_low();
                resources.lwr.set_high();
            }
            (true, false) => {
                // left obstacle, rotate clockwise
                resources.rwf.set_low();
                resources.rwr.set_high();
                resources.lwf.set_high();
                resources.lwr.set_low();
            }
            (false, false) => {
                // no obstacles, move forward
                resources.rwf.set_high();
                resources.rwr.set_low();
                resources.lwf.set_high();
                resources.lwr.set_low();
            }
        }

        resources.tmr2.start(50.hz());
    }
};

fn is_obstacle(val: u16, max: u16) -> bool {
    val < max - 200
}
