// Dummy robot:
// - all processing in timer interrupt: read adc, update motors
// - straightforward dummy logic for avoiding obstacles
// - trivial 'step' obstacle check using IR sensor input from ADC

#![allow(deprecated)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m as cm;
use cm::iprintln;

extern crate cortex_m_rt as rt;
extern crate panic_itm;

extern crate rtfm;
use rtfm::app;

extern crate stm32f1xx_hal as hal;
use hal::adc;
use hal::gpio;
use hal::prelude::*;
use hal::stm32;
use hal::timer;

use eziclean::motion;
use eziclean::motion::{Direction, Gear, Motion, Rotation};

/* */

pub struct FrontSensors {
    pub sll: u16,
    pub slc: u16,
    pub scc: u16,
    pub src: u16,
    pub srr: u16,
}

pub struct BottomSensors {
    pub sl: u16,
    pub sc: u16,
    pub sr: u16,
}

/* */

#[app(device = hal::stm32)]
const APP: () = {
    // basic mcu resources
    static mut tmr2: timer::Timer<stm32::TIM2> = ();
    static mut adc1: adc::Adc<stm32::ADC1> = ();
    static mut itm: hal::stm32::ITM = ();

    // front IR sensors
    static mut front_left_90: gpio::gpioc::PC0<gpio::Analog> = ();
    static mut front_left_45: gpio::gpioa::PA4<gpio::Analog> = ();
    static mut front_center: gpio::gpioa::PA6<gpio::Analog> = ();
    static mut front_right_45: gpio::gpioc::PC5<gpio::Analog> = ();
    static mut front_right_90: gpio::gpiob::PB0<gpio::Analog> = ();

    // bottom IR sensors
    static mut bottom_left: gpio::gpioc::PC1<gpio::Analog> = ();
    static mut bottom_center: gpio::gpioa::PA7<gpio::Analog> = ();
    static mut bottom_right: gpio::gpiob::PB1<gpio::Analog> = ();

    // Motion control
    static mut drive: Motion = ();

    #[init]
    fn init() {
        let mut rcc = device.RCC.constrain();
        let dbg = &mut core.ITM.stim[0];

        // configure clocks
        let mut flash = device.FLASH.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(8.mhz())
            .pclk1(8.mhz())
            .adcclk(4.mhz())
            .freeze(&mut flash.acr);

        iprintln!(dbg, "SYSCLK: {} Hz ...", clocks.sysclk().0);
        iprintln!(dbg, "PCLK2: {} Hz ...", clocks.pclk2().0);
        iprintln!(dbg, "ADCCLK: {} Hz ...", clocks.adcclk().0);

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

        // Motion controls

        let l_rev = gpiod.pd2.into_open_drain_output(&mut gpiod.crl);
        let l_fwd = gpiod.pd5.into_open_drain_output(&mut gpiod.crl);
        let r_rev = gpioe.pe14.into_open_drain_output(&mut gpioe.crh);
        let r_fwd = gpiob.pb11.into_open_drain_output(&mut gpiob.crh);

        let c1 = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
        let c2 = gpiob.pb7.into_alternate_push_pull(&mut gpiob.crl);
        let c3 = gpiob.pb8.into_alternate_push_pull(&mut gpiob.crh);
        let c4 = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);

        let pwm = device.TIM4.pwm(
            (c1, c2, c3, c4),
            &mut afio.mapr,
            500.hz(),
            clocks,
            &mut rcc.apb1,
        );

        let mut m = Motion::init(
            (pwm.1, pwm.0),
            (pwm.2, pwm.3),
            (l_fwd, l_rev),
            (r_fwd, r_rev),
        );
        m.stop().unwrap();

        // ADC setup
        let mut a1 = adc::Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);
        a1.set_sample_time(adc::AdcSampleTime::T_13);

        // configure and start TIM2 periodic timer
        let mut t2 = timer::Timer::tim2(device.TIM2, 50.hz(), clocks, &mut rcc.apb1);
        t2.listen(timer::Event::Update);

        // init late resources

        tmr2 = t2;
        adc1 = a1;
        itm = core.ITM;

        front_left_90 = ch10;
        front_left_45 = ch4;
        front_center = ch6;
        front_right_45 = ch15;
        front_right_90 = ch8;

        bottom_left = ch11;
        bottom_center = ch7;
        bottom_right = ch9;

        drive = m;
    }

    #[idle()]
    fn idle() -> ! {
        loop {
            cm::asm::wfi();
        }
    }

    #[interrupt(resources = [
                // hardware units
                tmr2, adc1, itm,
                // front IR sensors
                front_left_90,
                front_left_45,
                front_center,
                front_right_45,
                front_right_90,
                // bottom IR sensors
                bottom_left,
                bottom_center,
                bottom_right,
                // motion control
                drive
    ])]
    fn TIM2() {
        let dbg = &mut resources.itm.stim[0];
        let max_range: u16 = resources.adc1.max_sample();

        let front = FrontSensors {
            sll: resources.adc1.read(resources.front_left_90).unwrap(),
            slc: resources.adc1.read(resources.front_left_45).unwrap(),
            scc: resources.adc1.read(resources.front_center).unwrap(),
            src: resources.adc1.read(resources.front_right_45).unwrap(),
            srr: resources.adc1.read(resources.front_right_90).unwrap(),
        };

        let bottom = BottomSensors {
            sl: resources.adc1.read(resources.bottom_left).unwrap(),
            sc: resources.adc1.read(resources.bottom_center).unwrap(),
            sr: resources.adc1.read(resources.bottom_right).unwrap(),
        };

        iprintln!(
            dbg,
            "raw front sensors: ({},{},{},{},{})",
            front.sll,
            front.slc,
            front.scc,
            front.src,
            front.srr
        );

        let (dl, dc, dr) = make_decision(resources.drive, front, bottom, max_range).unwrap();
        iprintln!(dbg, "decision input: {} {} {}", dl, dc, dr);

        resources.tmr2.start(5.hz());
    }
};

fn make_decision(
    m: &mut Motion,
    front: FrontSensors,
    _bottom: BottomSensors,
    range: u16,
) -> Result<(bool, bool, bool), motion::Error> {
    let right_obstacle = is_obstacle(front.srr, range) || is_obstacle(front.src, range);
    let left_obstacle = is_obstacle(front.sll, range) || is_obstacle(front.slc, range);
    let center_obstacle = is_obstacle(front.scc, range);

    match (left_obstacle, center_obstacle, right_obstacle) {
        (_, true, _) | (true, false, true) => {
            // center obstacle: slow rotation on spot
            m.rotate(Rotation::Left, Gear::Low)?;
        }
        (false, false, true) => {
            // right obstacle, move rotate counter-clockwise
            m.set_right_wheel(Direction::Forward, Gear::Low)?;
            m.set_left_wheel(Direction::Reverse, Gear::Top)?;
        }
        (true, false, false) => {
            // left obstacle, move rotate clockwise
            m.set_right_wheel(Direction::Reverse, Gear::Top)?;
            m.set_left_wheel(Direction::Forward, Gear::Low)?;
        }
        (false, false, false) => {
            // no obstacles, move forward
            m.forward(Gear::Top)?;
        }
    }

    Ok((left_obstacle, center_obstacle, right_obstacle))
}

fn is_obstacle(val: u16, max: u16) -> bool {
    val < max - 200
}
