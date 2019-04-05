//
// Dummy robot:
// - all processing in timer interrupt: read adc, update motors
// - straightforward dummy logic for avoiding obstacles
// - trivial 'step' obstacle check using IR sensor input from ADC

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

pub enum WheelDirection {
    Forward,
    Reverse,
}

pub enum WheelSpeed {
    Slow,
    Fast,
}

pub struct Wheel {
    pub speed: WheelSpeed,
    pub dir: WheelDirection,
}

pub struct WheelControl {
    pub left: Wheel,
    pub right: Wheel,
}

/* */

#[app(device = hal::stm32)]
const APP: () = {
    // basic mcu resources
    static mut tmr2: timer::Timer<stm32::TIM2> = ();
    static mut adc1: adc::Adc<stm32::ADC1> = ();

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

    // left wheel controls
    static mut left_fwd: gpio::gpiod::PD5<gpio::Output<gpio::OpenDrain>> = ();
    static mut left_rev: gpio::gpiod::PD2<gpio::Output<gpio::OpenDrain>> = ();

    // right wheel controls
    static mut right_fwd: gpio::gpiob::PB11<gpio::Output<gpio::OpenDrain>> = ();
    static mut right_rev: gpio::gpioe::PE14<gpio::Output<gpio::OpenDrain>> = ();

    // pwm wheels control
    static mut pwm: (
        hal::pwm::Pwm<hal::device::TIM4, hal::pwm::C1>,
        hal::pwm::Pwm<hal::device::TIM4, hal::pwm::C2>,
        hal::pwm::Pwm<hal::device::TIM4, hal::pwm::C3>,
        hal::pwm::Pwm<hal::device::TIM4, hal::pwm::C4>,
    ) = ();

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
        pwm = pwm;

        front_left_90 = ch10;
        front_left_45 = ch4;
        front_center = ch6;
        front_right_45 = ch15;
        front_right_90 = ch8;

        bottom_left = ch11;
        bottom_center = ch7;
        bottom_right = ch9;

        left_fwd = left_fwd;
        left_rev = left_rev;
        right_fwd = right_fwd;
        right_rev = right_rev;
    }

    #[idle()]
    fn idle() -> ! {
        loop {
            cm::asm::wfi();
        }
    }

    #[interrupt(resources = [
                // hardware units
                tmr2, adc1, pwm,
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
                // left wheel
                left_fwd,
                left_rev,
                // right wheel
                right_fwd,
                right_rev
    ])]
    fn TIM2() {
        let max_range: u16 = resources.adc1.max_sample();
        let max_duty: u16 = resources.pwm.0.get_max_duty();
        let fast: u16 = 2 * max_duty / 3 as u16;
        let slow: u16 = 3 * max_duty / 4 as u16;

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

        let wheels = next_move(front, bottom, max_range);

        match wheels.left.dir {
            WheelDirection::Forward => {
                resources.left_fwd.set_high();
                resources.left_rev.set_low();
            }
            WheelDirection::Reverse => {
                resources.left_fwd.set_low();
                resources.left_rev.set_high();
            }
        };

        match wheels.left.speed {
            WheelSpeed::Slow => {
                resources.pwm.0.set_duty(slow);
                resources.pwm.1.set_duty(slow);
            }
            WheelSpeed::Fast => {
                resources.pwm.0.set_duty(fast);
                resources.pwm.1.set_duty(fast);
            }
        };

        match wheels.right.dir {
            WheelDirection::Forward => {
                resources.right_fwd.set_high();
                resources.right_rev.set_low();
            }
            WheelDirection::Reverse => {
                resources.right_fwd.set_low();
                resources.right_rev.set_high();
            }
        };

        match wheels.right.speed {
            WheelSpeed::Slow => {
                resources.pwm.2.set_duty(slow);
                resources.pwm.3.set_duty(slow);
            }
            WheelSpeed::Fast => {
                resources.pwm.2.set_duty(fast);
                resources.pwm.3.set_duty(fast);
            }
        };

        resources.tmr2.start(50.hz());
    }
};

fn next_move(front: FrontSensors, _bottom: BottomSensors, range: u16) -> WheelControl {
    let mut right_obstacle = false;
    let mut center_obstacle = false;
    let mut left_obstacle = false;

    if is_obstacle(front.sll, range) {
        left_obstacle = true;
    }

    if is_obstacle(front.slc, range) {
        left_obstacle = true;
    }

    if is_obstacle(front.scc, range) {
        center_obstacle = true;
    }

    if is_obstacle(front.src, range) {
        right_obstacle = true;
    }

    if is_obstacle(front.srr, range) {
        right_obstacle = true;
    }

    hprintln!(
        "sensors: ({},{},{})",
        left_obstacle,
        center_obstacle,
        right_obstacle
    )
    .unwrap();

    match (left_obstacle, center_obstacle, right_obstacle) {
        (_, true, _) | (true, false, true) => {
            // center obstacle: slow rotation on spot
            return WheelControl {
                left: Wheel {
                    speed: WheelSpeed::Slow,
                    dir: WheelDirection::Reverse,
                },
                right: Wheel {
                    speed: WheelSpeed::Slow,
                    dir: WheelDirection::Forward,
                },
            };
        }
        (false, false, true) => {
            // right obstacle, move rotate counter-clockwise
            return WheelControl {
                left: Wheel {
                    speed: WheelSpeed::Fast,
                    dir: WheelDirection::Reverse,
                },
                right: Wheel {
                    speed: WheelSpeed::Slow,
                    dir: WheelDirection::Forward,
                },
            };
        }
        (true, false, false) => {
            // left obstacle, move rotate clockwise
            return WheelControl {
                left: Wheel {
                    speed: WheelSpeed::Slow,
                    dir: WheelDirection::Forward,
                },
                right: Wheel {
                    speed: WheelSpeed::Fast,
                    dir: WheelDirection::Reverse,
                },
            };
        }
        (false, false, false) => {
            // no obstacles, move forward
            return WheelControl {
                left: Wheel {
                    speed: WheelSpeed::Fast,
                    dir: WheelDirection::Forward,
                },
                right: Wheel {
                    speed: WheelSpeed::Fast,
                    dir: WheelDirection::Forward,
                },
            };
        }
    }
}

fn is_obstacle(val: u16, max: u16) -> bool {
    val < max - 200
}
