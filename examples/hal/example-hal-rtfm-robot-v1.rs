// Dummy robot:
// - all processing in timer interrupt: read adc, update motors
// - straightforward dummy logic for avoiding obstacles
// - trivial 'step' obstacle check using IR sensor input from ADC

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m as cm;
use embedded_hal::digital::v2::OutputPin;
use hal::adc;
use hal::prelude::*;
use hal::stm32;
use hal::timer;
use panic_rtt_target as _;
use rtfm::app;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal as hal;

use eziclean::hw::adc::{Analog, BottomSensorsData, FrontSensorsData};
use eziclean::hw::motion::{Error, Motion};
use eziclean::hw::utils::*;
use eziclean::sw::comm::{Direction, Gear, Rotation};

/* */

#[app(device = stm32f1xx_hal::stm32, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    // basic hardware resources
    struct Resources {
        tmr2: timer::CountDownTimer<stm32::TIM2>,
        analog: Analog,
        drive: Motion,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let mut rcc = cx.device.RCC.constrain();

        rtt_init_print!();

        // configure clocks
        let mut flash = cx.device.FLASH.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(8.mhz())
            .pclk1(8.mhz())
            .adcclk(4.mhz())
            .freeze(&mut flash.acr);

        rprintln!("SYSCLK: {} Hz ...", clocks.sysclk().0);
        rprintln!("PCLK2: {} Hz ...", clocks.pclk2().0);
        rprintln!("ADCCLK: {} Hz ...", clocks.adcclk().0);

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = cx.device.GPIOC.split(&mut rcc.apb2);
        let mut gpiod = cx.device.GPIOD.split(&mut rcc.apb2);
        let mut gpioe = cx.device.GPIOE.split(&mut rcc.apb2);

        /* Motion controls */

        let l_rev = gpiod.pd2.into_open_drain_output(&mut gpiod.crl);
        let l_fwd = gpiod.pd5.into_open_drain_output(&mut gpiod.crl);
        let r_rev = gpioe.pe14.into_open_drain_output(&mut gpioe.crh);
        let r_fwd = gpiob.pb11.into_open_drain_output(&mut gpiob.crh);

        let c1 = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
        let c2 = gpiob.pb7.into_alternate_push_pull(&mut gpiob.crl);
        let c3 = gpiob.pb8.into_alternate_push_pull(&mut gpiob.crh);
        let c4 = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);

        let mut afio = cx.device.AFIO.constrain(&mut rcc.apb2);
        let pwm = timer::Timer::tim4(cx.device.TIM4, &clocks, &mut rcc.apb1).pwm(
            (c1, c2, c3, c4),
            &mut afio.mapr,
            500.hz(),
        );

        let mut m = Motion::init(
            (pwm.1, pwm.0),
            (pwm.2, pwm.3),
            (l_fwd, l_rev),
            (r_fwd, r_rev),
        );
        m.stop().unwrap();

        /* IR LEDs for obstacle sensors */

        // PC7: GPIO push-pull output: configure and disable IR LEDs of all the front sensors
        let mut front_leds = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        front_leds.set_low().unwrap();

        // PD9: push-pull output: configure and disable IR LEDs of all 3 floor sensors
        let mut bottom_leds = gpiod.pd9.into_push_pull_output(&mut gpiod.crh);
        bottom_leds.set_low().unwrap();

        /* Analog measurements */

        // ADC setup
        let adc = adc::Adc::adc1(cx.device.ADC1, &mut rcc.apb2, clocks);
        let mut a = Analog::init(adc, adc::SampleTime::T_13);

        // front sensor channels
        let ch4 = gpioa.pa4.into_analog(&mut gpioa.crl);
        let ch6 = gpioa.pa6.into_analog(&mut gpioa.crl);
        let ch8 = gpiob.pb0.into_analog(&mut gpiob.crl);
        let ch10 = gpioc.pc0.into_analog(&mut gpioc.crl);
        let ch15 = gpioc.pc5.into_analog(&mut gpioc.crl);

        a.init_front_sensors(front_leds, ch10, ch4, ch6, ch15, ch8);

        // bottom sensor channels
        let ch11 = gpioc.pc1.into_analog(&mut gpioc.crl);
        let ch7 = gpioa.pa7.into_analog(&mut gpioa.crl);
        let ch9 = gpiob.pb1.into_analog(&mut gpiob.crl);

        a.init_bottom_sensors(bottom_leds, ch11, ch7, ch9);

        /* configure and start TIM2 periodic timer */

        let mut t2 =
            timer::Timer::tim2(cx.device.TIM2, &clocks, &mut rcc.apb1).start_count_down(50.hz());
        t2.listen(timer::Event::Update);

        /* init late resources */

        init::LateResources {
            tmr2: t2,
            analog: a,
            drive: m,
        }
    }

    #[idle()]
    fn idle(_: idle::Context) -> ! {
        loop {
            cm::asm::nop();
        }
    }

    #[task(
        binds = TIM2,
        resources = [
                // hardware units
                tmr2,
                // analog readings
                analog,
                // motion control
                drive
    ])]
    fn tim2(cx: tim2::Context) {
        let max_range: u16 = cx.resources.analog.get_max_sample();

        let front = cx.resources.analog.get_front_sensors(true).unwrap();
        let bottom = cx.resources.analog.get_bottom_sensors(true).unwrap();

        rprintln!(
            "raw front sensors: ({},{},{},{},{})",
            front.fll,
            front.flc,
            front.fcc,
            front.frc,
            front.frr
        );

        let (dl, dc, dr) = make_decision(cx.resources.drive, front, bottom, max_range).unwrap();
        rprintln!("decision input: {} {} {}", dl, dc, dr);

        cx.resources.tmr2.start(5.hz());
    }
};

fn make_decision(
    m: &mut Motion,
    front: FrontSensorsData,
    _bottom: BottomSensorsData,
    range: u16,
) -> Result<(bool, bool, bool), Error> {
    let right_obstacle = is_obstacle(front.frr, range) || is_obstacle(front.frc, range);
    let left_obstacle = is_obstacle(front.fll, range) || is_obstacle(front.flc, range);
    let center_obstacle = is_obstacle(front.fcc, range);

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
