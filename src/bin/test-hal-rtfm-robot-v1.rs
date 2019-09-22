// Dummy robot:
// - all processing in timer interrupt: read adc, update motors
// - straightforward dummy logic for avoiding obstacles
// - trivial 'step' obstacle check using IR sensor input from ADC

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use embedded_hal::digital::v2::OutputPin;

use cm::iprintln;
use cortex_m as cm;

use panic_itm as _;

use rtfm::app;

use hal::adc;
use hal::prelude::*;
use hal::stm32;
use hal::timer;
use stm32f1xx_hal as hal;

use eziclean::hw::adc::{Analog, BottomSensorsData, FrontSensorsData};
use eziclean::hw::motion::{Direction, Error, Gear, Motion, Rotation};
use eziclean::hw::utils::*;

/* */

#[app(device = stm32f1xx_hal::stm32)]
const APP: () = {
    // basic hardware resources
    static mut tmr2: timer::CountDownTimer<stm32::TIM2> = ();
    static mut itm: hal::stm32::ITM = ();
    // analog readings
    static mut analog: Analog = ();
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

        /* Motion controls */

        let l_rev = gpiod.pd2.into_open_drain_output(&mut gpiod.crl);
        let l_fwd = gpiod.pd5.into_open_drain_output(&mut gpiod.crl);
        let r_rev = gpioe.pe14.into_open_drain_output(&mut gpioe.crh);
        let r_fwd = gpiob.pb11.into_open_drain_output(&mut gpiob.crh);

        let c1 = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
        let c2 = gpiob.pb7.into_alternate_push_pull(&mut gpiob.crl);
        let c3 = gpiob.pb8.into_alternate_push_pull(&mut gpiob.crh);
        let c4 = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);

        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let pwm = timer::Timer::tim4(device.TIM4, &clocks, &mut rcc.apb1).pwm(
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
        let adc = adc::Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);
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
            timer::Timer::tim2(device.TIM2, &clocks, &mut rcc.apb1).start_count_down(50.hz());
        t2.listen(timer::Event::Update);

        /* init late resources */

        tmr2 = t2;
        itm = core.ITM;
        analog = a;
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
                tmr2, itm,
                // analog readings
                analog,
                // motion control
                drive
    ])]
    fn TIM2() {
        let dbg = &mut resources.itm.stim[0];
        let max_range: u16 = resources.analog.get_max_sample();

        let front = resources.analog.get_front_sensors(true).unwrap();
        let bottom = resources.analog.get_bottom_sensors(true).unwrap();

        iprintln!(
            dbg,
            "raw front sensors: ({},{},{},{},{})",
            front.fll,
            front.flc,
            front.fcc,
            front.frc,
            front.frr
        );

        let (dl, dc, dr) = make_decision(resources.drive, front, bottom, max_range).unwrap();
        iprintln!(dbg, "decision input: {} {} {}", dl, dc, dr);

        resources.tmr2.start(5.hz());
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
