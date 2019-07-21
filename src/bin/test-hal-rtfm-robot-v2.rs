// TBD

#![allow(deprecated)]
#![no_main]
#![no_std]

use cm::iprintln;
use cortex_m as cm;

use panic_itm as _;

use rtfm::app;

use hal::adc;
use hal::prelude::*;
use hal::stm32;
use hal::timer;
use stm32f1xx_hal as hal;

use eziclean::adc::{Analog, BottomSensorsData, FrontSensorsData};
use eziclean::display::Display;
use eziclean::motion::{Direction, Gear, Motion, Rotation};

#[app(device = stm32f1xx_hal::stm32)]
const APP: () = {
    // basic hardware resources
    static mut exti: stm32::EXTI = ();
    static mut itm: stm32::ITM = ();
    // analog readings
    static mut analog: Analog = ();
    // Motion control
    static mut drive: Motion = ();
    // Display
    static mut screen: Display<timer::Timer<stm32::TIM2>> = ();

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
        iprintln!(dbg, "ADCCLK: {} Hz ...", clocks.adcclk().0);

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);
        let mut gpiod = device.GPIOD.split(&mut rcc.apb2);
        let mut gpioe = device.GPIOE.split(&mut rcc.apb2);

        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

        /* Motion controls */

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

        /* IR LEDs for obstacle sensors */

        // PC7: enable IR LEDs of all the front sensors
        let mut front_leds = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        front_leds.set_high();

        // PD9: enable IR LEDs of all 3 floor sensors
        let mut bottom_leds = gpiod.pd9.into_push_pull_output(&mut gpiod.crh);
        bottom_leds.set_high();

        /* Analog measurements */

        let adc = adc::Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);
        let mut a = Analog::init(adc, adc::AdcSampleTime::T_13);

        // front sensor channels
        let ch4 = gpioa.pa4.into_analog(&mut gpioa.crl);
        let ch6 = gpioa.pa6.into_analog(&mut gpioa.crl);
        let ch8 = gpiob.pb0.into_analog(&mut gpiob.crl);
        let ch10 = gpioc.pc0.into_analog(&mut gpioc.crl);
        let ch15 = gpioc.pc5.into_analog(&mut gpioc.crl);

        a.init_front_sensors(ch10, ch4, ch6, ch15, ch8);

        // bottom sensor channels
        let ch11 = gpioc.pc1.into_analog(&mut gpioc.crl);
        let ch7 = gpioa.pa7.into_analog(&mut gpioa.crl);
        let ch9 = gpiob.pb1.into_analog(&mut gpiob.crl);

        a.init_bottom_sensors(ch11, ch7, ch9);

        /* Display */

        let tmp = gpioe.pe15.into_floating_input(&mut gpioe.crh);
        let clk = gpioc.pc8.into_push_pull_output(&mut gpioc.crh);
        let dio = gpiod.pd14.into_push_pull_output(&mut gpiod.crh);
        let stb = gpioa.pa11.into_push_pull_output(&mut gpioa.crh);
        let timer = timer::Timer::tim2(device.TIM2, 200.khz(), clocks, &mut rcc.apb1);

        let d = Display::init(stb, dio, clk, tmp, timer);

        /* Sensor button */

        // PD1: one-channel touch sensor
        gpiod.pd1.into_floating_input(&mut gpiod.crl);

        // select PD1 as source input for line EXTI1
        afio.exticr1
            .exticr1()
            .modify(|_, w| unsafe { w.exti1().bits(0b0011) });

        // enable EXTI1 line and configure interrupt on falling edge
        device.EXTI.imr.modify(|_, w| w.mr1().set_bit());
        device.EXTI.ftsr.modify(|_, w| w.tr1().set_bit());

        /* Wheel encoders */

        // PD13: GPIO open-drain output: IR LEDs for both main motors encoders, active low
        let mut pd13 = gpiod.pd13.into_open_drain_output(&mut gpiod.crh);
        pd13.set_low();

        // PC12, PE8: IR diodes for both main motors encoders
        gpioc.pc12.into_floating_input(&mut gpioc.crh);
        gpioe.pe8.into_floating_input(&mut gpioe.crh);

        // select PE8 as source input for line EXTI8
        afio.exticr3
            .exticr3()
            .modify(|_, w| unsafe { w.exti8().bits(0b0100) });

        // select PC12 as source input for line EXTI12
        afio.exticr4
            .exticr4()
            .modify(|_, w| unsafe { w.exti12().bits(0b0010) });

        // enable EXTI8 and EXTI12 lines and configure interrupt on rising edge
        device.EXTI.imr.modify(|_, w| w.mr8().set_bit());
        device.EXTI.rtsr.modify(|_, w| w.tr8().set_bit());

        device.EXTI.imr.modify(|_, w| w.mr12().set_bit());
        device.EXTI.rtsr.modify(|_, w| w.tr12().set_bit());

        /* Charger plug */

        gpioe.pe4.into_floating_input(&mut gpioe.crl);

        // select PE4 as source input for line EXTI4
        afio.exticr2
            .exticr2()
            .modify(|_, w| unsafe { w.exti4().bits(0b0100) });

        // enable EXTI4 and configure interrupt on rising and falling edge
        device.EXTI.imr.modify(|_, w| w.mr4().set_bit());
        device.EXTI.rtsr.modify(|_, w| w.tr4().set_bit());
        device.EXTI.ftsr.modify(|_, w| w.tr4().set_bit());

        /* init late resources */

        itm = core.ITM;
        exti = device.EXTI;
        analog = a;
        drive = m;
        screen = d;
    }

    #[idle()]
    fn idle() -> ! {
        loop {
            cm::asm::wfi();
        }
    }

    #[interrupt(resources = [exti, itm])]
    fn EXTI1() {
        let d = &mut resources.itm.stim[0];
        iprintln!(d, ">>> EXTI1: BUTTON");

        resources.exti.pr.modify(|_, w| w.pr1().set_bit());
        // TODO: queue button Event
    }

    #[interrupt(resources = [exti, itm])]
    fn EXTI4() {
        let d = &mut resources.itm.stim[0];
        iprintln!(d, ">>> EXTI4: CHARGER");

        resources.exti.pr.modify(|_, w| w.pr4().set_bit());
        // TODO: queue charger Event
    }

    #[interrupt(resources = [exti, itm])]
    fn EXTI9_5() {
        let d = &mut resources.itm.stim[0];
        let r = resources.exti.pr.read();

        if r.pr8().bit_is_set() {
            iprintln!(d, ">>> EXTI9_5: RENC");
            resources.exti.pr.modify(|_, w| w.pr8().set_bit());
            // TODO: collect stats to get rotation speed
        }
    }

    #[interrupt(resources = [exti, itm])]
    fn EXTI15_10() {
        let d = &mut resources.itm.stim[0];
        let r = resources.exti.pr.read();

        if r.pr12().bit_is_set() {
            iprintln!(d, ">>> EXTI15_10: LENC");
            resources.exti.pr.modify(|_, w| w.pr12().set_bit());
            // TODO: collect stats to get rotation speed
        }
    }
};
