// TBD

#![allow(deprecated)]
#![no_main]
#![no_std]

use cm::iprintln;
use cortex_m as cm;

use panic_itm as _;

use rtfm::app;

use hal::adc;
use hal::gpio;
use hal::gpio::{OpenDrain, Output};
use hal::prelude::*;
use hal::stm32;
use hal::timer;
use stm32f1xx_hal as hal;

use bitbang_hal as bb;

use eziclean::adc::{Analog, BottomSensorsData, FrontSensorsData};
use eziclean::display::Display;
use eziclean::motion::{Direction, Gear, Motion, Rotation};

use kxcj9::ic::G8Device;
use kxcj9::{GScale8, Kxcj9, Resolution, SlaveAddr};
use kxcj9::{InterruptPinLatching, InterruptPinPolarity};
use kxcj9::{WakeUpInterruptConfig, WakeUpOutputDataRate, WakeUpTriggerMotion};

use nb;
use shared_bus;

/* Types */

type SpiStbType = gpio::gpioa::PA11<hal::gpio::Output<hal::gpio::PushPull>>;
type SpiDioType = gpio::gpiod::PD14<hal::gpio::Output<hal::gpio::PushPull>>;
type SpiClkType = gpio::gpioc::PC8<hal::gpio::Output<hal::gpio::PushPull>>;
type SpiTmpType = gpio::gpioe::PE15<hal::gpio::Input<hal::gpio::Floating>>;

type I2cSclType = gpio::gpioe::PE7<Output<OpenDrain>>;
type I2cSdaType = gpio::gpiob::PB2<Output<OpenDrain>>;

type TmrType = timer::Timer<stm32::TIM2>;
type TmrProxyType = shared_bus::proxy::BusProxy<
    'static,
    cm::interrupt::Mutex<core::cell::RefCell<TmrType>>,
    TmrType,
>;

/* */

static mut TMR_BUS: Option<
    shared_bus::BusManager<cm::interrupt::Mutex<core::cell::RefCell<TmrType>>, TmrType>,
> = None;

/* */

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
    static mut screen: Display<
        bb::spi::SPI<SpiTmpType, SpiDioType, SpiClkType, TmrProxyType>,
        SpiStbType,
    > = ();
    // Accelerometer
    static mut accel: Kxcj9<bb::i2c::I2cBB<I2cSclType, I2cSdaType, TmrProxyType>, G8Device> = ();

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

        /*
         * Enable shared access to TMR2 using shared_bus crate
         *
         */

        // trick to workaround rtfm late init of static resources:
        // tmr_bus needs to be 'static
        let tmr_bus = {
            let timer = timer::Timer::tim2(device.TIM2, 200.khz(), clocks, &mut rcc.apb1);
            let bus = shared_bus::BusManager::<
                cm::interrupt::Mutex<core::cell::RefCell<TmrType>>,
                TmrType,
            >::new(timer);

            unsafe {
                TMR_BUS = Some(bus);
                // This reference is now &'static
                &TMR_BUS.as_ref().unwrap()
            }
        };

        /*
         * Motion controls
         *
         */

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

        /*
         * Analog measurements
         *
         */

        let adc = adc::Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);
        let mut a = Analog::init(adc, adc::AdcSampleTime::T_13);

        // front sensors LEDs
        let mut front_leds = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        front_leds.set_low();

        // front sensors channels
        let ch4 = gpioa.pa4.into_analog(&mut gpioa.crl);
        let ch6 = gpioa.pa6.into_analog(&mut gpioa.crl);
        let ch8 = gpiob.pb0.into_analog(&mut gpiob.crl);
        let ch10 = gpioc.pc0.into_analog(&mut gpioc.crl);
        let ch15 = gpioc.pc5.into_analog(&mut gpioc.crl);

        a.init_front_sensors(front_leds, ch10, ch4, ch6, ch15, ch8);

        // bottom sensors LEDs
        let mut bottom_leds = gpiod.pd9.into_push_pull_output(&mut gpiod.crh);
        bottom_leds.set_low();

        // bottom sensor channels
        let ch11 = gpioc.pc1.into_analog(&mut gpioc.crl);
        let ch7 = gpioa.pa7.into_analog(&mut gpioa.crl);
        let ch9 = gpiob.pb1.into_analog(&mut gpiob.crl);

        a.init_bottom_sensors(bottom_leds, ch11, ch7, ch9);

        /*
         * Display
         *
         */

        let tmp = gpioe.pe15.into_floating_input(&mut gpioe.crh);
        let clk = gpioc.pc8.into_push_pull_output(&mut gpioc.crh);
        let dio = gpiod.pd14.into_push_pull_output(&mut gpiod.crh);
        let stb = gpioa.pa11.into_push_pull_output(&mut gpioa.crh);

        let mut spi = bb::spi::SPI::new(bb::spi::MODE_3, tmp, dio, clk, tmr_bus.acquire());
        spi.set_bit_order(bb::spi::BitOrder::LSBFirst);

        let d = Display::init(spi, stb);

        /*
         * Sensor button
         *
         */

        // PD1: one-channel touch sensor
        gpiod.pd1.into_floating_input(&mut gpiod.crl);

        // select PD1 as source input for line EXTI1
        afio.exticr1
            .exticr1()
            .modify(|_, w| unsafe { w.exti1().bits(0b0011) });

        // enable EXTI1 line and configure interrupt on falling edge
        device.EXTI.imr.modify(|_, w| w.mr1().set_bit());
        device.EXTI.ftsr.modify(|_, w| w.tr1().set_bit());

        /*
         * Wheel encoders
         *
         */

        // PD13: GPIO open-drain output: IR LEDs for both main motors encoders, active low
        let mut pd13 = gpiod.pd13.into_open_drain_output(&mut gpiod.crh);
        // FIXME: disable for now (active low), need to enable only for measurements ???
        pd13.set_high();

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

        /*
         * Wheel encoders
         *
         */

        gpioe.pe4.into_floating_input(&mut gpioe.crl);

        // select PE4 as source input for line EXTI4
        afio.exticr2
            .exticr2()
            .modify(|_, w| unsafe { w.exti4().bits(0b0100) });

        // enable EXTI4 and configure interrupt on rising and falling edge
        device.EXTI.imr.modify(|_, w| w.mr4().set_bit());
        device.EXTI.rtsr.modify(|_, w| w.tr4().set_bit());
        device.EXTI.ftsr.modify(|_, w| w.tr4().set_bit());

        /*
         * Accelerometer
         *
         */

        let scl = gpioe.pe7.into_open_drain_output(&mut gpioe.crl);
        let sda = gpiob.pb2.into_open_drain_output(&mut gpiob.crl);
        let _irq = gpioe.pe9.into_floating_input(&mut gpioe.crh);

        // select PE9 as source input for line EXTI9
        afio.exticr3
            .exticr3()
            .modify(|_, w| unsafe { w.exti9().bits(0b0100) });

        // enable EXTI9 and configure interrupt on rising edge
        device.EXTI.imr.modify(|_, w| w.mr9().set_bit());
        device.EXTI.rtsr.modify(|_, w| w.tr9().set_bit());

        let i2c = bb::i2c::I2cBB::new(scl, sda, tmr_bus.acquire());
        let address = SlaveAddr::Alternative(true);

        let mut acc = Kxcj9::new_kxcj9_1008(i2c, address);

        nb::block!(acc.reset()).ok();
        acc.enable().unwrap();
        acc.set_scale(GScale8::G2).unwrap();
        acc.set_resolution(Resolution::Low).unwrap();

        acc.set_interrupt_pin_polarity(InterruptPinPolarity::ActiveHigh)
            .unwrap();
        acc.set_interrupt_pin_latching(InterruptPinLatching::Latching)
            .unwrap();
        acc.enable_interrupt_pin().unwrap();

        let config = WakeUpInterruptConfig {
            trigger_motion: WakeUpTriggerMotion {
                x_negative: true,
                x_positive: true,
                y_negative: true,
                y_positive: true,
                z_negative: true,
                z_positive: true,
            },
            data_rate: WakeUpOutputDataRate::Hz25,
            fault_count: 1,
            threshold: 0.5,
        };

        acc.enable_wake_up_interrupt(config).unwrap();

        /*
         * init late resources
         *
         */

        itm = core.ITM;
        exti = device.EXTI;
        analog = a;
        drive = m;
        screen = d;
        accel = acc;
    }

    #[idle(resources = [itm, screen])]
    fn idle() -> ! {
        let mut n: u16 = 0;

        loop {
            resources.screen.lock(|d| {
                d.print_num(n).unwrap();
            });

            n = if n < 9999 { n + 1 } else { 0 };
            cm::asm::wfi();
        }
    }

    #[interrupt(resources = [exti, itm, screen])]
    fn EXTI1() {
        let d = &mut resources.itm.stim[0];
        iprintln!(d, ">>> EXTI1: BUTTON");

        resources.screen.print4([1, 1, 1, 1]).unwrap();
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

    #[interrupt(resources = [exti, itm, accel])]
    fn EXTI9_5() {
        let d = &mut resources.itm.stim[0];
        let r = resources.exti.pr.read();

        if r.pr8().bit_is_set() {
            iprintln!(d, ">>> EXTI9_5: RENC");
            resources.exti.pr.modify(|_, w| w.pr8().set_bit());
            // TODO: collect stats to get rotation speed
        }

        if r.pr9().bit_is_set() {
            let info = resources.accel.read_interrupt_info().unwrap();
            iprintln!(d, ">>> EXTI9_5: ACCEL {:?}", info);
            resources.accel.clear_interrupts().unwrap();
            resources.exti.pr.modify(|_, w| w.pr9().set_bit());
            // TODO: send accelerometer event
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
