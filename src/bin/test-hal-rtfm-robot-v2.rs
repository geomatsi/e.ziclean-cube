// TBD

#![no_main]
#![no_std]

use cm::iprintln;
use cortex_m as cm;

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

use panic_itm as _;

use rtfm::app;
use rtfm::Instant;

use hal::adc;
use hal::gpio;
use hal::gpio::{OpenDrain, Output};
use hal::prelude::*;
use hal::stm32;
use hal::timer;
use stm32f1xx_hal as hal;

use bitbang_hal as bb;

use eziclean::adc::Analog;
use eziclean::display::Display;
use eziclean::events::Events;
use eziclean::motion::Motion;

use heapless::binary_heap::{BinaryHeap, Max};
use heapless::consts::*;

use kxcj9::ic::G8Device;
use kxcj9::{GScale8, Kxcj9, Resolution, SlaveAddr};
use kxcj9::{InterruptPinLatching, InterruptPinPolarity};
use kxcj9::{WakeUpInterruptConfig, WakeUpOutputDataRate, WakeUpTriggerMotion};

use nb;

/* Types */

type Tmr2Type = timer::Timer<stm32::TIM2>;
type Tmr3Type = timer::Timer<stm32::TIM3>;

type DockGpioType = gpio::gpioe::PE5<hal::gpio::Input<hal::gpio::Floating>>;
type ButtonGpioType = gpio::gpiod::PD1<hal::gpio::Input<hal::gpio::Floating>>;
type ChargerGpioType = gpio::gpioe::PE4<hal::gpio::Input<hal::gpio::Floating>>;

type SpiStbType = gpio::gpioa::PA11<hal::gpio::Output<hal::gpio::PushPull>>;
type SpiDioType = gpio::gpiod::PD14<hal::gpio::Output<hal::gpio::PushPull>>;
type SpiClkType = gpio::gpioc::PC8<hal::gpio::Output<hal::gpio::PushPull>>;
type SpiTmpType = gpio::gpioe::PE15<hal::gpio::Input<hal::gpio::Floating>>;
type SpiScreen = bb::spi::SPI<SpiTmpType, SpiDioType, SpiClkType, Tmr2Type>;

type I2cSclType = gpio::gpioe::PE7<Output<OpenDrain>>;
type I2cSdaType = gpio::gpiob::PB2<Output<OpenDrain>>;
type I2cAccel = bb::i2c::I2cBB<I2cSclType, I2cSdaType, Tmr3Type>;

/* */

const PROC_PERIOD: u32 = 400_000; /* 1/20 sec */
const SENSE_PERIOD: u32 = 800_000; /* 1/10 sec */
const POWER_PERIOD: u32 = 80_000_000; /* 10 sec */

/* */

#[app(device = stm32f1xx_hal::stm32)]
const APP: () = {
    // Event queue
    static mut queue: BinaryHeap<Events, U8, Max> = ();

    // basic hardware resources
    static mut exti: stm32::EXTI = ();
    static mut itm: stm32::ITM = ();

    // buttons and chargers
    static mut dock: DockGpioType = ();
    static mut button: ButtonGpioType = ();
    static mut charger: ChargerGpioType = ();

    // analog readings
    static mut analog: Analog = ();

    // Motion control
    static mut drive: Motion = ();

    // Display
    static mut screen: Display<SpiScreen, SpiStbType> = ();

    // Accelerometer
    static mut accel: Kxcj9<I2cAccel, G8Device> = ();

    #[init(schedule = [proc_task, sense_task, power_task])]
    fn init() {
        let mut rcc = device.RCC.constrain();
        let dbg = &mut core.ITM.stim[0];

        // setup event queues
        let queue = BinaryHeap(heapless::i::BinaryHeap::new());

        // configure clocks
        let mut flash = device.FLASH.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(8.mhz())
            .pclk1(8.mhz())
            .adcclk(2.mhz())
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
        front_leds.set_low().unwrap();

        // front sensors channels
        let ch4 = gpioa.pa4.into_analog(&mut gpioa.crl);
        let ch6 = gpioa.pa6.into_analog(&mut gpioa.crl);
        let ch8 = gpiob.pb0.into_analog(&mut gpiob.crl);
        let ch10 = gpioc.pc0.into_analog(&mut gpioc.crl);
        let ch15 = gpioc.pc5.into_analog(&mut gpioc.crl);

        a.init_front_sensors(front_leds, ch10, ch4, ch6, ch15, ch8);

        // bottom sensors LEDs
        let mut bottom_leds = gpiod.pd9.into_push_pull_output(&mut gpiod.crh);
        bottom_leds.set_low().unwrap();

        // bottom sensor channels
        let ch11 = gpioc.pc1.into_analog(&mut gpioc.crl);
        let ch7 = gpioa.pa7.into_analog(&mut gpioa.crl);
        let ch9 = gpiob.pb1.into_analog(&mut gpiob.crl);

        a.init_bottom_sensors(bottom_leds, ch11, ch7, ch9);

        /*
         * Sensor button
         *
         */

        // PD1: one-channel touch sensor
        let button = gpiod.pd1.into_floating_input(&mut gpiod.crl);

        // select PD1 as source input for line EXTI1
        afio.exticr1
            .exticr1()
            .modify(|_, w| unsafe { w.exti1().bits(0b0011) });

        // enable EXTI1 line and configure interrupt on both falling and rising edges
        device.EXTI.imr.modify(|_, w| w.mr1().set_bit());
        device.EXTI.ftsr.modify(|_, w| w.tr1().set_bit());
        device.EXTI.rtsr.modify(|_, w| w.tr1().set_bit());

        /*
         * Wheel encoders
         *
         */

        // PD13: GPIO open-drain output: IR LEDs for both main motors encoders, active low
        let mut pd13 = gpiod.pd13.into_open_drain_output(&mut gpiod.crh);
        // FIXME: disable for now (active low), need to enable only for measurements ???
        pd13.set_high().unwrap();

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
         * Charging cable detection
         *
         */

        let charger = gpioe.pe4.into_floating_input(&mut gpioe.crl);

        // select PE4 as source input for line EXTI4
        afio.exticr2
            .exticr2()
            .modify(|_, w| unsafe { w.exti4().bits(0b0100) });

        // enable EXTI4 and configure interrupt on rising and falling edge
        device.EXTI.imr.modify(|_, w| w.mr4().set_bit());
        device.EXTI.rtsr.modify(|_, w| w.tr4().set_bit());
        device.EXTI.ftsr.modify(|_, w| w.tr4().set_bit());

        /*
         * Dock station detection
         *
         */

        let dock = gpioe.pe5.into_floating_input(&mut gpioe.crl);

        // select PE5 as source input for line EXTI9_5
        afio.exticr2
            .exticr2()
            .modify(|_, w| unsafe { w.exti5().bits(0b0100) });

        // enable EXTI5 and configure interrupt on rising and falling edge
        device.EXTI.imr.modify(|_, w| w.mr5().set_bit());
        device.EXTI.rtsr.modify(|_, w| w.tr5().set_bit());
        device.EXTI.ftsr.modify(|_, w| w.tr5().set_bit());

        /*
         * Display
         *
         */

        // FIXME: for now use TIM2 exclusively for display, however it should be shared with accel
        let tim2 = timer::Timer::tim2(device.TIM2, 200.khz(), clocks, &mut rcc.apb1);

        let tmp = gpioe.pe15.into_floating_input(&mut gpioe.crh);
        let clk = gpioc.pc8.into_push_pull_output(&mut gpioc.crh);
        let dio = gpiod.pd14.into_push_pull_output(&mut gpiod.crh);
        let stb = gpioa.pa11.into_push_pull_output(&mut gpioa.crh);

        let mut spi = bb::spi::SPI::new(bb::spi::MODE_3, tmp, dio, clk, tim2);
        spi.set_bit_order(bb::spi::BitOrder::LSBFirst);

        let d = Display::init(spi, stb);

        /*
         * Accelerometer
         *
         */

        // FIXME: for now use TIM3 for accel, but it is used for brushes PWM
        // NOTE: in release build mode decrease clock to 50kHz
        let tim3 = timer::Timer::tim3(device.TIM3, 50.khz(), clocks, &mut rcc.apb1);

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

        let i2c = bb::i2c::I2cBB::new(scl, sda, tim3);
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
         * schedule periodic tasks
         *
         */

        schedule
            .proc_task(Instant::now() + PROC_PERIOD.cycles())
            .unwrap();
        schedule
            .sense_task(Instant::now() + SENSE_PERIOD.cycles())
            .unwrap();
        schedule
            .power_task(Instant::now() + POWER_PERIOD.cycles())
            .unwrap();

        /*
         * init late resources
         *
         */

        queue = queue;
        itm = core.ITM;
        exti = device.EXTI;
        analog = a;
        drive = m;
        screen = d;
        accel = acc;
        dock = dock;
        button = button;
        charger = charger;
    }

    /*
     * Idle loop processes events with normal priority
     *
     */
    #[idle]
    fn idle() -> ! {
        loop {
            cm::asm::wfi();
        }
    }

    /*
     * This interrupt is used to dispatch timer queue tasks.
     * FIXME:  not all interrupts work here, e.g. UART4 does not work
     *
     */
    extern "C" {
        fn EXTI2();
    }

    /*
     * Brain: main processing task
     *
     */
    #[task(schedule = [proc_task], resources = [itm, queue])]
    fn proc_task() {
        let d = &mut resources.itm.stim[0];
        let eq = &mut resources.queue;

        while !eq.is_empty() {
            if let Some(e) = eq.pop() {
                iprintln!(d, ">>> prio event {:?}", e);
            }
        }

        schedule
            .proc_task(scheduled + PROC_PERIOD.cycles())
            .unwrap();
    }

    /*
     * Sense: sensor processing task to watch for obstacles using IR obstacle sensors
     * TODO: setup ADC+DMA and convert this task into ADC DMA interrupt handler
     *
     */
    #[task(schedule = [sense_task], resources = [itm, analog, queue])]
    fn sense_task() {
        let d = &mut resources.itm.stim[0];
        let adc = &mut resources.analog;
        let eq = &mut resources.queue;
        let max = adc.get_max_sample();

        if let Ok(fs) = adc.get_front_sensors() {
            let fll = is_obstacle(fs.fll, max);
            let flc = is_obstacle(fs.flc, max);
            let fcc = is_obstacle(fs.fcc, max);
            let frc = is_obstacle(fs.frc, max);
            let frr = is_obstacle(fs.frr, max);

            if fll || flc || fcc || frc || frr {
                iprintln!(
                    d,
                    ">>> sensor {} {} {} {} {}",
                    max - fs.fll,
                    max - fs.flc,
                    max - fs.fcc,
                    max - fs.frc,
                    max - fs.frr
                );
                //eq.push(Events::FrontSensor(fll, flc, fcc, frc, frr)).ok();
            }
        }

        // TODO: read and analyze bottom sensors

        schedule
            .sense_task(scheduled + SENSE_PERIOD.cycles())
            .unwrap();
    }

    /*
     * Power: task checking battery, dock station and charger plug
     *
     */
    #[task(schedule = [power_task], resources = [dock, charger, queue])]
    fn power_task() {
        let c = resources.charger.is_high().unwrap_or(false);
        let d = resources.dock.is_high().unwrap_or(false);
        let eq = &mut resources.queue;

        eq.push(Events::ChargerEvent(c)).ok();
        eq.push(Events::DockEvent(d)).ok();

        schedule
            .power_task(scheduled + POWER_PERIOD.cycles())
            .unwrap();
    }

    #[interrupt(resources = [exti, screen, button, queue])]
    fn EXTI1() {
        let pressed = resources.button.is_low().unwrap_or(false);
        let eq = &mut resources.queue;
        let e = Events::ButtonEvent(pressed);

        resources.screen.print_num(0000).ok();
        resources.exti.pr.modify(|_, w| w.pr1().set_bit());
        eq.push(e).ok();
    }

    #[interrupt(resources = [exti, charger, screen, queue])]
    fn EXTI4() {
        let plugged = resources.charger.is_high().unwrap_or(false);
        let eq = &mut resources.queue;
        let e = Events::ChargerEvent(plugged);

        resources.screen.print_num(1111).ok();
        resources.exti.pr.modify(|_, w| w.pr4().set_bit());
        eq.push(e).ok();
    }

    #[interrupt(resources = [exti, screen, accel, dock, queue])]
    fn EXTI9_5() {
        let eq = &mut resources.queue;
        let r = resources.exti.pr.read();

        if r.pr5().bit_is_set() {
            let dock = resources.dock.is_high().unwrap_or(false);
            let e = Events::DockEvent(dock);
            resources.exti.pr.modify(|_, w| w.pr4().set_bit());
            eq.push(e).ok();
        }

        if r.pr8().bit_is_set() {
            resources.exti.pr.modify(|_, w| w.pr8().set_bit());
            // TODO: collect stats to get rotation speed
        }

        if r.pr9().bit_is_set() {
            let info = resources.accel.read_interrupt_info().unwrap();
            let x = info.wake_up_x_negative | info.wake_up_x_positive;
            let y = info.wake_up_y_negative | info.wake_up_y_positive;
            let z = info.wake_up_z_negative | info.wake_up_z_positive;
            let e = Events::AccEvent(x, y, z);

            resources.screen.print_num(2222).ok();
            resources.accel.clear_interrupts().unwrap();
            resources.exti.pr.modify(|_, w| w.pr9().set_bit());
            eq.push(e).ok();
        }
    }

    #[interrupt(resources = [exti])]
    fn EXTI15_10() {
        let r = resources.exti.pr.read();

        if r.pr12().bit_is_set() {
            resources.exti.pr.modify(|_, w| w.pr12().set_bit());
            // TODO: collect stats to get rotation speed
        }
    }
};

//

fn is_obstacle(val: u16, max: u16) -> bool {
    val < max - 200
}
