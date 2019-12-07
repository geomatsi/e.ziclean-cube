#![allow(clippy::many_single_char_names)]
#![no_main]
#![no_std]

use cm::iprintln;
use cortex_m as cm;

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

use panic_itm as _;

use rtfm::app;
use rtfm::cyccnt::Instant;
use rtfm::cyccnt::U32Ext;

use hal::adc;
use hal::gpio;
use hal::gpio::{Floating, Input, OpenDrain, Output, PushPull};
use hal::prelude::*;
use hal::stm32;
use hal::timer::Timer;
use stm32f1xx_hal as hal;

use bitbang_hal as bb;

use eziclean::hw::adc::Analog;
use eziclean::hw::beep::Beeper;
use eziclean::hw::clean::Cleaner;
use eziclean::hw::display::Display;
use eziclean::hw::motion::Motion;
use eziclean::hw::poll_timer::PollTimer;
use eziclean::hw::utils::*;
use eziclean::sw::comm::Events;

use heapless::binary_heap::{BinaryHeap, Max};
use heapless::consts::*;

use kxcj9::ic::G8Device;
use kxcj9::{GScale8, Kxcj9, Resolution, SlaveAddr};
use kxcj9::{InterruptPinLatching, InterruptPinPolarity};
use kxcj9::{WakeUpInterruptConfig, WakeUpOutputDataRate, WakeUpTriggerMotion};

use nb;

/* Types */

type ButtonGpioType = gpio::gpiod::PD1<Input<Floating>>;
type ChargerGpioType = gpio::gpioe::PE4<Input<Floating>>;
type DockGpioType = gpio::gpioe::PE5<Input<Floating>>;
type BatteryGpioType = gpio::gpioe::PE6<Input<Floating>>;

type SpiStbType = gpio::gpioa::PA11<Output<PushPull>>;
type SpiDioType = gpio::gpiod::PD14<Output<PushPull>>;
type SpiClkType = gpio::gpioc::PC8<Output<PushPull>>;
type SpiTmpType = gpio::gpioe::PE15<Input<Floating>>;
type SpiScreen = bb::spi::SPI<SpiTmpType, SpiDioType, SpiClkType, PollTimer>;

type I2cSclType = gpio::gpioe::PE7<Output<OpenDrain>>;
type I2cSdaType = gpio::gpiob::PB2<Output<OpenDrain>>;
type I2cAccel = bb::i2c::I2cBB<I2cSclType, I2cSdaType, PollTimer>;

type BeepGpioType = gpio::gpioe::PE0<Output<PushPull>>;

/* */

const PROC_PERIOD: u32 = 400_000; /* 1/20 sec */
const SENSE_PERIOD: u32 = 800_000; /* 1/10 sec */
const POWER_PERIOD: u32 = 80_000_000; /* 10 sec */

/* */

#[app(device = stm32f1xx_hal::stm32, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        // Event queue
        queue: BinaryHeap<Events, U8, Max>,

        // basic hardware resources
        exti: stm32::EXTI,
        itm: stm32::ITM,

        // buttons and chargers
        dock: DockGpioType,
        button: ButtonGpioType,
        charger: ChargerGpioType,
        battery: BatteryGpioType,

        // analog readings
        analog: Analog,

        // Motion control
        drive: Motion,

        // Cleaner control
        cleaner: Cleaner,

        // Display
        screen: Display<SpiScreen, SpiStbType>,

        // Accelerometer
        accel: Kxcj9<I2cAccel, G8Device>,

        // Beeper
        beeper: Beeper<PollTimer, BeepGpioType>,
    }

    #[init(schedule = [proc_task, sense_task, power_task, init_task])]
    fn init(mut cx: init::Context) -> init::LateResources {
        let mut rcc = cx.device.RCC.constrain();
        let dbg = &mut cx.core.ITM.stim[0];

        // setup event queues
        let queue = BinaryHeap(heapless::i::BinaryHeap::new());

        // configure clocks
        let mut flash = cx.device.FLASH.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(8.mhz())
            .pclk1(8.mhz())
            .adcclk(2.mhz())
            .freeze(&mut flash.acr);

        iprintln!(dbg, "SYSCLK: {} Hz ...", clocks.sysclk().0);
        iprintln!(dbg, "ADCCLK: {} Hz ...", clocks.adcclk().0);

        let mut gpioa = cx.device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = cx.device.GPIOC.split(&mut rcc.apb2);
        let mut gpiod = cx.device.GPIOD.split(&mut rcc.apb2);
        let mut gpioe = cx.device.GPIOE.split(&mut rcc.apb2);

        let mut afio = cx.device.AFIO.constrain(&mut rcc.apb2);

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

        let pwm = Timer::tim4(cx.device.TIM4, &clocks, &mut rcc.apb1).pwm(
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

        /*
         * Analog measurements
         *
         */

        let adc = adc::Adc::adc1(cx.device.ADC1, &mut rcc.apb2, clocks);
        let mut a = Analog::init(adc, adc::SampleTime::T_28);

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

        // battery control channels
        let ch1 = gpioa.pa1.into_analog(&mut gpioa.crl);
        let ch2 = gpioa.pa2.into_analog(&mut gpioa.crl);

        a.init_battery_controls(ch1, ch2);

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
        cx.device.EXTI.imr.modify(|_, w| w.mr1().set_bit());
        cx.device.EXTI.ftsr.modify(|_, w| w.tr1().set_bit());
        cx.device.EXTI.rtsr.modify(|_, w| w.tr1().set_bit());

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
        cx.device.EXTI.imr.modify(|_, w| w.mr8().set_bit());
        cx.device.EXTI.rtsr.modify(|_, w| w.tr8().set_bit());

        cx.device.EXTI.imr.modify(|_, w| w.mr12().set_bit());
        cx.device.EXTI.rtsr.modify(|_, w| w.tr12().set_bit());

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
        cx.device.EXTI.imr.modify(|_, w| w.mr4().set_bit());
        cx.device.EXTI.rtsr.modify(|_, w| w.tr4().set_bit());
        cx.device.EXTI.ftsr.modify(|_, w| w.tr4().set_bit());

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
        cx.device.EXTI.imr.modify(|_, w| w.mr5().set_bit());
        cx.device.EXTI.rtsr.modify(|_, w| w.tr5().set_bit());
        cx.device.EXTI.ftsr.modify(|_, w| w.tr5().set_bit());

        /*
         * Battery presence detection
         *
         */

        let battery = gpioe.pe6.into_floating_input(&mut gpioe.crl);

        // select PE6 as source input for line EXTI9_5
        afio.exticr2
            .exticr2()
            .modify(|_, w| unsafe { w.exti6().bits(0b0100) });

        // enable EXTI6 and configure interrupt on rising and falling edge
        cx.device.EXTI.imr.modify(|_, w| w.mr6().set_bit());
        cx.device.EXTI.rtsr.modify(|_, w| w.tr6().set_bit());
        cx.device.EXTI.ftsr.modify(|_, w| w.tr6().set_bit());

        /*
         * Beeper
         *
         */

        let pe0 = gpioe.pe0.into_push_pull_output(&mut gpioe.crl);
        let beep_tmr = PollTimer::init(8.mhz(), 10.hz());
        let beeper = Beeper::create(beep_tmr, pe0);

        /*
         * Cleaner: brushes and pump
         *
         */

        // Use this to configure NJTRST as PB4
        let (_pa15, _pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let pb4 = pb4.into_alternate_push_pull(&mut gpiob.crl);
        let pb5 = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);

        let (pump, brush) = Timer::tim3(cx.device.TIM3, &clocks, &mut rcc.apb1).pwm(
            (pb4, pb5),
            &mut afio.mapr,
            10.khz(),
        );

        let cleaner = Cleaner::create(brush, pump);

        /*
         * Display and Accelerometer
         *
         * Note: there are no spare hardware timers on device:
         * - TIM2 for charging
         * - TIM3 for brushes
         * - TIM4 for main motors
         *
         * Two more timers are needed for accelerometer/display bit-bang i2c/spi. It is possible
         * to use TIM2 in active mode since charging is not needed during normal operations.
         * However rather than sharing TIM2 using some kind of shared_bus approach, here we use
         * simple blocking PollTimer based on SysTick. In fact PollTimer is a delay implementation
         * based on rtfm::Instant.
         *
         * Note that this delay is coarse, since interrupts are not disabled and wait loop can be
         * preempted by any interrupt. This is ok for our use-case though:
         * - we don't want to disable interrupts: need to watch for obstacles
         * - I2C and SPI of the devices in use are fairly tolerant to relaxed clock timings
         *
         * One more note regarding the use of PollTimer. RTFM enables SysTick after init.
         * So in init method there is no way to use PollTimer. That is why here in init method both
         * accelerometer and display handlers are created, but their actual enablement, that
         * requires I2C/SPI communication, is postponed to special s/w task scheduled immediately
         * after SysTick is disabled.
         *
         * This approach should be carefully checked for races. E.g. peripherals (display) may be
         * accessed from other interrupt handlers before post-init task is called.
         * For the peripherals in question (accelerometer and display) this does not lead to any
         * issues.
         */

        // Display

        let tmp = gpioe.pe15.into_floating_input(&mut gpioe.crh);
        let clk = gpioc.pc8.into_push_pull_output(&mut gpioc.crh);
        let dio = gpiod.pd14.into_push_pull_output(&mut gpiod.crh);
        let stb = gpioa.pa11.into_push_pull_output(&mut gpioa.crh);

        let disp_tmr = PollTimer::init(8.mhz(), 200.khz());
        let mut spi = bb::spi::SPI::new(bb::spi::MODE_3, tmp, dio, clk, disp_tmr);
        spi.set_bit_order(bb::spi::BitOrder::LSBFirst);

        let scr = Display::new(spi, stb);

        // Accelerometer

        afio.exticr3
            .exticr3()
            .modify(|_, w| unsafe { w.exti9().bits(0b0100) });

        cx.device.EXTI.imr.modify(|_, w| w.mr9().set_bit());
        cx.device.EXTI.rtsr.modify(|_, w| w.tr9().set_bit());

        let scl = gpioe.pe7.into_open_drain_output(&mut gpioe.crl);
        let sda = gpiob.pb2.into_open_drain_output(&mut gpiob.crl);
        let _irq = gpioe.pe9.into_floating_input(&mut gpioe.crh);

        // NOTE: in release build mode decrease clock to 50kHz
        let acc_tmr = PollTimer::init(8.mhz(), 50.khz());
        let i2c = bb::i2c::I2cBB::new(scl, sda, acc_tmr);
        let address = SlaveAddr::Alternative(true);
        let acc = Kxcj9::new_kxcj9_1008(i2c, address);

        /*
         * Enable the monotonic timer based on CYCCNT
         *
         */
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        /*
         * schedule tasks
         *
         */
        cx.schedule.init_task(Instant::now()).unwrap();
        cx.schedule
            .proc_task(Instant::now() + PROC_PERIOD.cycles())
            .unwrap();
        cx.schedule
            .sense_task(Instant::now() + SENSE_PERIOD.cycles())
            .unwrap();
        cx.schedule
            .power_task(Instant::now() + POWER_PERIOD.cycles())
            .unwrap();

        /*
         * init late resources
         *
         */

        init::LateResources {
            queue: queue,
            itm: cx.core.ITM,
            exti: cx.device.EXTI,
            analog: a,
            drive: m,
            screen: scr,
            accel: acc,
            dock: dock,
            button: button,
            charger: charger,
            battery: battery,
            beeper: beeper,
            cleaner: cleaner,
        }
    }

    /*
     * Idle loop processes events with normal priority
     *
     */
    #[idle]
    fn idle(_: idle::Context) -> ! {
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
    fn proc_task(mut cx: proc_task::Context) {
        let dbg = &mut cx.resources.itm.stim[0];
        let eq = &mut cx.resources.queue;

        while !eq.is_empty() {
            if let Some(e) = eq.pop() {
                iprintln!(dbg, ">>> prio event {:?}", e);
            }
        }

        cx.schedule
            .proc_task(cx.scheduled + PROC_PERIOD.cycles())
            .unwrap();
    }

    /*
     * Late init task: accelerometer and display
     *
     */

    #[task(resources = [itm, screen, accel])]
    fn init_task(cx: init_task::Context) {
        let _dbg = &mut cx.resources.itm.stim[0];
        let scr = cx.resources.screen;
        let acc = cx.resources.accel;

        // init display
        scr.enable().unwrap();

        // init accelerometer
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
    }

    /*
     * Sense: sensor processing task to watch for obstacles using IR obstacle sensors
     *
     */
    #[task(schedule = [sense_task], resources = [itm, analog, queue])]
    fn sense_task(mut cx: sense_task::Context) {
        let _dbg = &mut cx.resources.itm.stim[0];
        let adc = &mut cx.resources.analog;
        let eq = &mut cx.resources.queue;

        if let (Ok(fs1), Ok(fs2)) = (adc.get_front_sensors(false), adc.get_front_sensors(true)) {
            let fll = is_front_obstacle(fs1.fll, fs2.fll);
            let flc = is_front_obstacle(fs1.flc, fs2.flc);
            let fcc = is_front_obstacle(fs1.fcc, fs2.fcc);
            let frc = is_front_obstacle(fs1.frc, fs2.frc);
            let frr = is_front_obstacle(fs1.frr, fs2.frr);

            if fll || flc || fcc || frc || frr {
                eq.push(Events::FrontSensor(fll, flc, fcc, frc, frr)).ok();
            }
        }

        // TODO: read and analyze bottom sensors

        cx.schedule
            .sense_task(cx.scheduled + SENSE_PERIOD.cycles())
            .unwrap();
    }

    /*
     * Power: task checking battery, dock station and charger plug
     *
     */
    #[task(schedule = [power_task], resources = [itm, dock, charger, battery, analog, queue])]
    fn power_task(mut cx: power_task::Context) {
        let _dbg = &mut cx.resources.itm.stim[0];
        let c = cx.resources.charger.is_high().unwrap_or(false);
        let b = cx.resources.battery.is_high().unwrap_or(false);
        let d = cx.resources.dock.is_high().unwrap_or(false);
        let adc = &mut cx.resources.analog;
        let eq = &mut cx.resources.queue;

        eq.push(Events::Charger(c)).ok();
        eq.push(Events::Dock(d)).ok();
        eq.push(Events::Battery(b)).ok();

        if let Ok(batt) = adc.get_battery() {
            if is_battery_low(batt.voltage) {
                eq.push(Events::BatteryLow).ok();
            }
        }

        cx.schedule
            .power_task(cx.scheduled + POWER_PERIOD.cycles())
            .unwrap();
    }

    #[task(binds = EXTI1, resources = [exti, screen, button, queue])]
    fn exti1(mut cx: exti1::Context) {
        let pressed = cx.resources.button.is_low().unwrap_or(false);
        let eq = &mut cx.resources.queue;
        let e = Events::Button(pressed);

        cx.resources.screen.print_num(0000).ok();
        cx.resources.exti.pr.modify(|_, w| w.pr1().set_bit());
        eq.push(e).ok();
    }

    #[task(binds = EXTI4, resources = [exti, charger, screen, queue])]
    fn exti4(mut cx: exti4::Context) {
        let plugged = cx.resources.charger.is_high().unwrap_or(false);
        let eq = &mut cx.resources.queue;
        let e = Events::Charger(plugged);

        cx.resources.screen.print_num(1111).ok();
        cx.resources.exti.pr.modify(|_, w| w.pr4().set_bit());
        eq.push(e).ok();
    }

    #[task(binds = EXTI9_5, resources = [exti, screen, accel, dock, battery, queue])]
    fn exti9_5(mut cx: exti9_5::Context) {
        let eq = &mut cx.resources.queue;
        let r = cx.resources.exti.pr.read();

        if r.pr5().bit_is_set() {
            let d = cx.resources.dock.is_high().unwrap_or(false);
            let e = Events::Dock(d);
            cx.resources.exti.pr.modify(|_, w| w.pr5().set_bit());
            eq.push(e).ok();
        }

        if r.pr6().bit_is_set() {
            let b = cx.resources.battery.is_high().unwrap_or(false);
            let e = Events::Battery(b);
            cx.resources.exti.pr.modify(|_, w| w.pr6().set_bit());
            eq.push(e).ok();
        }

        if r.pr8().bit_is_set() {
            cx.resources.exti.pr.modify(|_, w| w.pr8().set_bit());
            // TODO: collect stats to get rotation speed
        }

        if r.pr9().bit_is_set() {
            let info = cx.resources.accel.read_interrupt_info().unwrap();
            let x = info.wake_up_x_negative | info.wake_up_x_positive;
            let y = info.wake_up_y_negative | info.wake_up_y_positive;
            let z = info.wake_up_z_negative | info.wake_up_z_positive;
            let e = Events::Accel(x, y, z);

            cx.resources.screen.print_num(2222).ok();
            cx.resources.accel.clear_interrupts().unwrap();
            cx.resources.exti.pr.modify(|_, w| w.pr9().set_bit());
            eq.push(e).ok();
        }
    }

    #[task(binds = EXTI15_10, resources = [exti])]
    fn exti15_10(cx: exti15_10::Context) {
        let r = cx.resources.exti.pr.read();

        if r.pr12().bit_is_set() {
            cx.resources.exti.pr.modify(|_, w| w.pr12().set_bit());
            // TODO: collect stats to get rotation speed
        }
    }
};
