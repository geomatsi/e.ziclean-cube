#![allow(clippy::many_single_char_names)]
#![no_main]
#![no_std]

use cm::iprintln;
use cm::singleton;
use cortex_m as cm;

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

use panic_itm as _;

use rtfm::app;
use rtfm::cyccnt::Instant;
use rtfm::cyccnt::U32Ext;

use hal::adc;
use hal::adc::Scan;
use hal::adc::SetChannels;
use hal::dma;
use hal::dma::{Transfer, W};
use hal::gpio;
use hal::gpio::{Analog, Floating, Input, OpenDrain, Output, PushPull};
use hal::prelude::*;
use hal::stm32;
use hal::timer::Timer;
use stm32f1xx_hal as hal;

use bitbang_hal as bb;

use eziclean::hw::beep::Beeper;
use eziclean::hw::clean::Cleaner;
use eziclean::hw::display::Display;
use eziclean::hw::ir_rc::IRReceiver;
use eziclean::hw::ir_rc::ReceiverResult;
use eziclean::hw::motion::Motion;
use eziclean::hw::poll_timer::PollTimer;
use eziclean::hw::utils::*;

use eziclean::sw::brain::debug::DebugBrain;
use eziclean::sw::brain::Brain;
use eziclean::sw::comm::Actions;
use eziclean::sw::comm::Events;
use eziclean::sw::comm::View;

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
type InfraredTopGpioType = gpio::gpiod::PD15<Input<Floating>>;

type FrontSensorLedType = gpio::gpioc::PC7<Output<PushPull>>;
type BottomSensorLedType = gpio::gpiod::PD9<Output<PushPull>>;

type SpiStbType = gpio::gpioa::PA11<Output<PushPull>>;
type SpiDioType = gpio::gpiod::PD14<Output<PushPull>>;
type SpiClkType = gpio::gpioc::PC8<Output<PushPull>>;
type SpiTmpType = gpio::gpioe::PE15<Input<Floating>>;
type SpiScreen = bb::spi::SPI<SpiTmpType, SpiDioType, SpiClkType, PollTimer>;

type I2cSclType = gpio::gpioe::PE7<Output<OpenDrain>>;
type I2cSdaType = gpio::gpiob::PB2<Output<OpenDrain>>;
type I2cAccel = bb::i2c::I2cBB<I2cSclType, I2cSdaType, PollTimer>;

type AdcDmaType = adc::AdcDma<AdcPins, Scan>;
type DmaBufType = &'static mut [u16; 10];
type FrontSensorsBufType = &'static mut [u16; 5];
type BottomSensorsBufType = &'static mut [u16; 3];

/* structs */

pub struct AdcPins(
    gpio::gpioa::PA1<Analog>,
    gpio::gpioa::PA2<Analog>,
    gpio::gpioa::PA4<Analog>,
    gpio::gpioa::PA6<Analog>,
    gpio::gpioa::PA7<Analog>,
    gpio::gpiob::PB0<Analog>,
    gpio::gpiob::PB1<Analog>,
    gpio::gpioc::PC0<Analog>,
    gpio::gpioc::PC1<Analog>,
    gpio::gpioc::PC5<Analog>,
);

impl SetChannels<AdcPins> for adc::Adc<stm32::ADC1> {
    fn set_samples(&mut self) {
        self.set_channel_sample_time(1, adc::SampleTime::T_28);
        self.set_channel_sample_time(2, adc::SampleTime::T_28);
        self.set_channel_sample_time(4, adc::SampleTime::T_28);
        self.set_channel_sample_time(6, adc::SampleTime::T_28);
        self.set_channel_sample_time(7, adc::SampleTime::T_28);
        self.set_channel_sample_time(8, adc::SampleTime::T_28);
        self.set_channel_sample_time(9, adc::SampleTime::T_28);
        self.set_channel_sample_time(10, adc::SampleTime::T_28);
        self.set_channel_sample_time(11, adc::SampleTime::T_28);
        self.set_channel_sample_time(15, adc::SampleTime::T_28);
    }

    fn set_sequence(&mut self) {
        self.set_regular_sequence(&[1, 2, 4, 6, 7, 8, 9, 10, 11, 15]);
    }
}

pub struct AdcDmaMode {
    count: u8,
    obstacle: bool,
    drop: bool,
    leds: bool,
}

/* cpu sysclk: 8MHz (no external quartz) */

const MSEC_PERIOD: u32 = 8000; /* 1 msec */
const PROC_PERIOD: u32 = 400_000; /* 50 msec */
const SENSE_PERIOD: u32 = 800_000; /* 100 msec */
const POWER_PERIOD: u32 = 80_000_000; /* 10 sec */
const HBEAT_PERIOD: u32 = 4_000_000; /* 500 msec */

const IR_FREQ: u32 = 5000;
const IR_PERIOD: u32 = 8_000_000 / IR_FREQ;
const IR_WAIT_PERIOD: u32 = 4_000_000; /* 1/2 sec */
const IR_ERR_PERIOD: u32 = 200_000; /* 25 msec */

/* */

#[app(device = stm32f1xx_hal::stm32, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        // controller: 'Brain'
        brain: DebugBrain,

        // basic hardware resources
        exti: stm32::EXTI,
        itm: stm32::ITM,

        // buttons and chargers
        dock: DockGpioType,
        button: ButtonGpioType,
        charger: ChargerGpioType,
        battery: BatteryGpioType,

        // analog readings using DMA
        transfer: Option<Transfer<W, DmaBufType, AdcDmaType>>,
        adc_dma: Option<AdcDmaType>,
        buffer: Option<DmaBufType>,
        mode: AdcDmaMode,

        front_leds: FrontSensorLedType,
        front_buf: FrontSensorsBufType,

        bottom_leds: BottomSensorLedType,
        bottom_buf: BottomSensorsBufType,

        // Motion control
        drive: Motion,

        // Cleaner control
        cleaner: Cleaner,

        // Display
        screen: Display<SpiScreen, SpiStbType>,

        // Accelerometer
        accel: Kxcj9<I2cAccel, G8Device>,

        // Beeper
        beeper: Beeper<PollTimer>,

        // IR RC
        ir_pin: InfraredTopGpioType,
        ir_decoder: IRReceiver,
        ir_count: u32,
    }

    #[init(schedule = [proc_task, start_adc_dma_task, power_task, init_task, ir_decode_task, ir_enable_task, beep_stop_task, hb_task])]
    fn init(mut cx: init::Context) -> init::LateResources {
        let mut rcc = cx.device.RCC.constrain();
        let dbg = &mut cx.core.ITM.stim[0];

        // setup control center
        let brain = DebugBrain::create();

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
         * Analog measurements: ADC over DMA
         *
         */

        // front sensors LEDs : setup and disable
        let mut front_leds = gpioc.pc7.into_push_pull_output(&mut gpioc.crl);
        front_leds.set_low().unwrap();

        // front sensors buffer
        let front_buffer = singleton!(: [u16; 5] = [4095; 5]).unwrap();

        // front sensors channels
        let ch4 = gpioa.pa4.into_analog(&mut gpioa.crl);
        let ch6 = gpioa.pa6.into_analog(&mut gpioa.crl);
        let ch8 = gpiob.pb0.into_analog(&mut gpiob.crl);
        let ch10 = gpioc.pc0.into_analog(&mut gpioc.crl);
        let ch15 = gpioc.pc5.into_analog(&mut gpioc.crl);

        // bottom sensors LEDs: setup and disable
        let mut bottom_leds = gpiod.pd9.into_push_pull_output(&mut gpiod.crh);
        bottom_leds.set_low().unwrap();

        // front sensors buffer
        let bottom_buffer = singleton!(: [u16; 3] = [4095; 3]).unwrap();

        // bottom sensor channels
        let ch11 = gpioc.pc1.into_analog(&mut gpioc.crl);
        let ch7 = gpioa.pa7.into_analog(&mut gpioa.crl);
        let ch9 = gpiob.pb1.into_analog(&mut gpiob.crl);

        // battery control channels
        let ch1 = gpioa.pa1.into_analog(&mut gpioa.crl);
        let ch2 = gpioa.pa2.into_analog(&mut gpioa.crl);

        // dma channel #1
        let mut dma_ch1 = cx.device.DMA1.split(&mut rcc.ahb).1;
        dma_ch1.listen(dma::Event::TransferComplete);

        // setup ADC
        let adc = adc::Adc::adc1(cx.device.ADC1, &mut rcc.apb2, clocks);

        // configure ADC+DMA
        let adc_pins = AdcPins(ch1, ch2, ch4, ch6, ch7, ch8, ch9, ch10, ch11, ch15);
        let adc_dma = adc.with_scan_dma(adc_pins, dma_ch1);
        let buffer = singleton!(: [u16; 10] = [0; 10]).unwrap();

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
        /* TODO:
         *  - Encoders IR LEDs are active low
         *  - disable since no usage for that data so far
         */
        let mut pd13 = gpiod.pd13.into_open_drain_output(&mut gpiod.crh);
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

        // Infrared Remote Control

        // top IR diode: signal connected to PD15
        let irr = gpiod.pd15.into_floating_input(&mut gpiod.crh);
        let ird = IRReceiver::new(IR_FREQ);

        // select PD15 as source input for line EXTI10_15
        afio.exticr4
            .exticr4()
            .modify(|_, w| unsafe { w.exti15().bits(0b0011) });

        // configure interrupt on falling edge
        cx.device.EXTI.ftsr.modify(|_, w| w.tr15().set_bit());
        cx.device.EXTI.rtsr.modify(|_, w| w.tr15().clear_bit());

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
            .start_adc_dma_task(Instant::now() + SENSE_PERIOD.cycles())
            .unwrap();
        cx.schedule
            .power_task(Instant::now() + POWER_PERIOD.cycles())
            .unwrap();
        cx.schedule
            .ir_enable_task(Instant::now() + IR_WAIT_PERIOD.cycles())
            .unwrap();
        cx.schedule
            .hb_task(Instant::now() + HBEAT_PERIOD.cycles())
            .unwrap();

        /*
         * init late resources
         *
         */

        init::LateResources {
            brain: brain,
            itm: cx.core.ITM,
            exti: cx.device.EXTI,
            transfer: None,
            adc_dma: Some(adc_dma),
            buffer: Some(buffer),
            front_buf: front_buffer,
            front_leds: front_leds,
            bottom_buf: bottom_buffer,
            bottom_leds: bottom_leds,
            mode: AdcDmaMode {
                count: 0,
                obstacle: false,
                drop: false,
                leds: false,
            },
            drive: m,
            screen: scr,
            accel: acc,
            dock: dock,
            button: button,
            charger: charger,
            battery: battery,
            beeper: beeper,
            cleaner: cleaner,
            ir_decoder: ird,
            ir_count: 0,
            ir_pin: irr,
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
     * Brain: main processing task
     *
     */
    #[task(schedule = [proc_task, beep_stop_task], resources = [itm, brain, beeper, cleaner, drive, screen])]
    fn proc_task(cx: proc_task::Context) {
        let dbg = &mut cx.resources.itm.stim[0];

        while cx.resources.brain.ready() {
            match cx.resources.brain.action() {
                Actions::None => {
                    break;
                }
                Actions::Beep(msecs) => {
                    cx.resources.beeper.beep_on();
                    cx.schedule
                        .beep_stop_task(Instant::now() + (msecs * MSEC_PERIOD).cycles())
                        .unwrap();
                }
                Actions::Debug(e) => {
                    iprintln!(dbg, "event: {:?}", e);
                }
                Actions::Clean(c) => {
                    cx.resources.cleaner.clean(c);
                }
                Actions::Motion(direction, gear) => {
                    cx.resources.drive.motion(direction, gear).unwrap();
                }
                Actions::Rotation(direction, gear) => {
                    cx.resources.drive.rotate(direction, gear).unwrap();
                }
                Actions::Wheels((ld, lg), (rd, rg)) => {
                    cx.resources.drive.set_left_wheel(ld, lg).unwrap();
                    cx.resources.drive.set_right_wheel(rd, rg).unwrap();
                }
                Actions::Display(data) => match data {
                    View::Number(num) => {
                        cx.resources.screen.print_num(num).unwrap();
                    }
                    View::Time(hh, mm, cn) => {
                        cx.resources.screen.print_time([hh, mm], cn).unwrap();
                    }
                    View::Digits(nums) => {
                        cx.resources.screen.print4(nums).unwrap();
                    }
                    View::Clear => {
                        cx.resources.screen.clear().unwrap();
                    }
                },
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
     * Decoding Infrared Remote Control messages
     *
     */
    #[task(schedule = [ir_decode_task, ir_enable_task], resources = [itm, ir_pin, ir_count, ir_decoder, exti, brain])]
    fn ir_decode_task(cx: ir_decode_task::Context) {
        let dbg = &mut cx.resources.itm.stim[0];
        let val = cx.resources.ir_pin.is_high().unwrap();

        match cx.resources.ir_decoder.sample(val, *cx.resources.ir_count) {
            ReceiverResult::Done(v) => {
                let e = Events::InfraredCommand(v);
                cx.resources.brain.notify(e);
                cx.resources.ir_decoder.reset();
                *cx.resources.ir_count = 0;
                cx.schedule
                    .ir_enable_task(Instant::now() + IR_WAIT_PERIOD.cycles())
                    .unwrap();
            }
            ReceiverResult::Fail(e) => {
                iprintln!(dbg, "ir: err: {}", e);
                cx.resources.ir_decoder.reset();
                *cx.resources.ir_count = 0;
                cx.schedule
                    .ir_enable_task(Instant::now() + IR_ERR_PERIOD.cycles())
                    .unwrap();
            }
            ReceiverResult::Proc => {
                *cx.resources.ir_count = cx.resources.ir_count.wrapping_add(1);
                cx.schedule
                    .ir_decode_task(cx.scheduled + IR_PERIOD.cycles())
                    .unwrap();
            }
        }
    }

    #[task(schedule = [hb_task], resources = [brain])]
    fn hb_task(cx: hb_task::Context) {
        cx.resources.brain.notify(Events::HeartBeat);
        cx.schedule
            .hb_task(cx.scheduled + HBEAT_PERIOD.cycles())
            .unwrap();
    }

    #[task(resources = [beeper])]
    fn beep_stop_task(cx: beep_stop_task::Context) {
        cx.resources.beeper.beep_off();
    }

    #[task(resources = [exti])]
    fn ir_enable_task(cx: ir_enable_task::Context) {
        // enable IR RC interrupts
        cx.resources.exti.imr.modify(|_, w| w.mr15().set_bit());
    }

    /*
     * Sense: analog readings using DMA
     *   - start_adc_dma_task starts ADC/DMA scan
     *   - DMA_CHANNEL1 interrupt reads results
     *   - DMA_CHANNEL1 interrupt schedules start_adc_dma_task
     */
    #[task(binds = DMA1_CHANNEL1, schedule = [start_adc_dma_task], resources = [itm, transfer, adc_dma, buffer, mode, front_leds, bottom_leds, front_buf, bottom_buf, brain])]
    fn dma1_channel1(mut cx: dma1_channel1::Context) {
        let dbg = &mut cx.resources.itm.stim[0];

        if let Some(xfer) = cx.resources.transfer.take() {
            let (buf, mut scan) = xfer.wait();

            /* increment adc dma counter */

            cx.resources.mode.count = cx.resources.mode.count.wrapping_add(1);

            /* turn off sensor IR LEDs */

            cx.resources.front_leds.set_low().unwrap();
            cx.resources.bottom_leds.set_low().unwrap();

            /* check front sensor */

            let fll = is_front_obstacle(buf[7], cx.resources.front_buf[0]);
            let flc = is_front_obstacle(buf[2], cx.resources.front_buf[1]);
            let fcc = is_front_obstacle(buf[3], cx.resources.front_buf[2]);
            let frc = is_front_obstacle(buf[9], cx.resources.front_buf[3]);
            let frr = is_front_obstacle(buf[5], cx.resources.front_buf[4]);

            cx.resources.front_buf[0] = buf[7];
            cx.resources.front_buf[1] = buf[2];
            cx.resources.front_buf[2] = buf[3];
            cx.resources.front_buf[3] = buf[9];
            cx.resources.front_buf[4] = buf[5];

            if fll || flc || fcc || frc || frr {
                cx.resources
                    .brain
                    .notify(Events::FrontSensor(fll, flc, fcc, frc, frr));
                cx.resources.mode.obstacle = true;
            } else {
                if cx.resources.mode.obstacle {
                    cx.resources
                        .brain
                        .notify(Events::FrontSensor(fll, flc, fcc, frc, frr));
                    cx.resources.mode.obstacle = false;
                }
            }

            /* check bottom sensor */

            let bl = is_bottom_drop(buf[8], cx.resources.bottom_buf[0]);
            let bc = is_bottom_drop(buf[4], cx.resources.bottom_buf[1]);
            let br = is_bottom_drop(buf[6], cx.resources.bottom_buf[2]);

            cx.resources.bottom_buf[0] = buf[8];
            cx.resources.bottom_buf[1] = buf[4];
            cx.resources.bottom_buf[2] = buf[6];

            if bl || bc || br {
                cx.resources.brain.notify(Events::BottomSensor(bl, bc, br));
                cx.resources.mode.drop = true;
            } else {
                if cx.resources.mode.drop {
                    cx.resources.brain.notify(Events::BottomSensor(bl, bc, br));
                    cx.resources.mode.drop = false;
                }
            }

            /* check battery voltage once per 255 cycles: one-shot adc read is needed for vref */

            if cx.resources.mode.count == 0 {
                let (mut adc, mut pins, chan) = scan.split();

                let v_ref: u32 = adc.read_vref().into();
                let v_ch1: u32 = adc.read(&mut pins.0).unwrap();
                // As per PCB investigation, battery voltage divider:
                // v_ch1 (mV) = v_bat * 20k / (200k + 20k) */
                let v_bat: u32 = 11 * v_ch1 * 1200 / v_ref;

                if is_battery_low(v_bat) {
                    cx.resources.brain.notify(Events::BatteryLow);
                }

                scan = adc.with_scan_dma(pins, chan);
            }

            /* TODO: check battery charger current */
            /* TODO: check motors current */

            /* done with ADC checks */

            *cx.resources.adc_dma = Some(scan);
            *cx.resources.buffer = Some(buf);

            cx.schedule
                .start_adc_dma_task(Instant::now() + SENSE_PERIOD.cycles())
                .unwrap();
        } else {
            iprintln!(dbg, "DMA1_CH1 IRQ: ERR: no xfer");
        }
    }

    #[task(resources = [itm, transfer, adc_dma, buffer, front_leds, bottom_leds, mode])]
    fn start_adc_dma_task(cx: start_adc_dma_task::Context) {
        let dbg = &mut cx.resources.itm.stim[0];

        if let (Some(scan), Some(buf)) = (cx.resources.adc_dma.take(), cx.resources.buffer.take()) {
            cx.resources.mode.leds = match cx.resources.mode.leds {
                true => {
                    cx.resources.front_leds.set_high().unwrap();
                    cx.resources.bottom_leds.set_high().unwrap();
                    false
                }
                false => {
                    cx.resources.front_leds.set_low().unwrap();
                    cx.resources.bottom_leds.set_low().unwrap();
                    true
                }
            };

            /* start next adc dma transfer */
            let xfer = scan.read(buf);
            *cx.resources.transfer = Some(xfer);
        } else {
            iprintln!(dbg, "IDLE: ERR: no rdma");
        }
    }

    /*
     * Power: task checking battery, dock station and charger plug
     *
     */
    #[task(schedule = [power_task], resources = [itm, dock, charger, battery, brain])]
    fn power_task(cx: power_task::Context) {
        let _dbg = &mut cx.resources.itm.stim[0];
        let c = cx.resources.charger.is_high().unwrap_or(false);
        let b = cx.resources.battery.is_high().unwrap_or(false);
        let d = cx.resources.dock.is_high().unwrap_or(false);

        cx.resources.brain.notify(Events::Charger(c));
        cx.resources.brain.notify(Events::Dock(d));
        cx.resources.brain.notify(Events::Battery(b));

        cx.schedule
            .power_task(cx.scheduled + POWER_PERIOD.cycles())
            .unwrap();
    }

    #[task(binds = EXTI1, resources = [exti, screen, button, brain])]
    fn exti1(cx: exti1::Context) {
        let pressed = cx.resources.button.is_low().unwrap_or(false);
        let e = Events::Button(pressed);

        cx.resources.screen.print_num(0000).ok();
        cx.resources.exti.pr.modify(|_, w| w.pr1().set_bit());
        cx.resources.brain.notify(e);
    }

    #[task(binds = EXTI4, resources = [exti, charger, screen, brain])]
    fn exti4(cx: exti4::Context) {
        let plugged = cx.resources.charger.is_high().unwrap_or(false);
        let e = Events::Charger(plugged);

        cx.resources.screen.print_num(1111).ok();
        cx.resources.exti.pr.modify(|_, w| w.pr4().set_bit());
        cx.resources.brain.notify(e);
    }

    #[task(binds = EXTI9_5, resources = [exti, screen, accel, dock, battery, brain])]
    fn exti9_5(cx: exti9_5::Context) {
        let r = cx.resources.exti.pr.read();

        if r.pr5().bit_is_set() {
            let d = cx.resources.dock.is_high().unwrap_or(false);
            let e = Events::Dock(d);
            cx.resources.exti.pr.modify(|_, w| w.pr5().set_bit());
            cx.resources.brain.notify(e);
        }

        if r.pr6().bit_is_set() {
            let b = cx.resources.battery.is_high().unwrap_or(false);
            let e = Events::Battery(b);
            cx.resources.exti.pr.modify(|_, w| w.pr6().set_bit());
            cx.resources.brain.notify(e);
        }

        if r.pr8().bit_is_set() {
            cx.resources.exti.pr.modify(|_, w| w.pr8().set_bit());
            // TODO: collect right wheel encoder data
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
            cx.resources.brain.notify(e);
        }
    }

    #[task(binds = EXTI15_10, schedule = [ir_decode_task], resources = [exti])]
    fn exti15_10(cx: exti15_10::Context) {
        let r = cx.resources.exti.pr.read();

        if r.pr12().bit_is_set() {
            cx.resources.exti.pr.modify(|_, w| w.pr12().set_bit());
            // TODO: collect left wheel encoder data
        }

        if r.pr15().bit_is_set() {
            // acknowledge and disable IR pin interrupt
            cx.resources.exti.pr.modify(|_, w| w.pr15().set_bit());
            cx.resources.exti.imr.modify(|_, w| w.mr15().clear_bit());
            cx.schedule
                .ir_decode_task(Instant::now() + IR_PERIOD.cycles())
                .unwrap();
        }
    }

    /*
     * This interrupt is used to dispatch timer queue tasks.
     * TODO:  not all interrupts work here, e.g. UART4 does not work
     *
     */
    extern "C" {
        fn EXTI2();
    }
};
