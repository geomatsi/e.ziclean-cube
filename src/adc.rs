use embedded_hal::adc::OneShot;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal as hal;

// ADC1
type ADC = hal::adc::Adc<hal::stm32::ADC1>;

// front LED type
type FLD = hal::gpio::gpioc::PC7<hal::gpio::Output<hal::gpio::PushPull>>;

// front IR sensor types
type FLL = hal::gpio::gpioc::PC0<hal::gpio::Analog>;
type FLC = hal::gpio::gpioa::PA4<hal::gpio::Analog>;
type FCC = hal::gpio::gpioa::PA6<hal::gpio::Analog>;
type FRC = hal::gpio::gpioc::PC5<hal::gpio::Analog>;
type FRR = hal::gpio::gpiob::PB0<hal::gpio::Analog>;

// bottom LED type
type BLD = hal::gpio::gpiod::PD9<hal::gpio::Output<hal::gpio::PushPull>>;

// bottom IR sensor types
type BL = hal::gpio::gpioc::PC1<hal::gpio::Analog>;
type BC = hal::gpio::gpioa::PA7<hal::gpio::Analog>;
type BR = hal::gpio::gpiob::PB1<hal::gpio::Analog>;

// battery control types
type BatteryVoltage = hal::gpio::gpioa::PA1<hal::gpio::Analog>;
type BatteryCurrent = hal::gpio::gpioa::PA2<hal::gpio::Analog>;

// motor current control types
type RightMotorCurrent = hal::gpio::gpioc::PC4<hal::gpio::Analog>;
type LeftMotorCurrent = hal::gpio::gpioa::PA5<hal::gpio::Analog>;
type BrushCurrent = hal::gpio::gpioc::PC2<hal::gpio::Analog>;
type PumpCurrent = hal::gpio::gpioc::PC3<hal::gpio::Analog>;

/// Front sensors
struct FrontSensors {
    led: FLD,
    fll: FLL,
    flc: FLC,
    fcc: FCC,
    frc: FRC,
    frr: FRR,
}

/// Front sensor measurements
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub struct FrontSensorsData {
    pub fll: u16,
    pub flc: u16,
    pub fcc: u16,
    pub frc: u16,
    pub frr: u16,
}

/// Bottom sensors
struct BottomSensors {
    led: BLD,
    bl: BL,
    bc: BC,
    br: BR,
}

/// Bottom sensor measurements
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub struct BottomSensorsData {
    pub bl: u16,
    pub bc: u16,
    pub br: u16,
}

/// Battery controls
struct Battery {
    voltage: BatteryVoltage,
    current: BatteryCurrent,
}

/// Battery measurements
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub struct BatteryData {
    pub voltage: u32,
    pub current: u32,
}

/// Motor current controls
struct Motor {
    left: LeftMotorCurrent,
    right: RightMotorCurrent,
    brush: BrushCurrent,
    pump: PumpCurrent,
}

/// Motor current measurements
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub struct MotorData {
    pub left: u16,
    pub right: u16,
    pub brush: u16,
    pub pump: u16,
}

/// Analog errors
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Error {
    /// Function not intialized
    Uninitialized,
    /// Hardware error
    HwError,
}

/// Analog
pub struct Analog {
    /// ADC block
    adc: ADC,
    /// front sensors
    front: Option<FrontSensors>,
    /// bottom sensors
    bottom: Option<BottomSensors>,
    /// battery controls
    battery: Option<Battery>,
    /// motor current controls
    motor: Option<Motor>,
}

impl Analog {
    pub fn init(adc: ADC, sample: hal::adc::AdcSampleTime) -> Self {
        let mut a = Analog {
            adc,
            front: None,
            bottom: None,
            battery: None,
            motor: None,
        };

        a.adc.set_sample_time(sample);

        a
    }

    pub fn get_max_sample(&mut self) -> u16 {
        self.adc.max_sample()
    }

    pub fn init_front_sensors(
        &mut self,
        led: FLD,
        fll: FLL,
        flc: FLC,
        fcc: FCC,
        frc: FRC,
        frr: FRR,
    ) {
        self.front = Some(FrontSensors {
            led,
            fll,
            flc,
            fcc,
            frc,
            frr,
        });
    }

    pub fn get_front_sensors(&mut self, led: bool) -> Result<FrontSensorsData, Error> {
        if let Some(ref mut f) = self.front {
            if led {
                f.led.set_high().map_err(|_| Error::HwError)?;
            }

            let data = FrontSensorsData {
                flc: self.adc.read(&mut f.flc).unwrap(),
                fcc: self.adc.read(&mut f.fcc).unwrap(),
                frc: self.adc.read(&mut f.frc).unwrap(),
                frr: self.adc.read(&mut f.frr).unwrap(),
                // FIXME: FLL diode seems to be too slow: no reaction right after LED is on
                fll: self.adc.read(&mut f.fll).unwrap(),
            };

            if led {
                f.led.set_low().map_err(|_| Error::HwError)?;
            }

            Ok(data)
        } else {
            Err(Error::Uninitialized)
        }
    }

    pub fn init_bottom_sensors(&mut self, led: BLD, bl: BL, bc: BC, br: BR) {
        self.bottom = Some(BottomSensors { led, bl, bc, br });
    }

    pub fn get_bottom_sensors(&mut self, led: bool) -> Result<BottomSensorsData, Error> {
        if let Some(ref mut b) = self.bottom {
            if led {
                b.led.set_high().map_err(|_| Error::HwError)?;
            }

            let data = BottomSensorsData {
                bl: self.adc.read(&mut b.bl).unwrap(),
                bc: self.adc.read(&mut b.bc).unwrap(),
                br: self.adc.read(&mut b.br).unwrap(),
            };

            if led {
                b.led.set_low().map_err(|_| Error::HwError)?;
            }

            Ok(data)
        } else {
            Err(Error::Uninitialized)
        }
    }

    pub fn init_battery_controls(&mut self, voltage: BatteryVoltage, current: BatteryCurrent) {
        self.battery = Some(Battery { voltage, current });
    }

    pub fn get_battery(&mut self) -> Result<BatteryData, Error> {
        if let Some(ref mut b) = self.battery {
            // Read ADC values
            let v_ref: u32 = self.adc.read_vref().into();
            let v_ch1: u32 = self.adc.read(&mut b.voltage).unwrap();
            let v_ch2: u32 = self.adc.read(&mut b.current).unwrap();

            // As per PCB investigation, battery voltage divider:
            // v_ch1 (mV) = v_bat * 20k / (200k + 20k) */
            let v_bat: u32 = 11 * v_ch1 * 1200 / v_ref;

            // As per PCB investigation, OpAmp-1 (voltage subtractor) + OpAmp-2 (voltage follower):
            // v_ch2 (mV) = v_shunt * (200k / 10k)
            let v_shunt: u32 = v_ch2 * 1200 * 10 / v_ref / 200;
            // TODO: convert shunt voltage to current in mA

            Ok(BatteryData {
                voltage: v_bat,
                current: v_shunt,
            })
        } else {
            Err(Error::Uninitialized)
        }
    }

    pub fn init_motor_controls(
        &mut self,
        left: LeftMotorCurrent,
        right: RightMotorCurrent,
        brush: BrushCurrent,
        pump: PumpCurrent,
    ) {
        self.motor = Some(Motor {
            left,
            right,
            brush,
            pump,
        });
    }

    pub fn get_motors(&mut self) -> Result<MotorData, Error> {
        if let Some(ref mut m) = self.motor {
            Ok(MotorData {
                left: self.adc.read(&mut m.left).unwrap(),
                right: self.adc.read(&mut m.right).unwrap(),
                brush: self.adc.read(&mut m.brush).unwrap(),
                pump: self.adc.read(&mut m.pump).unwrap(),
            })
        } else {
            Err(Error::Uninitialized)
        }
    }
}
