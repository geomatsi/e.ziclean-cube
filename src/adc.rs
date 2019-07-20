#![allow(deprecated)]

use embedded_hal::adc::OneShot;

use super::*;

// ADC1
type ADC = hal::adc::Adc<hal::stm32::ADC1>;

// front IR sensor types
type FLL = hal::gpio::gpioc::PC0<hal::gpio::Analog>;
type FLC = hal::gpio::gpioa::PA4<hal::gpio::Analog>;
type FCC = hal::gpio::gpioa::PA6<hal::gpio::Analog>;
type FRC = hal::gpio::gpioc::PC5<hal::gpio::Analog>;
type FRR = hal::gpio::gpiob::PB0<hal::gpio::Analog>;

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
    pub voltage: u16,
    pub current: u16,
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

    pub fn init_front_sensors(&mut self, fll: FLL, flc: FLC, fcc: FCC, frc: FRC, frr: FRR) {
        self.front = Some(FrontSensors {
            fll,
            flc,
            fcc,
            frc,
            frr,
        });
    }

    pub fn get_front_sensors(&mut self) -> Result<FrontSensorsData, Error> {
        if let Some(ref mut f) = self.front {
            Ok(FrontSensorsData {
                fll: self.adc.read(&mut f.fll).unwrap(),
                flc: self.adc.read(&mut f.flc).unwrap(),
                fcc: self.adc.read(&mut f.fcc).unwrap(),
                frc: self.adc.read(&mut f.frc).unwrap(),
                frr: self.adc.read(&mut f.frr).unwrap(),
            })
        } else {
            Err(Error::Uninitialized)
        }
    }

    pub fn init_bottom_sensors(&mut self, bl: BL, bc: BC, br: BR) {
        self.bottom = Some(BottomSensors { bl, bc, br });
    }

    pub fn get_bottom_sensors(&mut self) -> Result<BottomSensorsData, Error> {
        if let Some(ref mut b) = self.bottom {
            Ok(BottomSensorsData {
                bl: self.adc.read(&mut b.bl).unwrap(),
                bc: self.adc.read(&mut b.bc).unwrap(),
                br: self.adc.read(&mut b.br).unwrap(),
            })
        } else {
            Err(Error::Uninitialized)
        }
    }

    pub fn init_battery_controls(&mut self, voltage: BatteryVoltage, current: BatteryCurrent) {
        self.battery = Some(Battery { voltage, current });
    }

    pub fn get_battery(&mut self) -> Result<BatteryData, Error> {
        if let Some(ref mut b) = self.battery {
            Ok(BatteryData {
                voltage: self.adc.read(&mut b.voltage).unwrap(),
                current: self.adc.read(&mut b.current).unwrap(),
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
