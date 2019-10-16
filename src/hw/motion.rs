use crate::sw::comm::{Direction, Gear, Rotation};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;
use stm32f1xx_hal as hal;

// left wheel gear control
type LeftWheelRevGear = hal::pwm::Pwm<hal::device::TIM4, hal::pwm::C1>;
type LeftWheelFwdGear = hal::pwm::Pwm<hal::device::TIM4, hal::pwm::C2>;

type LeftGear = (LeftWheelFwdGear, LeftWheelRevGear);

// right wheel gear control
type RightWheelFwdGear = hal::pwm::Pwm<hal::device::TIM4, hal::pwm::C3>;
type RightWheelRevGear = hal::pwm::Pwm<hal::device::TIM4, hal::pwm::C4>;

type RightGear = (RightWheelFwdGear, RightWheelRevGear);

// left wheel dir controls
type LeftWheelFwdControl = hal::gpio::gpiod::PD5<hal::gpio::Output<hal::gpio::OpenDrain>>;
type LeftWheelRevControl = hal::gpio::gpiod::PD2<hal::gpio::Output<hal::gpio::OpenDrain>>;

type LeftDir = (LeftWheelFwdControl, LeftWheelRevControl);

// right wheel dir controls
type RightWheelFwdControl = hal::gpio::gpiob::PB11<hal::gpio::Output<hal::gpio::OpenDrain>>;
type RightWheelRevControl = hal::gpio::gpioe::PE14<hal::gpio::Output<hal::gpio::OpenDrain>>;

type RightDir = (RightWheelFwdControl, RightWheelRevControl);

/// e.ziclean motion
///
/// Pins PB6-PB9 (TIM4 output channels):
///
/// PB6 (TIM4_CH1) left wheel reverse speed
/// PB7 (TIM4_CH2) left wheel forward speed
/// PB8 (TIM4_CH3) right wheel forward speed
/// PB9 (TIM4_CH4) right wheel reverse speed
///
/// Pins PD2, PD5: left main motor direction control
///  _________________________
/// | PD2  | PD5  | FWD | REV |
///  _________________________
/// |  0   |   0  |  -  |  -  |
///  _________________________
/// |  1   |   0  |  -  |  +  |
///  _________________________
/// |  0   |   1  |  +  |  -  |
///  _________________________
/// |  1   |   1  |  -  |  -  |
///  _________________________
///
/// Pins PE14, PB11: right main motor direction control
///  _________________________
/// | PE14 | PB11 | FWD | REV |
///  _________________________
/// |  0   |   0  |  -  |  -  |
///  _________________________
/// |  1   |   0  |  -  |  +  |
///  _________________________
/// |  0   |   1  |  +  |  -  |
///  _________________________
/// |  1   |   1  |  -  |  -  |
///  _________________________

/// Motion error
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Error {
    /// HardwareError
    HardwareError,
}

impl Into<usize> for Gear {
    fn into(self) -> usize {
        match self {
            Gear::Low => 0,
            Gear::Medium => 1,
            Gear::Top => 2,
        }
    }
}

/// Left Wheel
struct LeftWheel {
    gc: LeftGear,
    dc: LeftDir,
}

/// Right Wheel
struct RightWheel {
    gc: RightGear,
    dc: RightDir,
}

/// Motion
pub struct Motion {
    /// Right wheel state
    right: RightWheel,
    /// Left wheel state
    left: LeftWheel,
    /// max pwm duty
    max_duty: u16,
    /// gear pwm duty options
    duty: [u16; 3],
}

impl Motion {
    pub fn init(
        (lgf, lgr): LeftGear,
        (rgf, rgr): RightGear,
        (lcf, lcr): LeftDir,
        (rcf, rcr): RightDir,
    ) -> Self {
        let mut m = Motion {
            left: LeftWheel {
                gc: (lgf, lgr),
                dc: (lcf, lcr),
            },
            right: RightWheel {
                gc: (rgf, rgr),
                dc: (rcf, rcr),
            },
            duty: [0; 3],
            max_duty: 0,
        };

        let max_duty = m.right.gc.0.get_max_duty();

        m.left.dc.0.set_low().ok();
        m.left.dc.1.set_low().ok();

        // NB: max duty corresponds to lowest gear
        m.left.gc.0.set_duty(max_duty);
        m.left.gc.1.set_duty(max_duty);

        m.left.gc.0.enable();
        m.left.gc.1.enable();

        m.right.dc.0.set_low().ok();
        m.right.dc.1.set_low().ok();

        // NB: max duty corresponds to lowest gear
        m.right.gc.0.set_duty(max_duty);
        m.right.gc.1.set_duty(max_duty);

        m.right.gc.0.enable();
        m.right.gc.1.enable();

        m.duty[0] = (3 * u32::from(max_duty) / 4) as u16;
        m.duty[1] = (2 * u32::from(max_duty) / 3) as u16;
        m.duty[2] = (1 * u32::from(max_duty) / 2) as u16;
        m.max_duty = max_duty;

        m
    }

    pub fn stop(&mut self) -> Result<(), Error> {
        self.set_right_wheel(Direction::None, Gear::Low)?;
        self.set_left_wheel(Direction::None, Gear::Low)?;

        Ok(())
    }

    pub fn forward(&mut self, gear: Gear) -> Result<(), Error> {
        self.set_right_wheel(Direction::Forward, gear)?;
        self.set_left_wheel(Direction::Forward, gear)?;

        Ok(())
    }

    pub fn backwards(&mut self, gear: Gear) -> Result<(), Error> {
        self.set_right_wheel(Direction::Reverse, gear)?;
        self.set_left_wheel(Direction::Reverse, gear)?;

        Ok(())
    }

    pub fn rotate(&mut self, rot: Rotation, gear: Gear) -> Result<(), Error> {
        match rot {
            Rotation::Left => {
                self.set_right_wheel(Direction::Forward, gear)?;
                self.set_left_wheel(Direction::Reverse, gear)?;
            }
            Rotation::Right => {
                self.set_right_wheel(Direction::Reverse, gear)?;
                self.set_left_wheel(Direction::Forward, gear)?;
            }
        }

        Ok(())
    }

    pub fn set_left_wheel(&mut self, dir: Direction, gear: Gear) -> Result<(), Error> {
        match dir {
            Direction::None => {
                self.left.gc.0.set_duty(self.max_duty);
                self.left.dc.0.set_low().map_err(|_| Error::HardwareError)?;
                self.left.gc.1.set_duty(self.max_duty);
                self.left.dc.1.set_low().map_err(|_| Error::HardwareError)?;
            }
            Direction::Forward => {
                self.left.gc.0.set_duty(self.duty[gear as usize]);
                self.left
                    .dc
                    .0
                    .set_high()
                    .map_err(|_| Error::HardwareError)?;
                self.left.gc.1.set_duty(self.max_duty);
                self.left.dc.1.set_low().map_err(|_| Error::HardwareError)?;
            }
            Direction::Reverse => {
                self.left.gc.0.set_duty(self.max_duty);
                self.left.dc.0.set_low().map_err(|_| Error::HardwareError)?;
                self.left.gc.1.set_duty(self.duty[gear as usize]);
                self.left
                    .dc
                    .1
                    .set_high()
                    .map_err(|_| Error::HardwareError)?;
            }
        };

        Ok(())
    }

    pub fn set_right_wheel(&mut self, dir: Direction, gear: Gear) -> Result<(), Error> {
        match dir {
            Direction::None => {
                self.right.gc.0.set_duty(self.max_duty);
                self.right
                    .dc
                    .0
                    .set_low()
                    .map_err(|_| Error::HardwareError)?;
                self.right.gc.1.set_duty(self.max_duty);
                self.right
                    .dc
                    .1
                    .set_low()
                    .map_err(|_| Error::HardwareError)?;
            }
            Direction::Forward => {
                self.right.gc.0.set_duty(self.duty[gear as usize]);
                self.right
                    .dc
                    .0
                    .set_high()
                    .map_err(|_| Error::HardwareError)?;
                self.right.gc.1.set_duty(self.max_duty);
                self.right
                    .dc
                    .1
                    .set_low()
                    .map_err(|_| Error::HardwareError)?;
            }
            Direction::Reverse => {
                self.right.gc.0.set_duty(self.max_duty);
                self.right
                    .dc
                    .0
                    .set_low()
                    .map_err(|_| Error::HardwareError)?;
                self.right.gc.1.set_duty(self.duty[gear as usize]);
                self.right
                    .dc
                    .1
                    .set_high()
                    .map_err(|_| Error::HardwareError)?;
            }
        };

        Ok(())
    }
}
