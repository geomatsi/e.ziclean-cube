use super::*;

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

/// Wheel rotation direction
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Direction {
    Forward,
    Reverse,
}

/// Vehicle rotation direction
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Rotation {
    Left,
    Right,
}

/// Gear
#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Gear {
    Neutral,
    Low,
    Medium,
    Top,
}

/// Left Wheel
struct LeftWheel {
    dir: Direction,
    gear: Gear,
    gc: LeftGear,
    dc: LeftDir,
}

/// Right Wheel
struct RightWheel {
    dir: Direction,
    gear: Gear,
    gc: RightGear,
    dc: RightDir,
}

/// Motion
pub struct Motion {
    /// Right wheel state
    right: RightWheel,
    /// Left wheel state
    left: LeftWheel,
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
                dir: Direction::Forward,
                gear: Gear::Neutral,
                gc: (lgf, lgr),
                dc: (lcf, lcr),
            },
            right: RightWheel {
                dir: Direction::Forward,
                gear: Gear::Neutral,
                gc: (rgf, rgr),
                dc: (rcf, rcr),
            },
        };

        m.left.gc.0.disable();
        m.left.gc.1.disable();

        m.left.dc.0.set_low();
        m.left.dc.1.set_low();

        m.right.gc.0.disable();
        m.right.gc.1.disable();

        m.right.dc.0.set_low();
        m.right.dc.1.set_low();

        m
    }

    pub fn forward(&mut self, gear: Gear) -> Result<(), Error> {
        self.set_right_wheel(Direction::Forward, gear)?;
        self.set_left_wheel(Direction::Forward, gear)?;

        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), Error> {
        self.forward(Gear::Neutral)?;

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
        Ok(())
    }

    pub fn set_right_wheel(&mut self, dir: Direction, gear: Gear) -> Result<(), Error> {
        Ok(())
    }
}
