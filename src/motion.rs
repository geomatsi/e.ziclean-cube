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
#[derive(Debug, Eq, PartialEq)]
pub enum Error {
    /// HardwareError
    HardwareError,
}

/// Wheel rotation direction
#[derive(Debug, Eq, PartialEq)]
pub enum Direction {
    Forward,
    Reverse,
}

/// Vehicle rotation direction
#[derive(Debug, Eq, PartialEq)]
pub enum Rotation {
    Left,
    Right,
}

/// Gear
#[derive(Debug, Eq, PartialEq)]
pub enum Gear {
    Low,
    Medium,
    Top,
}

/// Motion
pub struct Motion
{
    pwm: u32,
}

impl Motion
{
    pub fn init(pwm: u32) -> Self {
        Motion { pwm }
    }

    pub fn forward(gear: Gear) -> Result<(), Error> {
        Ok(())
    }

    pub fn backwards(gear: Gear) -> Result<(), Error> {
        Ok(())
    }

    pub fn rotate(rot: Rotation, gear: Gear) -> Result<(), Error> {
        Ok(())
    }

    pub fn set_left_wheel(dir: Direction, gear: Gear) -> Result<(), Error> {
        Ok(())
    }

    pub fn set_right_wheel(dir: Direction, gear: Gear) -> Result<(), Error> {
        Ok(())
    }
}
