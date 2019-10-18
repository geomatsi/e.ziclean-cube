use crate::sw::comm::Clean;
use embedded_hal::PwmPin;
use stm32f1xx_hal as hal;

type PumpPwm = hal::pwm::Pwm<hal::device::TIM3, hal::pwm::C1>;
type BrushPwm = hal::pwm::Pwm<hal::device::TIM3, hal::pwm::C2>;

/// Cleaner control
pub struct Cleaner {
    /// Brush PWM
    brush: BrushPwm,
    /// Pump PWM
    pump: PumpPwm,
    /// Slow brushes
    slow: u16,
    /// Fast brushes
    fast: u16,
}

impl Cleaner {
    pub fn create(mut brush: BrushPwm, mut pump: PumpPwm) -> Self {
        brush.disable();
        pump.disable();

        let max_duty = brush.get_max_duty();
        let slow = (1 * u32::from(max_duty) / 2) as u16;
        let fast = (5 * u32::from(max_duty) / 6) as u16;

        Cleaner {
            brush,
            pump,
            slow,
            fast,
        }
    }

    pub fn stop(&mut self) {
        self.brush.disable();
        self.pump.disable();
    }

    pub fn slow(&mut self) {
        self.brush.enable();
        self.brush.set_duty(self.slow);

        self.pump.enable();
        self.pump.set_duty(self.slow);
    }

    pub fn fast(&mut self) {
        self.brush.enable();
        self.brush.set_duty(self.fast);

        self.pump.enable();
        self.pump.set_duty(self.fast);
    }

    pub fn clean(&mut self, action: Clean) {
        match action {
            Clean::None => {
                self.stop();
            }
            Clean::Slow => {
                self.slow();
            }
            Clean::Fast => {
                self.fast();
            }
        }
    }
}
