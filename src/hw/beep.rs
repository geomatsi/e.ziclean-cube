use embedded_hal::digital::v2::OutputPin;
use embedded_hal::timer::CountDown;
use embedded_hal::timer::Periodic;
use nb::block;

/// Beeper
pub struct Beeper<TMR, PIN>
where
    TMR: Periodic + CountDown,
    PIN: OutputPin,
{
    /// timer
    tmr: TMR,
    /// pin
    pin: PIN,
}

impl<TMR, PIN> Beeper<TMR, PIN>
where
    TMR: Periodic + CountDown,
    PIN: OutputPin,
{
    pub fn create(tmr: TMR, pin: PIN) -> Self {
        Beeper { tmr, pin }
    }

    pub fn beep(&mut self, n: u32) {
        self.pin.set_high().ok();

        for _ in 0..n {
            block!(self.tmr.wait()).ok();
        }

        self.pin.set_low().ok();
    }

    pub fn beep_on(&mut self) {
        self.pin.set_high().ok();
    }
    pub fn beep_off(&mut self) {
        self.pin.set_low().ok();
    }
}
