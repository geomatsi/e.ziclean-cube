use embedded_hal::digital::v2::OutputPin;
use embedded_hal::timer::CountDown;
use embedded_hal::timer::Periodic;
use nb::block;
use stm32f1xx_hal as hal;

type BeepGpio = hal::gpio::gpioe::PE0<hal::gpio::Output<hal::gpio::PushPull>>;

/// Beeper
pub struct Beeper<TMR>
where
    TMR: Periodic + CountDown,
{
    /// timer
    tmr: TMR,
    /// pin
    pin: BeepGpio,
}

impl<TMR> Beeper<TMR>
where
    TMR: Periodic + CountDown,
{
    pub fn create(tmr: TMR, pin: BeepGpio) -> Self {
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
