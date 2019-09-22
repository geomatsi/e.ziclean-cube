use embedded_hal::timer::CountDown;
use embedded_hal::timer::Periodic;
use rtfm::Instant;
use stm32f1xx_hal::time::*;
use void::Void;

pub struct PollTimer {
    /// System clock: cycles per second
    clk: u32,
    /// Timer period in system clock cycles
    tmr: u32,
}

impl PollTimer {
    pub fn init<T1, T2>(clk: T1, tmr: T2) -> Self
    where
        T1: Into<Hertz>,
        T2: Into<Hertz>,
    {
        let sfreq = clk.into().0;
        let tfreq = tmr.into().0;

        let stick: u32 = sfreq;
        let ptick: u32 = if sfreq < tfreq {
            sfreq
        } else {
            sfreq / tfreq as u32
        };

        PollTimer {
            clk: stick,
            tmr: ptick,
        }
    }
}

impl Periodic for PollTimer {}

impl CountDown for PollTimer {
    type Time = Hertz;

    fn start<T>(&mut self, timeout: T)
    where
        T: Into<Hertz>,
    {
        self.tmr = self.clk / timeout.into().0 as u32;
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        let start = Instant::now();

        while start.elapsed().as_cycles() < self.tmr {}

        Ok(())
    }
}
