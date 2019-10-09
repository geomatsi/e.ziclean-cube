#[derive(Debug, Clone, Copy)]
enum ReceiverState {
    Waiting,
    Recv,
    Recv0,
    Recv1,
    Done,
    Error,
}

#[derive(Debug, Clone, Copy)]
pub enum ReceiverResult {
    Proc,
    Done(u8),
    Fail(u8),
}

pub struct IRReceiver {
    state: ReceiverState,
    curr_stamp: u32,
    prev_stamp: u32,
    rate: u32,
    pin: bool,
    res: u8,
    num: u8,
}

impl IRReceiver {
    pub fn new(sample_rate: u32) -> Self {
        IRReceiver {
            state: ReceiverState::Waiting,
            rate: sample_rate,
            curr_stamp: 0,
            prev_stamp: 0,
            pin: true,
            res: 0u8,
            num: 0u8,
        }
    }

    pub fn sample(&mut self, new_pin: bool, stamp: u32) -> ReceiverResult {
        if new_pin == self.pin {
            return ReceiverResult::Proc;
        }

        // FIXME: (curr - prev) in usec
        let mut delta = stamp.wrapping_sub(self.prev_stamp);
        delta *= 1_000_000 / self.rate;

        // new_val = 1 => val = 0 = > rising
        let rising = new_pin;

        let prev_stamp = self.prev_stamp;
        self.prev_stamp = stamp;
        self.pin = new_pin;

        match self.state {
            ReceiverState::Waiting => {
                if rising {
                    self.state = ReceiverState::Error;
                    return ReceiverResult::Fail(1);
                }

                if prev_stamp != 0 && delta < 4000 {
                    self.state = ReceiverState::Error;
                    return ReceiverResult::Fail(2);
                }

                self.state = ReceiverState::Recv;
                ReceiverResult::Proc
            }
            ReceiverState::Recv => {
                if !rising {
                    self.state = ReceiverState::Error;
                    return ReceiverResult::Fail(3);
                }

                // FIXME
                if delta > (800 - 400) && delta < (800 + 400) {
                    if self.num == 7 {
                        self.state = ReceiverState::Done;
                        return ReceiverResult::Done(self.res);
                    }
                    self.state = ReceiverState::Recv1;
                    return ReceiverResult::Proc;
                }

                // FIXME
                if delta > (2500 - 1000) && delta < (2500 + 1000) {
                    if self.num == 7 {
                        self.state = ReceiverState::Done;
                        return ReceiverResult::Done(self.res);
                    }
                    self.state = ReceiverState::Recv0;
                    return ReceiverResult::Proc;
                }

                self.state = ReceiverState::Error;
                ReceiverResult::Fail(4)
            }
            ReceiverState::Recv0 => {
                if rising {
                    self.state = ReceiverState::Error;
                    return ReceiverResult::Fail(5);
                }

                // FIXME
                if delta > (800 - 400) && delta < (800 + 400) {
                    self.res |= 0 << self.num;
                    self.num += 1;

                    self.state = ReceiverState::Recv;
                    return ReceiverResult::Proc;
                }

                self.state = ReceiverState::Error;
                ReceiverResult::Fail(6)
            }
            ReceiverState::Recv1 => {
                if rising {
                    self.state = ReceiverState::Error;
                    return ReceiverResult::Fail(7);
                }

                if delta > (2500 - 1000) && delta < (2500 + 1000) {
                    self.res |= 1 << self.num;
                    self.num += 1;

                    self.state = ReceiverState::Recv;
                    return ReceiverResult::Proc;
                }

                self.state = ReceiverState::Error;
                ReceiverResult::Fail(8)
            }
            ReceiverState::Error => ReceiverResult::Fail(9),
            _ => unreachable!(),
        }
    }

    pub fn reset(&mut self) {
        self.state = ReceiverState::Waiting;
        self.curr_stamp = 0;
        self.prev_stamp = 0;
        self.pin = true;
        self.res = 0u8;
        self.num = 0u8;
    }
}
