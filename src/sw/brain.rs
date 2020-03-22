use crate::sw::comm::Actions;
use crate::sw::comm::Events;

pub trait Brain {
    fn notify(&mut self, event: Events);
    fn action(&mut self) -> Actions;
    fn ready(&self) -> bool;
}

pub mod debug;
