use crate::sw::comm::Actions;
use crate::sw::comm::Events;

use super::Brain;

use heapless::binary_heap::{BinaryHeap, Max};
use heapless::consts::*;

pub struct DebugBrain {
    queue: BinaryHeap<Events, U8, Max>,
}

impl DebugBrain {
    pub fn create() -> Self {
        let q = BinaryHeap(heapless::i::BinaryHeap::new());

        DebugBrain { queue: q }
    }
}

impl Brain for DebugBrain {
    fn notify(&mut self, e: Events) {
        self.queue.push(e).ok();
    }

    fn action(&mut self) -> Actions {
        if let Some(e) = self.queue.pop() {
            match e {
                Events::Button(true) => Actions::Beep(200),
                _ => Actions::Debug(e),
            }
        } else {
            Actions::None
        }
    }

    fn ready(&self) -> bool {
        !self.queue.is_empty()
    }
}
