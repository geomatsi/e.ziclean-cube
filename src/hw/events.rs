use core::cmp::Ordering;

#[derive(Debug, Clone, Copy)]
pub enum Events {
    /// Empty event
    None,
    /// Battery low event
    BatteryLow,
    /// Accelerometer Event: motion (X-axis motion, Y-axis motion, Z-axis motion)
    Accel(bool, bool, bool),
    /// Button event: (pressed)
    Button(bool),
    /// Charger event: (plugged)
    Charger(bool),
    /// Dock Station event: (detected)
    Dock(bool),
    /// Dock Station event: (detected)
    Battery(bool),
    /// Front sensors event: (ll, lc, cc, rc, rr)
    FrontSensor(bool, bool, bool, bool, bool),
    /// Bottom sensors event: (l, c, r)
    BottomSensor(bool, bool, bool),
}

impl Default for Events {
    fn default() -> Self {
        Events::None
    }
}

impl Events {
    fn prio(self) -> u8 {
        match self {
            Events::None => 0u8,
            Events::Dock(_) => 10u8,
            Events::Charger(_) => 20u8,
            Events::Battery(_) => 25u8,
            Events::BatteryLow => 30u8,
            Events::Button(_) => 40u8,
            Events::Accel(_, _, _) => 80u8,
            Events::FrontSensor(_, _, _, _, _) => 90u8,
            Events::BottomSensor(_, _, _) => 100u8,
        }
    }
}

/* simple ordering of events based only on their priority */

impl Eq for Events {}

impl PartialEq for Events {
    fn eq(&self, other: &Events) -> bool {
        self.prio() == other.prio()
    }
}

impl PartialOrd for Events {
    fn partial_cmp(&self, other: &Events) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Events {
    fn cmp(&self, other: &Events) -> Ordering {
        match self {
            Events::None => match other {
                Events::None => Ordering::Equal,
                _ => self.prio().cmp(&other.prio()),
            },
            Events::Charger(_) => match other {
                Events::Charger(_) => Ordering::Equal,
                _ => self.prio().cmp(&other.prio()),
            },
            Events::Dock(_) => match other {
                Events::Dock(_) => Ordering::Equal,
                _ => self.prio().cmp(&other.prio()),
            },
            Events::Battery(_) => match other {
                Events::Battery(_) => Ordering::Equal,
                _ => self.prio().cmp(&other.prio()),
            },
            Events::BatteryLow => match other {
                Events::BatteryLow => Ordering::Equal,
                _ => self.prio().cmp(&other.prio()),
            },
            Events::Button(_) => match other {
                Events::Button(_) => Ordering::Equal,
                _ => self.prio().cmp(&other.prio()),
            },
            Events::Accel(_, _, _) => match other {
                Events::Accel(_, _, _) => Ordering::Equal,
                _ => self.prio().cmp(&other.prio()),
            },
            Events::FrontSensor(_, _, _, _, _) => match other {
                Events::FrontSensor(_, _, _, _, _) => Ordering::Equal,
                _ => self.prio().cmp(&other.prio()),
            },
            Events::BottomSensor(_, _, _) => match other {
                Events::BottomSensor(_, _, _) => Ordering::Equal,
                _ => self.prio().cmp(&other.prio()),
            },
        }
    }
}