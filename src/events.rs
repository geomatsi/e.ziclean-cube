#[derive(Debug, Eq, PartialEq, Clone, Copy)]
pub enum Events {
    /// Empty event
    None,
    /// Battery low event
    BatteryLow,
    /// Accelerometer Event: motion (X-axis motion, Y-axis motion, Z-axis motion)
    AccEvent(bool, bool, bool),
    /// Button event: (pressed)
    ButtonEvent(bool),
    /// Charger event: (plugged)
    ChargerEvent(bool),
    /// Dock Station event: (detected)
    DockEvent(bool),
    /// Front sensors event: (ll, lc, cc, rc, rr)
    FrontSensor(bool, bool, bool, bool, bool),
    /// Bottom sensors event: (l, c, r)
    BottomSensor(bool, bool, bool),
}
