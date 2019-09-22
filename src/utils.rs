/*
 *
 */
pub fn is_obstacle(val: u16, max: u16) -> bool {
    val < max - 200
}

/*
 *
 */
pub fn is_front_obstacle(v1: u16, v2: u16) -> bool {
    (v1 > v2 + 50) | (v2 > v1 + 50)
}

/*
 *
 */
pub fn is_bottom_drop(_v1: u16, _v2: u16) -> bool {
    /* TODO */
    false
}

/*
 *
 */
pub fn is_battery_low(v: u32) -> bool {
    v < 15000
}
