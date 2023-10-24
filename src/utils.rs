#[inline(always)]
pub fn is_bit_set(value: u32, bit: u8) -> bool {
    (value & (1 << bit)) != 0
}

#[inline(always)]
pub fn is_bit_set_64(value: u64, bit: u8) -> bool {
    (value & (1 << bit)) != 0
}

#[inline(always)]
pub fn set_bit_state(value: u32, bit: u8, state: bool) -> u32 {
    let bit_state = if state { 1 } else { 0 };
    let mask = 1 << bit;
    (value & !mask) | ((bit_state << bit) & mask)
}

#[inline(always)]
pub fn set_bit_state_inline(value: &mut u32, bit: u8, state: bool) {
    let bit_state = if state { 1 } else { 0 };
    let mask = 1 << bit;
    *value = (*value & !mask) | ((bit_state << bit) & mask)
}
