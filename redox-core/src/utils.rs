const BIT_MASKS: [u32; 32] = [
    0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000,
    0x8000, 0x10000, 0x20000, 0x40000, 0x80000, 0x100000, 0x200000, 0x400000, 0x800000, 0x1000000,
    0x2000000, 0x4000000, 0x8000000, 0x10000000, 0x20000000, 0x40000000, 0x80000000,
];

#[inline(always)]
pub fn is_bit_set(value: u32, bit: u8) -> bool {
    unsafe { (value & BIT_MASKS.get_unchecked(bit as usize)) != 0 }
}

#[inline(always)]
pub fn set_bit_state(value: u32, bit: u8, state: bool) -> u32 {
    unsafe {
        let mask = BIT_MASKS.get_unchecked(bit as usize);
        (value & !mask) | (((state as u32) << bit) & mask)
    }
}

#[inline(always)]
pub fn set_bit_state_inline(value: &mut u32, bit: u8, state: bool) {
    unsafe {
        let mask = BIT_MASKS.get_unchecked(bit as usize);
        *value = (*value & !mask) | (((state as u32) << bit) & mask)
    }
}
