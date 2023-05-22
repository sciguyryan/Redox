use std::collections::HashMap;

use super::register::Register;

pub enum RegisterNames {
    // [ User Registers ] //
    /// Data register 1.
    R1,
    /// Data register 2.
    R2,
    /// Data register 3.
    R3,
    /// Data register 4.
    R4,
    /// Data register 5.
    R5,
    /// Data register 6.
    R6,
    /// Data register 7.
    R7,
    /// Data register 8.
    R8,

    // [System Registers] //
    /// Instruction pointer register.
    IP,
    /// Stack pointer register.
    SP,
    /// CPU flags register.
    FL,
    /// Program counter register.
    PC,
    /// A debug testing register.
    T0,
}

pub struct Registers {
    pub registers: HashMap<RegisterNames, Register>,
}

impl Registers {
    pub fn new() -> Self {
        Self {
            registers: HashMap::new(),
        }
    }
}

impl Default for Registers {
    fn default() -> Self {
        Self::new()
    }
}
