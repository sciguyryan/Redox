use std::collections::HashMap;

use super::register::{Register, RegisterPermission, RegisterValue};

#[derive(Debug, Eq, PartialEq, Hash)]
pub enum RegisterId {
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
    pub registers: HashMap<RegisterId, Register>,
}

impl Registers {
    pub fn new() -> Self {
        let rw_permissions = RegisterPermission::R | RegisterPermission::W;

        Self {
            registers: HashMap::from([(
                RegisterId::R1,
                Register::new(RegisterId::R1, rw_permissions, RegisterValue::U32(0)),
            )]),
        }
    }
}

impl Default for Registers {
    fn default() -> Self {
        Self::new()
    }
}
