use std::collections::HashMap;

use super::register::{Register, RegisterPermission, RegisterValue};

#[derive(Debug, Eq, PartialEq, Hash)]
#[repr(u8)]
pub enum RegisterId {
    // Layout
    // bits  : 8 16 24 32
    // bytes : 1 2  3  4

    // Bits  : 8 16
    // Bytes : 1  2
    // Data register 1: two highest order bytes.
    //AX,
    // Bits  : 16
    // Byte  : 2
    // Data register 1: second highest order byte.
    //AH,
    // Bits  : 8
    // Byte  : 1
    // Data register 1 - highest order byte.
    //AL,

    // [ User Registers ] //
    /// Data register 1.
    R1,
    /// Data register 1, two highest order bytes.
    //R1X,
    /// Data register 1, highest order byte.
    //R1H,
    /// Data register 1, lowest order byte.
    //R1L,

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
    /// Accumulator register.
    AC,
    /// Instruction pointer register.
    IP,
    /// Stack pointer register.
    SP,
    /// Frame pointer register.
    FP,
    /// CPU flags register.
    FL,
    /// Program counter register.
    PC,
    /// A debug testing register.
    T0,
}

macro_rules! register {
    ($a:expr, $b:expr, $c:expr) => {{
        ($a, Register::new($a, $b.clone(), $c))
    }};
}

pub struct Registers {
    pub registers: HashMap<RegisterId, Register>,
}

impl Registers {
    pub fn new() -> Self {
        let rw = RegisterPermission::R | RegisterPermission::W;
        let rpw = RegisterPermission::R | RegisterPermission::PW;
        let prpw = RegisterPermission::PR | RegisterPermission::PW;

        Self {
            registers: HashMap::from([
                // [ User Registers ] //
                register!(RegisterId::R1, &rw, RegisterValue::U32(0)),
                register!(RegisterId::R2, &rw, RegisterValue::U32(0)),
                register!(RegisterId::R3, &rw, RegisterValue::U32(0)),
                register!(RegisterId::R4, &rw, RegisterValue::U32(0)),
                register!(RegisterId::R5, &rw, RegisterValue::U32(0)),
                register!(RegisterId::R6, &rw, RegisterValue::U32(0)),
                register!(RegisterId::R7, &rw, RegisterValue::U32(0)),
                register!(RegisterId::R8, &rw, RegisterValue::U32(0)),
                // [System Registers] //
                register!(RegisterId::AC, &rw, RegisterValue::U32(0)),
                register!(RegisterId::IP, &rw, RegisterValue::U32(0)),
                register!(RegisterId::SP, &prpw, RegisterValue::U32(0)),
                register!(RegisterId::FP, &prpw, RegisterValue::U32(0)),
                register!(RegisterId::FL, &prpw, RegisterValue::U32(0)),
                //register!(RegisterId::T0, &prpw, RegisterValue::U32(0)),
                register!(RegisterId::PC, &rpw, RegisterValue::U32(0)),
            ]),
        }
    }
}

impl Default for Registers {
    fn default() -> Self {
        Self::new()
    }
}
