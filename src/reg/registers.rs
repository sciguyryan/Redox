use std::collections::HashMap;

use super::register::{RegisterF32, RegisterPermission, RegisterU32};

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

macro_rules! register_u32 {
    ($b:expr, $c:expr, $d:expr) => {{
        use super::register::RegisterU32;
        ($b, RegisterU32::new($b, $c.clone(), $d))
    }};
}

pub struct Registers {
    /// A hashmap of the u32 registers.
    pub registers_u32: HashMap<RegisterId, RegisterU32>,
    /// A hashmap of the f32 registers.
    pub registers_f32: HashMap<RegisterId, RegisterF32>,
}

impl Registers {
    pub fn new() -> Self {
        let rw = RegisterPermission::R | RegisterPermission::W;
        let rpw = RegisterPermission::R | RegisterPermission::PW;
        let prpw = RegisterPermission::PR | RegisterPermission::PW;

        Self {
            registers_u32: HashMap::from([
                // [ User Registers ] //
                register_u32!(RegisterId::R1, &rw, 0),
                register_u32!(RegisterId::R2, &rw, 0),
                register_u32!(RegisterId::R3, &rw, 0),
                register_u32!(RegisterId::R4, &rw, 0),
                register_u32!(RegisterId::R5, &rw, 0),
                register_u32!(RegisterId::R6, &rw, 0),
                register_u32!(RegisterId::R7, &rw, 0),
                register_u32!(RegisterId::R8, &rw, 0),
                // [System Registers] //
                register_u32!(RegisterId::AC, &rw, 0),
                register_u32!(RegisterId::IP, &rw, 0),
                register_u32!(RegisterId::SP, &prpw, 0),
                register_u32!(RegisterId::FP, &prpw, 0),
                register_u32!(RegisterId::FL, &prpw, 0),
                //register!(RegisterId::T0, &prpw, 0),
                register_u32!(RegisterId::PC, &rpw, 0),
            ]),
            registers_f32: HashMap::new(),
        }
    }
}

impl Default for Registers {
    fn default() -> Self {
        Self::new()
    }
}
