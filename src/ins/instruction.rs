use std::fmt::{self, Display, Formatter};

use crate::reg::registers::RegisterId;

#[derive(Debug)]
pub enum Instruction {
    Nop(),
    Hlt(),
    AddU32LitReg(u32, RegisterId),
}

impl Display for Instruction {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        let asm_format = match *self {
            Instruction::Nop() => String::from("nop"),
            Instruction::AddU32LitReg(literal, reg) => format!("add.u32 {:02X}, {}", literal, reg),
            Instruction::Hlt() => String::from("hlt"),
        };
        write!(f, "{}", asm_format)
    }
}
