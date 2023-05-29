use std::fmt::{self, Display, Formatter};

use crate::reg::registers::RegisterId;

#[derive(Debug)]
pub enum Instruction {
    Nop,
    Hlt,
    AddU32LitReg(u32, RegisterId),
}

impl Display for Instruction {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        let asm_format = match *self {
            Instruction::Nop => String::from("nop"),
            Instruction::AddU32LitReg(literal, reg) => format!("add.u32 {:02X}, {}", literal, reg),
            Instruction::Hlt => String::from("hlt"),
        };
        write!(f, "{}", asm_format)
    }
}

impl Instruction {
    pub fn get_instruction_len(&self) -> u32 {
        match self {
            Instruction::Nop => 2,
            Instruction::Hlt => 2,
            Instruction::AddU32LitReg(_, _) => {
                // 2 bytes for the instruction, 4 bytes for the literal, 1 byte for the register.
                2 + 4 + 1
            }
        }
    }
}
