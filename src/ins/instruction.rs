use std::fmt::{self, Display, Formatter};

use crate::reg::registers::RegisterId;

/// The size of the instruction, in bytes.
const INSTRUCTION_SIZE: u32 = 2;
/// The size of a u32 argument, in bytes.
const ARG_U32_LIT_SIZE: u32 = 4;
/// The size of a register ID argument, in bytes.
const ARG_REG_ID_SIZE: u32 = 1;

#[derive(Debug)]
pub enum Instruction {
    Nop,
    AddU32LitU32Reg(u32, RegisterId),
    AddU32RegU32Reg(RegisterId, RegisterId),
    Ret,
    Mret,
    Hlt,
}

impl Display for Instruction {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        let asm_format = match *self {
            Instruction::Nop => String::from("nop"),
            Instruction::AddU32LitU32Reg(literal, reg) => {
                format!("add.u32 {:02X}, {}", literal, reg)
            }
            Instruction::AddU32RegU32Reg(reg1, reg2) => format!("add.reg {}, {}", reg1, reg2),
            Instruction::Ret => String::from("ret"),
            Instruction::Mret => String::from("mret"),
            Instruction::Hlt => String::from("hlt"),
        };
        write!(f, "{}", asm_format)
    }
}

impl Instruction {
    pub fn get_instruction_size(&self) -> u32 {
        let arg_size = match self {
            Instruction::Nop => 0,
            Instruction::AddU32LitU32Reg(_, _) => ARG_U32_LIT_SIZE + ARG_REG_ID_SIZE,
            Instruction::AddU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::Ret => 0,
            Instruction::Mret => 0,
            Instruction::Hlt => 0,
        };

        INSTRUCTION_SIZE + arg_size
    }
}
