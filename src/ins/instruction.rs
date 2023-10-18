use std::fmt::{self, Display, Formatter};

use crate::reg::registers::RegisterId;

/// The size of the instruction, in bytes.
const INSTRUCTION_SIZE: u32 = 2;
/// The size of a u32 argument, in bytes.
const ARG_U32_IMM_SIZE: u32 = std::mem::size_of::<u32>() as u32;
/// The size of a memory address argument, in bytes.
const ARG_MEM_ADDR_SIZE: u32 = std::mem::size_of::<u32>() as u32;
/// The size of a register ID argument, in bytes.
const ARG_REG_ID_SIZE: u32 = std::mem::size_of::<RegisterId>() as u32;

#[derive(Debug)]
pub enum Instruction {
    Nop,
    AddU32ImmU32Reg(u32, RegisterId),
    AddU32RegU32Reg(RegisterId, RegisterId),
    MovU32ImmU32Reg(u32, RegisterId),
    MovU32RegU32Reg(RegisterId, RegisterId),
    MovU32ImmMem(u32, u32),
    MovU32RegMem(RegisterId, u32),
    Ret,
    Mret,
    Hlt,
}

impl Display for Instruction {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        let asm_format = match *self {
            Instruction::Nop => String::from("nop"),
            Instruction::AddU32ImmU32Reg(literal, reg) => {
                format!("add.u32l_u32r 0x{:02X}, {}", literal, reg)
            }
            Instruction::AddU32RegU32Reg(reg1, reg2) => format!("add.u32r_u32r {}, {}", reg1, reg2),
            Instruction::MovU32ImmU32Reg(literal, reg) => {
                format!("move.u32i_u32r 0x{:02X}, {}", literal, reg)
            }
            Instruction::MovU32RegU32Reg(reg1, reg2) => {
                format!("move.u32r_u32r {}, {}", reg1, reg2)
            }
            Instruction::MovU32ImmMem(lit, addr) => {
                format!("move.u32i_mem 0x{:02X}, 0x{:04X}", lit, addr)
            }
            Instruction::MovU32RegMem(reg, addr) => {
                format!("move.u32r_mem {}, 0x{:04X}", reg, addr)
            }
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
            Instruction::AddU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::AddU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::MovU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::MovU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::MovU32ImmMem(_, _) => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            Instruction::MovU32RegMem(_, _) => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            Instruction::Ret => 0,
            Instruction::Mret => 0,
            Instruction::Hlt => 0,
        };

        INSTRUCTION_SIZE + arg_size
    }
}
