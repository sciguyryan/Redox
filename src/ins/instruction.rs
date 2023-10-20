use std::fmt::{self, Display, Formatter};

use crate::reg::registers::RegisterId;

use super::op_codes::OpCode;

/// The size of the instruction, in bytes.
const INSTRUCTION_SIZE: u32 = 2;
/// The size of an extended instruction, in bytes.
const EXT_INSTRUCTION_SIZE: u32 = 4;
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
    MovMemU32Reg(u32, RegisterId),
    MovU32RegPtrU32Reg(RegisterId, RegisterId),
    Ret,
    Mret,
    Hlt,
}

impl Into<OpCode> for Instruction {
    fn into(self) -> OpCode {
        match self {
            Instruction::Nop => OpCode::Nop,
            Instruction::AddU32ImmU32Reg(_, _) => OpCode::AddU32ImmU32Reg,
            Instruction::AddU32RegU32Reg(_, _) => OpCode::AddU32RegU32Reg,
            Instruction::MovU32ImmU32Reg(_, _) => OpCode::MovU32ImmU32Reg,
            Instruction::MovU32RegU32Reg(_, _) => OpCode::MovU32RegU32Reg,
            Instruction::MovU32ImmMem(_, _) => OpCode::MovU32ImmMem,
            Instruction::MovU32RegMem(_, _) => OpCode::MovU32RegMem,
            Instruction::MovMemU32Reg(_, _) => OpCode::MovMemU32Reg,
            Instruction::MovU32RegPtrU32Reg(_, _) => OpCode::MovMemU32Reg,
            Instruction::Ret => OpCode::Ret,
            Instruction::Mret => OpCode::Mret,
            Instruction::Hlt => OpCode::Hlt,
        }
    }
}

impl Display for Instruction {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        // Note: square brackets are used to indicate an address.
        let asm_format = match *self {
            Instruction::Nop => String::from("nop"),
            Instruction::AddU32ImmU32Reg(imm, reg) => {
                format!("add ${:02X}, {}", imm, reg)
            }
            Instruction::AddU32RegU32Reg(reg1, reg2) => {
                format!("add {}, {}", reg1, reg2)
            }
            Instruction::MovU32ImmU32Reg(imm, reg) => {
                format!("move ${:02X}, {}", imm, reg)
            }
            Instruction::MovU32RegU32Reg(reg1, reg2) => {
                format!("move {}, {}", reg1, reg2)
            }
            Instruction::MovU32ImmMem(imm, addr) => {
                format!("move ${:02X}, [${:04X}]", imm, addr)
            }
            Instruction::MovU32RegMem(reg, addr) => {
                format!("move {}, [${:04X}]", reg, addr)
            }
            Instruction::MovMemU32Reg(addr, reg) => {
                format!("move [${:04X}], {}", addr, reg)
            }
            Instruction::MovU32RegPtrU32Reg(reg1, reg2) => {
                format!("move [{}], {}", reg1, reg2)
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
        let (arg_size, extended) = match self {
            Instruction::Nop => (0, false),
            Instruction::AddU32ImmU32Reg(_, _) => (ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE, false),
            Instruction::AddU32RegU32Reg(_, _) => (ARG_REG_ID_SIZE + ARG_REG_ID_SIZE, false),
            Instruction::MovU32ImmU32Reg(_, _) => (ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE, false),
            Instruction::MovU32RegU32Reg(_, _) => (ARG_REG_ID_SIZE + ARG_REG_ID_SIZE, false),
            Instruction::MovU32ImmMem(_, _) => (ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE, false),
            Instruction::MovU32RegMem(_, _) => (ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE, false),
            Instruction::MovMemU32Reg(_, _) => (ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE, false),
            Instruction::MovU32RegPtrU32Reg(_, _) => (ARG_REG_ID_SIZE + ARG_REG_ID_SIZE, false),
            Instruction::Ret => (0, false),
            Instruction::Mret => (0, false),
            Instruction::Hlt => (0, false),
        };

        if !extended {
            INSTRUCTION_SIZE + arg_size
        } else {
            EXT_INSTRUCTION_SIZE + arg_size
        }
    }
}
