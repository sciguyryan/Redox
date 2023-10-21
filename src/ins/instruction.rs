use std::fmt::{self, Display, Formatter};

use crate::{ins::op_codes::OpCode, reg::registers::RegisterId};

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

#[derive(Clone, Copy, Debug)]
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
    SwapU32RegU32Reg(RegisterId, RegisterId),
    Ret,
    Mret,
    Hlt,
}

impl Display for Instruction {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        // Note: square brackets are used to indicate we are working with an address.
        let asm_format = match *self {
            Instruction::Nop => String::from("nop"),
            Instruction::AddU32ImmU32Reg(imm, reg) => {
                format!("add ${imm:02X}, {reg}")
            }
            Instruction::AddU32RegU32Reg(in_reg, out_reg) => {
                format!("add {in_reg}, {out_reg}")
            }
            Instruction::MovU32ImmU32Reg(imm, reg) => {
                format!("move ${imm:02X}, {reg}")
            }
            Instruction::MovU32RegU32Reg(in_reg, out_reg) => {
                format!("move {in_reg}, {out_reg}")
            }
            Instruction::MovU32ImmMem(imm, addr) => {
                format!("move ${imm:02X}, [${addr:04X}]")
            }
            Instruction::MovU32RegMem(reg, addr) => {
                format!("move {reg}, [${addr:04X}]")
            }
            Instruction::MovMemU32Reg(addr, reg) => {
                format!("move [${addr:04X}], {reg}")
            }
            Instruction::MovU32RegPtrU32Reg(in_reg, out_reg) => {
                format!("move [{in_reg}], {out_reg}")
            }
            Instruction::SwapU32RegU32Reg(reg1, reg2) => {
                format!("swap {reg1}, {reg2}")
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
            Instruction::SwapU32RegU32Reg(_, _) => (ARG_REG_ID_SIZE + ARG_REG_ID_SIZE, false),
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

    pub fn get_bytecode(&self) -> Vec<u8> {
        let mut bytecode: Vec<u8> = Vec::new();

        let opcode = OpCode::from(*self);
        let opcode_value = opcode as u32;
        let opcode_bytes = opcode_value.to_le_bytes();

        // First we push the bytes for the opcode.
        if opcode.is_extended() {
            bytecode.extend_from_slice(&opcode_bytes);
        } else {
            bytecode.extend_from_slice(&opcode_bytes[0..2]);
        }

        // Next, we need to push the argument bytes. This part is more interesting.
        match self {
            Instruction::Nop => {}
            Instruction::AddU32ImmU32Reg(imm, reg) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.push(*reg as u8);
            }
            Instruction::AddU32RegU32Reg(in_reg, out_reg) => {
                bytecode.push(*in_reg as u8);
                bytecode.push(*out_reg as u8);
            }
            Instruction::MovU32ImmU32Reg(imm, reg) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.push(*reg as u8);
            }
            Instruction::MovU32RegU32Reg(in_reg, out_reg) => {
                bytecode.push(*in_reg as u8);
                bytecode.push(*out_reg as u8);
            }
            Instruction::MovU32ImmMem(imm, addr) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.extend_from_slice(&addr.to_le_bytes());
            }
            Instruction::MovU32RegMem(reg, addr) => {
                bytecode.push(*reg as u8);
                bytecode.extend_from_slice(&addr.to_le_bytes());
            }
            Instruction::MovMemU32Reg(addr, reg) => {
                bytecode.extend_from_slice(&addr.to_le_bytes());
                bytecode.push(*reg as u8);
            }
            Instruction::MovU32RegPtrU32Reg(in_reg, out_reg) => {
                bytecode.push(*in_reg as u8);
                bytecode.push(*out_reg as u8);
            }
            Instruction::SwapU32RegU32Reg(reg1, reg2) => {
                bytecode.push(*reg1 as u8);
                bytecode.push(*reg2 as u8);
            }
            Instruction::Ret => todo!(),
            Instruction::Mret => {}
            Instruction::Hlt => {}
        }

        bytecode
    }
}
