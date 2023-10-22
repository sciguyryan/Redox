use std::fmt::{self, Display, Formatter};

use crate::{
    ins::{move_expressions::MoveExpressionHandler, op_codes::OpCode},
    reg::registers::RegisterId,
};

/// The size of the instruction, in bytes.
const INSTRUCTION_SIZE: u32 = 2;
/// The size of an extended instruction, in bytes.
const EXT_INSTRUCTION_SIZE: u32 = 4;
/// The size of a u16 argument, in bytes.
const ARG_U16_IMM_SIZE: u32 = std::mem::size_of::<u16>() as u32;
/// The size of a u32 argument, in bytes.
const ARG_U32_IMM_SIZE: u32 = std::mem::size_of::<u32>() as u32;
/// The size of a memory address argument, in bytes.
const ARG_MEM_ADDR_SIZE: u32 = std::mem::size_of::<u32>() as u32;
/// The size of a register ID argument, in bytes.
const ARG_REG_ID_SIZE: u32 = std::mem::size_of::<RegisterId>() as u32;

#[derive(Clone, Copy, Debug)]
pub enum Instruction {
    Nop,

    /******** [Arithmetic Instructions] ********/
    AddU32ImmU32Reg(u32, RegisterId),
    AddU32RegU32Reg(RegisterId, RegisterId),

    /******** [Simple Move Instructions - NO EXPRESSIONS] ********/
    SwapU32RegU32Reg(RegisterId, RegisterId),
    MovU32ImmU32Reg(u32, RegisterId),
    MovU32RegU32Reg(RegisterId, RegisterId),
    MovU32ImmMemRelSimple(u32, u32),
    MovU32RegMemRelSimple(RegisterId, u32),
    MovMemU32RegRelSimple(u32, RegisterId),
    MovU32RegPtrU32RegRelSimple(RegisterId, RegisterId),

    /******** [Complex Move Instructions - WITH EXPRESSIONS] ********/
    MovU32ImmMemRelExpr(u32, u32),

    /******** [Special Instructions] ********/
    Ret,
    Mret,
    Hlt,
}

impl Display for Instruction {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        // NOTE: square brackets are used to indicate we are working with an address.
        // NOTE: once expressions are added any that reference addresses could be modified
        //       to use the encoded expression system instead.
        //       https://www.cs.virginia.edu/~evans/cs216/guides/x86.html (Addressing Memory)
        let asm_format = match *self {
            Instruction::Nop => String::from("nop"),

            /******** [Arithmetic Instructions] ********/
            Instruction::AddU32ImmU32Reg(imm, reg) => {
                format!("add ${imm:02X}, {reg}")
            }
            Instruction::AddU32RegU32Reg(in_reg, out_reg) => {
                format!("add {in_reg}, {out_reg}")
            }

            /******** [Simple Move Instructions - NO EXPRESSIONS] ********/
            Instruction::SwapU32RegU32Reg(reg1, reg2) => {
                format!("swap {reg1}, {reg2}")
            }
            Instruction::MovU32ImmU32Reg(imm, reg) => {
                format!("mov ${imm:02X}, {reg}")
            }
            Instruction::MovU32RegU32Reg(in_reg, out_reg) => {
                format!("mov {in_reg}, {out_reg}")
            }
            Instruction::MovU32ImmMemRelSimple(imm, addr) => {
                format!("mov.rs ${imm:02X}, [${addr:04X}]")
            }
            Instruction::MovU32RegMemRelSimple(reg, addr) => {
                format!("mov.rs {reg}, [${addr:04X}]")
            }
            Instruction::MovMemU32RegRelSimple(addr, reg) => {
                format!("mov.rs [${addr:04X}], {reg}")
            }
            Instruction::MovU32RegPtrU32RegRelSimple(in_reg, out_reg) => {
                format!("mov.rs [{in_reg}], {out_reg}")
            }

            /******** [Complex Move Instructions - WITH EXPRESSIONS] ********/
            Instruction::MovU32ImmMemRelExpr(imm, expr) => {
                let mut expression_decoder = MoveExpressionHandler::new();
                expression_decoder.decode(expr);

                format!("mov.rs ${imm:02X}, [{expression_decoder}]")
            }

            /******** [Special Instructions] ********/
            Instruction::Ret => String::from("ret"),
            Instruction::Mret => String::from("mret"),
            Instruction::Hlt => String::from("hlt"),
        };
        write!(f, "{asm_format}")
    }
}

impl Instruction {
    pub fn get_instruction_size(&self) -> u32 {
        let (arg_size, extended) = match self {
            Instruction::Nop => (0, false),

            /******** [Arithmetic Instructions] ********/
            Instruction::AddU32ImmU32Reg(_, _) => (ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE, false),
            Instruction::AddU32RegU32Reg(_, _) => (ARG_REG_ID_SIZE + ARG_REG_ID_SIZE, false),

            /******** [Simple Move Instructions - NO EXPRESSIONS] ********/
            Instruction::SwapU32RegU32Reg(_, _) => (ARG_REG_ID_SIZE + ARG_REG_ID_SIZE, false),
            Instruction::MovU32ImmU32Reg(_, _) => (ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE, false),
            Instruction::MovU32RegU32Reg(_, _) => (ARG_REG_ID_SIZE + ARG_REG_ID_SIZE, false),
            Instruction::MovU32ImmMemRelSimple(_, _) => {
                (ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE, false)
            }
            Instruction::MovU32RegMemRelSimple(_, _) => {
                (ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE, false)
            }
            Instruction::MovMemU32RegRelSimple(_, _) => {
                (ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE, false)
            }
            Instruction::MovU32RegPtrU32RegRelSimple(_, _) => {
                (ARG_REG_ID_SIZE + ARG_REG_ID_SIZE, false)
            }

            /******** [Complex Move Instructions - WITH EXPRESSIONS] ********/
            Instruction::MovU32ImmMemRelExpr(_, _) => (ARG_U32_IMM_SIZE + ARG_U16_IMM_SIZE, false),

            /******** [Special Instructions] ********/
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
        let mut bytecode = Vec::new();

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

            /******** [Arithmetic Instructions] ********/
            Instruction::AddU32ImmU32Reg(imm, reg) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.push(*reg as u8);
            }
            Instruction::AddU32RegU32Reg(in_reg, out_reg) => {
                bytecode.push(*in_reg as u8);
                bytecode.push(*out_reg as u8);
            }

            /******** [Simple Move Instructions - NO EXPRESSIONS] ********/
            Instruction::SwapU32RegU32Reg(reg1, reg2) => {
                bytecode.push(*reg1 as u8);
                bytecode.push(*reg2 as u8);
            }
            Instruction::MovU32ImmU32Reg(imm, reg) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.push(*reg as u8);
            }
            Instruction::MovU32RegU32Reg(in_reg, out_reg) => {
                bytecode.push(*in_reg as u8);
                bytecode.push(*out_reg as u8);
            }
            Instruction::MovU32ImmMemRelSimple(imm, addr) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.extend_from_slice(&addr.to_le_bytes());
            }
            Instruction::MovU32RegMemRelSimple(reg, addr) => {
                bytecode.push(*reg as u8);
                bytecode.extend_from_slice(&addr.to_le_bytes());
            }
            Instruction::MovMemU32RegRelSimple(addr, reg) => {
                bytecode.extend_from_slice(&addr.to_le_bytes());
                bytecode.push(*reg as u8);
            }
            Instruction::MovU32RegPtrU32RegRelSimple(in_reg, out_reg) => {
                bytecode.push(*in_reg as u8);
                bytecode.push(*out_reg as u8);
            }

            /******** [Complex Move Instructions - WITH EXPRESSIONS] ********/
            Instruction::MovU32ImmMemRelExpr(imm, expr) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.extend_from_slice(&expr.to_le_bytes());
            }

            /******** [Special Instructions] ********/
            Instruction::Ret => todo!(),
            Instruction::Mret => {}
            Instruction::Hlt => {}
        }

        bytecode
    }
}
