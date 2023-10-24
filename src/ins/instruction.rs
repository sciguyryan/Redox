use std::fmt::{self, Display, Formatter};

use crate::{
    ins::{move_expressions::MoveExpressionHandler, op_codes::OpCode},
    reg::registers::RegisterId,
};

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
    MovU32MemU32RegRelExpr(u32, RegisterId),

    /******** [Special Instructions] ********/
    Ret,
    Mret,
    Hlt,
}

impl Display for Instruction {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        // NOTE: square brackets are used to indicate we are working with an address.
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
                let mut decoder = MoveExpressionHandler::new();
                decoder.decode(expr);

                format!("mov.rc ${imm:02X}, [{decoder}]")
            }
            Instruction::MovU32MemU32RegRelExpr(expr, reg) => {
                let mut decoder = MoveExpressionHandler::new();
                decoder.decode(expr);

                format!("mov.rc [{decoder}], {reg}")
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
    pub fn into_bytecode(self) -> Vec<u8> {
        let opcode_value = OpCode::from(self) as u32;
        let opcode_bytes = opcode_value.to_le_bytes();

        let mut bytecode = Vec::new();

        // First, we push the bytes for the opcode.
        bytecode.extend_from_slice(&opcode_bytes);

        // Next, we need to push the argument bytes.
        match self {
            Instruction::Nop => {}

            /******** [Arithmetic Instructions] ********/
            Instruction::AddU32ImmU32Reg(imm, reg) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.push(reg as u8);
            }
            Instruction::AddU32RegU32Reg(in_reg, out_reg) => {
                bytecode.push(in_reg as u8);
                bytecode.push(out_reg as u8);
            }

            /******** [Simple Move Instructions - NO EXPRESSIONS] ********/
            Instruction::SwapU32RegU32Reg(reg1, reg2) => {
                bytecode.push(reg1 as u8);
                bytecode.push(reg2 as u8);
            }
            Instruction::MovU32ImmU32Reg(imm, reg) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.push(reg as u8);
            }
            Instruction::MovU32RegU32Reg(in_reg, out_reg) => {
                bytecode.push(in_reg as u8);
                bytecode.push(out_reg as u8);
            }
            Instruction::MovU32ImmMemRelSimple(imm, addr) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.extend_from_slice(&addr.to_le_bytes());
            }
            Instruction::MovU32RegMemRelSimple(reg, addr) => {
                bytecode.push(reg as u8);
                bytecode.extend_from_slice(&addr.to_le_bytes());
            }
            Instruction::MovMemU32RegRelSimple(addr, reg) => {
                bytecode.extend_from_slice(&addr.to_le_bytes());
                bytecode.push(reg as u8);
            }
            Instruction::MovU32RegPtrU32RegRelSimple(in_reg, out_reg) => {
                bytecode.push(in_reg as u8);
                bytecode.push(out_reg as u8);
            }

            /******** [Complex Move Instructions - WITH EXPRESSIONS] ********/
            Instruction::MovU32ImmMemRelExpr(imm, expr) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.extend_from_slice(&expr.to_le_bytes());
            }
            Instruction::MovU32MemU32RegRelExpr(expr, reg) => {
                bytecode.extend_from_slice(&expr.to_le_bytes());
                bytecode.push(reg as u8);
            }

            /******** [Special Instructions] ********/
            Instruction::Ret => todo!(),
            Instruction::Mret => {}
            Instruction::Hlt => {}
        }

        bytecode
    }
}
