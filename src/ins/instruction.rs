use std::fmt::{self, Display, Formatter};

use crate::{ins::move_expressions::MoveExpressionHandler, reg::registers::RegisterId};

#[derive(Clone, Copy, Debug)]
pub enum Instruction {
    Nop,

    /******** [Bit Operation Instructions] ********/
    LeftShiftU32ImmU32Reg(u32, RegisterId),

    /******** [Arithmetic Instructions] ********/
    AddU32ImmU32Reg(u32, RegisterId),
    AddU32RegU32Reg(RegisterId, RegisterId),

    /******** [Move Instructions - NO EXPRESSIONS] ********/
    SwapU32RegU32Reg(RegisterId, RegisterId),
    MovU32ImmU32Reg(u32, RegisterId),
    MovU32RegU32Reg(RegisterId, RegisterId),
    MovU32ImmMemRelSimple(u32, u32),
    MovU32RegMemRelSimple(RegisterId, u32),
    MovMemU32RegRelSimple(u32, RegisterId),
    MovU32RegPtrU32RegRelSimple(RegisterId, RegisterId),

    /******** [Move Instructions - WITH EXPRESSIONS] ********/
    MovU32ImmMemExprRel(u32, u32),
    MovMemExprU32RegRel(u32, RegisterId),
    MovU32RegMemExprRel(RegisterId, u32),

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

            /******** [Bit Operation Instructions] ********/
            Instruction::LeftShiftU32ImmU32Reg(imm, reg) => {
                format!("lsf ${imm:02X}, {reg}")
            }

            /******** [Move Instructions - NO EXPRESSIONS] ********/
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

            /******** [Move Instructions - WITH EXPRESSIONS] ********/
            Instruction::MovU32ImmMemExprRel(imm, expr) => {
                let mut decoder = MoveExpressionHandler::new();
                decoder.decode(expr);

                format!("mov.rc ${imm:02X}, [{decoder}]")
            }
            Instruction::MovMemExprU32RegRel(expr, reg) => {
                let mut decoder = MoveExpressionHandler::new();
                decoder.decode(expr);

                format!("mov.rc [{decoder}], {reg}")
            }
            Instruction::MovU32RegMemExprRel(reg, expr) => {
                let mut decoder = MoveExpressionHandler::new();
                decoder.decode(expr);

                format!("mov.rc {reg}, [{decoder}]")
            }

            /******** [Special Instructions] ********/
            Instruction::Ret => String::from("ret"),
            Instruction::Mret => String::from("mret"),
            Instruction::Hlt => String::from("hlt"),
        };
        write!(f, "{asm_format}")
    }
}
