use std::{
    fmt::{self, Display, Formatter},
    mem,
};

use crate::{ins::move_expressions::MoveExpressionHandler, reg::registers::RegisterId};

use super::op_codes::OpCode;

/// The size of the instruction, in bytes.
const INSTRUCTION_SIZE: u32 = mem::size_of::<u32>() as u32;
/// The size of a u32 argument, in bytes.
const ARG_U32_IMM_SIZE: u32 = mem::size_of::<u32>() as u32;
/// The size of a u8 argument, in bytes.
const ARG_U8_IMM_SIZE: u32 = 1;
/// The size of a memory address argument, in bytes.
const ARG_MEM_ADDR_SIZE: u32 = mem::size_of::<u32>() as u32;
/// The size of a register ID argument, in bytes.
const ARG_REG_ID_SIZE: u32 = mem::size_of::<RegisterId>() as u32;

#[derive(Clone, Copy, Debug)]
pub enum Instruction {
    /// No operation.
    Nop,

    /******** [Arithmetic Instructions] ********/
    /// Add u32 immediate to u32 register. The result is stored in the accumulator register.
    AddU32ImmU32Reg(u32, RegisterId),
    /// Add u32 register to u32 register. The result is stored in the accumulator register.
    AddU32RegU32Reg(RegisterId, RegisterId),

    /******** [Bit Operation Instructions] ********/
    /// Left-shift a u32 register by a u32 immediate. The result remains in the origin register.
    LeftShiftU32ImmU32Reg(u32, RegisterId),
    /// Left-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    LeftShiftU32RegU32Reg(RegisterId, RegisterId),
    /// Arithmetic left-shift a u32 register by a u32 immediate. The result remains in the origin register.
    ArithLeftShiftU32ImmU32Reg(u32, RegisterId),
    /// Arithmetic left-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    ArithLeftShiftU32RegU32Reg(RegisterId, RegisterId),
    /// Right-shift a u32 register by a u32 immediate. The result remains in the origin register.
    RightShiftU32ImmU32Reg(u32, RegisterId),
    /// Right-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    RightShiftU32RegU32Reg(RegisterId, RegisterId),
    /// Arithmetic right-shift a u32 register by a u32 immediate. The result remains in the origin register.
    ArithRightShiftU32ImmU32Reg(u32, RegisterId),
    /// Arithmetic right-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    ArithRightShiftU32RegU32Reg(RegisterId, RegisterId),

    /******** [Data Instructions] ********/
    /******** [Move Instructions - NO EXPRESSIONS] ********/
    /// Swap the values of the two registers.
    SwapU32RegU32Reg(RegisterId, RegisterId),
    /// Move a u32 immediate to u32 register (A). The result is copied into register A.
    MovU32ImmU32Reg(u32, RegisterId),
    /// Move a u32 register (B) to u32 register (A). The result is copied into register A.
    MovU32RegU32Reg(RegisterId, RegisterId),
    /// Move a u32 iImmediate to memory (relative to the base address of the code block). The result is copied into the specified memory address.
    MovU32ImmMemRelSimple(u32, u32),
    /// Move a u32 register to memory (relative to the base address of the code block). The result is copied into the specified memory address.
    MovU32RegMemRelSimple(RegisterId, u32),
    /// Move a u32 value from memory (relative to the base address of the code block) to a u32 register. The result is copied into the specified register.
    MovMemU32RegRelSimple(u32, RegisterId),
    /// Move the value from the memory address specified by a register (relative to the base address of the code block). The result is copied into the specified register.
    MovU32RegPtrU32RegRelSimple(RegisterId, RegisterId),

    /******** [Move Instructions - WITH EXPRESSIONS] ********/
    /// Move a u32 immediate to memory (relative to the base address of the code block). The result is copied into the specified memory address.
    MovU32ImmMemExprRel(u32, u32),
    /// Move the address as given by an expression (from memory, relative to the base address of the code block). The result is copied into the specified register.
    MovMemExprU32RegRel(u32, RegisterId),
    /// Move the value of a register to the address given by an expression (relative to the base address of the code block). The result is copied into the specified memory address.
    MovU32RegMemExprRel(RegisterId, u32),

    /******** [Logic Instructions] ********/
    /// Test the value of a bit in a register. The CF flag will be set to the value of the bit.
    BitTestU32Reg(u8, RegisterId),

    /******** [Special Instructions] ********/
    /// Return from a subroutine.
    Ret,
    /// Machine return - downgrade the privilege level of the processor.
    Mret,
    /// Halt the execution of the processor.
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
                format!("shl ${imm:02X}, {reg}")
            }
            Instruction::LeftShiftU32RegU32Reg(shift_reg, reg) => {
                format!("shl {shift_reg}, {reg}")
            }
            Instruction::ArithLeftShiftU32ImmU32Reg(imm, reg) => {
                format!("sal ${imm:02X}, {reg}")
            }
            Instruction::ArithLeftShiftU32RegU32Reg(shift_reg, reg) => {
                format!("sal {shift_reg}, {reg}")
            }
            Instruction::RightShiftU32ImmU32Reg(imm, reg) => {
                format!("shr ${imm:02X}, {reg}")
            }
            Instruction::RightShiftU32RegU32Reg(shift_reg, reg) => {
                format!("shr {shift_reg}, {reg}")
            }
            Instruction::ArithRightShiftU32ImmU32Reg(imm, reg) => {
                format!("sar ${imm:02X}, {reg}")
            }
            Instruction::ArithRightShiftU32RegU32Reg(shift_reg, reg) => {
                format!("sar {shift_reg}, {reg}")
            }

            /******** [Data Instructions] ********/
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

            /******** [Logic Instructions] ********/
            Instruction::BitTestU32Reg(bit, reg) => {
                format!("bt {bit}, {reg}")
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
    pub fn get_instruction_arg_size(&self) -> u32 {
        match *self {
            Instruction::Nop => 0,

            /******** [Arithmetic Instructions] ********/
            Instruction::AddU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::AddU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,

            /******** [Bit Operation Instructions] ********/
            Instruction::LeftShiftU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::LeftShiftU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::ArithLeftShiftU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::ArithLeftShiftU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::RightShiftU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::RightShiftU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::ArithRightShiftU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::ArithRightShiftU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,

            /******** [Data Instructions] ********/
            /******** [Move Instructions - NO EXPRESSIONS] ********/
            Instruction::SwapU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::MovU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::MovU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::MovU32ImmMemRelSimple(_, _) => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            Instruction::MovU32RegMemRelSimple(_, _) => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            Instruction::MovMemU32RegRelSimple(_, _) => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            Instruction::MovU32RegPtrU32RegRelSimple(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,

            /******** [Move Instructions - WITH EXPRESSIONS] ********/
            Instruction::MovU32ImmMemExprRel(_, _) => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            Instruction::MovMemExprU32RegRel(_, _) => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            Instruction::MovU32RegMemExprRel(_, _) => ARG_REG_ID_SIZE + ARG_U32_IMM_SIZE,

            Instruction::BitTestU32Reg(_, _) => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,

            /******** [Special Instructions] ********/
            Instruction::Ret => 0,
            Instruction::Mret => 0,
            Instruction::Hlt => 0,
        }
    }

    pub fn get_instruction_arg_size_from_op(opcode: OpCode) -> u32 {
        // Note: Yes, yes... code duplication since this is the same code as
        //       get_instruction_arg_size above, but I don't care here.
        match opcode {
            OpCode::Nop => 0,

            /******** [Arithmetic Instructions] ********/
            OpCode::AddU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::AddU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,

            /******** [Bit Operation Instructions] ********/
            OpCode::LeftShiftU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::LeftShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::ArithLeftShiftU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::ArithLeftShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::RightShiftU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::RightShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::ArithRightShiftU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::ArithRightShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,

            /******** [Data Instructions] ********/
            /******** [Move Instructions - NO EXPRESSIONS] ********/
            OpCode::SwapU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32ImmMemRelSimple => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::MovU32RegMemRelSimple => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::MovMemU32RegRelSimple => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32RegPtrU32RegRelSimple => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,

            /******** [Logic Instructions] ********/
            OpCode::BitTestU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,

            /******** [Move Instructions - WITH EXPRESSIONS] ********/
            OpCode::MovU32ImmMemExprRel => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::MovMemExprU32RegRel => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32RegMemExprRel => ARG_REG_ID_SIZE + ARG_U32_IMM_SIZE,

            /******** [Special Instructions] ********/
            OpCode::Ret => 0,
            OpCode::Mret => 0,
            OpCode::Hlt => 0,
        }
    }

    pub fn get_total_instruction_size(&self) -> u32 {
        self.get_instruction_arg_size() + INSTRUCTION_SIZE
    }
}
