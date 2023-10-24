use crate::reg::registers::RegisterId;

use super::instruction::Instruction;

use num_derive::FromPrimitive;
use std::mem;

/// The size of the instruction, in bytes.
const INSTRUCTION_SIZE: u32 = 4;
/// The size of a u32 argument, in bytes.
const ARG_U32_IMM_SIZE: u32 = mem::size_of::<u32>() as u32;
/// The size of a memory address argument, in bytes.
const ARG_MEM_ADDR_SIZE: u32 = mem::size_of::<u32>() as u32;
/// The size of a register ID argument, in bytes.
const ARG_REG_ID_SIZE: u32 = mem::size_of::<RegisterId>() as u32;

#[repr(u32)]
/// The opcode for an instruction.
#[derive(Clone, Copy, Debug, FromPrimitive)]
pub enum OpCode {
    /// Subroutine - a pseudo-opcode used to identify a subroutine position.
    //Subroutine = -2,
    /// Label - a pseudo-opcode used to identify a labels position.
    //Label = -1,

    /// No Operation - a non-operation instruction.
    Nop,

    /******** [Arithmetic Instructions] ********/
    /// Add u32 Immediate to u32 Register. Result is moved into Accumulator register.
    AddU32ImmU32Reg,
    /// Add u32 Register to u32 Register. Result is moved into Accumulator register.
    AddU32RegU32Reg,

    /******** [Bit Operation Instructions] ********/
    LeftShiftU32ImmU32Reg,
    //LeftShiftU32RegU32Reg,

    /******** [Simple Move Instructions - NO EXPRESSIONS] ********/
    /// Swap the values of the two registers.
    SwapU32RegU32Reg,
    /// Move a u32 Immediate to u32 Register. Result is copied into the register.
    MovU32ImmU32Reg,
    /// Move a u32 Register to u32 Register. Result is copied into the register.
    MovU32RegU32Reg,
    /// Move a u32 Immediate to Memory (relative to the base address of the code block). Result is copied into Memory.
    MovU32ImmMemRelSimple,
    /// Move a u32 Register to Memory (relative to the base address of the code block). Result is copied into Memory.
    MovU32RegMemRelSimple,
    /// Move a u32 value from Memory (relative to the base address of the code block) u32 Register. Result is copied into the Register.
    MovMemU32RegRelSimple,
    /// Move the value from the memory address specified by a Register (relative to the base address of the code block). Result is copied into the other register.
    MovU32RegPtrU32RegRelSimple,

    /******** [Complex Move Instructions - WITH EXPRESSIONS] ********/
    /// Move a u32 Immediate to Memory (relative to the base address of the code block). Result is copied into Memory.
    MovU32ImmMemExprRel,
    /// Move the address as given by an expression (from Memory, relative to the base address of the code block). Result is copied into the Register.
    MovMemExprU32RegRel,
    /// Move the value of a register to the address given by an expression (relative to the base address of the code block). Result is copied into Memory.
    MovU32RegMemExprRel,

    /******** [Special Instructions] ********/
    /// Return - return from a subroutine.
    Ret = 32765,
    /// Machine return - downgrade the privilege level of the processor.
    Mret = 32766,
    /// Halt - halt the execution of the virtual machine.
    Hlt = 32767,
    // Values higher than 32767 (0x7FFF) are extended instructions and require four bytes to represent.
}

impl OpCode {
    pub fn get_instruction_arg_size(&self) -> u32 {
        match self {
            OpCode::Nop => 0,

            /******** [Arithmetic Instructions] ********/
            OpCode::AddU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::AddU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,

            /******** [Bit Operation Instructions] ********/
            OpCode::LeftShiftU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,

            /******** [Move Instructions - NO EXPRESSIONS] ********/
            OpCode::SwapU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32ImmMemRelSimple => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::MovU32RegMemRelSimple => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::MovMemU32RegRelSimple => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32RegPtrU32RegRelSimple => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,

            /******** [Move Instructions - WITH EXPRESSIONS] ********/
            OpCode::MovU32ImmMemExprRel => ARG_U32_IMM_SIZE + ARG_U32_IMM_SIZE,
            OpCode::MovMemExprU32RegRel => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
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

impl From<Instruction> for OpCode {
    fn from(val: Instruction) -> Self {
        match val {
            Instruction::Nop => OpCode::Nop,

            /******** [Arithmetic Instructions] ********/
            Instruction::AddU32ImmU32Reg(_, _) => OpCode::AddU32ImmU32Reg,
            Instruction::AddU32RegU32Reg(_, _) => OpCode::AddU32RegU32Reg,
            Instruction::MovU32ImmU32Reg(_, _) => OpCode::MovU32ImmU32Reg,
            Instruction::MovU32RegU32Reg(_, _) => OpCode::MovU32RegU32Reg,

            /******** [Bit Operation Instructions] ********/
            Instruction::LeftShiftU32ImmU32Reg(_, _) => OpCode::LeftShiftU32ImmU32Reg,

            /******** [Move Instructions - NO EXPRESSIONS] ********/
            Instruction::SwapU32RegU32Reg(_, _) => OpCode::SwapU32RegU32Reg,
            Instruction::MovU32ImmMemRelSimple(_, _) => OpCode::MovU32ImmMemRelSimple,
            Instruction::MovU32RegMemRelSimple(_, _) => OpCode::MovU32RegMemRelSimple,
            Instruction::MovMemU32RegRelSimple(_, _) => OpCode::MovMemU32RegRelSimple,
            Instruction::MovU32RegPtrU32RegRelSimple(_, _) => OpCode::MovU32RegPtrU32RegRelSimple,

            /******** [Move Instructions - WITH EXPRESSIONS] ********/
            Instruction::MovU32ImmMemExprRel(_, _) => OpCode::MovU32ImmMemExprRel,
            Instruction::MovMemExprU32RegRel(_, _) => OpCode::MovMemExprU32RegRel,
            Instruction::MovU32RegMemExprRel(_, _) => OpCode::MovU32RegMemExprRel,

            /******** [Special Instructions] ********/
            Instruction::Ret => OpCode::Ret,
            Instruction::Mret => OpCode::Mret,
            Instruction::Hlt => OpCode::Hlt,
        }
    }
}
