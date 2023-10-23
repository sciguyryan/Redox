use crate::reg::registers::RegisterId;

use super::instruction::Instruction;

use num_derive::FromPrimitive;

/// The size of the instruction, in bytes.
const INSTRUCTION_SIZE: u32 = 4;
/// The size of a u32 argument, in bytes.
const ARG_U32_IMM_SIZE: u32 = std::mem::size_of::<u32>() as u32;
/// The size of a memory address argument, in bytes.
const ARG_MEM_ADDR_SIZE: u32 = std::mem::size_of::<u32>() as u32;
/// The size of a register ID argument, in bytes.
const ARG_REG_ID_SIZE: u32 = std::mem::size_of::<RegisterId>() as u32;

#[repr(u32)]
/// The opcode for an instruction.
/// TODO - If the highest bit is set then the instruction is extended and the size of the instruction
/// TODO - is 32 bits, otherwise it can be shortened to just 16 bits.
#[derive(Clone, Copy, Debug, FromPrimitive)]
pub enum OpCode {
    /// Subroutine - a pseudo-opcode used to identify a subroutine position.
    //Subroutine = -2,
    /// Label - a pseudo-opcode used to identify a labels position.
    //Label = -1,

    /// No Operation - a non-operation instruction.
    Nop = 0,

    /******** [Arithmetic Instructions] ********/
    /// Add u32 Immediate to u32 Register. Result is moved into Accumulator register.
    AddU32ImmU32Reg = 1,
    /// Add u32 Register to u32 Register. Result is moved into Accumulator register.
    AddU32RegU32Reg = 2,

    /******** [Simple Move Instructions - NO EXPRESSIONS] ********/
    /// Swap the values of the two registers.
    SwapU32RegU32Reg = 3,
    /// Move a u32 Immediate to u32 Register. Result is copied into the register.
    MovU32ImmU32Reg = 4,
    /// Move a u32 Register to u32 Register. Result is copied into the register.
    MovU32RegU32Reg = 5,
    /// Move a u32 Immediate to Memory (relative to the base address of the code block). Result is copied into memory.
    MovU32ImmMemRelSimple = 6,
    /// Move a u32 Register to Memory (relative to the base address of the code block). Result is copied into memory.
    MovU32RegMemRelSimple = 7,
    /// Move a u32 value from Memory (relative to the base address of the code block) u32 Register. Result is copied into the register.
    MovMemU32RegRelSimple = 8,
    /// Move the value from the memory address specified by a Register (relative to the base address of the code block). Result is copied into the other register.
    MovU32RegPtrU32RegRelSimple = 9,

    /******** [Complex Move Instructions - WITH EXPRESSIONS] ********/
    /// Move a u32 Immediate to Memory (relative to the base address of the code block). The expression contains a register and a constant.
    /// Result is copied into memory.
    MovU32ImmMemRelExpr = 10,

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
    pub fn get_total_instruction_size(&self) -> u32 {
        let size = match self {
            OpCode::Nop => 0,

            /******** [Arithmetic Instructions] ********/
            OpCode::AddU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::AddU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,

            /******** [Simple Move Instructions - NO EXPRESSIONS] ********/
            OpCode::SwapU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32ImmMemRelSimple => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::MovU32RegMemRelSimple => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::MovMemU32RegRelSimple => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32RegPtrU32RegRelSimple => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,

            /******** [Complex Move Instructions - WITH EXPRESSIONS] ********/
            OpCode::MovU32ImmMemRelExpr => ARG_U32_IMM_SIZE + ARG_U32_IMM_SIZE,

            /******** [Special Instructions] ********/
            OpCode::Ret => 0,
            OpCode::Mret => 0,
            OpCode::Hlt => 0,
        };

        size + INSTRUCTION_SIZE
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

            /******** [Simple Move Instructions - NO EXPRESSIONS] ********/
            Instruction::SwapU32RegU32Reg(_, _) => OpCode::SwapU32RegU32Reg,
            Instruction::MovU32ImmMemRelSimple(_, _) => OpCode::MovU32ImmMemRelSimple,
            Instruction::MovU32RegMemRelSimple(_, _) => OpCode::MovU32RegMemRelSimple,
            Instruction::MovMemU32RegRelSimple(_, _) => OpCode::MovMemU32RegRelSimple,
            Instruction::MovU32RegPtrU32RegRelSimple(_, _) => OpCode::MovU32RegPtrU32RegRelSimple,

            /******** [Complex Move Instructions - WITH EXPRESSIONS] ********/
            Instruction::MovU32ImmMemRelExpr(_, _) => OpCode::MovU32ImmMemRelExpr,

            /******** [Special Instructions] ********/
            Instruction::Ret => OpCode::Ret,
            Instruction::Mret => OpCode::Mret,
            Instruction::Hlt => OpCode::Hlt,
        }
    }
}
