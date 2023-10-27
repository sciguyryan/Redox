use super::instruction::Instruction;

use num_derive::FromPrimitive;

#[repr(u32)]
/// The opcode for an instruction.
#[derive(Clone, Copy, Debug, FromPrimitive)]
pub enum OpCode {
    /// No Operation - an empty instruction.
    Nop,

    /******** [Arithmetic Instructions] ********/
    /// Add u32 immediate to u32 register. The result is stored in the accumulator register.
    AddU32ImmU32Reg,
    /// Add u32 register to u32 register. The result is stored in the accumulator register.
    AddU32RegU32Reg,

    /******** [Bit Operation Instructions] ********/
    /// Left-shift a u32 register by a u32 immediate. The result remains in the origin register.
    LeftShiftU32ImmU32Reg,
    /// Left-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    LeftShiftU32RegU32Reg,
    /// Arithmetic left-shift a u32 register by a u32 immediate. The result remains in the origin register.
    ArithLeftShiftU32ImmU32Reg,
    /// Arithmetic left-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    ArithLeftShiftU32RegU32Reg,
    /// Right-shift a u32 register by a u32 immediate. The result remains in the origin register.
    RightShiftU32ImmU32Reg,
    /// Right-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    RightShiftU32RegU32Reg,
    /// Arithmetic right-shift a u32 register by a u32 immediate. The result remains in the origin register.
    ArithRightShiftU32ImmU32Reg,
    /// Arithmetic right-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    ArithRightShiftU32RegU32Reg,

    /******** [Simple Move Instructions - NO EXPRESSIONS] ********/
    /// Swap the values of the two registers.
    SwapU32RegU32Reg,
    /// Move a u32 immediate to u32 register (A). The result is copied into register A.
    MovU32ImmU32Reg,
    /// Move a u32 register (B) to u32 register (A). The result is copied into register A.
    MovU32RegU32Reg,
    /// Move a u32 iImmediate to memory (relative to the base address of the code block). The result is copied into the specified memory address.
    MovU32ImmMemRelSimple,
    /// Move a u32 register to memory (relative to the base address of the code block). The result is copied into the specified memory address.
    MovU32RegMemRelSimple,
    /// Move a u32 value from memory (relative to the base address of the code block) to a u32 register. The result is copied into the specified register.
    MovMemU32RegRelSimple,
    /// Move the value from the memory address specified by a register (relative to the base address of the code block). The result is copied into the specified register.
    MovU32RegPtrU32RegRelSimple,

    /******** [Complex Move Instructions - WITH EXPRESSIONS] ********/
    /// Move a u32 immediate to memory (relative to the base address of the code block). The result is copied into the specified memory address.
    MovU32ImmMemExprRel,
    /// Move the address as given by an expression (from memory, relative to the base address of the code block). The result is copied into the specified register.
    MovMemExprU32RegRel,
    /// Move the value of a register to the address given by an expression (relative to the base address of the code block). The result is copied into the specified memory address.
    MovU32RegMemExprRel,

    /******** [Special Instructions] ********/
    /// Return from a subroutine.
    Ret = 32765,
    /// Machine return - downgrade the privilege level of the processor.
    Mret = 32766,
    /// Halt the execution of the virtual machine.
    Hlt = 32767,
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
            Instruction::LeftShiftU32RegU32Reg(_, _) => OpCode::LeftShiftU32RegU32Reg,
            Instruction::ArithLeftShiftU32ImmU32Reg(_, _) => OpCode::ArithLeftShiftU32ImmU32Reg,
            Instruction::ArithLeftShiftU32RegU32Reg(_, _) => OpCode::ArithLeftShiftU32RegU32Reg,
            Instruction::RightShiftU32ImmU32Reg(_, _) => OpCode::RightShiftU32ImmU32Reg,
            Instruction::RightShiftU32RegU32Reg(_, _) => OpCode::RightShiftU32RegU32Reg,
            Instruction::ArithRightShiftU32ImmU32Reg(_, _) => OpCode::ArithRightShiftU32ImmU32Reg,
            Instruction::ArithRightShiftU32RegU32Reg(_, _) => OpCode::ArithRightShiftU32RegU32Reg,

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
