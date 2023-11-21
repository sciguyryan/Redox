use super::instruction::Instruction;

use num_derive::FromPrimitive;

#[repr(u32)]
/// The opcode for an instruction.
#[derive(Clone, Copy, Debug, FromPrimitive)]
pub enum OpCode {
    /// No Operation - an empty instruction.
    Nop,

    /******** [Arithmetic Instructions] ********/
    /// Add a u32 immediate to u32 register. The result is stored in the accumulator register.
    AddU32ImmU32Reg,
    /// Add a u32 register to u32 register. The result is stored in the accumulator register.
    AddU32RegU32Reg,
    /// Subtract a u32 immediate from a u32 register. The result is stored in the accumulator register.
    SubU32ImmU32Reg,
    /// Subtract a u32 register from a u32 immediate. The result is stored in the accumulator register.
    SubU32RegU32Imm,
    /// Subtract a u32 register (A) from a u32 register (B). The result is stored in the accumulator register.
    SubU32RegU32Reg,
    /// Increment a u32 register.
    IncU32Reg,

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

    /******** [Data Instructions] ********/
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
    /// Move a u32 immediate to memory (relative to the base address of the code block). The result is copied into the specified memory address.
    MovU32ImmMemExprRel,
    /// Move the address as given by an expression (from memory, relative to the base address of the code block). The result is copied into the specified register.
    MovMemExprU32RegRel,
    /// Move the value of a register to the address given by an expression (relative to the base address of the code block). The result is copied into the specified memory address.
    MovU32RegMemExprRel,
    /// Reverse the order of bytes in a specified register.
    ByteSwapU32,
    /// Zero the high bits of the source value starting from a specified index.
    ///
    /// The carry and zero flags may be set depending on the result and the overflow flag will always be cleared.
    ZeroHighBitsByIndexU32Reg,
    /// Zero the high bits of the source value starting from a specified index.
    ///
    /// The carry and zero flags may be set depending on the result and the overflow flag will always be cleared.
    ZeroHighBitsByIndexU32RegU32Imm,

    /******** [Logic Instructions] ********/
    /// Test the state of a bit from a u32 register. The CF flag will be set to the state of the bit.
    BitTestU32Reg,
    /// Test the state of a bit from a u32 value (starting at the specified memory address). The CF flag will be set to the state of the bit.
    BitTestU32Mem,
    /// Test the state of a bit from a u32 register and clear the bit. The CF flag will be set to the original state of the bit.
    BitTestResetU32Reg,
    /// Test the state of a bit of a u32 value (starting at the specified memory address) and clear the bit. The CF flag will be set to the original state of the bit.
    BitTestResetU32Mem,
    /// Test the state of a bit of a u32 register and set the bit. The CF flag will be set to the original state of the bit.
    BitTestSetU32Reg,
    /// Test the state of a bit of a u32 value (starting at the specified memory address) and set the bit. The CF flag will be set to the original state of the bit.
    BitTestSetU32Mem,
    /// Search for the most significant set bit of a u32 register (A) and store the index of the bit in a u32 register (B).
    BitScanReverseU32RegU32Reg,
    /// Search for the most significant set bit of a u32 value (starting at the specified memory address) and store the index of the bit in a u32 register.
    BitScanReverseU32MemU32Reg,
    /// Search for the most significant set bit of a u32 register and store the index of the bit as a u32 value starting at a specified memory address.
    BitScanReverseU32RegMemU32,
    /// Search for the most significant set bit of a u32 value (starting at the specified memory address) and store the index of the bit as a u32 value starting at a specified memory address.
    BitScanReverseU32MemU32Mem,
    /// Search for the least significant set bit of a u32 register (A) and store the index of the bit in a u32 register (B).
    BitScanForwardU32RegU32Reg,
    /// Search for the least significant set bit of a u32 value (starting at the specified memory address) and store the index of the bit in a u32 register.
    BitScanForwardU32MemU32Reg,
    /// Search for the least significant set bit of a u32 register and store the index of the bit as a u32 value starting at a specified memory address.
    BitScanForwardU32RegMemU32,
    /// Search for the least significant set bit of a u32 value (starting at the specified memory address) and store the index of the bit as a u32 value starting at a specified memory address.
    BitScanForwardU32MemU32Mem,

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
            Instruction::SubU32ImmU32Reg(_, _) => OpCode::SubU32ImmU32Reg,
            Instruction::SubU32RegU32Imm(_, _) => OpCode::SubU32RegU32Imm,
            Instruction::SubU32RegU32Reg(_, _) => OpCode::SubU32RegU32Reg,
            Instruction::IncU32Reg(_) => OpCode::IncU32Reg,

            /******** [Bit Operation Instructions] ********/
            Instruction::LeftShiftU32ImmU32Reg(_, _) => OpCode::LeftShiftU32ImmU32Reg,
            Instruction::LeftShiftU32RegU32Reg(_, _) => OpCode::LeftShiftU32RegU32Reg,
            Instruction::ArithLeftShiftU32ImmU32Reg(_, _) => OpCode::ArithLeftShiftU32ImmU32Reg,
            Instruction::ArithLeftShiftU32RegU32Reg(_, _) => OpCode::ArithLeftShiftU32RegU32Reg,
            Instruction::RightShiftU32ImmU32Reg(_, _) => OpCode::RightShiftU32ImmU32Reg,
            Instruction::RightShiftU32RegU32Reg(_, _) => OpCode::RightShiftU32RegU32Reg,
            Instruction::ArithRightShiftU32ImmU32Reg(_, _) => OpCode::ArithRightShiftU32ImmU32Reg,
            Instruction::ArithRightShiftU32RegU32Reg(_, _) => OpCode::ArithRightShiftU32RegU32Reg,

            /******** [Data Instructions] ********/
            Instruction::SwapU32RegU32Reg(_, _) => OpCode::SwapU32RegU32Reg,
            Instruction::MovU32ImmU32Reg(_, _) => OpCode::MovU32ImmU32Reg,
            Instruction::MovU32RegU32Reg(_, _) => OpCode::MovU32RegU32Reg,
            Instruction::MovU32ImmMemRelSimple(_, _) => OpCode::MovU32ImmMemRelSimple,
            Instruction::MovU32RegMemRelSimple(_, _) => OpCode::MovU32RegMemRelSimple,
            Instruction::MovMemU32RegRelSimple(_, _) => OpCode::MovMemU32RegRelSimple,
            Instruction::MovU32RegPtrU32RegRelSimple(_, _) => OpCode::MovU32RegPtrU32RegRelSimple,
            Instruction::MovU32ImmMemExprRel(_, _) => OpCode::MovU32ImmMemExprRel,
            Instruction::MovMemExprU32RegRel(_, _) => OpCode::MovMemExprU32RegRel,
            Instruction::MovU32RegMemExprRel(_, _) => OpCode::MovU32RegMemExprRel,
            Instruction::ByteSwapU32(_) => OpCode::ByteSwapU32,
            Instruction::ZeroHighBitsByIndexU32Reg(_, _, _) => OpCode::ZeroHighBitsByIndexU32Reg,
            Instruction::ZeroHighBitsByIndexU32RegU32Imm(_, _, _) => {
                OpCode::ZeroHighBitsByIndexU32RegU32Imm
            }

            /******** [Logic Instructions] ********/
            Instruction::BitTestU32Reg(_, _) => OpCode::BitTestU32Reg,
            Instruction::BitTestU32Mem(_, _) => OpCode::BitTestU32Mem,
            Instruction::BitTestResetU32Reg(_, _) => OpCode::BitTestResetU32Reg,
            Instruction::BitTestResetU32Mem(_, _) => OpCode::BitTestResetU32Mem,
            Instruction::BitTestSetU32Reg(_, _) => OpCode::BitTestSetU32Reg,
            Instruction::BitTestSetU32Mem(_, _) => OpCode::BitTestSetU32Mem,
            Instruction::BitScanReverseU32RegU32Reg(_, _) => OpCode::BitScanReverseU32RegU32Reg,
            Instruction::BitScanReverseU32MemU32Reg(_, _) => OpCode::BitScanReverseU32MemU32Reg,
            Instruction::BitScanReverseU32RegMemU32(_, _) => OpCode::BitScanReverseU32RegMemU32,
            Instruction::BitScanReverseU32MemU32Mem(_, _) => OpCode::BitScanReverseU32MemU32Mem,
            Instruction::BitScanForwardU32RegU32Reg(_, _) => OpCode::BitScanForwardU32RegU32Reg,
            Instruction::BitScanForwardU32MemU32Reg(_, _) => OpCode::BitScanForwardU32MemU32Reg,
            Instruction::BitScanForwardU32RegMemU32(_, _) => OpCode::BitScanForwardU32RegMemU32,
            Instruction::BitScanForwardU32MemU32Mem(_, _) => OpCode::BitScanForwardU32MemU32Mem,

            /******** [Special Instructions] ********/
            Instruction::Ret => OpCode::Ret,
            Instruction::Mret => OpCode::Mret,
            Instruction::Hlt => OpCode::Hlt,
        }
    }
}
