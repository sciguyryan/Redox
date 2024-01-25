use num_derive::FromPrimitive;
use strum_macros::EnumIter;

use super::instruction::Instruction;

/// The opcode for an instruction.
#[repr(u32)]
#[derive(
    Clone, Copy, Debug, Default, Eq, PartialOrd, Ord, PartialEq, Hash, FromPrimitive, EnumIter,
)]
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
    /// Unsigned multiplication of the register ER1 by a u32 immediate. The result is stored in the register ER1.
    MulU32Imm,
    /// Unsigned multiplication of the register ER1 by a u32 register. The result is stored in the register ER1.
    MulU32Reg,
    /// Unsigned division of the register ER1 by a u32 immediate. The quotient is stored in the register ER1 and the modulo is stored in ER4.
    DivU32Imm,
    /// Unsigned division of the register ER1 by a u32 register. The quotient is stored in the register ER1 and the modulo is stored in ER4.
    DivU32Reg,
    /// Increment a u32 register.
    IncU32Reg,
    /// Decrement a u32 register.
    DecU32Reg,
    /// Perform a logical AND operation on a u32 immediate and a u32 register.
    AndU32ImmU32Reg,

    /******** [Bit Operation Instructions] ********/
    /// Left-shift a u32 register by a u8 immediate. The result remains in the origin register.
    LeftShiftU8ImmU32Reg,
    /// Left-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    LeftShiftU32RegU32Reg,
    /// Arithmetic left-shift a u32 register by a u8 immediate. The result remains in the origin register.
    ArithLeftShiftU8ImmU32Reg,
    /// Arithmetic left-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    ArithLeftShiftU32RegU32Reg,
    /// Right-shift a u32 register by a u8 immediate. The result remains in the origin register.
    RightShiftU8ImmU32Reg,
    /// Right-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    RightShiftU32RegU32Reg,
    /// Arithmetic right-shift a u32 register by a u32 immediate. The result remains in the origin register.
    ArithRightShiftU8ImmU32Reg,
    /// Arithmetic right-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    ArithRightShiftU32RegU32Reg,

    /******** [Branching Instructions] ********/
    /// Call a subroutine by a provided u32 immediate address.
    CallU32Imm,
    /// Call a subroutine by the address as specified by a u32 register.
    CallU32Reg,
    /// Return from a subroutine that had zero or more u32 arguments supplied.
    RetArgsU32,
    /// Trigger a specific type of interrupt handler.
    Int,
    /// Returns from an interrupt handler.
    IntRet,
    /// Unconditional jump to a specified address.
    JumpAbsU32Imm,
    /// Unconditional jump to an address as specified by a u32 register.
    JumpAbsU32Reg,

    /******** [Data Instructions] ********/
    /// Swap the values of the two registers.
    SwapU32RegU32Reg,
    /// Move a u32 immediate to u32 register (A). The result is copied into register A.
    MovU32ImmU32Reg,
    /// Move a u32 register (B) to u32 register (A). The result is copied into register A.
    MovU32RegU32Reg,
    /// Move a u32 immediate to memory. The result is copied into the specified memory address.
    MovU32ImmMemSimple,
    /// Move a u32 register to memory. The result is copied into the specified memory address.
    MovU32RegMemSimple,
    /// Move a u32 value from memory to a u32 register. The result is copied into the specified register.
    MovMemU32RegSimple,
    /// Move a u32 value from the memory address specified by a register. The result is copied into the specified register.
    MovU32RegPtrU32RegSimple,
    /// Move a u32 value to the memory address as given by an expression. The result is copied into the specified memory address.
    MovU32ImmMemExpr,
    /// Move a u32 value from address as given by an expression to a u32 register. The result is copied into the specified register.
    MovMemExprU32Reg,
    /// Move the value of a u32 register to the address given by an expression. The result is copied into the specified memory address.
    MovU32RegMemExpr,
    /// Reverse the order of bytes in a specified u32 register.
    ByteSwapU32,
    /// Zero the high bits of the source value starting from a specified index.
    ///
    /// The carry and zero flags may be set depending on the result and the overflow flag will always be cleared.
    ZeroHighBitsByIndexU32Reg,
    /// Zero the high bits of the source value starting from a specified index.
    ///
    /// The carry and zero flags may be set depending on the result and the overflow flag will always be cleared.
    ZeroHighBitsByIndexU32RegU32Imm,
    /// Push a f32 immediate value onto the stack.
    PushF32Imm,
    /// Push a u32 immediate value onto the stack.
    PushU32Imm,
    /// Push the value of a u32 register onto the stack.
    PushU32Reg,
    /// Pop a f32 value from the stack to a f32 register.
    PopF32ToF32Reg,
    /// Pop a u32 value from the stack to a u32 register.
    PopU32ToU32Reg,

    /******** [IO Instructions] ********/
    /// Output a f32 immediate value to a specific port.
    OutF32Imm,
    /// Output a u32 immediate value to a specific port.
    OutU32Imm,
    /// Output a u32 register value value to a specific port.
    OutU32Reg,
    /// Output a u8 immediate value to a specific port.
    OutU8Imm,
    /// Input a u8 value from a specific port into a specified register.
    InU8Reg,
    /// Input a u8 value from a specific port into a specified memory address.
    InU8Mem,
    /// Input a u32 value from a specific port into a specified register.
    InU32Reg,
    /// Input a u32 value from a specific port into a specified memory address.
    InU32Mem,
    /// Input a f32 value from a specific port into a specified register.
    InF32Reg,
    /// Input a f32 value from a specific port into a specified memory address.
    InF32Mem,

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
    /// Mask a specific interrupt.
    MaskInterrupt,
    /// Unmask a specific interrupt.
    UnmaskInterrupt,
    /// Set the location of the interrupt vector table.
    ///
    /// # Note
    ///
    /// This can only be done in machine mode.
    LoadIVTAddrU32Imm,
    /// Machine-mode return - downgrade the privilege level of the processor.
    MachineReturn = 65534,
    /// Halt the execution of the virtual machine.
    Halt = 65535,

    // Anything above u32::MAX - 10 (0xfffffff5) is an invalid opcode. These are reserved for future or special use.
    Reserved1 = u32::MAX - 10,
    Reserved2,
    Reserved3,
    Reserved4,
    Reserved5,
    Reserved6,
    Reserved7,
    Reserved8,
    Reserved9,

    /// A placeholder opcode for labels. These do not directly compile to anything and are for used for computing jumps. This should never be constructed directly.
    Label,
    /// A placeholder for instances where the opcode isn't recognized. This should never be constructed directly.
    #[default]
    Unknown,
}

impl From<&Instruction> for OpCode {
    fn from(val: &Instruction) -> Self {
        use Instruction as I;
        use OpCode as O;

        match val {
            I::Nop => O::Nop,

            /******** [Arithmetic Instructions] ********/
            I::AddU32ImmU32Reg(_, _) => O::AddU32ImmU32Reg,
            I::AddU32RegU32Reg(_, _) => O::AddU32RegU32Reg,
            I::SubU32ImmU32Reg(_, _) => O::SubU32ImmU32Reg,
            I::SubU32RegU32Imm(_, _) => O::SubU32RegU32Imm,
            I::SubU32RegU32Reg(_, _) => O::SubU32RegU32Reg,
            I::MulU32Imm(_) => O::MulU32Imm,
            I::MulU32Reg(_) => O::MulU32Reg,
            I::DivU32Imm(_) => O::DivU32Imm,
            I::DivU32Reg(_) => O::DivU32Reg,
            I::IncU32Reg(_) => O::IncU32Reg,
            I::DecU32Reg(_) => O::DecU32Reg,
            I::AndU32ImmU32Reg(_, _) => O::AndU32ImmU32Reg,

            /******** [Bit Operation Instructions] ********/
            I::LeftShiftU8ImmU32Reg(_, _) => O::LeftShiftU8ImmU32Reg,
            I::LeftShiftU32RegU32Reg(_, _) => O::LeftShiftU32RegU32Reg,
            I::ArithLeftShiftU8ImmU32Reg(_, _) => O::ArithLeftShiftU8ImmU32Reg,
            I::ArithLeftShiftU32RegU32Reg(_, _) => O::ArithLeftShiftU32RegU32Reg,
            I::RightShiftU8ImmU32Reg(_, _) => O::RightShiftU8ImmU32Reg,
            I::RightShiftU32RegU32Reg(_, _) => O::RightShiftU32RegU32Reg,
            I::ArithRightShiftU8ImmU32Reg(_, _) => O::ArithRightShiftU8ImmU32Reg,
            I::ArithRightShiftU32RegU32Reg(_, _) => O::ArithRightShiftU32RegU32Reg,

            /******** [Branching Instructions] ********/
            I::CallU32Imm(_) => O::CallU32Imm,
            I::CallU32Reg(_) => O::CallU32Reg,
            I::RetArgsU32 => O::RetArgsU32,
            I::Int(_) => O::Int,
            I::IntRet => O::IntRet,
            I::JumpAbsU32Imm(_) => O::JumpAbsU32Imm,
            I::JumpAbsU32Reg(_) => O::JumpAbsU32Reg,

            /******** [Data Instructions] ********/
            I::SwapU32RegU32Reg(_, _) => O::SwapU32RegU32Reg,
            I::MovU32ImmU32Reg(_, _) => O::MovU32ImmU32Reg,
            I::MovU32RegU32Reg(_, _) => O::MovU32RegU32Reg,
            I::MovU32ImmMemSimple(_, _) => O::MovU32ImmMemSimple,
            I::MovU32RegMemSimple(_, _) => O::MovU32RegMemSimple,
            I::MovMemU32RegSimple(_, _) => O::MovMemU32RegSimple,
            I::MovU32RegPtrU32RegSimple(_, _) => O::MovU32RegPtrU32RegSimple,
            I::MovU32ImmMemExpr(_, _) => O::MovU32ImmMemExpr,
            I::MovMemExprU32Reg(_, _) => O::MovMemExprU32Reg,
            I::MovU32RegMemExpr(_, _) => O::MovU32RegMemExpr,
            I::ByteSwapU32(_) => O::ByteSwapU32,
            I::ZeroHighBitsByIndexU32Reg(_, _, _) => O::ZeroHighBitsByIndexU32Reg,
            I::ZeroHighBitsByIndexU32RegU32Imm(_, _, _) => O::ZeroHighBitsByIndexU32RegU32Imm,
            I::PushU32Imm(_) => O::PushU32Imm,
            I::PushF32Imm(_) => O::PushF32Imm,
            I::PushU32Reg(_) => O::PushU32Reg,
            I::PopF32ToF32Reg(_) => O::PopF32ToF32Reg,
            I::PopU32ToU32Reg(_) => O::PopU32ToU32Reg,

            /******** [IO Instructions] ********/
            I::OutF32Imm(_, _) => O::OutF32Imm,
            I::OutU32Imm(_, _) => O::OutU32Imm,
            I::OutU32Reg(_, _) => O::OutU32Reg,
            I::OutU8Imm(_, _) => O::OutU8Imm,
            I::InU8Reg(_, _) => O::InU8Reg,
            I::InU8Mem(_, _) => O::InU8Mem,
            I::InU32Reg(_, _) => O::InU32Reg,
            I::InU32Mem(_, _) => O::InU32Mem,
            I::InF32Reg(_, _) => O::InF32Reg,
            I::InF32Mem(_, _) => O::InF32Mem,

            /******** [Logic Instructions] ********/
            I::BitTestU32Reg(_, _) => O::BitTestU32Reg,
            I::BitTestU32Mem(_, _) => O::BitTestU32Mem,
            I::BitTestResetU32Reg(_, _) => O::BitTestResetU32Reg,
            I::BitTestResetU32Mem(_, _) => O::BitTestResetU32Mem,
            I::BitTestSetU32Reg(_, _) => O::BitTestSetU32Reg,
            I::BitTestSetU32Mem(_, _) => O::BitTestSetU32Mem,
            I::BitScanReverseU32RegU32Reg(_, _) => O::BitScanReverseU32RegU32Reg,
            I::BitScanReverseU32MemU32Reg(_, _) => O::BitScanReverseU32MemU32Reg,
            I::BitScanReverseU32RegMemU32(_, _) => O::BitScanReverseU32RegMemU32,
            I::BitScanReverseU32MemU32Mem(_, _) => O::BitScanReverseU32MemU32Mem,
            I::BitScanForwardU32RegU32Reg(_, _) => O::BitScanForwardU32RegU32Reg,
            I::BitScanForwardU32MemU32Reg(_, _) => O::BitScanForwardU32MemU32Reg,
            I::BitScanForwardU32RegMemU32(_, _) => O::BitScanForwardU32RegMemU32,
            I::BitScanForwardU32MemU32Mem(_, _) => O::BitScanForwardU32MemU32Mem,

            /******** [Special Instructions] ********/
            I::MaskInterrupt(_) => O::MaskInterrupt,
            I::UnmaskInterrupt(_) => O::UnmaskInterrupt,
            I::LoadIVTAddrU32Imm(_) => O::LoadIVTAddrU32Imm,
            I::MachineReturn => O::MachineReturn,
            I::Halt => O::Halt,

            /******** [Reserved Instructions] ********/
            I::Reserved1 => O::Reserved1,
            I::Reserved2 => O::Reserved2,
            I::Reserved3 => O::Reserved3,
            I::Reserved4 => O::Reserved4,
            I::Reserved5 => O::Reserved5,
            I::Reserved6 => O::Reserved6,
            I::Reserved7 => O::Reserved7,
            I::Reserved8 => O::Reserved8,
            I::Reserved9 => O::Reserved9,

            /******** [Pseudo Instructions] ********/
            I::Label(_) => O::Label,
            I::Unknown(_) => O::Unknown,
        }
    }
}

#[cfg(test)]
mod tests_opcodes {
    use strum::IntoEnumIterator;

    use crate::ins::instruction::Instruction;

    use super::OpCode;

    /// Test to ensure every [`OpCode`] is mapped to an [`Instruction`] and vice versa. Also checks to ensure they have equal sizes.
    #[test]
    fn test_mapped_opcodes() {
        let opcodes: Vec<OpCode> = OpCode::iter().collect();
        let instructions: Vec<Instruction> = Instruction::iter().collect();

        // Ensure that the collections are the same length.
        assert_eq!(opcodes.len(), instructions.len());

        let mut success = true;

        // Ensure that each instruction produces the same opcode back.
        for (op, ins) in opcodes.iter().zip(instructions.iter()) {
            // Get the opcode from the instruction.
            let op_from_ins: OpCode = ins.into();
            if op_from_ins != *op {
                eprintln!("instruction {ins} has an opcode mismatch - expected {op_from_ins:?} but got {:?}", *op);
                success = false;
            }
        }

        assert!(
            success,
            "one or more opcodes failed to match with an instruction"
        );
    }
}
