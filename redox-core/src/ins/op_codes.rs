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
    /// Multiply a u32 register by a u32 immediate. The result is stored in the accumulator register.
    MulU32ImmU32Reg,
    /// Multiply a u32 register by a u32 register. The result is stored in the accumulator register.
    MulU32RegU32Reg,
    /// Divide a u32 register by a u32 immediate. The result is stored in the accumulator register.
    DivU32ImmU32Reg,
    /// Divide a u32 immediate by a u32 register. The result is stored in the accumulator register.
    DivU32RegU32Imm,
    /// Divide a u32 register (B) by a u32 register (A). The result is stored in the accumulator register.
    DivU32RegU32Reg,
    /// Calculate the modulo of a u32 register by a u32 immediate. The result is stored in the accumulator register.
    ModU32ImmU32Reg,
    /// Calculate the modulo of a u32 immediate by a u32 register. The result is stored in the accumulator register.
    ModU32RegU32Imm,
    /// Calculate the modulo of a u32 register (B) by a u32 register (A). The result is stored in the accumulator register.
    ModU32RegU32Reg,
    /// Increment a u32 register.
    IncU32Reg,
    /// Decrement a u32 register.
    DecU32Reg,
    /// Perform a logical AND operation on a u32 immediate and a u32 register.
    AndU32ImmU32Reg,

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
    /// Move the value from the memory address specified by a register. The result is copied into the specified register.
    MovU32RegPtrU32RegSimple,
    /// Move a u32 immediate to memory. The result is copied into the specified memory address.
    MovU32ImmMemExpr,
    /// Move the value at the address as given by an expression to a u32 register. The result is copied into the specified register.
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
    /// Push a u32 immediate value onto the stack.
    PushU32Imm,
    /// Push a f32 immediate value onto the stack.
    PushF32Imm,
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
        use Instruction::*;

        match val {
            Instruction::Nop => OpCode::Nop,

            /******** [Arithmetic Instructions] ********/
            AddU32ImmU32Reg(_, _) => OpCode::AddU32ImmU32Reg,
            AddU32RegU32Reg(_, _) => OpCode::AddU32RegU32Reg,
            SubU32ImmU32Reg(_, _) => OpCode::SubU32ImmU32Reg,
            SubU32RegU32Imm(_, _) => OpCode::SubU32RegU32Imm,
            SubU32RegU32Reg(_, _) => OpCode::SubU32RegU32Reg,
            MulU32ImmU32Reg(_, _) => OpCode::MulU32ImmU32Reg,
            MulU32RegU32Reg(_, _) => OpCode::MulU32RegU32Reg,
            DivU32ImmU32Reg(_, _) => OpCode::DivU32ImmU32Reg,
            DivU32RegU32Imm(_, _) => OpCode::DivU32RegU32Imm,
            DivU32RegU32Reg(_, _) => OpCode::DivU32RegU32Reg,
            ModU32ImmU32Reg(_, _) => OpCode::ModU32ImmU32Reg,
            ModU32RegU32Imm(_, _) => OpCode::ModU32RegU32Imm,
            ModU32RegU32Reg(_, _) => OpCode::ModU32RegU32Reg,
            IncU32Reg(_) => OpCode::IncU32Reg,
            DecU32Reg(_) => OpCode::DecU32Reg,
            AndU32ImmU32Reg(_, _) => OpCode::AndU32ImmU32Reg,

            /******** [Bit Operation Instructions] ********/
            LeftShiftU32ImmU32Reg(_, _) => OpCode::LeftShiftU32ImmU32Reg,
            LeftShiftU32RegU32Reg(_, _) => OpCode::LeftShiftU32RegU32Reg,
            ArithLeftShiftU32ImmU32Reg(_, _) => OpCode::ArithLeftShiftU32ImmU32Reg,
            ArithLeftShiftU32RegU32Reg(_, _) => OpCode::ArithLeftShiftU32RegU32Reg,
            RightShiftU32ImmU32Reg(_, _) => OpCode::RightShiftU32ImmU32Reg,
            RightShiftU32RegU32Reg(_, _) => OpCode::RightShiftU32RegU32Reg,
            ArithRightShiftU32ImmU32Reg(_, _) => OpCode::ArithRightShiftU32ImmU32Reg,
            ArithRightShiftU32RegU32Reg(_, _) => OpCode::ArithRightShiftU32RegU32Reg,

            /******** [Branching Instructions] ********/
            CallU32Imm(_) => OpCode::CallU32Imm,
            CallU32Reg(_) => OpCode::CallU32Reg,
            RetArgsU32 => OpCode::RetArgsU32,
            Int(_) => OpCode::Int,
            IntRet => OpCode::IntRet,
            JumpAbsU32Imm(_) => OpCode::JumpAbsU32Imm,
            JumpAbsU32Reg(_) => OpCode::JumpAbsU32Reg,

            /******** [Data Instructions] ********/
            SwapU32RegU32Reg(_, _) => OpCode::SwapU32RegU32Reg,
            MovU32ImmU32Reg(_, _) => OpCode::MovU32ImmU32Reg,
            MovU32RegU32Reg(_, _) => OpCode::MovU32RegU32Reg,
            MovU32ImmMemSimple(_, _) => OpCode::MovU32ImmMemSimple,
            MovU32RegMemSimple(_, _) => OpCode::MovU32RegMemSimple,
            MovMemU32RegSimple(_, _) => OpCode::MovMemU32RegSimple,
            MovU32RegPtrU32RegSimple(_, _) => OpCode::MovU32RegPtrU32RegSimple,
            MovU32ImmMemExpr(_, _) => OpCode::MovU32ImmMemExpr,
            MovMemExprU32Reg(_, _) => OpCode::MovMemExprU32Reg,
            MovU32RegMemExpr(_, _) => OpCode::MovU32RegMemExpr,
            ByteSwapU32(_) => OpCode::ByteSwapU32,
            ZeroHighBitsByIndexU32Reg(_, _, _) => OpCode::ZeroHighBitsByIndexU32Reg,
            ZeroHighBitsByIndexU32RegU32Imm(_, _, _) => OpCode::ZeroHighBitsByIndexU32RegU32Imm,
            PushU32Imm(_) => OpCode::PushU32Imm,
            PushF32Imm(_) => OpCode::PushF32Imm,
            PushU32Reg(_) => OpCode::PushU32Reg,
            PopF32ToF32Reg(_) => OpCode::PopF32ToF32Reg,
            PopU32ToU32Reg(_) => OpCode::PopU32ToU32Reg,

            /******** [IO Instructions] ********/
            OutF32Imm(_, _) => OpCode::OutF32Imm,
            OutU32Imm(_, _) => OpCode::OutU32Imm,
            OutU32Reg(_, _) => OpCode::OutU32Reg,
            OutU8Imm(_, _) => OpCode::OutU8Imm,
            InU8Reg(_, _) => OpCode::InU8Reg,
            InU8Mem(_, _) => OpCode::InU8Mem,
            InU32Reg(_, _) => OpCode::InU32Reg,
            InU32Mem(_, _) => OpCode::InU32Mem,
            InF32Reg(_, _) => OpCode::InF32Reg,
            InF32Mem(_, _) => OpCode::InF32Mem,

            /******** [Logic Instructions] ********/
            BitTestU32Reg(_, _) => OpCode::BitTestU32Reg,
            BitTestU32Mem(_, _) => OpCode::BitTestU32Mem,
            BitTestResetU32Reg(_, _) => OpCode::BitTestResetU32Reg,
            BitTestResetU32Mem(_, _) => OpCode::BitTestResetU32Mem,
            BitTestSetU32Reg(_, _) => OpCode::BitTestSetU32Reg,
            BitTestSetU32Mem(_, _) => OpCode::BitTestSetU32Mem,
            BitScanReverseU32RegU32Reg(_, _) => OpCode::BitScanReverseU32RegU32Reg,
            BitScanReverseU32MemU32Reg(_, _) => OpCode::BitScanReverseU32MemU32Reg,
            BitScanReverseU32RegMemU32(_, _) => OpCode::BitScanReverseU32RegMemU32,
            BitScanReverseU32MemU32Mem(_, _) => OpCode::BitScanReverseU32MemU32Mem,
            BitScanForwardU32RegU32Reg(_, _) => OpCode::BitScanForwardU32RegU32Reg,
            BitScanForwardU32MemU32Reg(_, _) => OpCode::BitScanForwardU32MemU32Reg,
            BitScanForwardU32RegMemU32(_, _) => OpCode::BitScanForwardU32RegMemU32,
            BitScanForwardU32MemU32Mem(_, _) => OpCode::BitScanForwardU32MemU32Mem,

            /******** [Special Instructions] ********/
            MaskInterrupt(_) => OpCode::MaskInterrupt,
            UnmaskInterrupt(_) => OpCode::UnmaskInterrupt,
            LoadIVTAddrU32Imm(_) => OpCode::LoadIVTAddrU32Imm,
            MachineReturn => OpCode::MachineReturn,
            Halt => OpCode::Halt,

            /******** [Reserved Instructions] ********/
            Reserved1 => OpCode::Reserved1,
            Reserved2 => OpCode::Reserved2,
            Reserved3 => OpCode::Reserved3,
            Reserved4 => OpCode::Reserved4,
            Reserved5 => OpCode::Reserved5,
            Reserved6 => OpCode::Reserved6,
            Reserved7 => OpCode::Reserved7,
            Reserved8 => OpCode::Reserved8,
            Reserved9 => OpCode::Reserved9,

            /******** [Pseudo Instructions] ********/
            Label(_) => OpCode::Label,
            Unknown(_) => OpCode::Unknown,
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
