use super::instruction::Instruction;

use num_derive::FromPrimitive;

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

    /// Add u32 Immediate to u32 Register. Result is moved into Accumulator register.
    AddU32ImmU32Reg = 1,
    /// Add u32 Register to u32 Register. Result is moved into Accumulator register.
    AddU32RegU32Reg = 2,

    /// Move a u32 Immediate to u32 Register. Result is copied into the register.
    MovU32ImmU32Reg = 3,
    /// Move a u32 Register to u32 Register. Result is copied into the register.
    MovU32RegU32Reg = 4,
    /// Move a u32 Immediate to Memory. Result is copied into memory.
    MovU32ImmMem = 5,
    /// Move a u32 Register to Memory. Result is copied into memory.
    MovU32RegMem = 6,
    /// Move a u32 value from Memory u32 Register. Result is copied into the register.
    MovMemU32Reg = 7,
    /// Move the value from the memory address specified by a Register. Result is copied into the other register.
    MovU32RegPtrU32Reg = 8,
    /// Swap the values of the two registers.
    SwapU32RegU32Reg = 9,

    /// Return - return from a subroutine.
    Ret = 32765,
    /// Machine return - downgrade the privilege level of the processor.
    Mret = 32766,

    /// Halt - halt the execution of the virtual machine.
    Hlt = 32767,
    // Values higher than 32767 (0x7FFF) are extended instructions and require four bytes to represent.
}

impl OpCode {
    pub fn is_extended(&self) -> bool {
        let val = *self as u32;
        val > 32767
    }
}

impl From<Instruction> for OpCode {
    fn from(val: Instruction) -> Self {
        match val {
            Instruction::Nop => OpCode::Nop,
            Instruction::AddU32ImmU32Reg(_, _) => OpCode::AddU32ImmU32Reg,
            Instruction::AddU32RegU32Reg(_, _) => OpCode::AddU32RegU32Reg,
            Instruction::MovU32ImmU32Reg(_, _) => OpCode::MovU32ImmU32Reg,
            Instruction::MovU32RegU32Reg(_, _) => OpCode::MovU32RegU32Reg,
            Instruction::MovU32ImmMem(_, _) => OpCode::MovU32ImmMem,
            Instruction::MovU32RegMem(_, _) => OpCode::MovU32RegMem,
            Instruction::MovMemU32Reg(_, _) => OpCode::MovMemU32Reg,
            Instruction::MovU32RegPtrU32Reg(_, _) => OpCode::MovU32RegPtrU32Reg,
            Instruction::SwapU32RegU32Reg(_, _) => OpCode::SwapU32RegU32Reg,
            Instruction::Ret => OpCode::Ret,
            Instruction::Mret => OpCode::Mret,
            Instruction::Hlt => OpCode::Hlt,
        }
    }
}
