#[repr(u32)]
/// The opcode for an instruction.
/// TODO - If the highest bit is set then the instruction is extended and the size of the instruction
/// TODO - is 32 bits, otherwise it can be shortened to just 16 bits.
pub enum OpCode {
    /// Subroutine - a pseudo-opcode used to identify a subroutine position.
    //Subroutine = -2,
    /// Label - a pseudo-opcode used to identify a labels position.
    //Label = -1,

    /// No Operation - a non-operation instruction.
    Nop = 0,

    /// Add u32 Immediate to u32 Register. Result is moved into Accumulator register.
    AddU32ImmU32Reg = 0b0000_0000_0000_0001,
    /// Add u32 Register to u32 Register. Result is moved into Accumulator register.
    AddU32RegU32Reg = 0b0000_0000_0000_0010,

    /// Move a u32 Immediate to u32 Register. Result is copied into the register.
    MovU32ImmU32Reg = 0b0000_0000_0000_0011,
    /// Move a u32 Register to u32 Register. Result is copied into the register.
    MovU32RegU32Reg = 0b0000_0000_0000_0100,
    /// Move a u32 Immediate to Memory. Result is copied into memory.
    MovU32ImmMem = 0b0000_0000_0000_0101,
    /// Move a u32 Register to Memory. Result is copied into memory.
    MovU32RegMem = 0b0000_0000_0000_0110,
    /// Move a u32 value from Memory u32 Register. Result is copied into memory.
    MovMemU32Reg = 0b0000_0000_0000_0111,

    /// Return - return from a subroutine.
    Ret = 0b0111_1111_1111_1101,
    /// Machine return - downgrade the privilege level of the processor.
    Mret = 0b0111_1111_1111_1110,

    /// Halt - halt the execution of the virtual machine.
    Hlt = 0b0111_1111_1111_1111,
}
