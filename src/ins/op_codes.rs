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
