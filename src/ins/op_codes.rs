// If bit 7 of an opcode is set, then we are working with an extended opcode.
// This means that we need to read two bytes instead of one byte.
// This is a size optimization, the most common opcodes will only take 1 byte.
#[repr(u16)]
pub enum OpCode {
    /// Subroutine - a pseudo-opcode used to identify a subroutine position.
    //Subroutine = -2,
    /// Label - a pseudo-opcode used to identify a labels position.
    //Label = -1,

    /// No Operation - a non-operation instruction.
    Nop = 0,

    /// Add u32 Literal to Register.
    AddU32LitReg = 1,

    /// Halt - halt the execution of the virtual machine.
    Hlt = 65535,
}
