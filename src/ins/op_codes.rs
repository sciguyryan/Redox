#[repr(u16)]
pub enum OpCode {
    /// Subroutine - a pseudo-opcode used to identify a subroutine position.
    //Subroutine = -2,
    /// Label - a pseudo-opcode used to identify a labels position.
    //Label = -1,

    /// No Operation - a non-operation instruction.
    Nop = 0,

    /// Add u32 Literal to u32 Register. Result is moved into Accumulator register.
    AddU32LitU32Reg,

    /// Add u32 Register to u32 Register. Result is moved into Accumulator register.
    AddU32RegU32Reg,

    /// Move a u32 Literal to u32 Register. Result is moved into the register.
    MovU32LitU32Reg,

    /// Move a u32 Register to u32 Register. Result is moved into the register.
    MovU32RegU32Reg,

    /// Return - return from a subroutine.
    Ret = 65533,

    /// Machine return - downgrade the privilege level of the processor.
    Mret = 65534,

    /// Halt - halt the execution of the virtual machine.
    Hlt = 65535,
}
