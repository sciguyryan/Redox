use std::fmt::{self, Display, Formatter};

#[cfg(test)]
use strum_macros::EnumIter;

use crate::{ins::expression::Expression, reg::registers::RegisterId};

use super::op_codes::OpCode;

/// The size of the instruction, in bytes.
const INSTRUCTION_SIZE: usize = 4;
/// The size of a f32 argument, in bytes.
const ARG_F32_IMM_SIZE: usize = 4;
/// The size of a u32 argument, in bytes.
const ARG_U32_IMM_SIZE: usize = 4;
/// The size of a u8 argument, in bytes.
const ARG_U8_IMM_SIZE: usize = 1;
/// The size of a memory address argument, in bytes.
const ARG_MEM_ADDR_SIZE: usize = 4;
/// The size of a register ID argument, in bytes.
const ARG_REG_ID_SIZE: usize = 1;

#[derive(Clone, Debug)]
#[cfg_attr(test, derive(EnumIter))]
pub enum Instruction {
    /// No operation.
    Nop,

    /******** [Arithmetic Instructions] ********/
    /// Add a u32 immediate to u32 register. The result is stored in the accumulator register.
    AddU32ImmU32Reg(u32, RegisterId),
    /// Add a u32 register to u32 register. The result is stored in the accumulator register.
    AddU32RegU32Reg(RegisterId, RegisterId),
    /// Subtract a u32 immediate from a u32 register. The result is stored in the accumulator register.
    SubU32ImmU32Reg(u32, RegisterId),
    /// Subtract a u32 register (A) from a u32 register (B). The result is stored in the accumulator register.
    SubU32RegU32Reg(RegisterId, RegisterId),
    /// Unsigned multiplication of the register ER1 by a u32 immediate. The result is stored in the register ER1.
    MulU32Imm(u32),
    /// Unsigned multiplication of the register ER1 by a u32 register. The result is stored in the register ER1.
    MulU32Reg(RegisterId),
    /// Unsigned division of the register ER1 by a u32 immediate. The quotient is stored in the register ER1 and the modulo is stored in ER4.
    DivU32Imm(u32),
    /// Unsigned division of the register ER1 by a u32 register. The quotient is stored in the register ER1 and the modulo is stored in ER4.
    DivU32Reg(RegisterId),
    /// Increment a u32 register.
    IncU32Reg(RegisterId),
    /// Decrement a u32 register.
    DecU32Reg(RegisterId),
    /// Perform a logical AND operation on a u32 immediate and a u32 register. The resulting value is stored in the register.
    AndU32ImmU32Reg(u32, RegisterId),

    /******** [Bit Operation Instructions] ********/
    /// Left-shift a u32 register by a u8 immediate. The result remains in the origin register.
    LeftShiftU8ImmU32Reg(u8, RegisterId),
    /// Left-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    LeftShiftU32RegU32Reg(RegisterId, RegisterId),
    /// Arithmetic left-shift a u32 register by a u8 immediate. The result remains in the origin register.
    ArithLeftShiftU8ImmU32Reg(u8, RegisterId),
    /// Arithmetic left-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    ArithLeftShiftU32RegU32Reg(RegisterId, RegisterId),
    /// Right-shift a u32 register by a u8 immediate. The result remains in the origin register.
    RightShiftU8ImmU32Reg(u8, RegisterId),
    /// Right-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    RightShiftU32RegU32Reg(RegisterId, RegisterId),
    /// Arithmetic right-shift a u32 register by a u8 immediate. The result remains in the origin register.
    ArithRightShiftU8ImmU32Reg(u8, RegisterId),
    /// Arithmetic right-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    ArithRightShiftU32RegU32Reg(RegisterId, RegisterId),

    /******** [Branching Instructions] ********/
    /// Call a subroutine at a specified absolute u32 immediate address.
    CallAbsU32Imm(u32),
    /// Call a subroutine at an absolute address as specified by a u32 register.
    CallAbsU32Reg(RegisterId),
    /// Call a subroutine at an address as given by an offset from the address pointed to a u32 register.
    CallRelU32RegU32Offset(u32, RegisterId),
    /// Call a subroutine at an address as given the ECS register and offset by a u32 immediate.
    CallRelCSU32Offset(u32),
    /// Call a subroutine at an address as given the ECS register and offset by the value of a u32 register.
    CallRelCSU32RegOffset(RegisterId),
    /// Return from a subroutine that had zero or more u32 arguments supplied.
    RetArgsU32,
    /// Trigger a specific type of interrupt handler.
    Int(u8),
    /// Returns from an interrupt handler.
    IntRet,
    /// Unconditional jump to a specified address.
    JumpAbsU32Imm(u32),
    /// Unconditional jump to an address as specified by a u32 register.
    JumpAbsU32Reg(RegisterId),

    /******** [Data Instructions] ********/
    /// Swap the values of the two registers.
    SwapU32RegU32Reg(RegisterId, RegisterId),
    /// Move a u32 immediate to u32 register (A). The result is copied into register A.
    MovU32ImmU32Reg(u32, RegisterId),
    /// Move a u32 register (B) to u32 register (A). The result is copied into register A.
    MovU32RegU32Reg(RegisterId, RegisterId),
    /// Move a u32 immediate to memory. The result is copied into the specified memory address.
    MovU32ImmMemSimple(u32, u32),
    /// Move a u32 register to memory. The result is copied into the specified memory address.
    MovU32RegMemSimple(RegisterId, u32),
    /// Move a u32 value from memory to a u32 register. The result is copied into the specified register.
    MovMemU32RegSimple(u32, RegisterId),
    /// Move a u32 value from the memory address specified by a register. The result is copied into the specified register.
    MovU32RegPtrU32RegSimple(RegisterId, RegisterId),
    /// Move a u32 value to the memory address as given by an expression. The result is copied into the specified memory address.
    MovU32ImmMemExpr(u32, u32),
    /// Move a u32 value from the address as given by an expression to a u32 register. The result is copied into the specified register.
    MovMemExprU32Reg(u32, RegisterId),
    /// Move the value of a u32 register to the address given by an expression. The result is copied into the specified memory address.
    MovU32RegMemExpr(RegisterId, u32),
    /// Reverse the order of bytes in a specified u32 register.
    ByteSwapU32(RegisterId),
    /// Zero the high bits of the source value starting from a specified index.
    ZeroHighBitsByIndexU32Reg(RegisterId, RegisterId, RegisterId),
    /// Zero the high bits of the source value starting from a specified index.
    ZeroHighBitsByIndexU32RegU32Imm(u32, RegisterId, RegisterId),
    /// Push a f32 immediate value onto the stack.
    PushF32Imm(f32),
    /// Push a u32 immediate value onto the stack.
    PushU32Imm(u32),
    /// Push the value of a u32 register onto the stack.
    PushU32Reg(RegisterId),
    /// Pop a f32 value from the stack to a f32 register.
    PopF32ToF32Reg(RegisterId),
    /// Pop a u32 value from the stack to a u32 register.
    PopU32ToU32Reg(RegisterId),

    /******** [IO Instructions] ********/
    /// Output a f32 immediate value to a specific port.
    OutF32Imm(f32, u8),
    /// Output a u32 immediate value to a specific port.
    OutU32Imm(u32, u8),
    /// Output a u32 register value value to a specific port.
    OutU32Reg(RegisterId, u8),
    /// Output a u8 immediate value to a specific port.
    OutU8Imm(u8, u8),
    /// Input a u8 value from a specific port into a specified register.
    InU8Reg(u8, RegisterId),
    /// Input a u8 value from a specific port into a specified memory address.
    InU8Mem(u8, u32),
    /// Input a u32 value from a specific port into a specified register.
    InU32Reg(u8, RegisterId),
    /// Input a u32 value from a specific port into a specified memory address.
    InU32Mem(u8, u32),
    /// Input a f32 value from a specific port into a specified register.
    InF32Reg(u8, RegisterId),
    /// Input a f32 value from a specific port into a specified memory address.
    InF32Mem(u8, u32),

    /******** [Logic Instructions] ********/
    /// Test the state of a bit from a u32 register. The CF flag will be set to the state of the bit.
    BitTestU32Reg(u8, RegisterId),
    /// Test the state of a bit from a u32 value (starting at the specified memory address). The CF flag will be set to the state of the bit.
    BitTestU32Mem(u8, u32),
    /// Test the state of a bit from a u32 register and clear the bit. The CF flag will be set to the original state of the bit.
    BitTestResetU32Reg(u8, RegisterId),
    /// Test the state of a bit of a u32 value (starting at the specified memory address) and clear the bit. The CF flag will be set to the original state of the bit.
    BitTestResetU32Mem(u8, u32),
    /// Test the state of a bit of a u32 register and set the bit. The CF flag will be set to the original state of the bit.
    BitTestSetU32Reg(u8, RegisterId),
    /// Test the state of a bit of a u32 value (starting at the specified memory address) and set the bit. The CF flag will be set to the original state of the bit.
    BitTestSetU32Mem(u8, u32),
    /// Search for the most significant set bit of a u32 register (A) and store the index of the bit in a u32 register (B).
    BitScanReverseU32RegU32Reg(RegisterId, RegisterId),
    /// Search for the most significant set bit of a u32 value (starting at the specified memory address) and store the index of the bit in a u32 register.
    BitScanReverseU32MemU32Reg(u32, RegisterId),
    /// Search for the most significant set bit of a u32 register and store the index of the bit as a u32 value starting at a specified memory address.
    BitScanReverseU32RegMemU32(RegisterId, u32),
    /// Search for the most significant set bit of a u32 value (starting at the specified memory address) and store the index of the bit as a u32 value starting at a specified memory address.
    BitScanReverseU32MemU32Mem(u32, u32),
    /// Search for the least significant set bit of a u32 register (A) and store the index of the bit in a u32 register (B).
    BitScanForwardU32RegU32Reg(RegisterId, RegisterId),
    /// Search for the least significant set bit of a u32 value (starting at the specified memory address) and store the index of the bit in a u32 register.
    BitScanForwardU32MemU32Reg(u32, RegisterId),
    /// Search for the least significant set bit of a u32 register and store the index of the bit as a u32 value starting at a specified memory address.
    BitScanForwardU32RegMemU32(RegisterId, u32),
    /// Search for the least significant set bit of a u32 value (starting at the specified memory address) and store the index of the bit as a u32 value starting at a specified memory address.
    BitScanForwardU32MemU32Mem(u32, u32),

    /******** [Special Instructions] ********/
    /// Mask a specific interrupt.
    MaskInterrupt(u8),
    /// Unmask a specific interrupt.
    UnmaskInterrupt(u8),
    /// Set the location of the interrupt vector table.
    ///
    /// # Note
    ///
    /// This can only be done in machine mode.
    LoadIVTAddrU32Imm(u32),
    /// Machine-mode return - downgrade the privilege level of the processor.
    MachineReturn,
    /// Halt the execution of the processor.
    Halt,

    /******** [Reserved Instructions] ********/
    Reserved1,
    Reserved2,
    Reserved3,
    Reserved4,
    Reserved5,
    Reserved6,
    Reserved7,
    Reserved8,
    Reserved9,

    /******** [Pseudo Instructions] ********/
    /// A placeholder opcode for labels. These do not directly compile to anything and are intended on being a hint when compiling. This should never be constructed directly.
    Label(String),
    /// A placeholder for instances where the opcode isn't recognized. This should never be constructed directly.
    Unknown(u32),
}

// NOTE - This has to be manually implemented due to the issues with comparing f32 values.
impl PartialEq for Instruction {
    fn eq(&self, other: &Self) -> bool {
        core::mem::discriminant(self) == core::mem::discriminant(other)
    }
}

impl Display for Instruction {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        use Instruction as I;

        // Syntax notes:
        // A set of square brackets are used to indicate an expression.
        // An "&" prefix before an argument type indicates that it is being used as a pointer to a memory address.
        let asm_format = match self {
            I::Nop => String::from("nop"),

            /******** [Arithmetic Instructions] ********/
            I::AddU32ImmU32Reg(imm, reg) => {
                format!("add 0x{imm:08x}, {reg}")
            }
            I::AddU32RegU32Reg(in_reg, out_reg) => {
                format!("add {in_reg}, {out_reg}")
            }
            I::SubU32ImmU32Reg(imm, reg) => {
                format!("sub 0x{imm:08x}, {reg}")
            }
            I::SubU32RegU32Reg(reg_1, reg_2) => {
                format!("sub {reg_1}, {reg_2}")
            }
            I::MulU32Imm(imm) => {
                format!("mul 0x{imm:08x}")
            }
            I::MulU32Reg(reg) => {
                format!("mul {reg}")
            }
            I::DivU32Imm(imm) => {
                format!("div 0x{imm:08x}")
            }
            I::DivU32Reg(reg) => {
                format!("div {reg}")
            }
            I::IncU32Reg(reg) => {
                format!("inc {reg}")
            }
            I::DecU32Reg(reg) => {
                format!("dec {reg}")
            }
            I::AndU32ImmU32Reg(imm, reg) => {
                format!("and 0x{imm:08x}, {reg}")
            }

            /******** [Bit Operation Instructions] ********/
            I::LeftShiftU8ImmU32Reg(imm, reg) => {
                format!("shl 0x{imm:02x}, {reg}")
            }
            I::LeftShiftU32RegU32Reg(shift_reg, reg) => {
                format!("shl {shift_reg}, {reg}")
            }
            I::ArithLeftShiftU8ImmU32Reg(imm, reg) => {
                format!("sal 0x{imm:02x}, {reg}")
            }
            I::ArithLeftShiftU32RegU32Reg(shift_reg, reg) => {
                format!("sal {shift_reg}, {reg}")
            }
            I::RightShiftU8ImmU32Reg(imm, reg) => {
                format!("shr 0x{imm:02x}, {reg}")
            }
            I::RightShiftU32RegU32Reg(shift_reg, reg) => {
                format!("shr {shift_reg}, {reg}")
            }
            I::ArithRightShiftU8ImmU32Reg(imm, reg) => {
                format!("sar 0x{imm:02x}, {reg}")
            }
            I::ArithRightShiftU32RegU32Reg(shift_reg, reg) => {
                format!("sar {shift_reg}, {reg}")
            }

            /******** [Branching Instructions] ********/
            I::CallAbsU32Imm(addr) => {
                // TODO - apply labels to these jumps - either dynamically generated or via binary file metadata.
                format!("call &0x{addr:08x}")
            }
            I::CallAbsU32Reg(reg) => {
                format!("call &{reg}")
            }
            I::CallRelU32RegU32Offset(offset, reg) => {
                format!("callr 0x{offset:08x}, &{reg}")
            }
            I::CallRelCSU32Offset(offset) => {
                format!("callr 0x{offset:08x}")
            }
            I::CallRelCSU32RegOffset(offset_reg) => {
                format!("callr {offset_reg}")
            }
            I::RetArgsU32 => String::from("iret"),
            I::Int(int_code) => {
                format!("int 0x{int_code:02x}")
            }
            I::IntRet => String::from("intret"),
            I::JumpAbsU32Imm(addr) => {
                // TODO - apply labels to these jumps - either dynamically generated or via binary file metadata.
                format!("jmp &0x{addr:08x}")
            }
            I::JumpAbsU32Reg(reg) => {
                format!("jmp &{reg}")
            }

            /******** [Data Instructions] ********/
            I::SwapU32RegU32Reg(reg_1, reg_2) => {
                format!("swap {reg_1}, {reg_2}")
            }
            I::MovU32ImmU32Reg(imm, reg) => {
                format!("mov 0x{imm:08x}, {reg}")
            }
            I::MovU32RegU32Reg(in_reg, out_reg) => {
                format!("mov {in_reg}, {out_reg}")
            }
            I::MovU32ImmMemSimple(imm, addr) => {
                format!("mov 0x{imm:08x}, &0x{addr:08x}")
            }
            I::MovU32RegMemSimple(reg, addr) => {
                format!("mov {reg}, &0x{addr:08x}")
            }
            I::MovMemU32RegSimple(addr, reg) => {
                format!("mov &0x{addr:08x}, {reg}")
            }
            I::MovU32RegPtrU32RegSimple(in_reg, out_reg) => {
                format!("mov &{in_reg}, {out_reg}")
            }
            I::MovU32ImmMemExpr(imm, expr) => {
                let mut decoder = Expression::new();
                decoder.unpack(*expr);

                format!("mov 0x{imm:08x}, &[{decoder}]")
            }
            I::MovMemExprU32Reg(expr, reg) => {
                let mut decoder = Expression::new();
                decoder.unpack(*expr);

                format!("mov &[{decoder}], {reg}")
            }
            I::MovU32RegMemExpr(reg, expr) => {
                let mut decoder = Expression::new();
                decoder.unpack(*expr);

                format!("mov {reg}, &[{decoder}]")
            }
            I::ByteSwapU32(reg) => {
                format!("bswap {reg}")
            }
            I::ZeroHighBitsByIndexU32Reg(index_reg, in_reg, out_reg) => {
                format!("zhbi {index_reg}, {in_reg}, {out_reg}")
            }
            I::ZeroHighBitsByIndexU32RegU32Imm(index, in_reg, out_reg) => {
                format!("zhbi 0x{index:08x}, {in_reg}, {out_reg}")
            }
            I::PushF32Imm(value) => {
                format!("push {value}")
            }
            I::PushU32Imm(value) => {
                format!("push 0x{value:08x}")
            }
            I::PushU32Reg(reg) => {
                format!("push {reg}")
            }
            I::PopF32ToF32Reg(reg) => {
                format!("pop {reg}")
            }
            I::PopU32ToU32Reg(reg) => {
                format!("pop {reg}")
            }

            /******** [IO Instructions] ********/
            I::OutF32Imm(value, port) => {
                format!("outf {value:.1}, 0x{port:02x}")
            }
            I::OutU32Imm(value, port) => {
                format!("outwd 0x{value:08x}, 0x{port:02x}")
            }
            I::OutU32Reg(reg, port) => {
                format!("outrw {reg}, 0x{port:02x}")
            }
            I::OutU8Imm(value, port) => {
                format!("outb 0x{value:02x}, 0x{port:02x}")
            }
            I::InU8Reg(port, reg) => {
                format!("inrb 0x{port:02x}, {reg}")
            }
            I::InU32Reg(port, reg) => {
                format!("inrw 0x{port:02x}, {reg}")
            }
            I::InF32Reg(port, reg) => {
                format!("inrf 0x{port:02x}, {reg}")
            }
            I::InU8Mem(port, addr) => {
                format!("inmb 0x{port:02x}, &0x{addr:08x}")
            }
            I::InU32Mem(port, addr) => {
                format!("inmw 0x{port:02x}, &0x{addr:08x}")
            }
            I::InF32Mem(port, addr) => {
                format!("inmf 0x{port:02x}, &0x{addr:08x}")
            }

            /******** [Logic Instructions] ********/
            I::BitTestU32Reg(bit, reg) => {
                format!("bt 0x{bit:02x}, {reg}")
            }
            I::BitTestU32Mem(bit, addr) => {
                format!("bt 0x{bit:02x}, &0x{addr:08x}")
            }
            I::BitTestResetU32Reg(bit, reg) => {
                format!("btr 0x{bit:02x}, {reg}")
            }
            I::BitTestResetU32Mem(bit, addr) => {
                format!("btr 0x{bit:02x}, &0x{addr:08x}")
            }
            I::BitTestSetU32Reg(bit, reg) => {
                format!("bts 0x{bit:02x}, {reg}")
            }
            I::BitTestSetU32Mem(bit, addr) => {
                format!("bts 0x{bit:02x}, &0x{addr:08x}")
            }
            I::BitScanReverseU32RegU32Reg(in_reg, out_reg) => {
                format!("bsr {in_reg}, {out_reg}")
            }
            I::BitScanReverseU32MemU32Reg(addr, reg) => {
                format!("bsr &0x{addr:08x}, {reg}")
            }
            I::BitScanReverseU32RegMemU32(reg, out_addr) => {
                format!("bsr {reg}, &0x{out_addr:08x}")
            }
            I::BitScanReverseU32MemU32Mem(in_addr, out_addr) => {
                format!("bsr &0x{in_addr:08x}, &0x{out_addr:08x}")
            }
            I::BitScanForwardU32RegU32Reg(in_reg, out_reg) => {
                format!("bsf {in_reg}, {out_reg}")
            }
            I::BitScanForwardU32MemU32Reg(addr, reg) => {
                format!("bsf &0x{addr:08x}, {reg}")
            }
            I::BitScanForwardU32RegMemU32(reg, out_addr) => {
                format!("bsf {reg}, &0x{out_addr:08x}")
            }
            I::BitScanForwardU32MemU32Mem(in_addr, out_addr) => {
                format!("bsf &0x{in_addr:08x}, &0x{out_addr:08x}")
            }

            /******** [Special Instructions] ********/
            I::MaskInterrupt(int_code) => {
                format!("mint 0x{int_code:02x}")
            }
            I::UnmaskInterrupt(int_code) => {
                format!("umint 0x{int_code:02x}")
            }
            I::LoadIVTAddrU32Imm(addr) => {
                format!("livt &0x{addr:08x}")
            }
            I::MachineReturn => String::from("mret"),
            I::Halt => String::from("hlt"),

            /******** [Reserved Instructions] ********/
            I::Reserved1
            | I::Reserved2
            | I::Reserved3
            | I::Reserved4
            | I::Reserved5
            | I::Reserved6
            | I::Reserved7
            | I::Reserved8
            | I::Reserved9 => unreachable!("attempted to use a reserved instruction"),

            /******** [Pseudo Instructions] ********/
            I::Label(_label) => {
                todo!();
            }
            I::Unknown(id) => format!("UNKNOWN! ID = 0x{id:08x}"),
        };
        write!(f, "{asm_format}")
    }
}

impl Instruction {
    /// Get the expected argument size of an [`Instruction`], in bytes.
    ///
    /// # Returns
    ///
    /// A usize giving the expected argument size, in bytes.
    #[inline(always)]
    pub fn get_instruction_arg_size(&self) -> usize {
        Instruction::get_instruction_arg_size_from_op(self.into())
    }

    /// Get the expected argument size of an [`OpCode`], in bytes.
    ///
    /// # Arguments
    ///
    /// * `opcode` - The [`OpCode`] which should be used to calculate the argument size
    ///
    /// # Returns
    ///
    /// A usize giving the expected argument size, in bytes.
    #[inline(always)]
    pub fn get_instruction_arg_size_from_op(opcode: OpCode) -> usize {
        use OpCode as O;

        match opcode {
            O::Nop => 0,

            /******** [Arithmetic Instructions] ********/
            O::AddU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            O::AddU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            O::SubU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            O::SubU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            O::MulU32Imm => ARG_U32_IMM_SIZE,
            O::MulU32Reg => ARG_REG_ID_SIZE,
            O::DivU32Imm => ARG_U32_IMM_SIZE,
            O::DivU32Reg => ARG_REG_ID_SIZE,
            O::IncU32Reg => ARG_REG_ID_SIZE,
            O::DecU32Reg => ARG_REG_ID_SIZE,
            O::AndU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,

            /******** [Bit Operation Instructions] ********/
            O::LeftShiftU8ImmU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            O::LeftShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            O::ArithLeftShiftU8ImmU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            O::ArithLeftShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            O::RightShiftU8ImmU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            O::RightShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            O::ArithRightShiftU8ImmU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            O::ArithRightShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,

            /******** [Branching Instructions] ********/
            O::CallAbsU32Imm => ARG_MEM_ADDR_SIZE,
            O::CallAbsU32Reg => ARG_REG_ID_SIZE,
            O::CallRelU32RegU32Offset => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            O::CallRelCSU32Offset => ARG_U32_IMM_SIZE,
            O::CallRelCSU32RegOffset => ARG_REG_ID_SIZE,
            O::RetArgsU32 => 0,
            O::Int => ARG_U8_IMM_SIZE,
            O::IntRet => 0,
            O::JumpAbsU32Imm => ARG_MEM_ADDR_SIZE,
            O::JumpAbsU32Reg => ARG_REG_ID_SIZE,

            /******** [Data Instructions] ********/
            O::SwapU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            O::MovU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            O::MovU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            O::MovU32ImmMemSimple => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            O::MovU32RegMemSimple => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            O::MovMemU32RegSimple => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            O::MovU32RegPtrU32RegSimple => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            O::MovU32ImmMemExpr => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            O::MovMemExprU32Reg => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            O::MovU32RegMemExpr => ARG_REG_ID_SIZE + ARG_U32_IMM_SIZE,
            O::ByteSwapU32 => ARG_REG_ID_SIZE,
            O::ZeroHighBitsByIndexU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            O::ZeroHighBitsByIndexU32RegU32Imm => {
                ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE + ARG_REG_ID_SIZE
            }
            O::PushU32Imm => ARG_U32_IMM_SIZE,
            O::PushF32Imm => ARG_F32_IMM_SIZE,
            O::PushU32Reg => ARG_REG_ID_SIZE,
            O::PopF32ToF32Reg => ARG_REG_ID_SIZE,
            O::PopU32ToU32Reg => ARG_REG_ID_SIZE,

            /******** [IO Instructions] ********/
            O::OutF32Imm => ARG_F32_IMM_SIZE + ARG_U8_IMM_SIZE,
            O::OutU32Imm => ARG_U32_IMM_SIZE + ARG_U8_IMM_SIZE,
            O::OutU32Reg => ARG_REG_ID_SIZE + ARG_U8_IMM_SIZE,
            O::OutU8Imm => ARG_U8_IMM_SIZE + ARG_U8_IMM_SIZE,
            O::InU8Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            O::InU8Mem => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            O::InU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            O::InU32Mem => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            O::InF32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            O::InF32Mem => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,

            /******** [Logic Instructions] ********/
            O::BitTestU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            O::BitTestU32Mem => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            O::BitTestResetU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            O::BitTestResetU32Mem => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            O::BitTestSetU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            O::BitTestSetU32Mem => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            O::BitScanReverseU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            O::BitScanReverseU32MemU32Reg => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            O::BitScanReverseU32RegMemU32 => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            O::BitScanReverseU32MemU32Mem => ARG_MEM_ADDR_SIZE + ARG_MEM_ADDR_SIZE,
            O::BitScanForwardU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            O::BitScanForwardU32MemU32Reg => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            O::BitScanForwardU32RegMemU32 => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            O::BitScanForwardU32MemU32Mem => ARG_MEM_ADDR_SIZE + ARG_MEM_ADDR_SIZE,

            /******** [Special Instructions] ********/
            O::MaskInterrupt => ARG_U8_IMM_SIZE,
            O::UnmaskInterrupt => ARG_U8_IMM_SIZE,
            O::LoadIVTAddrU32Imm => ARG_MEM_ADDR_SIZE,
            O::MachineReturn | O::Halt => 0,

            /******** [Reserved Instructions] ********/
            O::Reserved1
            | O::Reserved2
            | O::Reserved3
            | O::Reserved4
            | O::Reserved5
            | O::Reserved6
            | O::Reserved7
            | O::Reserved8
            | O::Reserved9 => 0,

            /******** [Pseudo Instructions] ********/
            O::Label | O::Unknown => 0,
        }
    }

    /// Get the total size of an [`Instruction`], in bytes.
    ///
    /// # Returns
    ///
    /// A usize giving the total size of the [`Instruction`], in bytes.
    #[inline(always)]
    pub fn get_total_instruction_size(&self) -> usize {
        let instruction_size = match self {
            Instruction::Reserved1
            | Instruction::Reserved2
            | Instruction::Reserved3
            | Instruction::Reserved4
            | Instruction::Reserved5
            | Instruction::Reserved6
            | Instruction::Reserved7
            | Instruction::Reserved8
            | Instruction::Reserved9
            | Instruction::Label(_) => 0,
            _ => INSTRUCTION_SIZE,
        };
        instruction_size + self.get_instruction_arg_size()
    }
}
