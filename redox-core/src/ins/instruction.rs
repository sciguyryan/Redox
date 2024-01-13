use std::fmt::{self, Display, Formatter};

#[cfg(test)]
use strum_macros::EnumIter;

use crate::{ins::move_expressions::MoveExpressionHandler, reg::registers::RegisterId};

use super::op_codes::OpCode;

/// The size of the instruction, in bytes.
const INSTRUCTION_SIZE: usize = 4;
/// The size of a u32 argument, in bytes.
const ARG_U32_IMM_SIZE: usize = 4;
/// The size of a u8 argument, in bytes.
const ARG_U8_IMM_SIZE: usize = 1;
/// The size of a memory address argument, in bytes.
const ARG_MEM_ADDR_SIZE: usize = 4;
/// The size of a register ID argument, in bytes.
const ARG_REG_ID_SIZE: usize = 1;

#[derive(Clone, Debug, Eq, PartialEq)]
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
    /// Subtract a u32 register from a u32 immediate. The result is stored in the accumulator register.
    SubU32RegU32Imm(RegisterId, u32),
    /// Subtract a u32 register (A) from a u32 register (B). The result is stored in the accumulator register.
    SubU32RegU32Reg(RegisterId, RegisterId),
    /// Multiply a u32 register by a u32 immediate. The result is stored in the accumulator register.
    MulU32ImmU32Reg(u32, RegisterId),
    /// Multiply a u32 register by a u32 register. The result is stored in the accumulator register.
    MulU32RegU32Reg(RegisterId, RegisterId),
    /// Divide a u32 register by a u32 immediate. The result is stored in the accumulator register.
    DivU32ImmU32Reg(u32, RegisterId),
    /// Divide a u32 immediate by a u32 register. The result is stored in the accumulator register.
    DivU32RegU32Imm(RegisterId, u32),
    /// Divide a u32 register (B) by a u32 register (A). The result is stored in the accumulator register.
    DivU32RegU32Reg(RegisterId, RegisterId),
    /// Calculate the modulo of a u32 register by a u32 immediate. The result is stored in the accumulator register.
    ModU32ImmU32Reg(u32, RegisterId),
    /// Calculate the modulo of a u32 immediate by a u32 register. The result is stored in the accumulator register.
    ModU32RegU32Imm(RegisterId, u32),
    /// Calculate the modulo of a u32 register (B) by a u32 register (A). The result is stored in the accumulator register.
    ModU32RegU32Reg(RegisterId, RegisterId),
    /// Increment a u32 register.
    IncU32Reg(RegisterId),
    /// Decrement a u32 register.
    DecU32Reg(RegisterId),
    /// Perform a logical AND operation on a u32 immediate and a u32 register.
    AndU32ImmU32Reg(u32, RegisterId),

    /******** [Bit Operation Instructions] ********/
    /// Left-shift a u32 register by a u32 immediate. The result remains in the origin register.
    LeftShiftU32ImmU32Reg(u32, RegisterId),
    /// Left-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    LeftShiftU32RegU32Reg(RegisterId, RegisterId),
    /// Arithmetic left-shift a u32 register by a u32 immediate. The result remains in the origin register.
    ArithLeftShiftU32ImmU32Reg(u32, RegisterId),
    /// Arithmetic left-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    ArithLeftShiftU32RegU32Reg(RegisterId, RegisterId),
    /// Right-shift a u32 register by a u32 immediate. The result remains in the origin register.
    RightShiftU32ImmU32Reg(u32, RegisterId),
    /// Right-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    RightShiftU32RegU32Reg(RegisterId, RegisterId),
    /// Arithmetic right-shift a u32 register by a u32 immediate. The result remains in the origin register.
    ArithRightShiftU32ImmU32Reg(u32, RegisterId),
    /// Arithmetic right-shift a u32 register (B) by a u32 register (A). The result remains in register A.
    ArithRightShiftU32RegU32Reg(RegisterId, RegisterId),

    /******** [Branching Instructions] ********/
    /// Call a subroutine by a provided u32 immediate address.
    CallU32Imm(u32),
    /// Call a subroutine by the address as specified by a u32 register.
    CallU32Reg(RegisterId),
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
    /// Move the value from the memory address specified by a register. The result is copied into the specified register.
    MovU32RegPtrU32RegSimple(RegisterId, RegisterId),
    /// Move a u32 immediate to memory. The result is copied into the specified memory address.
    MovU32ImmMemExpr(u32, u32),
    /// Move the address as given by an expression. The result is copied into the specified register.
    MovMemExprU32Reg(u32, RegisterId),
    /// Move the value of a register to the address given by an expression. The result is copied into the specified memory address.
    MovU32RegMemExpr(RegisterId, u32),
    /// Reverse the order of bytes in a specified register.
    ByteSwapU32(RegisterId),
    /// Zero the high bits of the source value starting from a specified index.
    ZeroHighBitsByIndexU32Reg(RegisterId, RegisterId, RegisterId),
    /// Zero the high bits of the source value starting from a specified index.
    ZeroHighBitsByIndexU32RegU32Imm(u32, RegisterId, RegisterId),
    /// Push a u32 immediate value onto the stack.
    PushU32Imm(u32),
    /// Push the value of a u32 register onto the stack.
    PushU32Reg(RegisterId),
    /// Pop a u32 value from the stack to a u32 register.
    PopU32ImmU32Reg(RegisterId),
    /// Output a u32 value to a specific port.
    OutU32Imm(u32, u8),

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

impl Display for Instruction {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        use Instruction::*;

        // NOTE: square brackets are used to indicate we are working with an address.
        let asm_format = match self {
            Nop => String::from("nop"),

            /******** [Arithmetic Instructions] ********/
            AddU32ImmU32Reg(imm, reg) => {
                format!("add ${imm:08x}, %{reg}")
            }
            AddU32RegU32Reg(in_reg, out_reg) => {
                format!("add %{in_reg}, %{out_reg}")
            }
            SubU32ImmU32Reg(imm, reg) => {
                format!("sub ${imm:08x}, %{reg}")
            }
            SubU32RegU32Imm(reg, imm) => {
                format!("sub %{reg}, ${imm:08x}")
            }
            SubU32RegU32Reg(reg_1, reg_2) => {
                format!("sub %{reg_1}, %{reg_2}")
            }
            MulU32ImmU32Reg(imm, reg) => {
                format!("mul ${imm:08x}, %{reg}")
            }
            MulU32RegU32Reg(reg_1, reg_2) => {
                format!("mul %{reg_1}, %{reg_2}")
            }
            DivU32ImmU32Reg(imm, reg) => {
                format!("div ${imm:08x}, %{reg}")
            }
            DivU32RegU32Imm(reg, imm) => {
                format!("div %{reg}, ${imm:08x}")
            }
            DivU32RegU32Reg(reg_1, reg_2) => {
                format!("div %{reg_1}, %{reg_2}")
            }
            ModU32ImmU32Reg(imm, reg) => {
                format!("mod ${imm:08x}, %{reg}")
            }
            ModU32RegU32Imm(reg, imm) => {
                format!("mod %{reg}, ${imm:08x}")
            }
            ModU32RegU32Reg(reg_1, reg_2) => {
                format!("mod %{reg_1}, %{reg_2}")
            }
            IncU32Reg(reg) => {
                format!("inc %{reg}")
            }
            DecU32Reg(reg) => {
                format!("dec %{reg}")
            }
            AndU32ImmU32Reg(imm, reg) => {
                format!("and ${imm:08x}, %{reg}")
            }

            /******** [Bit Operation Instructions] ********/
            LeftShiftU32ImmU32Reg(imm, reg) => {
                format!("shl ${imm:08x}, %{reg}")
            }
            LeftShiftU32RegU32Reg(shift_reg, reg) => {
                format!("shl {shift_reg}, %{reg}")
            }
            ArithLeftShiftU32ImmU32Reg(imm, reg) => {
                format!("sal ${imm:08x}, %{reg}")
            }
            ArithLeftShiftU32RegU32Reg(shift_reg, reg) => {
                format!("sal {shift_reg}, %{reg}")
            }
            RightShiftU32ImmU32Reg(imm, reg) => {
                format!("shr ${imm:08x}, %{reg}")
            }
            RightShiftU32RegU32Reg(shift_reg, reg) => {
                format!("shr %{shift_reg}, %{reg}")
            }
            ArithRightShiftU32ImmU32Reg(imm, reg) => {
                format!("sar ${imm:08x}, %{reg}")
            }
            ArithRightShiftU32RegU32Reg(shift_reg, reg) => {
                format!("sar {shift_reg}, %{reg}")
            }

            /******** [Branching Instructions] ********/
            CallU32Imm(addr) => {
                // TODO - apply labels to these jumps - either dynamically generated or via binary file metadata.
                format!("call [${addr:08x}]")
            }
            CallU32Reg(reg) => {
                format!("call [%{reg}]")
            }
            RetArgsU32 => String::from("iret"),
            Int(int_code) => {
                format!("int ${int_code:02x}")
            }
            IntRet => String::from("intret"),
            JumpAbsU32Imm(addr) => {
                // TODO - apply labels to these jumps - either dynamically generated or via binary file metadata.
                format!("jmp [${addr:08x}]")
            }
            JumpAbsU32Reg(reg) => {
                format!("jmp [%{reg}]")
            }

            /******** [Data Instructions] ********/
            SwapU32RegU32Reg(reg_1, reg_2) => {
                format!("swap %{reg_1}, %{reg_2}")
            }
            MovU32ImmU32Reg(imm, reg) => {
                format!("mov ${imm:08x}, %{reg}")
            }
            MovU32RegU32Reg(in_reg, out_reg) => {
                format!("mov %{in_reg}, %{out_reg}")
            }
            MovU32ImmMemSimple(imm, addr) => {
                format!("mov.s ${imm:08x}, [${addr:08x}]")
            }
            MovU32RegMemSimple(reg, addr) => {
                format!("mov.s %{reg}, [${addr:08x}]")
            }
            MovMemU32RegSimple(addr, reg) => {
                format!("mov.s [${addr:08x}], %{reg}")
            }
            MovU32RegPtrU32RegSimple(in_reg, out_reg) => {
                format!("mov.s [%{in_reg}], %{out_reg}")
            }
            MovU32ImmMemExpr(imm, expr) => {
                let mut decoder = MoveExpressionHandler::new();
                decoder.unpack(*expr);

                format!("mov.c ${imm:08x}, [{decoder}]")
            }
            MovMemExprU32Reg(expr, reg) => {
                let mut decoder = MoveExpressionHandler::new();
                decoder.unpack(*expr);

                format!("mov.c [{decoder}], %{reg}")
            }
            MovU32RegMemExpr(reg, expr) => {
                let mut decoder = MoveExpressionHandler::new();
                decoder.unpack(*expr);

                format!("mov.c %{reg}, [{decoder}]")
            }
            ByteSwapU32(reg) => {
                format!("bswap %{reg}")
            }
            ZeroHighBitsByIndexU32Reg(index_reg, in_reg, out_reg) => {
                format!("zhbi %{index_reg}, %{in_reg}, %{out_reg}")
            }
            ZeroHighBitsByIndexU32RegU32Imm(index, in_reg, out_reg) => {
                format!("zhbi {index}, %{in_reg}, %{out_reg}")
            }
            PushU32Imm(index) => {
                format!("push ${index:08x}")
            }
            PushU32Reg(reg) => {
                format!("push %{reg}")
            }
            PopU32ImmU32Reg(out_reg) => {
                format!("pop %{out_reg}")
            }
            OutU32Imm(value, device_id) => {
                format!("out ${value:08x}, ${device_id:02x}")
            }

            /******** [Logic Instructions] ********/
            BitTestU32Reg(bit, reg) => {
                format!("bt {bit}, %{reg}")
            }
            BitTestU32Mem(bit, addr) => {
                format!("bt {bit}, [${addr:08x}]")
            }
            BitTestResetU32Reg(bit, reg) => {
                format!("btr {bit}, %{reg}")
            }
            BitTestResetU32Mem(bit, addr) => {
                format!("btr {bit}, [${addr:08x}]")
            }
            BitTestSetU32Reg(bit, reg) => {
                format!("bts {bit}, %{reg}")
            }
            BitTestSetU32Mem(bit, addr) => {
                format!("bts {bit}, [${addr:08x}]")
            }
            BitScanReverseU32RegU32Reg(in_reg, out_reg) => {
                format!("bsr %{in_reg}, %{out_reg}")
            }
            BitScanReverseU32MemU32Reg(addr, reg) => {
                format!("bsr [${addr:08x}], %{reg}")
            }
            BitScanReverseU32RegMemU32(reg, out_addr) => {
                format!("bsr %{reg}, [${out_addr:08x}]")
            }
            BitScanReverseU32MemU32Mem(in_addr, out_addr) => {
                format!("bsr [${in_addr:08x}], [${out_addr:08x}]")
            }
            BitScanForwardU32RegU32Reg(in_reg, out_reg) => {
                format!("bsf %{in_reg}, %{out_reg}")
            }
            BitScanForwardU32MemU32Reg(addr, reg) => {
                format!("bsf [${addr:08x}], %{reg}")
            }
            BitScanForwardU32RegMemU32(reg, out_addr) => {
                format!("bsr %{reg}, [${out_addr:08x}]")
            }
            BitScanForwardU32MemU32Mem(in_addr, out_addr) => {
                format!("bsr [${in_addr:08x}], [${out_addr:08x}]")
            }

            /******** [Special Instructions] ********/
            MaskInterrupt(int_code) => {
                format!("mint ${int_code:02x}")
            }
            UnmaskInterrupt(int_code) => {
                format!("umint ${int_code:02x}")
            }
            LoadIVTAddrU32Imm(addr) => {
                format!("livt [${addr:08x}]")
            }
            MachineReturn => String::from("mret"),
            Halt => String::from("hlt"),

            /******** [Reserved Instructions] ********/
            Reserved1 | Reserved2 | Reserved3 | Reserved4 | Reserved5 | Reserved6 | Reserved7
            | Reserved8 | Reserved9 => unreachable!("attempted to use a reserved instruction"),

            /******** [Pseudo Instructions] ********/
            Label(_label) => {
                todo!();
            }
            Unknown(id) => format!("UNKNOWN! ID = {id:08x}"),
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
        use OpCode::*;

        match opcode {
            Nop => 0,

            /******** [Arithmetic Instructions] ********/
            AddU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            AddU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            SubU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            SubU32RegU32Imm => ARG_REG_ID_SIZE + ARG_U32_IMM_SIZE,
            SubU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            MulU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            MulU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            DivU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            DivU32RegU32Imm => ARG_REG_ID_SIZE + ARG_U32_IMM_SIZE,
            DivU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            ModU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            ModU32RegU32Imm => ARG_REG_ID_SIZE + ARG_U32_IMM_SIZE,
            ModU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            IncU32Reg => ARG_REG_ID_SIZE,
            DecU32Reg => ARG_REG_ID_SIZE,
            AndU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,

            /******** [Bit Operation Instructions] ********/
            LeftShiftU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            LeftShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            ArithLeftShiftU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            ArithLeftShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            RightShiftU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            RightShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            ArithRightShiftU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            ArithRightShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,

            /******** [Branching Instructions] ********/
            CallU32Imm => ARG_MEM_ADDR_SIZE,
            CallU32Reg => ARG_REG_ID_SIZE,
            RetArgsU32 => 0,
            Int => ARG_U8_IMM_SIZE,
            IntRet => 0,
            JumpAbsU32Imm => ARG_MEM_ADDR_SIZE,
            JumpAbsU32Reg => ARG_REG_ID_SIZE,

            /******** [Data Instructions] ********/
            SwapU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            MovU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            MovU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            MovU32ImmMemSimple => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            MovU32RegMemSimple => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            MovMemU32RegSimple => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            MovU32RegPtrU32RegSimple => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            MovU32ImmMemExpr => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            MovMemExprU32Reg => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            MovU32RegMemExpr => ARG_REG_ID_SIZE + ARG_U32_IMM_SIZE,
            ByteSwapU32 => ARG_REG_ID_SIZE,
            ZeroHighBitsByIndexU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            ZeroHighBitsByIndexU32RegU32Imm => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            PushU32Imm => ARG_U32_IMM_SIZE,
            PushU32Reg => ARG_REG_ID_SIZE,
            PopU32ImmU32Reg => ARG_REG_ID_SIZE,
            OutU32Imm => ARG_U32_IMM_SIZE + ARG_U8_IMM_SIZE,

            /******** [Logic Instructions] ********/
            BitTestU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            BitTestU32Mem => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            BitTestResetU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            BitTestResetU32Mem => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            BitTestSetU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            BitTestSetU32Mem => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            BitScanReverseU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            BitScanReverseU32MemU32Reg => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            BitScanReverseU32RegMemU32 => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            BitScanReverseU32MemU32Mem => ARG_MEM_ADDR_SIZE + ARG_MEM_ADDR_SIZE,
            BitScanForwardU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            BitScanForwardU32MemU32Reg => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            BitScanForwardU32RegMemU32 => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            BitScanForwardU32MemU32Mem => ARG_MEM_ADDR_SIZE + ARG_MEM_ADDR_SIZE,

            /******** [Special Instructions] ********/
            MaskInterrupt => ARG_U8_IMM_SIZE,
            UnmaskInterrupt => ARG_U8_IMM_SIZE,
            LoadIVTAddrU32Imm => ARG_MEM_ADDR_SIZE,
            MachineReturn | Halt => 0,

            /******** [Reserved Instructions] ********/
            Reserved1 | Reserved2 | Reserved3 | Reserved4 | Reserved5 | Reserved6 | Reserved7
            | Reserved8 | Reserved9 => 0,

            /******** [Pseudo Instructions] ********/
            Label | Unknown => 0,
        }
    }

    /// Get the total size of an [`Instruction`], in bytes.
    ///
    /// # Returns
    ///
    /// A usize giving the total size of the [`Instruction`], in bytes.
    #[inline(always)]
    pub fn get_total_instruction_size(&self) -> usize {
        self.get_instruction_arg_size() + INSTRUCTION_SIZE
    }
}
