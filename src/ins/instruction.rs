use std::{
    fmt::{self, Display, Formatter},
    mem,
};

use crate::{ins::move_expressions::MoveExpressionHandler, reg::registers::RegisterId};

use super::op_codes::OpCode;

/// The size of the instruction, in bytes.
const INSTRUCTION_SIZE: u32 = mem::size_of::<u32>() as u32;
/// The size of a u32 argument, in bytes.
const ARG_U32_IMM_SIZE: u32 = mem::size_of::<u32>() as u32;
/// The size of a u8 argument, in bytes.
const ARG_U8_IMM_SIZE: u32 = 1;
/// The size of a memory address argument, in bytes.
const ARG_MEM_ADDR_SIZE: u32 = mem::size_of::<u32>() as u32;
/// The size of a register ID argument, in bytes.
const ARG_REG_ID_SIZE: u32 = mem::size_of::<RegisterId>() as u32;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
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
    /// Increment a u32 register.
    IncU32Reg(RegisterId),
    /// Decrement a u32 register.
    DecU32Reg(RegisterId),

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

    /******** [Data Instructions] ********/
    /// Swap the values of the two registers.
    SwapU32RegU32Reg(RegisterId, RegisterId),
    /// Move a u32 immediate to u32 register (A). The result is copied into register A.
    MovU32ImmU32Reg(u32, RegisterId),
    /// Move a u32 register (B) to u32 register (A). The result is copied into register A.
    MovU32RegU32Reg(RegisterId, RegisterId),
    /// Move a u32 iImmediate to memory (relative to the base address of the code block). The result is copied into the specified memory address.
    MovU32ImmMemRelSimple(u32, u32),
    /// Move a u32 register to memory (relative to the base address of the code block). The result is copied into the specified memory address.
    MovU32RegMemRelSimple(RegisterId, u32),
    /// Move a u32 value from memory (relative to the base address of the code block) to a u32 register. The result is copied into the specified register.
    MovMemU32RegRelSimple(u32, RegisterId),
    /// Move the value from the memory address specified by a register (relative to the base address of the code block). The result is copied into the specified register.
    MovU32RegPtrU32RegRelSimple(RegisterId, RegisterId),
    /// Move a u32 immediate to memory (relative to the base address of the code block). The result is copied into the specified memory address.
    MovU32ImmMemExprRel(u32, u32),
    /// Move the address as given by an expression (from memory, relative to the base address of the code block). The result is copied into the specified register.
    MovMemExprU32RegRel(u32, RegisterId),
    /// Move the value of a register to the address given by an expression (relative to the base address of the code block). The result is copied into the specified memory address.
    MovU32RegMemExprRel(RegisterId, u32),
    /// Reverse the order of bytes in a specified register.
    ByteSwapU32(RegisterId),
    /// Zero the high bits of the source value starting from a specified index.
    ZeroHighBitsByIndexU32Reg(RegisterId, RegisterId, RegisterId),
    /// Zero the high bits of the source value starting from a specified index.
    ZeroHighBitsByIndexU32RegU32Imm(u32, RegisterId, RegisterId),

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
    /// Return from a subroutine.
    Ret,
    /// Machine return - downgrade the privilege level of the processor.
    Mret,
    /// Halt the execution of the processor.
    Hlt,
}

impl Display for Instruction {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        // NOTE: square brackets are used to indicate we are working with an address.
        let asm_format = match *self {
            Instruction::Nop => String::from("nop"),

            /******** [Arithmetic Instructions] ********/
            Instruction::AddU32ImmU32Reg(imm, reg) => {
                format!("add ${imm:02X}, {reg}")
            }
            Instruction::AddU32RegU32Reg(in_reg, out_reg) => {
                format!("add {in_reg}, {out_reg}")
            }
            Instruction::SubU32ImmU32Reg(imm, reg) => {
                format!("sub ${imm:02X}, {reg}")
            }
            Instruction::SubU32RegU32Imm(reg, imm) => {
                format!("sub {reg}, ${imm:02X}")
            }
            Instruction::SubU32RegU32Reg(reg_1, reg_2) => {
                format!("sub {reg_1}, ${reg_2}")
            }
            Instruction::IncU32Reg(reg) => {
                format!("inc {reg}")
            }
            Instruction::DecU32Reg(reg) => {
                format!("dec {reg}")
            }

            /******** [Bit Operation Instructions] ********/
            Instruction::LeftShiftU32ImmU32Reg(imm, reg) => {
                format!("shl ${imm:02X}, {reg}")
            }
            Instruction::LeftShiftU32RegU32Reg(shift_reg, reg) => {
                format!("shl {shift_reg}, {reg}")
            }
            Instruction::ArithLeftShiftU32ImmU32Reg(imm, reg) => {
                format!("sal ${imm:02X}, {reg}")
            }
            Instruction::ArithLeftShiftU32RegU32Reg(shift_reg, reg) => {
                format!("sal {shift_reg}, {reg}")
            }
            Instruction::RightShiftU32ImmU32Reg(imm, reg) => {
                format!("shr ${imm:02X}, {reg}")
            }
            Instruction::RightShiftU32RegU32Reg(shift_reg, reg) => {
                format!("shr {shift_reg}, {reg}")
            }
            Instruction::ArithRightShiftU32ImmU32Reg(imm, reg) => {
                format!("sar ${imm:02X}, {reg}")
            }
            Instruction::ArithRightShiftU32RegU32Reg(shift_reg, reg) => {
                format!("sar {shift_reg}, {reg}")
            }

            /******** [Data Instructions] ********/
            Instruction::SwapU32RegU32Reg(reg1, reg2) => {
                format!("swap {reg1}, {reg2}")
            }
            Instruction::MovU32ImmU32Reg(imm, reg) => {
                format!("mov ${imm:02X}, {reg}")
            }
            Instruction::MovU32RegU32Reg(in_reg, out_reg) => {
                format!("mov {in_reg}, {out_reg}")
            }
            Instruction::MovU32ImmMemRelSimple(imm, addr) => {
                format!("mov.rs ${imm:02X}, [${addr:04X}]")
            }
            Instruction::MovU32RegMemRelSimple(reg, addr) => {
                format!("mov.rs {reg}, [${addr:04X}]")
            }
            Instruction::MovMemU32RegRelSimple(addr, reg) => {
                format!("mov.rs [${addr:04X}], {reg}")
            }
            Instruction::MovU32RegPtrU32RegRelSimple(in_reg, out_reg) => {
                format!("mov.rs [{in_reg}], {out_reg}")
            }
            Instruction::MovU32ImmMemExprRel(imm, expr) => {
                let mut decoder = MoveExpressionHandler::new();
                decoder.decode(expr);

                format!("mov.rc ${imm:02X}, [{decoder}]")
            }
            Instruction::MovMemExprU32RegRel(expr, reg) => {
                let mut decoder = MoveExpressionHandler::new();
                decoder.decode(expr);

                format!("mov.rc [{decoder}], {reg}")
            }
            Instruction::MovU32RegMemExprRel(reg, expr) => {
                let mut decoder = MoveExpressionHandler::new();
                decoder.decode(expr);

                format!("mov.rc {reg}, [{decoder}]")
            }
            Instruction::ByteSwapU32(reg) => {
                format!("bswap {reg}")
            }
            Instruction::ZeroHighBitsByIndexU32Reg(index_reg, in_reg, out_reg) => {
                format!("zhbi {index_reg}, {in_reg}, {out_reg}")
            }
            Instruction::ZeroHighBitsByIndexU32RegU32Imm(index, in_reg, out_reg) => {
                format!("zhbi {index}, {in_reg}, {out_reg}")
            }

            /******** [Logic Instructions] ********/
            Instruction::BitTestU32Reg(bit, reg) => {
                format!("bt {bit}, {reg}")
            }
            Instruction::BitTestU32Mem(bit, addr) => {
                format!("bt {bit}, [${addr:04X}]")
            }
            Instruction::BitTestResetU32Reg(bit, reg) => {
                format!("btr {bit}, {reg}")
            }
            Instruction::BitTestResetU32Mem(bit, addr) => {
                format!("btr {bit}, [${addr:04X}]")
            }
            Instruction::BitTestSetU32Reg(bit, reg) => {
                format!("bts {bit}, {reg}")
            }
            Instruction::BitTestSetU32Mem(bit, addr) => {
                format!("bts {bit}, [${addr:04X}]")
            }
            Instruction::BitScanReverseU32RegU32Reg(in_reg, out_reg) => {
                format!("bsr {in_reg}, {out_reg}")
            }
            Instruction::BitScanReverseU32MemU32Reg(addr, reg) => {
                format!("bsr [${addr:04X}], {reg}")
            }
            Instruction::BitScanReverseU32RegMemU32(reg, out_addr) => {
                format!("bsr {reg}, [${out_addr:04X}]")
            }
            Instruction::BitScanReverseU32MemU32Mem(in_addr, out_addr) => {
                format!("bsr [${in_addr:04X}], [${out_addr:04X}]")
            }
            Instruction::BitScanForwardU32RegU32Reg(in_reg, out_reg) => {
                format!("bsf {in_reg}, {out_reg}")
            }
            Instruction::BitScanForwardU32MemU32Reg(addr, reg) => {
                format!("bsf [${addr:04X}], {reg}")
            }
            Instruction::BitScanForwardU32RegMemU32(reg, out_addr) => {
                format!("bsr {reg}, [${out_addr:04X}]")
            }
            Instruction::BitScanForwardU32MemU32Mem(in_addr, out_addr) => {
                format!("bsr [${in_addr:04X}], [${out_addr:04X}]")
            }

            /******** [Special Instructions] ********/
            Instruction::Ret => String::from("ret"),
            Instruction::Mret => String::from("mret"),
            Instruction::Hlt => String::from("hlt"),
        };
        write!(f, "{asm_format}")
    }
}

impl Instruction {
    pub fn get_instruction_arg_size(&self) -> u32 {
        match *self {
            Instruction::Nop => 0,

            /******** [Arithmetic Instructions] ********/
            Instruction::AddU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::AddU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::SubU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::SubU32RegU32Imm(_, _) => ARG_REG_ID_SIZE + ARG_U32_IMM_SIZE,
            Instruction::SubU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::IncU32Reg(_) => ARG_REG_ID_SIZE,
            Instruction::DecU32Reg(_) => ARG_REG_ID_SIZE,

            /******** [Bit Operation Instructions] ********/
            Instruction::LeftShiftU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::LeftShiftU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::ArithLeftShiftU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::ArithLeftShiftU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::RightShiftU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::RightShiftU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::ArithRightShiftU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::ArithRightShiftU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,

            /******** [Data Instructions] ********/
            Instruction::SwapU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::MovU32ImmU32Reg(_, _) => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::MovU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::MovU32ImmMemRelSimple(_, _) => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            Instruction::MovU32RegMemRelSimple(_, _) => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            Instruction::MovMemU32RegRelSimple(_, _) => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            Instruction::MovU32RegPtrU32RegRelSimple(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::MovU32ImmMemExprRel(_, _) => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            Instruction::MovMemExprU32RegRel(_, _) => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            Instruction::MovU32RegMemExprRel(_, _) => ARG_REG_ID_SIZE + ARG_U32_IMM_SIZE,
            Instruction::ByteSwapU32(_) => ARG_REG_ID_SIZE,
            Instruction::ZeroHighBitsByIndexU32Reg(_, _, _) => {
                ARG_REG_ID_SIZE + ARG_REG_ID_SIZE + ARG_REG_ID_SIZE
            }
            Instruction::ZeroHighBitsByIndexU32RegU32Imm(_, _, _) => {
                ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE + ARG_REG_ID_SIZE
            }

            /******** [Logic Instructions] ********/
            Instruction::BitTestU32Reg(_, _) => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::BitTestU32Mem(_, _) => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            Instruction::BitTestResetU32Reg(_, _) => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::BitTestResetU32Mem(_, _) => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            Instruction::BitTestSetU32Reg(_, _) => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            Instruction::BitTestSetU32Mem(_, _) => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            Instruction::BitScanReverseU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::BitScanReverseU32MemU32Reg(_, _) => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            Instruction::BitScanReverseU32RegMemU32(_, _) => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            Instruction::BitScanReverseU32MemU32Mem(_, _) => ARG_MEM_ADDR_SIZE + ARG_MEM_ADDR_SIZE,
            Instruction::BitScanForwardU32RegU32Reg(_, _) => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            Instruction::BitScanForwardU32MemU32Reg(_, _) => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            Instruction::BitScanForwardU32RegMemU32(_, _) => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            Instruction::BitScanForwardU32MemU32Mem(_, _) => ARG_MEM_ADDR_SIZE + ARG_MEM_ADDR_SIZE,

            /******** [Special Instructions] ********/
            Instruction::Ret => 0,
            Instruction::Mret => 0,
            Instruction::Hlt => 0,
        }
    }

    pub fn get_instruction_arg_size_from_op(opcode: OpCode) -> u32 {
        // Note: Yes, yes... code duplication since this is the same code as
        //       get_instruction_arg_size above, but I don't care here.
        match opcode {
            OpCode::Nop => 0,

            /******** [Arithmetic Instructions] ********/
            OpCode::AddU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::AddU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::SubU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::SubU32RegU32Imm => ARG_REG_ID_SIZE + ARG_U32_IMM_SIZE,
            OpCode::SubU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::IncU32Reg => ARG_REG_ID_SIZE,
            OpCode::DecU32Reg => ARG_REG_ID_SIZE,

            /******** [Bit Operation Instructions] ********/
            OpCode::LeftShiftU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::LeftShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::ArithLeftShiftU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::ArithLeftShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::RightShiftU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::RightShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::ArithRightShiftU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::ArithRightShiftU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,

            /******** [Data Instructions] ********/
            OpCode::SwapU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32ImmU32Reg => ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32ImmMemRelSimple => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::MovU32RegMemRelSimple => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::MovMemU32RegRelSimple => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32RegPtrU32RegRelSimple => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32ImmMemExprRel => ARG_U32_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::MovMemExprU32RegRel => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            OpCode::MovU32RegMemExprRel => ARG_REG_ID_SIZE + ARG_U32_IMM_SIZE,
            OpCode::ByteSwapU32 => ARG_REG_ID_SIZE,
            OpCode::ZeroHighBitsByIndexU32Reg => {
                ARG_REG_ID_SIZE + ARG_REG_ID_SIZE + ARG_REG_ID_SIZE
            }
            OpCode::ZeroHighBitsByIndexU32RegU32Imm => {
                ARG_U32_IMM_SIZE + ARG_REG_ID_SIZE + ARG_REG_ID_SIZE
            }

            /******** [Logic Instructions] ********/
            OpCode::BitTestU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::BitTestU32Mem => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::BitTestResetU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::BitTestResetU32Mem => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::BitTestSetU32Reg => ARG_U8_IMM_SIZE + ARG_REG_ID_SIZE,
            OpCode::BitTestSetU32Mem => ARG_U8_IMM_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::BitScanReverseU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::BitScanReverseU32MemU32Reg => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            OpCode::BitScanReverseU32RegMemU32 => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::BitScanReverseU32MemU32Mem => ARG_MEM_ADDR_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::BitScanForwardU32RegU32Reg => ARG_REG_ID_SIZE + ARG_REG_ID_SIZE,
            OpCode::BitScanForwardU32MemU32Reg => ARG_MEM_ADDR_SIZE + ARG_REG_ID_SIZE,
            OpCode::BitScanForwardU32RegMemU32 => ARG_REG_ID_SIZE + ARG_MEM_ADDR_SIZE,
            OpCode::BitScanForwardU32MemU32Mem => ARG_MEM_ADDR_SIZE + ARG_MEM_ADDR_SIZE,

            /******** [Special Instructions] ********/
            OpCode::Ret => 0,
            OpCode::Mret => 0,
            OpCode::Hlt => 0,
        }
    }

    pub fn get_total_instruction_size(&self) -> u32 {
        self.get_instruction_arg_size() + INSTRUCTION_SIZE
    }
}
