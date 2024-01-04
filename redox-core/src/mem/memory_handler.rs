use num_traits::FromPrimitive;
use prettytable::{row, Table};

use crate::{ins::instruction::Instruction, reg::registers::RegisterId};

use super::mapped_memory::MappedMemory;

/// The number of bytes in a megabyte.
pub const MEGABYTE: usize = 1024 * 1024;
/// The size of a u32 value, in bytes.
pub const BYTES_IN_U32: usize = 4;
/// The maximum permissible size of the system RAM segment.
pub const MAX_PHYSICAL_MEMORY: usize = MEGABYTE * 256;
/// The name of the RAM segment.
pub const RAM_SEGMENT_NAME: &str = "ram";

#[allow(unused)]
enum StackTypeHint {
    F32,
    U32,
    U8,
}

/*
 * The memory layout is currently as follows:
 * [USER SEGMENT]
 * [CODE SEGMENT]
 * [DATA SEGMENT]
 * [STACK SEGMENT]
 *
 * User memory represents anything prior to the code segment. The start of the user memory segment
 * contains the interrupt address table.
 *
 * Since the system does not have memory write enforcement, technically the entire thing is writable,
 * meaning that self-modifying code is technically also possible too.
 *
 * Certain memory areas are mapped to things outside of the emulated physical memory but can be
 * accessed as though they were. These mirrored segments will be used for graphics memory and the memory
 * of other emulated devices.
 * These areas are always mapped to segments outside the space of the emulated physical memory to avoid
 * causing conflicts.
 */

pub struct MemoryHandler {
    /// A list of the registered handlers for specially memory-mapped segments.
    mapped_memory: Vec<MappedMemory>,

    /// The start address of the user segment.
    pub user_segment_start: usize,
    /// The end address of the user segment.
    pub user_segment_end: usize,

    /// The start address of the code segment.
    pub code_segment_start: usize,
    /// The end address of the code segment.
    pub code_segment_end: usize,

    /// The start of the data segment.
    pub data_segment_start: usize,
    /// The end of the data segment.
    pub data_segment_end: usize,

    /// A list of type hints for the items currently on the stack.
    stack_type_hints: Vec<StackTypeHint>,
    /// The start address of the stack segment.
    pub stack_segment_start: usize,
    /// The end address of the stack segment.
    pub stack_segment_end: usize,
    /// The current stack pointer.
    stack_pointer: usize,
}

impl MemoryHandler {
    pub fn new(
        user_segment_capacity: usize,
        code_segment_bytes: &[u8],
        data_segment_bytes: &[u8],
        stack_segment_capacity: usize,
    ) -> Self {
        // The user segment is always the first in memory.
        let user_segment_start = 0;
        let user_segment_end = user_segment_capacity;

        // Next, if specified, will be the code segment.
        let code_segment_start = user_segment_end;
        let code_segment_end = code_segment_start
            + if !code_segment_bytes.is_empty() {
                code_segment_bytes.len()
            } else {
                0
            };

        // Next, if specified, will be the data segment.
        let data_segment_start = code_segment_end;
        let data_segment_end = data_segment_start
            + if !data_segment_bytes.is_empty() {
                data_segment_bytes.len()
            } else {
                0
            };

        // Next, if specified, will be the stack segment.
        let stack_segment_start = data_segment_end;
        let stack_segment_end = stack_segment_start
            + if stack_segment_capacity > 0 {
                stack_segment_capacity
            } else {
                0
            };

        // Assert that the entire memory will be less than a predefined size.
        // This will ensure that we can map the mirrored segments without having them
        // conflict with the actual main memory segments.
        assert!(stack_segment_end < MAX_PHYSICAL_MEMORY);

        let mut mem = Self {
            mapped_memory: vec![],
            user_segment_start,
            user_segment_end,
            code_segment_start,
            code_segment_end,
            data_segment_start,
            data_segment_end,
            stack_type_hints: Vec::with_capacity(stack_segment_capacity),
            stack_segment_start,
            stack_segment_end,
            stack_pointer: stack_segment_end,
        };

        // Insert the physical RAM memory-mapped segments that
        // represents the physical RAM.
        mem.add_mapped_memory_segment(
            0,
            stack_segment_end - user_segment_start,
            true,
            true,
            RAM_SEGMENT_NAME,
        );

        // Load the code segment bytes into RAM, if it has been provided.
        if !code_segment_bytes.is_empty() {
            mem.set_range(code_segment_start, code_segment_bytes);
        }

        // Load the data segment bytes into RAM, if it has been provided.
        if !data_segment_bytes.is_empty() {
            mem.set_range(data_segment_start, data_segment_bytes);
        }

        mem
    }

    /// Add a memory-mapped segment.
    ///
    /// # Arguments
    ///
    /// * `start` - The start address of the memory-mapped segment.
    /// * `length` - The length of the memory-mapped segment.
    /// * `can_read` - Can the segment be read from?
    /// * `can_write` - Can the segment be written to?
    /// * `name` - The name of the memory-mapped segment.
    ///
    /// # Returns
    ///
    /// A usize indicating the ID of the added memory-mapped segment.
    pub fn add_mapped_memory_segment(
        &mut self,
        start: usize,
        length: usize,
        can_read: bool,
        can_write: bool,
        name: &str,
    ) -> usize {
        // Check whether we have a memory-mapped segment that would intersect
        // with this one. We don't want to allow segments to cross like this.
        let end = start + length;
        assert!(!self
            .mapped_memory
            .iter()
            .any(|m| (start >= m.start && end <= m.end) || (start <= m.end && m.start <= end)));

        self.mapped_memory
            .push(MappedMemory::new(start, length, can_read, can_write, name));

        // Return the index of the segment we just added.
        self.mapped_memory.len() - 1
    }

    /// Can we pop a u32 value from the stack?
    #[inline(always)]
    pub fn can_pop_u32(&self) -> bool {
        self.stack_pointer >= self.stack_segment_start
            && self.stack_pointer + 4 <= self.stack_segment_end
    }

    /// Can we push a u32 value onto the stack?
    #[inline(always)]
    pub fn can_push_u32(&self) -> bool {
        self.stack_pointer - 4 >= self.stack_segment_start
            && self.stack_pointer <= self.stack_segment_end
    }

    /// Completely clear the physical (RAM) memory segment.
    pub fn clear(&mut self) {
        self.get_physical_memory_segment_mut().clear();
    }

    /// Attempt to decompile any instructions between a start and end memory location.
    ///
    /// # Arguments
    ///
    /// * `start` - The starting memory location.
    /// * `end` - The ending memory location
    ///
    /// # Returns
    ///
    /// A vector containing the decompiled instructions.
    pub fn decompile_instructions(&self, start: usize, end: usize) -> Vec<Instruction> {
        let mut instructions = vec![];

        let mut cursor = start;
        while cursor < end {
            let ins = self.get_instruction(cursor);
            instructions.push(ins);

            cursor += ins.get_total_instruction_size();
        }

        instructions
    }

    /// Attempt to get a reference to a byte in memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The position of the byte in memory.
    ///
    /// # Returns
    ///
    /// A pointer to the specific byte in memory.
    pub fn get_byte_ref(&self, pos: usize) -> &u8 {
        let segment = self
            .get_mapped_segment_by_address(pos)
            .expect("failed to get mapped memory segment for address");

        assert!(segment.can_read);

        &segment.data[pos - segment.start]
    }

    /// Attempt to read and clone a byte from memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The position of the byte in memory.
    ///
    /// # Returns
    ///
    /// A clone of the specific byte from memory.
    pub fn get_byte_clone(&self, pos: usize) -> u8 {
        let segment = self
            .get_mapped_segment_by_address(pos)
            .expect("failed to get mapped memory segment for address");

        assert!(segment.can_read);

        segment.data[pos - segment.start]
    }

    pub fn get_instruction(&self, pos: usize) -> Instruction {
        use crate::ins::op_codes::OpCode::*;

        // Read the OpCode ID.
        let opcode_id = self.get_u32(pos);

        // Validate the opcode is one of the ones we know about.
        // In the case we encounter an unrecognized opcode ID then we will
        // default to the Unknown opcode, which is useful for debugging.
        let opcode = FromPrimitive::from_u32(opcode_id).unwrap_or_default();

        // Calculate the length of the arguments, in bytes.
        let arg_len = Instruction::get_instruction_arg_size_from_op(opcode);
        let arg_bytes = self.get_range_ptr(pos + 4, arg_len);

        let mut cursor = 0;

        // Create our instruction instance.
        match opcode {
            Nop => Instruction::Nop,

            /******** [Arithmetic Instructions] ********/
            AddU32ImmU32Reg => {
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::AddU32ImmU32Reg(imm, reg)
            }
            AddU32RegU32Reg => {
                let reg_1 = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let reg_2 = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::AddU32RegU32Reg(reg_1, reg_2)
            }
            SubU32ImmU32Reg => {
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::SubU32ImmU32Reg(imm, reg)
            }
            SubU32RegU32Imm => {
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::SubU32RegU32Imm(reg, imm)
            }
            SubU32RegU32Reg => {
                let reg_1 = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let reg_2 = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::SubU32RegU32Reg(reg_1, reg_2)
            }
            MulU32ImmU32Reg => {
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::MulU32ImmU32Reg(imm, reg)
            }
            MulU32RegU32Reg => {
                let reg_1 = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let reg_2 = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::MulU32RegU32Reg(reg_1, reg_2)
            }
            DivU32ImmU32Reg => {
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::DivU32ImmU32Reg(imm, reg)
            }
            DivU32RegU32Imm => {
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::DivU32RegU32Imm(reg, imm)
            }
            DivU32RegU32Reg => {
                let reg_1 = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let reg_2 = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::DivU32RegU32Reg(reg_1, reg_2)
            }
            ModU32ImmU32Reg => {
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::ModU32ImmU32Reg(imm, reg)
            }
            ModU32RegU32Imm => {
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::ModU32RegU32Imm(reg, imm)
            }
            ModU32RegU32Reg => {
                let reg_1 = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let reg_2 = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::ModU32RegU32Reg(reg_1, reg_2)
            }
            IncU32Reg => {
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::IncU32Reg(reg)
            }
            DecU32Reg => {
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::DecU32Reg(reg)
            }
            AndU32ImmU32Reg => {
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::AndU32ImmU32Reg(imm, reg)
            }

            /******** [Bit Operation Instructions] ********/
            LeftShiftU32ImmU32Reg => {
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::LeftShiftU32ImmU32Reg(imm, reg)
            }
            LeftShiftU32RegU32Reg => {
                let shift_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::LeftShiftU32RegU32Reg(shift_reg, reg)
            }
            ArithLeftShiftU32ImmU32Reg => {
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::ArithLeftShiftU32ImmU32Reg(imm, reg)
            }
            ArithLeftShiftU32RegU32Reg => {
                let shift_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::ArithLeftShiftU32RegU32Reg(shift_reg, reg)
            }
            RightShiftU32ImmU32Reg => {
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::RightShiftU32ImmU32Reg(imm, reg)
            }
            RightShiftU32RegU32Reg => {
                let shift_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::RightShiftU32RegU32Reg(shift_reg, reg)
            }
            ArithRightShiftU32ImmU32Reg => {
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::ArithRightShiftU32ImmU32Reg(imm, reg)
            }
            ArithRightShiftU32RegU32Reg => {
                let shift_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::ArithRightShiftU32RegU32Reg(shift_reg, reg)
            }

            /******** [Branching Instructions] ********/
            Int => {
                let addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::Int(addr)
            }
            IntRet => Instruction::IntRet,
            JumpAbsU32Imm => {
                let addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::JumpAbsU32Imm(addr)
            }
            JumpAbsU32Reg => {
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::JumpAbsU32Reg(reg)
            }

            /******** [Data Instructions] ********/
            SwapU32RegU32Reg => {
                let reg1 = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let reg2 = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::SwapU32RegU32Reg(reg1, reg2)
            }
            MovU32ImmU32Reg => {
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::MovU32ImmU32Reg(imm, reg)
            }
            MovU32RegU32Reg => {
                let in_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let out_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::MovU32RegU32Reg(in_reg, out_reg)
            }
            MovU32ImmMemSimple => {
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::MovU32ImmMemSimple(imm, addr)
            }
            MovU32RegMemSimple => {
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::MovU32RegMemSimple(reg, addr)
            }
            MovMemU32RegSimple => {
                let addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::MovMemU32RegSimple(addr, reg)
            }
            MovU32RegPtrU32RegSimple => {
                let in_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let out_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::MovU32RegPtrU32RegSimple(in_reg, out_reg)
            }
            MovU32ImmMemExpr => {
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let expr = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::MovU32ImmMemExpr(imm, expr)
            }
            MovMemExprU32Reg => {
                let expr = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::MovMemExprU32Reg(expr, reg)
            }
            MovU32RegMemExpr => {
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let expr = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::MovU32RegMemExpr(reg, expr)
            }
            ByteSwapU32 => {
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::ByteSwapU32(reg)
            }
            ZeroHighBitsByIndexU32Reg => {
                let index_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let in_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let out_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::ZeroHighBitsByIndexU32Reg(index_reg, in_reg, out_reg)
            }
            ZeroHighBitsByIndexU32RegU32Imm => {
                let index = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let in_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let out_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::ZeroHighBitsByIndexU32RegU32Imm(index, in_reg, out_reg)
            }
            PushU32Imm => {
                let imm = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::PushU32Imm(imm)
            }

            /******** [Logic Instructions] ********/
            BitTestU32Reg => {
                let bit = MemoryHandler::read_u8(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::BitTestU32Reg(bit, reg)
            }
            BitTestU32Mem => {
                let bit = MemoryHandler::read_u8(arg_bytes, &mut cursor);
                let addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::BitTestU32Mem(bit, addr)
            }
            BitTestResetU32Reg => {
                let bit = MemoryHandler::read_u8(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::BitTestResetU32Reg(bit, reg)
            }
            BitTestResetU32Mem => {
                let bit = MemoryHandler::read_u8(arg_bytes, &mut cursor);
                let addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::BitTestResetU32Mem(bit, addr)
            }
            BitTestSetU32Reg => {
                let bit = MemoryHandler::read_u8(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::BitTestSetU32Reg(bit, reg)
            }
            BitTestSetU32Mem => {
                let bit = MemoryHandler::read_u8(arg_bytes, &mut cursor);
                let addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::BitTestSetU32Mem(bit, addr)
            }
            BitScanReverseU32RegU32Reg => {
                let in_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let out_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::BitScanReverseU32RegU32Reg(in_reg, out_reg)
            }
            BitScanReverseU32MemU32Reg => {
                let addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::BitScanReverseU32MemU32Reg(addr, reg)
            }
            BitScanReverseU32RegMemU32 => {
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let out_addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::BitScanReverseU32RegMemU32(reg, out_addr)
            }
            BitScanReverseU32MemU32Mem => {
                let in_addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let out_addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::BitScanReverseU32MemU32Mem(in_addr, out_addr)
            }
            BitScanForwardU32RegU32Reg => {
                let in_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let out_reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::BitScanForwardU32RegU32Reg(in_reg, out_reg)
            }
            BitScanForwardU32MemU32Reg => {
                let addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);

                Instruction::BitScanForwardU32MemU32Reg(addr, reg)
            }
            BitScanForwardU32RegMemU32 => {
                let reg = MemoryHandler::read_register_id(arg_bytes, &mut cursor);
                let out_addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::BitScanForwardU32RegMemU32(reg, out_addr)
            }
            BitScanForwardU32MemU32Mem => {
                let in_addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);
                let out_addr = MemoryHandler::read_u32(arg_bytes, &mut cursor);

                Instruction::BitScanForwardU32MemU32Mem(in_addr, out_addr)
            }

            /******** [Special Instructions] ********/
            Ret => Instruction::Ret,
            Mret => Instruction::Mret,
            Hlt => Instruction::Hlt,
            Unknown => Instruction::Unknown(opcode_id),
        }
    }

    /// Attempt to find the memory-mapped segment that is associated with a given address.
    ///
    /// # Arguments
    ///
    /// * `pos` - The position of the byte in memory.
    ///
    /// # Returns
    ///
    /// A reference to the associated [`MappedMemory`] segment.
    ///
    /// # Note
    ///
    /// This method will assert if no valid memory segment is located.
    #[inline(always)]
    pub fn get_mapped_segment_by_address(&self, pos: usize) -> Option<&MappedMemory> {
        self.mapped_memory
            .iter()
            .find(|e| e.can_contains_range(pos, pos))
    }

    /// Attempt to find the mapped memory segment that is associated with a given address.
    ///
    /// # Arguments
    ///
    /// * `pos` - The position of the byte in memory.
    ///
    /// # Returns
    ///
    /// A reference to the associated [`MappedMemory`] segment.
    ///
    /// # Note
    ///
    /// This method will assert if no valid memory segment is located.
    #[inline(always)]
    pub fn get_mapped_segment_by_address_mut(&mut self, pos: usize) -> Option<&mut MappedMemory> {
        self.mapped_memory
            .iter_mut()
            .find(|e| e.can_contains_range(pos, pos))
    }

    /// Attempt to find the memory-mapped segment that has the specified index.
    ///
    /// # Arguments
    ///
    /// * `index` - The index of the segment.
    ///
    /// # Returns
    ///
    /// A mutable reference to the [`MappedMemory`] segment.
    ///
    /// # Note
    ///
    /// This method will assert if no valid segment is located.
    #[inline(always)]
    pub fn get_mapped_segment_by_index_mut(&mut self, index: usize) -> Option<&mut MappedMemory> {
        if index >= self.mapped_memory.len() {
            return None;
        }

        Some(&mut self.mapped_memory[index])
    }

    /// Attempt to find the mapped memory segment that has a specific name.
    ///
    /// # Arguments
    ///
    /// * `name` - A string giving the name of the segment.
    ///
    /// # Returns
    ///
    /// A reference to the associated [`MappedMemory`] segment.
    ///
    /// # Note
    ///
    /// This method will assert if no valid memory segment is located.
    #[inline(always)]
    pub fn get_mapped_segment_by_name(&self, name: &str) -> Option<&MappedMemory> {
        self.mapped_memory.iter().find(|e| e.name == name)
    }

    /// Attempt to find the mapped memory segment that has a specific name.
    ///
    /// # Arguments
    ///
    /// * `name` - A string giving the name of the segment.
    ///
    /// # Returns
    ///
    /// A mutable reference to the associated [`MappedMemory`] segment.
    ///
    /// # Note
    ///
    /// This method will assert if no valid memory segment is located.
    #[inline(always)]
    pub fn get_mapped_segment_by_name_mut(&mut self, name: &str) -> Option<&mut MappedMemory> {
        self.mapped_memory.iter_mut().find(|e| e.name == name)
    }

    /// Get a reference to the physical (RAM) memory segment.
    #[inline(always)]
    pub fn get_physical_memory_segment(&self) -> &MappedMemory {
        self.get_mapped_segment_by_name(RAM_SEGMENT_NAME)
            .expect("failed to get physical RAM memory segment")
    }

    /// Get a mutable reference to the physical RAM memory segment.
    #[inline(always)]
    pub fn get_physical_memory_segment_mut(&mut self) -> &mut MappedMemory {
        self.get_mapped_segment_by_name_mut(RAM_SEGMENT_NAME)
            .expect("failed to get physical RAM memory segment")
    }

    /// Attempt to read a u32 value from memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The starting position of the bytes in memory.
    ///
    /// # Returns
    ///
    /// A u32 from the specified memory address.
    pub fn get_u32(&self, pos: usize) -> u32 {
        let bytes: [u8; 4] = self
            .get_range_ptr(pos, 4)
            .try_into()
            .expect("failed to create a u32 from memory bytes");

        u32::from_le_bytes(bytes)
    }

    /// Attempt to get a pointer to a range of bytes within memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The starting position of the bytes in memory.
    /// * `len` - The number of bytes to read from memory.
    ///
    /// # Returns
    ///
    /// A reference to a u8 slice from memory.
    #[inline(always)]
    pub fn get_range_ptr(&self, pos: usize, len: usize) -> &[u8] {
        let segment = self
            .get_mapped_segment_by_address(pos)
            .expect("failed to get mapped memory segment for address");

        assert!(segment.can_read);

        // Translate the global address into the local variant.
        let start = pos - segment.start;
        let end = start + len;
        segment.assert_within_bounds(end);

        // Translate the absolute address into the absolute local variant.
        &segment.data[start..end]
    }

    /// Attempt to clone a range of bytes from memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The starting position of the bytes in memory.
    /// * `len` - The number of bytes to read from memory.
    ///
    /// # Returns
    ///
    /// A vector containing the bytes cloned from memory.
    pub fn get_range_clone(&self, pos: usize, len: usize) -> Vec<u8> {
        self.get_range_ptr(pos, len).to_vec()
    }

    /// Get a slice of the code segment contents.
    ///
    /// # Returns
    ///
    /// A slice of u8 values.
    pub fn get_code_segment_storage(&self) -> &[u8] {
        &self.get_ram_storage()[self.code_segment_start..self.code_segment_end]
    }

    /// Get a slice of the data segment contents.
    ///
    /// # Returns
    ///
    /// A slice of u8 values.
    pub fn get_data_segment_storage(&self) -> &[u8] {
        &self.get_ram_storage()[self.data_segment_start..self.data_segment_end]
    }

    /// Get a slice of the raw stack segment contents.
    ///
    /// # Returns
    ///
    /// A slice of u8 values
    pub fn get_stack_segment_storage(&self) -> &[u8] {
        &self.get_ram_storage()[self.stack_segment_start..self.stack_segment_end]
    }

    /// Get a slice of the entire memory contents.
    ///
    /// # Returns
    ///
    /// A slice of u8 values, referencing every byte in memory.
    pub fn get_ram_storage(&self) -> &[u8] {
        &self.get_physical_memory_segment().data
    }

    /// Get a slice of the user segment contents.
    ///
    /// # Returns
    ///
    /// A slice of u8 values
    pub fn get_user_segment_storage(&self) -> &[u8] {
        &self.get_ram_storage()[..self.code_segment_start]
    }

    /// Is the physicla memory segment empty?
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Get the size of the physical memory segment.
    ///
    /// # Returns
    ///
    /// The length of the physical memory segment, in bytes.
    pub fn len(&self) -> usize {
        self.get_ram_storage().len()
    }

    // Attempt to pop a [`StackTypeHint`] from the stack hint list.
    #[cfg(feature = "stack-type-hints")]
    fn pop_type_hint(&mut self) {
        self.stack_type_hints.pop();
    }

    /// Attempt to pop a [`StackTypeHint`] from the stack hint list.
    #[cfg(not(feature = "stack-type-hints"))]
    fn pop_type_hint(&mut self) {}

    /// Attempt to pop a u32 value from the stack.
    pub fn pop_u32(&mut self) -> u32 {
        let value_start_pos = self.stack_pointer;
        assert!(
            self.can_pop_u32(),
            "insufficient space on the stack to pop a u32 value"
        );

        // Read the value from the stack.
        let result = self.get_u32(value_start_pos);
        self.pop_type_hint();

        // Update the stack pointer.
        self.stack_pointer = value_start_pos + 4;

        result
    }

    /// Attempt to push a [`StackTypeHint`] onto the stack hint list.
    ///
    /// # Arguments
    ///
    /// * `hint` - The [`StackTypeHint`] to be added to the hit list.
    #[cfg(feature = "stack-type-hints")]
    fn push_type_hint(&mut self, hint: StackTypeHint) {
        self.stack_type_hints.push(hint);
    }

    /// Attempt to push a [`StackTypeHint`] onto the stack hint list.
    #[cfg(not(feature = "stack-type-hints"))]
    fn push_type_hint(&mut self, _hint: StackTypeHint) {}

    /// Attempt to push a u32 value onto the stack.
    ///
    /// # Arguments
    ///
    /// * `value` - The u32 value to be pushed onto the stack.
    ///
    /// # Notes
    ///
    /// This method will automatically keep track of the stack pointer as held within
    /// the memory object, but the CPU registers **must** be updated separately or
    /// they will fall out of sync.
    pub fn push_u32(&mut self, value: u32) {
        assert!(
            self.can_push_u32(),
            "insufficient space on the stack to push a onto u32 value"
        );

        // This needs to be 3 (not 4) since we are working from the basis that this
        // will add a value to the last index. 0 to 3 is 4 positions.
        let value_start_pos = self.stack_pointer - 4;

        // Push the value to the stack.
        self.set_u32(value_start_pos, value);
        self.push_type_hint(StackTypeHint::U32);

        // Update the stack pointer.
        self.stack_pointer = value_start_pos;
    }

    /// Attempt to read a [`RegisterId`] from a memory slice.
    ///
    /// # Arguments
    ///
    /// * `bytes` - A slice of u8 values from which the register ID should be extracted.
    /// * `cursor` - A mutable reference to the cursor, which specifies the starting position within the slice.
    pub fn read_register_id(bytes: &[u8], cursor: &mut usize) -> RegisterId {
        FromPrimitive::from_u8(MemoryHandler::read_u8(bytes, cursor))
            .expect("failed to read register ID from memory")
    }

    /// Attempt to read a u32 value from a memory slice.
    ///
    /// # Arguments
    ///
    /// * `bytes` - A slice of u8 values from which the register ID should be extracted.
    /// * `cursor` - A mutable reference to the cursor, which specifies the starting position within the slice.
    pub fn read_u32(bytes: &[u8], cursor: &mut usize) -> u32 {
        // This will assert if the value is out of range anyway.
        let bytes: [u8; 4] = bytes[*cursor..(*cursor + 4)]
            .try_into()
            .expect("failed to read u32 value from memory");
        *cursor += 4;

        u32::from_le_bytes(bytes)
    }

    /// Attempt to read a u8 value from a memory slice.
    ///
    /// # Arguments
    ///
    /// * `bytes` - A slice of u8 values from which the register ID should be extracted.
    /// * `cursor` - A mutable reference to the cursor, which specifies the starting position within the slice.
    #[inline(always)]
    pub fn read_u8(bytes: &[u8], cursor: &mut usize) -> u8 {
        let byte = bytes[*cursor];
        *cursor += 1;
        byte
    }

    /// Set the value of a specific byte in memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The point in memory to be checked.
    /// * `value` - The value to be written into memory.
    pub fn set(&mut self, pos: usize, value: u8) {
        let segment = self
            .get_mapped_segment_by_address_mut(pos)
            .expect("failed to get mapped memory segment for address");

        assert!(segment.can_write);

        // Translate the global address into the local variant.
        segment.data[pos - segment.start] = value;
    }

    /// Set the value of a range of values in memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The absolute position of the first byte to be written into memory.
    /// * `values` - A slice of u8 values that are to be written into memory.
    #[inline(always)]
    pub fn set_range(&mut self, pos: usize, values: &[u8]) {
        let segment = self
            .get_mapped_segment_by_address_mut(pos)
            .expect("failed to get mapped memory segment for address");

        assert!(segment.can_write);

        // Translate the global address into the local variant.
        let start = pos - segment.start;
        let end = start + values.len();
        segment.assert_within_bounds(end);

        // This is safe since we have checked that the range is completely
        // contained within the specific memory segment.
        segment.data[start..end].copy_from_slice(values);
    }

    /// Write a u32 value into memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The position of the first byte to be written into memory.
    /// * `value` - The value to be written into memory.
    pub fn set_u32(&mut self, pos: usize, value: u32) {
        self.set_range(pos, &u32::to_le_bytes(value));
    }

    /// Print a list of the mapped memory segments.
    pub fn print_mapped_memory_segments(&self) {
        let mut table = Table::new();

        table.add_row(row!["ID", "Name", "Start", "End"]);

        for (i, segment) in self.mapped_memory.iter().enumerate() {
            table.add_row(row![
                i,
                segment.name,
                format!("{:0>8X}", segment.start),
                format!("{:0>8X}", segment.end)
            ]);
        }

        table.printstd();
    }

    /// Print the contents of the stack.
    pub fn print_stack(&self) {
        let mut table = Table::new();

        if cfg!(feature = "stack-type-hints") {
            if self.stack_type_hints.is_empty() {
                println!("The stack is currently empty.");
                return;
            }

            table.add_row(row!["Index", "Value", "Type"]);

            // We will walk backwards up the stack.
            // In the stack a lower index represents a value that has been more recently added to the stack.
            let mut current_pos = self.stack_segment_end;

            for (id, hint) in self.stack_type_hints.iter().enumerate() {
                match *hint {
                    StackTypeHint::F32 => todo!(),
                    StackTypeHint::U32 => {
                        let value = self.get_u32(current_pos - 4);
                        table.add_row(row![id, format!("{value}"), "u32"]);
                        current_pos -= 4;
                    }
                    StackTypeHint::U8 => todo!(),
                }
            }

            table.printstd();
        } else {
            println!("WARNING: stack type hints are disabled. A raw view of stack memory will be displayed.");
            println!("{:?}", self.get_stack_segment_storage());
        }
    }
}

impl From<&[u8]> for MemoryHandler {
    fn from(values: &[u8]) -> Self {
        MemoryHandler::new(100, values, &[], 0)
    }
}

#[cfg(test)]
mod tests_memory {
    use crate::ins::instruction::Instruction;

    use super::{MemoryHandler, MEGABYTE};

    fn fill_memory_sequential(mem: &mut MemoryHandler) {
        for (i, byte) in &mut mem
            .get_physical_memory_segment_mut()
            .data
            .iter_mut()
            .enumerate()
        {
            *byte = (i as u8) % 255;
        }
    }

    fn get_test_ram_instance() -> MemoryHandler {
        let mut ram = MemoryHandler::new(100, &[], &[], 0);

        // Fill the memory with debug data.
        fill_memory_sequential(&mut ram);

        ram
    }

    /// Test adding and working with mapped memory segments.
    #[test]
    fn test_adding_mapped_memory_segment() {
        let mut ram = MemoryHandler::new(100, &[], &[], 0);

        // Well past the boot memory segment.
        let start = MEGABYTE * 512;
        let length = 100;

        // Add the segment.
        ram.add_mapped_memory_segment(start, length, true, true, "tester");
        assert_eq!(ram.mapped_memory.len(), 2);

        // Next, testing we can read and write to the mapped memory segment.
        ram.set_u32(start, 0xDEADBEEF);

        // Next, test that we can read the value back from the mapped memory segment.
        assert_eq!(ram.get_u32(start), 0xDEADBEEF);
    }

    /// Test adding an overlapping memory segment panics.
    #[test]
    #[should_panic]
    fn test_adding_mapped_memory_segment_overlapping() {
        let mut ram = MemoryHandler::new(100, &[], &[], 0);

        // Well past the boot memory segment.
        let start = MEGABYTE * 512;
        let length = 100;

        // Add the segment.
        ram.add_mapped_memory_segment(start, length, true, true, "tester");
        assert_eq!(ram.mapped_memory.len(), 3);

        // Attempt to add another segment that intersects with the one we previously added.
        ram.add_mapped_memory_segment(start + 1, length, true, true, "tester 2");
    }

    /// Test reading from a mapped memory segment that doesn't support it.
    #[test]
    #[should_panic]
    fn test_mapped_memory_segment_no_reading() {
        let mut ram = MemoryHandler::new(100, &[], &[], 0);

        // Well past the boot memory segment.
        let start = MEGABYTE * 512;
        let length = 100;

        // Add the segment.
        ram.add_mapped_memory_segment(start, length, false, true, "tester");
        assert_eq!(ram.mapped_memory.len(), 3);

        // Can we read from the memory segment?
        let _ = ram.get_u32(start);
    }

    /// Test writing to a mapped memory segment that doesn't support it.
    #[test]
    #[should_panic]
    fn test_mapped_memory_segment_no_writing() {
        let mut ram = MemoryHandler::new(100, &[], &[], 0);

        // Well past the boot memory segment.
        let start = MEGABYTE * 512;
        let length = 100;

        // Add the segment.
        ram.add_mapped_memory_segment(start, length, true, false, "tester");
        assert_eq!(ram.mapped_memory.len(), 3);

        // Can we read from the memory segment?
        ram.set_u32(start, 0xDEAFBEEF);
    }

    /// Test attempting to read beyond the bounds of a mapped memory segment.
    #[test]
    #[should_panic]
    fn test_mapped_memory_segment_read_past_bounds() {
        let mut ram = MemoryHandler::new(100, &[], &[], 0);

        // Well past the boot memory segment.
        let start = MEGABYTE * 512;
        let length = 2;

        // Add the segment.
        ram.add_mapped_memory_segment(start, length, true, true, "tester");
        assert_eq!(ram.mapped_memory.len(), 3);

        // Can we read from the memory segment?
        let _ = ram.get_u32(start);
    }

    /// Test attempting to write beyond the bounds of a mapped memory segment.
    #[test]
    #[should_panic]
    fn test_mapped_memory_segment_write_past_bounds() {
        let mut ram = MemoryHandler::new(100, &[], &[], 0);

        // Well past the boot memory segment.
        let start = MEGABYTE * 512;
        let length = 2;

        // Add the segment.
        ram.add_mapped_memory_segment(start, length, true, true, "tester");
        assert_eq!(ram.mapped_memory.len(), 3);

        // Can we read from the memory segment?
        ram.set_u32(start, 0xDEADBEEF);
    }

    /// Test the basic aspects of creating a RAM module.
    #[test]
    fn test_ram_creation_simple() {
        let size = 100;
        let ram = MemoryHandler::new(size, &[], &[], 0);

        assert_eq!(
            ram.len(),
            100,
            "failed to create a RAM module of the specified size"
        );

        assert_eq!(
            ram.get_range_ptr(0, 100 - 1),
            vec![0x0; 100 - 1],
            "failed to correctly initialize RAM"
        );
    }

    /// Test the complex aspects of creating a RAM module.
    #[test]
    fn test_ram_creation_complex() {
        let user_size = 0x100;
        let stack_capacity_bytes = 2 * 4;
        let code = [0x1; 10];
        let data = [0x2; 10];
        let mut ram = MemoryHandler::new(user_size, &code, &data, stack_capacity_bytes);

        // Add some stack entries.
        let mut stack_bytes = vec![];
        ram.push_u32(123);
        ram.push_u32(321);

        stack_bytes.extend_from_slice(&321u32.to_le_bytes());
        stack_bytes.extend_from_slice(&123u32.to_le_bytes());

        assert_eq!(
            ram.len(),
            user_size + stack_capacity_bytes + code.len() + data.len(),
            "failed to create a RAM module of the specified size"
        );

        // Check the code segment is correct.
        assert_eq!(code, ram.get_code_segment_storage());

        // Check the data segment is correct.
        assert_eq!(data, ram.get_data_segment_storage());

        // Check the stack segment is correct.
        assert_eq!(stack_bytes, ram.get_stack_segment_storage());
    }

    /// Test reading beyond RAM bounds.
    #[test]
    #[should_panic]
    fn test_read_no_mapped_segment() {
        let ram = MemoryHandler::new(100, &[], &[], 0);

        _ = ram.get_range_ptr(usize::MAX, 1);
    }

    /// Test basic reading and writing.
    #[test]
    fn test_read_write_u8() {
        let mut ram = get_test_ram_instance();

        // Read/write a single byte.
        let byte = 0xff;
        ram.set(0, byte);
        assert_eq!(
            byte,
            *ram.get_byte_ref(0),
            "failed to read correct value from memory"
        );

        // Read/write a range of bytes.
        let input: Vec<u8> = (0..50).collect();
        ram.set_range(0, &input);
        assert_eq!(
            &input,
            ram.get_range_ptr(0, 50),
            "failed to read correct values from memory"
        );
    }

    /// Test a u32 round-trip by writing to and reading from memory.
    #[test]
    fn test_read_write_u32() {
        let mut ram = get_test_ram_instance();

        let input = 0xDEADBEEF;
        ram.set_u32(0, input);

        assert_eq!(input, ram.get_u32(0));
    }

    /// Test a stack push and pop round-trip.
    #[test]
    fn test_stack_u32_push_pop() {
        let mut ram = MemoryHandler::new(10, &[1, 2, 3, 4], &[9, 8, 7, 6, 5], 2 * 4);

        ram.push_u32(0x123);
        ram.push_u32(0x321);

        assert_eq!(ram.pop_u32(), 0x321);
        assert_eq!(ram.pop_u32(), 0x123);
    }

    /// Test for an assertion when popping an entry from an empty stack.
    #[test]
    #[should_panic]
    fn test_stack_pop_with_empty_stack() {
        let mut ram = MemoryHandler::new(100, &[], &[], 2);

        _ = ram.pop_u32();
    }

    /// Test for an assertion when pushing too many entries to the stack.
    #[test]
    #[should_panic]
    fn test_pushing_to_full_stack() {
        let mut ram = MemoryHandler::new(100, &[], &[], 1);

        ram.push_u32(0x123);
        ram.push_u32(0x321);
    }

    /// Test asserting when reading an invalid opcode ID, generating a special Unknown instruction.
    #[test]
    fn test_assert_invalid_opcode_id_preserve() {
        // This will generate an invalid opcode ID.
        let id = u32::MAX - 1;

        // Load the fake data into memory.
        let ram = MemoryHandler::new(100, &id.to_le_bytes(), &[], 0);

        // Attempt to decompile the instruction, yielding an invalid opcode.
        let result = ram.decompile_instructions(ram.code_segment_start, ram.code_segment_end);

        assert_eq!(result, vec![Instruction::Unknown(id)]);
    }
}
