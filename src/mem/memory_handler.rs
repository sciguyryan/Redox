use num_traits::FromPrimitive;
use prettytable::{row, Table};

use crate::ins::{instruction::Instruction, op_codes::OpCode};

use super::{mapped_memory::MappedMemory, memory_block_reader::MemoryBlockReader};

/// The number of bytes in a megabyte.
pub const MEGABYTE: usize = 1024 * 1024;

/// The maximum permissible size of the system RAM. Here it is 256 megabyte in size.
pub const MAX_PHYSICAL_MEMORY: usize = MEGABYTE * 256;

/// The name of the RAM region.
pub const RAM_REGION_NAME: &str = "ram";

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
 * accessed as though they were. These mirrored areas will be used for graphics memory and the memory
 * of other emulated devices.
 * These areas are always mapped to regions outside the space of the emulated physical memory to avoid
 * causing conflicts.
 */

pub struct MemoryHandler {
    /// A list of the registered handlers for specially mapped memory regions.
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
        let code_segment_start: usize;
        let code_segment_end: usize;
        if !code_segment_bytes.is_empty() {
            code_segment_start = user_segment_end;
            code_segment_end = code_segment_start + code_segment_bytes.len();
        } else {
            code_segment_start = user_segment_end;
            code_segment_end = code_segment_start;
        }

        // Next, if specified, will be the data segment.
        let data_segment_start: usize;
        let data_segment_end: usize;
        if !data_segment_bytes.is_empty() {
            data_segment_start = code_segment_end;
            data_segment_end = data_segment_start + data_segment_bytes.len();
        } else {
            data_segment_start = code_segment_end;
            data_segment_end = data_segment_start;
        }

        // Next, if specified, will be the stack segment.
        let stack_segment_start: usize;
        let stack_segment_end: usize;
        if stack_segment_capacity > 0 {
            stack_segment_start = data_segment_end;
            stack_segment_end = stack_segment_start + stack_segment_capacity;
        } else {
            stack_segment_start = data_segment_end;
            stack_segment_end = stack_segment_start;
        }

        // Assert that the entire memory will be less than 256 megabytes in size.
        // This will ensure that we can map the mirrored regions without having them
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
            stack_type_hints: vec![],
            stack_segment_start,
            stack_segment_end,
            stack_pointer: stack_segment_end,
        };

        // Insert the physical RAM memory-mapped region that represents the RAM.
        mem.add_mapped_memory_segment(
            0,
            stack_segment_end - user_segment_start,
            true,
            true,
            RAM_REGION_NAME,
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

    /// Attempt to get a pointer to a byte in memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The position of the byte in memory.
    ///
    /// # Returns
    ///
    /// A pointer to the specific byte in memory.
    pub fn get_byte_ptr(&self, pos: usize) -> &u8 {
        let region = self
            .get_mapped_segment_by_address(pos)
            .expect("failed to get mapped memory region for address");

        assert!(region.can_read);

        &region.data[pos - region.start]
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
        let region = self
            .get_mapped_segment_by_address(pos)
            .expect("failed to get mapped memory region for address");

        assert!(region.can_read);

        region.data[pos - region.start]
    }

    /// Attempt to read an instruction from memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The starting position of the instruction in memory.
    ///
    /// # Returns
    ///
    /// An [`Instruction`] instance, if the memory address contains a valid instruction.
    pub fn get_instruction(&self, pos: usize) -> Instruction {
        let opcode_id = self.get_u32(pos);

        // Validate the opcode is one of the ones we know about.
        // In the case we encounter an unrecognized opcode ID then we will
        // default to the Unknown opcode, which is useful for debugging.
        let opcode = FromPrimitive::from_u32(opcode_id).unwrap_or_default();

        // Calculate the length of the arguments, in bytes.
        let arg_len = Instruction::get_instruction_arg_size_from_op(opcode);
        let arg_bytes = self.get_range_ptr(pos + 4, arg_len);

        // Create a memory block reader.
        let mut block = MemoryBlockReader::new(arg_bytes);

        // Create our instruction instance.
        match opcode {
            OpCode::Nop => Instruction::Nop,

            /******** [Arithmetic Instructions] ********/
            OpCode::AddU32ImmU32Reg => {
                let imm = block.read_u32();
                let reg = block.read_register_id();

                Instruction::AddU32ImmU32Reg(imm, reg)
            }
            OpCode::AddU32RegU32Reg => {
                let reg_1 = block.read_register_id();
                let reg_2 = block.read_register_id();

                Instruction::AddU32RegU32Reg(reg_1, reg_2)
            }
            OpCode::SubU32ImmU32Reg => {
                let imm = block.read_u32();
                let reg = block.read_register_id();

                Instruction::SubU32ImmU32Reg(imm, reg)
            }
            OpCode::SubU32RegU32Imm => {
                let reg = block.read_register_id();
                let imm = block.read_u32();

                Instruction::SubU32RegU32Imm(reg, imm)
            }
            OpCode::SubU32RegU32Reg => {
                let reg_1 = block.read_register_id();
                let reg_2 = block.read_register_id();

                Instruction::SubU32RegU32Reg(reg_1, reg_2)
            }
            OpCode::MulU32ImmU32Reg => {
                let imm = block.read_u32();
                let reg = block.read_register_id();

                Instruction::MulU32ImmU32Reg(imm, reg)
            }
            OpCode::MulU32RegU32Reg => {
                let reg_1 = block.read_register_id();
                let reg_2 = block.read_register_id();

                Instruction::MulU32RegU32Reg(reg_1, reg_2)
            }
            OpCode::DivU32ImmU32Reg => {
                let imm = block.read_u32();
                let reg = block.read_register_id();

                Instruction::DivU32ImmU32Reg(imm, reg)
            }
            OpCode::DivU32RegU32Imm => {
                let reg = block.read_register_id();
                let imm = block.read_u32();

                Instruction::DivU32RegU32Imm(reg, imm)
            }
            OpCode::DivU32RegU32Reg => {
                let reg_1 = block.read_register_id();
                let reg_2 = block.read_register_id();

                Instruction::DivU32RegU32Reg(reg_1, reg_2)
            }
            OpCode::ModU32ImmU32Reg => {
                let imm = block.read_u32();
                let reg = block.read_register_id();

                Instruction::ModU32ImmU32Reg(imm, reg)
            }
            OpCode::ModU32RegU32Imm => {
                let reg = block.read_register_id();
                let imm = block.read_u32();

                Instruction::ModU32RegU32Imm(reg, imm)
            }
            OpCode::ModU32RegU32Reg => {
                let reg_1 = block.read_register_id();
                let reg_2 = block.read_register_id();

                Instruction::ModU32RegU32Reg(reg_1, reg_2)
            }
            OpCode::IncU32Reg => {
                let reg = block.read_register_id();

                Instruction::IncU32Reg(reg)
            }
            OpCode::DecU32Reg => {
                let reg = block.read_register_id();

                Instruction::DecU32Reg(reg)
            }
            OpCode::AndU32ImmU32Reg => {
                let imm = block.read_u32();
                let reg = block.read_register_id();

                Instruction::AndU32ImmU32Reg(imm, reg)
            }

            /******** [Bit Operation Instructions] ********/
            OpCode::LeftShiftU32ImmU32Reg => {
                let imm = block.read_u32();
                let reg = block.read_register_id();

                Instruction::LeftShiftU32ImmU32Reg(imm, reg)
            }
            OpCode::LeftShiftU32RegU32Reg => {
                let shift_reg = block.read_register_id();
                let reg = block.read_register_id();

                Instruction::LeftShiftU32RegU32Reg(shift_reg, reg)
            }
            OpCode::ArithLeftShiftU32ImmU32Reg => {
                let imm = block.read_u32();
                let reg = block.read_register_id();

                Instruction::ArithLeftShiftU32ImmU32Reg(imm, reg)
            }
            OpCode::ArithLeftShiftU32RegU32Reg => {
                let shift_reg = block.read_register_id();
                let reg = block.read_register_id();

                Instruction::ArithLeftShiftU32RegU32Reg(shift_reg, reg)
            }
            OpCode::RightShiftU32ImmU32Reg => {
                let imm = block.read_u32();
                let reg = block.read_register_id();

                Instruction::RightShiftU32ImmU32Reg(imm, reg)
            }
            OpCode::RightShiftU32RegU32Reg => {
                let shift_reg = block.read_register_id();
                let reg = block.read_register_id();

                Instruction::RightShiftU32RegU32Reg(shift_reg, reg)
            }
            OpCode::ArithRightShiftU32ImmU32Reg => {
                let imm = block.read_u32();
                let reg = block.read_register_id();

                Instruction::ArithRightShiftU32ImmU32Reg(imm, reg)
            }
            OpCode::ArithRightShiftU32RegU32Reg => {
                let shift_reg = block.read_register_id();
                let reg = block.read_register_id();

                Instruction::ArithRightShiftU32RegU32Reg(shift_reg, reg)
            }

            /******** [Branching Instructions] ********/
            OpCode::Int => {
                let addr = block.read_u32();

                Instruction::Int(addr)
            }
            OpCode::IntRet => Instruction::IntRet,

            /******** [Data Instructions] ********/
            OpCode::SwapU32RegU32Reg => {
                let reg1 = block.read_register_id();
                let reg2 = block.read_register_id();

                Instruction::SwapU32RegU32Reg(reg1, reg2)
            }
            OpCode::MovU32ImmU32Reg => {
                let imm = block.read_u32();
                let reg = block.read_register_id();

                Instruction::MovU32ImmU32Reg(imm, reg)
            }
            OpCode::MovU32RegU32Reg => {
                let in_reg = block.read_register_id();
                let out_reg = block.read_register_id();

                Instruction::MovU32RegU32Reg(in_reg, out_reg)
            }
            OpCode::MovU32ImmMemSimple => {
                let imm = block.read_u32();
                let addr = block.read_u32();

                Instruction::MovU32ImmMemSimple(imm, addr)
            }
            OpCode::MovU32RegMemSimple => {
                let reg = block.read_register_id();
                let addr = block.read_u32();

                Instruction::MovU32RegMemSimple(reg, addr)
            }
            OpCode::MovMemU32RegSimple => {
                let addr = block.read_u32();
                let reg = block.read_register_id();

                Instruction::MovMemU32RegSimple(addr, reg)
            }
            OpCode::MovU32RegPtrU32RegSimple => {
                let in_reg = block.read_register_id();
                let out_reg = block.read_register_id();

                Instruction::MovU32RegPtrU32RegSimple(in_reg, out_reg)
            }
            OpCode::MovU32ImmMemExpr => {
                let imm = block.read_u32();
                let expr = block.read_u32();

                Instruction::MovU32ImmMemExpr(imm, expr)
            }
            OpCode::MovMemExprU32Reg => {
                let expr = block.read_u32();
                let reg = block.read_register_id();

                Instruction::MovMemExprU32Reg(expr, reg)
            }
            OpCode::MovU32RegMemExpr => {
                let reg = block.read_register_id();
                let expr = block.read_u32();

                Instruction::MovU32RegMemExpr(reg, expr)
            }
            OpCode::ByteSwapU32 => {
                let reg = block.read_register_id();

                Instruction::ByteSwapU32(reg)
            }
            OpCode::ZeroHighBitsByIndexU32Reg => {
                let index_reg = block.read_register_id();
                let in_reg = block.read_register_id();
                let out_reg = block.read_register_id();

                Instruction::ZeroHighBitsByIndexU32Reg(index_reg, in_reg, out_reg)
            }
            OpCode::ZeroHighBitsByIndexU32RegU32Imm => {
                let index = block.read_u32();
                let in_reg = block.read_register_id();
                let out_reg = block.read_register_id();

                Instruction::ZeroHighBitsByIndexU32RegU32Imm(index, in_reg, out_reg)
            }
            OpCode::PushU32Imm => {
                let imm = block.read_u32();

                Instruction::PushU32Imm(imm)
            }

            /******** [Logic Instructions] ********/
            OpCode::BitTestU32Reg => {
                let bit = block.read_u8();
                let reg = block.read_register_id();

                Instruction::BitTestU32Reg(bit, reg)
            }
            OpCode::BitTestU32Mem => {
                let bit = block.read_u8();
                let addr = block.read_u32();

                Instruction::BitTestU32Mem(bit, addr)
            }
            OpCode::BitTestResetU32Reg => {
                let bit = block.read_u8();
                let reg = block.read_register_id();

                Instruction::BitTestResetU32Reg(bit, reg)
            }
            OpCode::BitTestResetU32Mem => {
                let bit = block.read_u8();
                let addr = block.read_u32();

                Instruction::BitTestResetU32Mem(bit, addr)
            }
            OpCode::BitTestSetU32Reg => {
                let bit = block.read_u8();
                let reg = block.read_register_id();

                Instruction::BitTestSetU32Reg(bit, reg)
            }
            OpCode::BitTestSetU32Mem => {
                let bit = block.read_u8();
                let addr = block.read_u32();

                Instruction::BitTestSetU32Mem(bit, addr)
            }
            OpCode::BitScanReverseU32RegU32Reg => {
                let in_reg = block.read_register_id();
                let out_reg = block.read_register_id();

                Instruction::BitScanReverseU32RegU32Reg(in_reg, out_reg)
            }
            OpCode::BitScanReverseU32MemU32Reg => {
                let addr = block.read_u32();
                let reg = block.read_register_id();

                Instruction::BitScanReverseU32MemU32Reg(addr, reg)
            }
            OpCode::BitScanReverseU32RegMemU32 => {
                let reg = block.read_register_id();
                let out_addr = block.read_u32();

                Instruction::BitScanReverseU32RegMemU32(reg, out_addr)
            }
            OpCode::BitScanReverseU32MemU32Mem => {
                let in_addr = block.read_u32();
                let out_addr = block.read_u32();

                Instruction::BitScanReverseU32MemU32Mem(in_addr, out_addr)
            }
            OpCode::BitScanForwardU32RegU32Reg => {
                let in_reg = block.read_register_id();
                let out_reg = block.read_register_id();

                Instruction::BitScanForwardU32RegU32Reg(in_reg, out_reg)
            }
            OpCode::BitScanForwardU32MemU32Reg => {
                let addr = block.read_u32();
                let reg = block.read_register_id();

                Instruction::BitScanForwardU32MemU32Reg(addr, reg)
            }
            OpCode::BitScanForwardU32RegMemU32 => {
                let reg = block.read_register_id();
                let out_addr = block.read_u32();

                Instruction::BitScanForwardU32RegMemU32(reg, out_addr)
            }
            OpCode::BitScanForwardU32MemU32Mem => {
                let in_addr = block.read_u32();
                let out_addr = block.read_u32();

                Instruction::BitScanForwardU32MemU32Mem(in_addr, out_addr)
            }

            /******** [Special Instructions] ********/
            OpCode::Ret => Instruction::Ret,
            OpCode::Mret => Instruction::Mret,
            OpCode::Hlt => Instruction::Hlt,
            OpCode::Unknown => Instruction::Unknown(opcode_id),
        }
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
    pub fn get_mapped_segment_by_address(&self, pos: usize) -> Option<&MappedMemory> {
        self.mapped_memory.iter().find(|e| e.contains_point(pos))
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
            .find(|e| e.contains_point(pos))
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
        self.get_mapped_segment_by_name(RAM_REGION_NAME)
            .expect("failed to get physical RAM memory segment")
    }

    /// Get a mutable reference to the physical RAM memory segment.
    #[inline(always)]
    pub fn get_physical_memory_segment_mut(&mut self) -> &mut MappedMemory {
        self.get_mapped_segment_by_name_mut(RAM_REGION_NAME)
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
    /// * `start` - The starting position of the bytes in memory.
    /// * `len` - The number of bytes to read from memory.
    ///
    /// # Returns
    ///
    /// A reference to a u8 slice from memory.
    #[inline(always)]
    pub fn get_range_ptr(&self, start: usize, len: usize) -> &[u8] {
        let region = self
            .get_mapped_segment_by_address(start)
            .expect("failed to get mapped memory region for address");

        assert!(region.can_read);

        // Does this region completely contain the address range?
        let absolute_end = start + len;
        assert!(region.contains_range(start, absolute_end));

        // Translate the absolute address into the absolute local variant.
        &region.data[(start - region.start)..(absolute_end - region.start)]
    }

    /// Attempt to clone a range of bytes from memory.
    ///
    /// # Arguments
    ///
    /// * `start` - The starting position of the bytes in memory.
    /// * `len` - The number of bytes to read from memory.
    ///
    /// # Returns
    ///
    /// A vector containing the bytes cloned from memory.
    pub fn get_range_clone(&self, start: usize, len: usize) -> Vec<u8> {
        self.get_range_ptr(start, len).to_vec()
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

    /// Checks whether the allocated memory region has a size greater than 0.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Get the size of the emulated physical memory.
    ///
    /// # Returns
    ///
    /// The length of the physical memory region, in bytes.
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

    /// Set the value of a specific byte in memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The point in memory to be checked.
    /// * `value` - The value to be written into memory.
    pub fn set(&mut self, pos: usize, value: u8) {
        let region = self
            .get_mapped_segment_by_address_mut(pos)
            .expect("failed to get mapped memory region for address");

        assert!(region.can_write);

        region.data[pos - region.start] = value;
    }

    /// Set the value of a range of values in memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The absolute position of the first byte to be written into memory.
    /// * `values` - A slice of u8 values that are to be written into memory.
    #[inline(always)]
    pub fn set_range(&mut self, pos: usize, values: &[u8]) {
        let region = self
            .get_mapped_segment_by_address_mut(pos)
            .expect("failed to get mapped memory region for address");

        assert!(region.can_write);

        // Does this region completely contain the address range?
        let absolute_end = pos + values.len();
        assert!(region.contains_range(pos, absolute_end));

        // Translate the absolute address into the absolute local variant.
        // This is safe since we have checked that the range is completely
        // valid relative to the mapped memory region.
        for (old, new) in region.data[(pos - region.start)..(absolute_end - region.start)]
            .iter_mut()
            .zip(values.iter())
        {
            *old = *new;
        }
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

    /// Print a list of the mapped memory regions.
    pub fn print_mapped_memory_regions(&self) {
        let mut table = Table::new();

        table.add_row(row!["ID", "Name", "Start", "End"]);

        for (i, region) in self.mapped_memory.iter().enumerate() {
            table.add_row(row![
                i,
                region.name,
                format!("{}", region.start),
                format!("{}", region.end)
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

    /// Test adding and working with mapped memory regions.
    #[test]
    fn test_adding_mapped_memory_region() {
        let mut ram = MemoryHandler::new(100, &[], &[], 0);

        // Well past the boot memory region.
        let start = MEGABYTE * 512;
        let length = 100;

        // Add the region.
        ram.add_mapped_memory_segment(start, length, true, true, "tester");
        assert_eq!(ram.mapped_memory.len(), 2);

        // Next, testing we can read and write to the mapped memory region.
        ram.set_u32(start, 0xDEADBEEF);

        // Next, test that we can read the value back from the mapped memory region.
        assert_eq!(ram.get_u32(start), 0xDEADBEEF);
    }

    /// Test adding an overlapping memory region panics.
    #[test]
    #[should_panic]
    fn test_adding_mapped_memory_region_overlapping() {
        let mut ram = MemoryHandler::new(100, &[], &[], 0);

        // Well past the boot memory region.
        let start = MEGABYTE * 512;
        let length = 100;

        // Add the region.
        ram.add_mapped_memory_segment(start, length, true, true, "tester");
        assert_eq!(ram.mapped_memory.len(), 3);

        // Attempt to add another region that intersects with the one we previously added.
        ram.add_mapped_memory_segment(start + 1, length, true, true, "tester 2");
    }

    /// Test reading from a mapped memory region that doesn't support it.
    #[test]
    #[should_panic]
    fn test_mapped_memory_region_no_reading() {
        let mut ram = MemoryHandler::new(100, &[], &[], 0);

        // Well past the boot memory region.
        let start = MEGABYTE * 512;
        let length = 100;

        // Add the region.
        ram.add_mapped_memory_segment(start, length, false, true, "tester");
        assert_eq!(ram.mapped_memory.len(), 3);

        // Can we read from the memory region?
        let _ = ram.get_u32(start);
    }

    /// Test writing to a mapped memory region that doesn't support it.
    #[test]
    #[should_panic]
    fn test_mapped_memory_region_no_writing() {
        let mut ram = MemoryHandler::new(100, &[], &[], 0);

        // Well past the boot memory region.
        let start = MEGABYTE * 512;
        let length = 100;

        // Add the region.
        ram.add_mapped_memory_segment(start, length, true, false, "tester");
        assert_eq!(ram.mapped_memory.len(), 3);

        // Can we read from the memory region?
        ram.set_u32(start, 0xDEAFBEEF);
    }

    /// Test attempting to read beyond the bounds of a mapped memory region.
    #[test]
    #[should_panic]
    fn test_mapped_memory_region_read_past_bounds() {
        let mut ram = MemoryHandler::new(100, &[], &[], 0);

        // Well past the boot memory region.
        let start = MEGABYTE * 512;
        let length = 2;

        // Add the region.
        ram.add_mapped_memory_segment(start, length, true, true, "tester");
        assert_eq!(ram.mapped_memory.len(), 3);

        // Can we read from the memory region?
        let _ = ram.get_u32(start);
    }

    /// Test attempting to write beyond the bounds of a mapped memory region.
    #[test]
    #[should_panic]
    fn test_mapped_memory_region_write_past_bounds() {
        let mut ram = MemoryHandler::new(100, &[], &[], 0);

        // Well past the boot memory region.
        let start = MEGABYTE * 512;
        let length = 2;

        // Add the region.
        ram.add_mapped_memory_segment(start, length, true, true, "tester");
        assert_eq!(ram.mapped_memory.len(), 3);

        // Can we read from the memory region?
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
    fn test_read_no_mapped_region() {
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
            *ram.get_byte_ptr(0),
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
