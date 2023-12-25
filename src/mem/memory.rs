use num_traits::FromPrimitive;
use prettytable::{row, Table};

use crate::ins::{instruction::Instruction, op_codes::OpCode};

use super::memory_block_reader::MemoryBlockReader;

/// The size of a 256 megabyte memory block.
const MEM_256_MB: usize = 1028 * 256;

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
 */

pub struct Memory {
    /// The raw byte storage of this memory module.
    storage: Vec<u8>,

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

impl Memory {
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
        assert!(stack_segment_end < MEM_256_MB);

        // Now we have the locations of the memory segments, we can create the memory
        let mut mem = Self {
            storage: vec![0x0; stack_segment_end],
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

    /// Assert that a specific memory position is within the valid memory region.
    ///
    /// # Arguments
    ///
    /// * `pos` - The point in memory to be checked.
    #[inline]
    fn assert_point_in_bounds(&self, pos: usize) {
        assert!(
            self.is_in_bounds(pos),
            "The memory location is outside of the valid memory bounds. Point = {pos}"
        );
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

    /// Completely clear the memory.
    pub fn clear(&mut self) {
        self.storage = vec![0; self.storage.len()];
    }

    /// Attempt to decompile any instructions between a start and end memory location.
    ///
    /// # Arguments
    ///
    /// * `start` - The starting memory location.
    /// * `end` - The ending memory location.
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

            cursor += ins.get_total_instruction_size() as usize;
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
        self.assert_point_in_bounds(pos);

        &self.storage[pos]
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
        assert!(self.is_in_bounds(pos));

        self.storage[pos]
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
        let opcode: OpCode =
            FromPrimitive::from_u32(opcode_id).expect("failed to read valid instruction opcode");

        // calculate the length of the arguments, in bytes.
        let arg_len = Instruction::get_instruction_arg_size_from_op(opcode) as usize;
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

                Instruction::MovU32ImmU32(imm, reg)
            }
            OpCode::MovU32RegU32Reg => {
                let in_reg = block.read_register_id();
                let out_reg = block.read_register_id();

                Instruction::MovU32RegU32(in_reg, out_reg)
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
        }
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
    pub fn get_range_ptr(&self, start: usize, len: usize) -> &[u8] {
        let end = start + len;
        self.assert_point_in_bounds(start);
        self.assert_point_in_bounds(end);

        &self.storage[start..end]
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
        &self.storage[self.code_segment_start..self.code_segment_end]
    }

    /// Get a slice of the data segment contents.
    ///
    /// # Returns
    ///
    /// A slice of u8 values.
    pub fn get_data_segment_storage(&self) -> &[u8] {
        &self.storage[self.data_segment_start..self.data_segment_end]
    }

    /// Get a slice of the raw stack segment contents.
    ///
    /// # Returns
    ///
    /// A slice of u8 values
    pub fn get_stack_segment_storage(&self) -> &[u8] {
        &self.storage[self.stack_segment_start..self.stack_segment_end]
    }

    /// Get a slice of the entire memory contents.
    ///
    /// # Returns
    ///
    /// A slice of u8 values, referencing every byte in memory.
    pub fn get_storage(&self) -> &[u8] {
        &self.storage
    }

    /// Get a slice of the user segment contents.
    ///
    /// # Returns
    ///
    /// A slice of u8 values
    pub fn get_user_segment_storage(&self) -> &[u8] {
        &self.storage[..self.code_segment_start]
    }

    /// Checks whether the allocated memory region has a size greater than 0.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Checks whether a specific memory position is within the valid memory region bounds.
    ///
    /// # Arguments
    ///
    /// * `pos` - The point in memory to be checked.
    ///
    /// # Returns
    ///
    /// True if the memory region is within the valid memory region bounds, false otherwise.
    #[inline]
    fn is_in_bounds(&self, pos: usize) -> bool {
        pos <= self.len()
    }

    /// Get the size of the currently allocated memory.
    ///
    /// # Returns
    ///
    /// The length of the memory, in bytes.
    pub fn len(&self) -> usize {
        self.storage.len()
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
        //assert!(
        //    value_start_pos >= self.stack_segment_start && value_start_pos < self.stack_segment_end
        //);

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
    /// This method will automatically keep track of the stack pointer
    /// as held within the memory object, but the CPU registers **must** be
    /// updated separately or they will fall out of sync.
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
        self.assert_point_in_bounds(pos);

        self.storage[pos] = value;
    }

    /// Set the value of a range of values in memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The position of the first byte to be written into memory.
    /// * `values` - A slice of values that are to be written into memory.
    pub fn set_range(&mut self, start: usize, values: &[u8]) {
        let end = start + values.len();
        self.assert_point_in_bounds(start);
        self.assert_point_in_bounds(end);

        for (i, b) in values.iter().enumerate() {
            self.storage[start + i] = *b;
        }
    }

    /// Write a u32 value into memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The position of the first byte to be written into memory.
    /// * `value` - The value to be written into memory.
    pub fn set_u32(&mut self, pos: usize, value: u32) {
        let bytes = u32::to_le_bytes(value);

        self.set_range(pos, &bytes);
    }

    /// Print te entire contents of memory.
    pub fn print(&self) {
        println!("{:?}", self.storage);
    }

    /// Print a specific range of bytes from memory.
    pub fn print_range(&self, start: usize, len: usize) {
        let end = start + len;
        self.assert_point_in_bounds(start);
        self.assert_point_in_bounds(end);

        println!("{:?}", self.storage[start..end].to_vec());
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

impl From<&[u8]> for Memory {
    fn from(values: &[u8]) -> Self {
        let mut mem = Memory::new(values.len(), &[], &[], 0);
        mem.storage = values.to_vec();

        mem
    }
}

#[cfg(test)]
mod tests_memory {
    use super::Memory;

    fn fill_memory_sequential(mem: &mut Memory) {
        for (i, byte) in &mut mem.storage.iter_mut().enumerate() {
            *byte = (i as u8) % 255;
        }
    }

    fn get_test_ram_instance() -> Memory {
        let mut ram = Memory::new(100, &[], &[], 0);

        // Fill the memory with debug data.
        fill_memory_sequential(&mut ram);

        ram
    }

    /// Test reading beyond RAM bounds.
    #[test]
    #[should_panic]
    fn test_overread_ram() {
        let ram = Memory::new(100, &[], &[], 0);

        _ = ram.get_range_ptr(usize::MAX, 1);
    }

    /// Test the basic aspects of creating a RAM module.
    #[test]
    fn test_ram_creation_simple() {
        let size = 100;
        let ram = Memory::new(size, &[], &[], 0);

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
        let mut ram = Memory::new(user_size, &code, &data, stack_capacity_bytes);

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
        let mut ram = Memory::new(10, &[1, 2, 3, 4], &[9, 8, 7, 6, 5], 2 * 4);

        ram.push_u32(0x123);
        ram.push_u32(0x321);
        assert_eq!(ram.pop_u32(), 0x321);
        assert_eq!(ram.pop_u32(), 0x123);
    }

    /// Test for an assertion when popping an entry from an empty stack.
    #[test]
    #[should_panic]
    fn test_stack_pop_with_empty_stack() {
        let mut ram = Memory::new(100, &[], &[], 2);

        _ = ram.pop_u32();
    }

    /// Test for an assertion when pushing too many entries to the stack.
    #[test]
    #[should_panic]
    fn test_pushing_to_full_stack() {
        let mut ram = Memory::new(100, &[], &[], 1);

        ram.push_u32(0x123);
        ram.push_u32(0x321);
    }
}
