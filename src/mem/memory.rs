use num_traits::FromPrimitive;
use prettytable::{row, Table};

use crate::ins::{instruction::Instruction, op_codes::OpCode};

use super::memory_block_reader::MemoryBlockReader;

enum StackTypeHint {
    F32,
    U32,
    U8,
}

pub struct Memory {
    /// The raw byte storage of this memory module.
    storage: Vec<u8>,

    /// A list of type hints for the items currently on the stack.
    stack_type_hints: Vec<StackTypeHint>,
    /// The number of u32 items that the stack can hold.
    stack_capacity: usize,
    /// The size of the stack, in bytes.
    stack_size: usize,
    /// The position that represents the start of the stack memory region.
    stack_start: usize,
    /// The position that represents the end of the stack memory region.
    stack_end: usize,
}

impl Memory {
    pub fn new(size: usize) -> Self {
        Self {
            storage: vec![0x0; size],

            stack_type_hints: vec![],
            stack_capacity: 0,
            stack_size: 0,
            stack_start: 0,
            stack_end: 0,
        }
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

    /// Completely clear the memory.
    pub fn clear(&mut self) {
        self.storage = vec![0; self.storage.len()];
    }

    /// Configure the stack memory region.
    ///
    /// # Arguments
    ///
    /// * `stack_start` - The starting point for the stack memory region.
    /// * `stack_capacity` - The number of items that can be stored in stack memory.
    ///
    /// # Returns
    ///
    /// A usize giving the end location of the stack memory region.
    pub fn configure_stack(&mut self, stack_start_pos: usize, stack_capacity: usize) -> usize {
        // The size of the stack is calculated to contain enough space
        // for a given number of u32 values (at 4 bytes each).
        let stack_size = stack_capacity * std::mem::size_of::<u32>();
        let stack_end = stack_start_pos + stack_size;
        assert!(stack_end < self.len());

        // Initialize the relevant stack fields.
        self.stack_type_hints = Vec::with_capacity(stack_capacity);
        self.stack_capacity = stack_capacity;
        self.stack_size = stack_size;
        self.stack_start = stack_start_pos;
        self.stack_end = stack_end;

        stack_end
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

                Instruction::MovU32ImmU32Reg(imm, reg)
            }
            OpCode::MovU32RegU32Reg => {
                let in_reg = block.read_register_id();
                let out_reg = block.read_register_id();

                Instruction::MovU32RegU32Reg(in_reg, out_reg)
            }
            OpCode::MovU32ImmMemRelSimple => {
                let imm = block.read_u32();
                let addr = block.read_u32();

                Instruction::MovU32ImmMemRelSimple(imm, addr)
            }
            OpCode::MovU32RegMemRelSimple => {
                let reg = block.read_register_id();
                let addr = block.read_u32();

                Instruction::MovU32RegMemRelSimple(reg, addr)
            }
            OpCode::MovMemU32RegRelSimple => {
                let addr = block.read_u32();
                let reg = block.read_register_id();

                Instruction::MovMemU32RegRelSimple(addr, reg)
            }
            OpCode::MovU32RegPtrU32RegRelSimple => {
                let in_reg = block.read_register_id();
                let out_reg = block.read_register_id();

                Instruction::MovU32RegPtrU32RegRelSimple(in_reg, out_reg)
            }
            OpCode::MovU32ImmMemExprRel => {
                let imm = block.read_u32();
                let expr = block.read_u32();

                Instruction::MovU32ImmMemExprRel(imm, expr)
            }
            OpCode::MovMemExprU32RegRel => {
                let expr = block.read_u32();
                let reg = block.read_register_id();

                Instruction::MovMemExprU32RegRel(expr, reg)
            }
            OpCode::MovU32RegMemExprRel => {
                let reg = block.read_register_id();
                let expr = block.read_u32();

                Instruction::MovU32RegMemExprRel(reg, expr)
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

    /// Get a reference to a slice of the entire memory contents.
    ///
    /// # Returns
    ///
    /// A slice of u8 values, referencing every byte in memory.
    pub fn get_storage(&self) -> &[u8] {
        &self.storage
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

    /// Attempt to pop a u32 value from the stack.
    pub fn pop_u32(&mut self) -> u32 {
        0
    }

    /// Attempt to push a u32 value onto the stack.
    ///
    /// # Arguments
    ///
    /// * `value` - The u32 value to be pushed onto the stack.
    pub fn push_u32(&mut self, value: u32, stack_pointer: usize) {
        let value_start_pos = stack_pointer - 4;
        assert!(value_start_pos <= self.stack_end);

        self.set_u32(value_start_pos, value);
        self.pop_type_hint(StackTypeHint::U32);

        println!("WARNING: be sure to update the stack pointer and stack frame size registers!");
    }

    #[cfg(feature = "stack-type-hints")]
    fn pop_type_hint(&mut self, hint: StackTypeHint) {
        self.stack_type_hints.push(hint);
    }

    #[cfg(not(feature = "stack-type-hints"))]
    fn pop_type_hint(&mut self, _hint: StackTypeHint) {}

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
            table.add_row(row!["Index", "Value", "Type"]);

            let mut stack_pos = self.stack_end;
            //println!("stack_start = {}", stack_pos);
            //println!("AAAAAAAAAAA {}", self.get_u32(stack_pos - 4));

            for (id, hint) in self.stack_type_hints.iter().enumerate() {
                match *hint {
                    StackTypeHint::F32 => todo!(),
                    StackTypeHint::U32 => {
                        let value = self.get_u32(stack_pos - 4);
                        table.add_row(row![id, format!("{value}"), "u32"]);
                        stack_pos -= 4;
                    }
                    StackTypeHint::U8 => todo!(),
                }
            }
        } else {
            println!("WARNING: stack type hints are disabled, all outputs will be assumed to be u32 values.");

            table.add_row(row!["Index", "Value"]);
        }

        /*for (id, reg) in &self.registers_u32 {
            let reg_value = *reg.read_unchecked();
            let formatted_value = format!("{reg_value:0>8X}");
            let mut notes = String::new();

            if *id == RegisterId::FL {
                notes = reg.get_flags_register_string();
            }

            table.add_row(row![id, formatted_value, "u32", notes]);
        }

        for (id, reg) in &self.registers_f32 {
            table.add_row(row![id, format!("{}", reg.read_unchecked()), "f32", ""]);
        }*/

        table.printstd();
    }
}

impl From<&[u8]> for Memory {
    fn from(values: &[u8]) -> Self {
        let mut mem = Memory::new(values.len());
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
        let mut ram = Memory::new(100);

        // Fill the memory with debug data.
        fill_memory_sequential(&mut ram);

        ram
    }

    /// Test reading above RAM bounds.
    #[test]
    #[should_panic]
    fn test_overread_ram() {
        let size = 100;
        let ram = Memory::new(size);

        _ = ram.get_range_ptr(100, 1);
    }

    /// Test the basic aspects of creating a RAM module.
    #[test]
    fn test_ram_creation() {
        let size = 100;
        let ram = Memory::new(size);

        assert_eq!(
            ram.len(),
            size,
            "failed to create a RAM module of the specified size"
        );

        assert_eq!(
            ram.get_range_ptr(0, size - 1),
            vec![0x0; size - 1],
            "failed to correctly initialize RAM"
        );
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
}
