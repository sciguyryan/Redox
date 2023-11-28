use num_traits::FromPrimitive;

use crate::ins::{instruction::Instruction, op_codes::OpCode};

use super::memory_block_reader::MemoryBlockReader;

pub struct Memory {
    /// The raw byte storage of this memory module.
    storage: Vec<u8>,
}

impl Memory {
    pub fn new(size: usize) -> Self {
        Self {
            storage: vec![0x0; size],
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

    pub fn get_byte_ptr(&self, pos: usize) -> &u8 {
        self.assert_point_in_bounds(pos);

        &self.storage[pos]
    }

    pub fn get_byte_clone(&self, pos: usize) -> u8 {
        assert!(self.is_in_bounds(pos));

        self.storage[pos]
    }

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
            OpCode::IncU32Reg => {
                let reg = block.read_register_id();

                Instruction::IncU32Reg(reg)
            }
            OpCode::DecU32Reg => {
                let reg = block.read_register_id();

                Instruction::DecU32Reg(reg)
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

    pub fn get_u32(&self, pos: usize) -> u32 {
        let bytes: [u8; 4] = self
            .get_range_ptr(pos, 4)
            .try_into()
            .expect("failed to create a u32 from memory bytes");

        u32::from_le_bytes(bytes)
    }

    pub fn get_range_ptr(&self, start: usize, len: usize) -> &[u8] {
        let end = start + len;
        self.assert_point_in_bounds(start);
        self.assert_point_in_bounds(end);

        &self.storage[start..end]
    }

    pub fn get_range_clone(&self, start: usize, len: usize) -> Vec<u8> {
        self.get_range_ptr(start, len).to_vec()
    }

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

    pub fn len(&self) -> usize {
        self.storage.len()
    }

    pub fn clear(&mut self) {
        self.storage = vec![0; self.storage.len()];
    }

    pub fn set(&mut self, pos: usize, value: u8) {
        self.assert_point_in_bounds(pos);

        self.storage[pos] = value;
    }

    pub fn set_range(&mut self, start: usize, values: &[u8]) {
        let end = start + values.len();
        self.assert_point_in_bounds(start);
        self.assert_point_in_bounds(end);

        for (i, b) in values.iter().enumerate() {
            self.storage[start + i] = *b;
        }
    }

    pub fn set_u32(&mut self, pos: usize, value: u32) {
        let bytes = u32::to_le_bytes(value);

        self.set_range(pos, &bytes);
    }

    pub fn print(&self) {
        println!("{:?}", self.storage);
    }

    pub fn print_range(&self, start: usize, len: usize) {
        let end = start + len;
        self.assert_point_in_bounds(start);
        self.assert_point_in_bounds(end);

        println!("{:?}", self.storage[start..end].to_vec());
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
