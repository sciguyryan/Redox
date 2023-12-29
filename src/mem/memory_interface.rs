use crate::ins::instruction::Instruction;

pub trait MemoryInterface {
    /// Does this device support reading?
    fn can_read() -> bool;

    /// Does this device support writing?
    fn can_write() -> bool;

    /// Attempt to get a pointer to a byte in memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The position of the byte in memory.
    ///
    /// # Returns
    ///
    /// A pointer to the specific byte in memory.
    fn get_byte_ptr(&self, pos: usize) -> &u8;

    /// Attempt to read and clone a byte from memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The position of the byte in memory.
    ///
    /// # Returns
    ///
    /// A clone of the specific byte from memory.
    fn get_byte_clone(&self, pos: usize) -> u8;

    /// Attempt to read an instruction from memory.
    ///
    /// # Arguments
    ///
    /// * `start` - The starting position of the instruction in memory.
    ///
    /// # Returns
    ///
    /// An [`Instruction`] instance, if the memory address contains a valid instruction.
    fn get_instruction(&self, start: usize) -> Instruction;

    /// Attempt to read a u32 value from memory.
    ///
    /// # Arguments
    ///
    /// * `start` - The starting position of the bytes in memory.
    ///
    /// # Returns
    ///
    /// A u32 from the specified memory address.
    fn get_u32(&self, start: usize) -> u32;

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
    fn get_range_ptr(&self, start: usize, len: usize) -> &[u8];

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
    fn get_range_clone(&self, start: usize, len: usize) -> Vec<u8>;

    /// The length of this memory region.
    fn len(&self) -> usize;

    /// Set the value of a specific byte in memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The point in memory to be checked.
    /// * `value` - The value to be written into memory.
    fn set(&mut self, pos: usize, value: u8);

    /// Set the value of a range of values in memory.
    ///
    /// # Arguments
    ///
    /// * `start` - The position of the first byte to be written into memory.
    /// * `values` - A slice of u8 values that are to be written into memory.
    fn set_range(&mut self, start: usize, values: &[u8]);

    /// Write a u32 value into memory.
    ///
    /// # Arguments
    ///
    /// * `start` - The position of the first byte to be written into memory.
    /// * `value` - The value to be written into memory.
    fn set_u32(&mut self, start: usize, value: u32);

    fn get_map_start(&self) -> usize;
    fn get_map_end(&self) -> usize;
    fn get_name(&self) -> String;
}

// Thanks to AlphaKeks on r/rust for the suggestion!
#[macro_export]
macro_rules! impl_mem_interface {
    ( $( $type:ty, $can_read:expr, $can_write:expr ),+) => {
        $(
        use $crate::ins::instruction::Instruction;
        use $crate::mem::memory_block_reader::MemoryBlockReader;
        use $crate::mem::memory_interface::MemoryInterface;
        use $crate::ins::op_codes::OpCode;

        use num_traits::FromPrimitive;

        impl MemoryInterface for $type {
            fn can_read() -> bool {
                $can_read
            }

            fn can_write() -> bool {
                $can_write
            }

            fn get_map_start(&self) -> usize {
                self.mem_map_start
            }

            fn get_map_end(&self) -> usize {
                self.mem_map_end
            }

            fn get_name(&self) -> String {
                self.mem_map_name
            }

            #[inline(always)]
            fn get_byte_ptr(&self, pos: usize) -> &u8 {
                assert!($can_read, "reading is not implemented for this device");

                &self.memory[pos]
            }

            #[inline(always)]
            fn get_byte_clone(&self, pos: usize) -> u8 {
                *self.get_byte_ptr(pos)
            }

            #[inline(always)]
            fn get_instruction(&self, pos: usize) -> Instruction {
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

            #[inline(always)]
            fn get_u32(&self, pos: usize) -> u32 {
                let bytes: [u8; 4] = self
                    .get_range_ptr(pos, 4)
                    .try_into()
                    .expect("failed to create a u32 from memory bytes");

                u32::from_le_bytes(bytes)
            }

            #[inline(always)]
            fn get_range_ptr(&self, start: usize, len: usize) -> &[u8] {
                let end = start + len;
                assert!($can_read && end < self.len());

                &self.memory[start..(start + len)]
            }

            #[inline(always)]
            fn get_range_clone(&self, start: usize, len: usize) -> Vec<u8> {
                self.get_range_ptr(start, len).to_vec()
            }

            #[inline(always)]
            fn len(&self) -> usize {
                self.memory.len()
            }

            #[inline(always)]
            fn set(&mut self, pos: usize, value: u8) {
                assert!($can_write);

                self.memory[pos] = value;
            }

            #[inline(always)]
            fn set_range(&mut self, start: usize, values: &[u8]) {
                let end = start + values.len();
                assert!($can_write && end < self.len());

                // Translate the absolute address into the absolute local variant.
                // This is safe since we have checked that the range is completely
                // valid relative to the mapped memory region.
                for (old, new) in self.memory[start..end]
                    .iter_mut()
                    .zip(values.iter())
                {
                    *old = *new;
                }
            }

            #[inline(always)]
            fn set_u32(&mut self, start: usize, value: u32) {
                self.set_range(start, &u32::to_le_bytes(value));
            }
        }
        )+
    };
}
