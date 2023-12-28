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
    /// * `pos` - The starting position of the instruction in memory.
    ///
    /// # Returns
    ///
    /// An [`Instruction`] instance, if the memory address contains a valid instruction.
    fn get_instruction(&self, pos: usize) -> Instruction;

    /// Attempt to read a u32 value from memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The starting position of the bytes in memory.
    ///
    /// # Returns
    ///
    /// A u32 from the specified memory address.
    fn get_u32(&self, pos: usize) -> u32;

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
    fn get_range_clone(&self, start: usize, len: usize) -> Vec<u8> {
        self.get_range_ptr(start, len).to_vec()
    }

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
    /// * `pos` - The absolute position of the first byte to be written into memory.
    /// * `values` - A slice of u8 values that are to be written into memory.
    fn set_range(&mut self, pos: usize, values: &[u8]);

    /// Write a u32 value into memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The position of the first byte to be written into memory.
    /// * `value` - The value to be written into memory.
    fn set_u32(&mut self, pos: usize, value: u32);
}

#[macro_export]
// Thanks to AlphaKeks on r/rust for the suggestion!
macro_rules! impl_mem_interface {
    ( $( $type:ty, $can_read:expr, $can_write:expr ),+) => {
        $(
        use $crate::ins::instruction::Instruction;
        use $crate::mem::memory_interface::MemoryInterface;

        impl MemoryInterface for $type {
            fn can_read() -> bool {
                $can_read
            }

            fn can_write() -> bool {
                $can_write
            }

            fn get_byte_ptr(&self, pos: usize) -> &u8 {
                assert!($can_read, "reading is not implemented for this device");

                &self.memory[pos]
            }

            fn get_byte_clone(&self, pos: usize) -> u8 {
                *self.get_byte_ptr(pos)
            }

            fn get_instruction(&self, pos: usize) -> Instruction {
                todo!();
            }

            fn get_u32(&self, pos: usize) -> u32 {
                todo!();
            }

            fn get_range_ptr(&self, start: usize, len: usize) -> &[u8] {
                todo!();
            }

            fn len(&self) -> usize {
                self.memory.len()
            }

            fn set(&mut self, pos: usize, value: u8) {
                todo!();
            }

            fn set_range(&mut self, pos: usize, values: &[u8]) {
                todo!();
            }

            fn set_u32(&mut self, pos: usize, value: u32) {
                todo!();
            }
        }
        )+
    };
}
