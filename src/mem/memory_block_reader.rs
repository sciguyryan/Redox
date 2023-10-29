use num_traits::FromPrimitive;

use crate::reg::registers::RegisterId;

pub struct MemoryBlockReader<'a> {
    cursor: usize,
    data: &'a [u8],
}

impl<'a> MemoryBlockReader<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self { cursor: 0, data }
    }

    pub fn read_register_id(&mut self) -> RegisterId {
        FromPrimitive::from_u8(*self.read_u8_internal())
            .expect("failed to read register ID from memory")
    }

    fn read_u8_internal(&mut self) -> &u8 {
        let pos = self.cursor;
        self.cursor += 1;
        &self.data[pos]
    }

    pub fn read_u8(&mut self) -> u8 {
        *self.read_u8_internal()
    }

    pub fn read_u32(&mut self) -> u32 {
        // This will assert if the value is out of range anyway.
        let bytes: [u8; 4] = self.data[self.cursor..self.cursor + 4]
            .try_into()
            .expect("failed to read u32 value from memory");
        self.cursor += 4;

        u32::from_le_bytes(bytes)
    }
}
