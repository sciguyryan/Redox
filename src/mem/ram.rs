use crate::impl_mem_interface;

/// The start index of the mapped memory region.
pub const MEM_MAP_START: usize = 0;
/// The name of the memory region.
pub const MEM_MAP_NAME: &str = "ram";

pub struct Ram {
    pub mem_map_start: usize,
    pub mem_map_end: usize,
    pub mem_map_name: String,
    pub memory: Vec<u8>,
}

impl Ram {
    pub fn new(length: usize) -> Self {
        Self {
            mem_map_start: MEM_MAP_START,
            mem_map_end: MEM_MAP_START + length,
            mem_map_name: MEM_MAP_NAME.to_string(),
            memory: vec![0x0; length],
        }
    }
}

impl_mem_interface!(Ram, true, true);