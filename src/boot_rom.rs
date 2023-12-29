use crate::{impl_mem_interface, mem::memory::MEGABYTE};

/// The start index of the mapped memory region.
pub const MEM_MAP_START: usize = 0x12_C00_000; // Starting at the 300 megabyte region.
/// The end index of the mapped memory region.
pub const SIZE: usize = MEGABYTE; // Extending for 1 megabyte.
/// The name of the memory region.
pub const MEM_MAP_NAME: String = String::from("boot");

// TODO - build a compiled boot region and set the instruction pointer to it.
// TODO - This can hold all of the initial setup code for the CPU, replacing the stuff that is
// TODO - currently setup in registers.rs, etc.

pub struct BootROM {
    pub mem_map_start: usize,
    pub mem_map_end: usize,
    pub mem_map_name: String,
    pub memory: Vec<u8>,
}

impl BootROM {
    pub fn new() -> Self {
        Self {
            mem_map_start: MEM_MAP_START,
            mem_map_end: MEM_MAP_START + SIZE,
            mem_map_name: MEM_MAP_NAME.to_string(),
            memory: vec![0x0; SIZE],
        }
    }
}

impl_mem_interface!(BootROM, true, false);
