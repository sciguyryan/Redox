use crate::{impl_mem_interface, mem::memory::MEGABYTE};

/// The start index of the boot mapped memory region.
pub const BOOT_MEMORY_START: usize = 0x12_C00_000; // Starting at the 300 megabyte region.
/// The end index of the boot mapped memory region.
pub const BOOT_MEMORY_LENGTH: usize = MEGABYTE; // Extending for 1 megabyte.
/// The name of the boot ROM memory region.
pub const BOOT_NAME: &str = "boot";
/// The ID of the boot ROM mapped memory region.
pub const BOOT_MEMORY_ID: usize = 1;

// TODO - build a compiled boot region and set the instruction pointer to it.
// TODO - This can hold all of the initial setup code for the CPU, replacing the stuff that is
// TODO - currently setup in registers.rs, etc.

pub struct BootROM {
    pub memory: Vec<u8>,
}

impl BootROM {
    pub fn new() -> Self {
        Self {
            memory: vec![0x0; BOOT_MEMORY_LENGTH],
        }
    }
}

impl_mem_interface!(BootROM, true, false);
