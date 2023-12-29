use crate::mem::memory_handler::MEGABYTE;

/// The start index of the boot mapped memory region.
pub const BOOT_MEMORY_START: usize = 0x12_C00_000; // Starting at the 300 megabyte region.
/// The end index of the boot mapped memory region.
pub const BOOT_MEMORY_LENGTH: usize = MEGABYTE; // Extending for 1 megabyte.
/// The name of the region.
pub const BOOT_REGION_NAME: &str = "boot";

pub struct BootRom {
    pub compiled: Vec<u8>,
}

impl BootRom {
    pub fn new() -> Self {
        let mut s = Self { compiled: vec![] };

        s.compile();

        s
    }

    pub fn compile(&mut self) {}
}
