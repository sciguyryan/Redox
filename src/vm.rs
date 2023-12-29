use crate::{
    cpu::Cpu,
    ins::instruction::Instruction,
    mem::memory_handler::{MemoryHandler, MEGABYTE},
};

pub const MIN_USER_SEGMENT_SIZE: usize = MEGABYTE * 32;
pub const U32_STACK_CAPACITY: usize = 1000;
pub const BYTES_IN_U32: usize = 4;

/// The start index of the boot mapped memory region.
pub const BOOT_MEMORY_START: usize = 0x12_C00_000; // Starting at the 300 megabyte region.
/// The end index of the boot mapped memory region.
pub const BOOT_MEMORY_LENGTH: usize = MEGABYTE; // Extending for 1 megabyte.

pub struct VirtualMachine {
    pub mem: MemoryHandler,
    pub cpu: Cpu,
}

impl VirtualMachine {
    /// Build a virtual machine instance.
    ///
    /// # Arguments
    ///
    /// * `user_segment_size` - The size of the user memory segment, in bytes.
    /// * `code_segment_bytes` - The contents of the code segment.
    /// * `data_segment_bytes` - The contents of the data segment.
    /// * `stack_segment_size` - The size of the stack memory segment, in bytes.
    pub fn new(
        user_segment_size: usize,
        code_segment_bytes: &[u8],
        data_segment_bytes: &[u8],
        stack_segment_size: usize,
    ) -> Self {
        // We have a minimum memory condition for this VM to ensure that certain assumptions
        // around the placement of things in memory remain sound.
        if cfg!(not(test)) && user_segment_size <= MIN_USER_SEGMENT_SIZE {
            println!("WARNING: attempting to create a virtual machine with a user memory segment size that is smaller than the suggested minimum. Some features may not work correctly.");
        }

        // Construct out virtual machine.
        let mut vm = Self {
            mem: MemoryHandler::new(
                user_segment_size,
                code_segment_bytes,
                data_segment_bytes,
                stack_segment_size,
            ),
            cpu: Cpu::default(),
        };

        // Insert the boot memory mapped region.
        vm.mem
            .add_mapped_memory_region(BOOT_MEMORY_START, BOOT_MEMORY_LENGTH, true, false, "boot");

        // Synchronize certain CPU registers with the local variables
        // held in the memory instance.
        vm.cpu.synchronize_registers(&vm.mem);

        vm
    }

    pub fn run(&mut self) {
        self.cpu.run(&mut self.mem);
    }

    pub fn run_instructions(&mut self, instructions: &[Instruction]) {
        self.cpu.run_instructions(&mut self.mem, instructions);
    }
}
