use crate::{
    boot_rom::{BootRom, BOOT_MEMORY_LENGTH, BOOT_MEMORY_START, BOOT_REGION_NAME},
    cpu::Cpu,
    ins::instruction::Instruction,
    mem::memory_handler::{MemoryHandler, MEGABYTE},
};

/// The smallest permitted size of the user memory segment.
pub const MIN_USER_SEGMENT_SIZE: usize = MEGABYTE * 32;
/// The default size of the stack, in terms of how many u32 values can be held.
pub const U32_STACK_CAPACITY: usize = 1000;

pub struct VirtualMachine {
    pub mem: MemoryHandler,
    pub cpu: Cpu,
}

impl VirtualMachine {
    /// Build a new [`VirtualMachine`] instance.
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
        #[cfg(all(debug_assertions, not(test)))]
        {
            if user_segment_size <= MIN_USER_SEGMENT_SIZE {
                println!("WARNING: attempting to create a virtual machine with a user memory segment size that is smaller than the suggested minimum. Some features may not work correctly.");
            }
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

        // Build and load the boot ROM.
        vm.load_boot_rom();

        vm
    }

    /// Create and load the bootable ROM into memory.
    fn load_boot_rom(&mut self) {
        // Create the bootable ROM.
        let boot_rom = BootRom::compile(&self.mem);

        // Create a new memory segment tp hold the bootable ROM data.
        let boot_mem_id = self.mem.add_mapped_memory_segment(
            BOOT_MEMORY_START,
            BOOT_MEMORY_LENGTH,
            true,
            false,
            BOOT_REGION_NAME,
        );

        // Load the contents of the boot into the newly created memory segment.
        self.mem
            .get_mapped_segment_by_index_mut(boot_mem_id)
            .expect("failed to get memory segment")
            .set_contents(&boot_rom);
    }

    /// Run the virtual machine until completion.
    pub fn run(&mut self) {
        self.cpu.run(&mut self.mem);
    }

    /// Run a sequence of [`Instruction`]s within the virtual machine.
    pub fn run_instructions(&mut self, instructions: &[Instruction]) {
        self.cpu.run_instructions(&mut self.mem, instructions);
    }
}
