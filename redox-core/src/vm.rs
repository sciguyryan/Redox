use hashbrown::HashMap;

use crate::{
    boot_rom::{BootRom, BOOT_MEMORY_LENGTH, BOOT_MEMORY_START, BOOT_REGION_NAME},
    cpu::Cpu,
    ins::instruction::Instruction,
    mem::memory_handler::{MemoryHandler, MEGABYTE},
};

/// The smallest permitted size of the user memory segment.
pub const MIN_USER_SEGMENT_SIZE: usize = MEGABYTE * 5;
/// The default size of the stack, in terms of how many u32 values can be held.
pub const U32_STACK_CAPACITY: usize = 1000;

pub struct VirtualMachine {
    /// The [`MemoryHandler`] for this virtual machine.
    pub mem: MemoryHandler,
    /// The [`Cpu`] for this virtual machine.
    pub cpu: Cpu,

    /// The default interrupt vector table handlers.
    default_ivt_handlers: HashMap<u8, u32>,
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
        // We have a minimum memory condition for this VM to ensure that certain
        //assumptions around the placement of things in memory remain sound.
        assert!(user_segment_size >= MIN_USER_SEGMENT_SIZE);

        // The first 1024 bytes (1 kilobyte) of user memory are used for
        // the interrupt vector table (IVT). This is comprised of 256
        // entries of 4 bytes, each representing a potential address for the
        // interrupt handler. Some interrupts aren't

        // Construct out virtual machine.
        let mut vm = Self {
            mem: MemoryHandler::new(
                user_segment_size,
                code_segment_bytes,
                data_segment_bytes,
                stack_segment_size,
            ),
            cpu: Cpu::default(),
            default_ivt_handlers: HashMap::new(),
        };

        // Build and load the boot ROM.
        // This will return a list of our default interrupt vector handlers
        // from the compiled boot ROM.
        let default_ivt_handlers = vm.load_boot_rom();

        // Load the default interrupt vector table entries into memory.
        vm.load_default_ivt_handlers(default_ivt_handlers);

        vm
    }

    /// Create and load the bootable ROM into memory.
    fn load_boot_rom(&mut self) -> HashMap<u8, u32> {
        // Create the bootable ROM.
        let (boot_rom, default_ivt_handlers) = BootRom::compile(&self.mem);

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

        // Return the default IVT handlers for further processing.
        default_ivt_handlers
    }

    fn load_default_ivt_handlers(&mut self, handlers: HashMap<u8, u32>) {
        for (int_code, address) in handlers {}
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
