use crate::{
    boot_rom::{BootRom, BOOT_MEMORY_LENGTH, BOOT_MEMORY_START, BOOT_REGION_NAME},
    com_bus::communication_bus::CommunicationBus,
    cpu::Cpu,
    ins::instruction::Instruction,
    mem::memory_handler::{MemoryHandler, MEGABYTE},
    reg::registers::Registers,
};

/// The smallest permitted size of the user memory segment.
pub const MIN_USER_SEGMENT_SIZE: usize = MEGABYTE * 5;
/// The default size of the stack, in terms of how many u32 values can be held.
pub const U32_STACK_CAPACITY: usize = 1000;

pub struct VirtualMachine {
    /// The [`Cpu`] for this virtual machine.
    pub cpu: Cpu,
    /// The [`CommunicationBus`] for this virtual machine.
    pub com_bus: CommunicationBus,
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

        // The first 1024 bytes (1 kilobyte) of user memory are typically used for
        // the interrupt vector table (IVT).
        // This is comprised of 256 entries of 4 bytes, each representing a potential
        // address for the interrupt handler.
        let mem = MemoryHandler::new(
            user_segment_size,
            code_segment_bytes,
            data_segment_bytes,
            stack_segment_size,
        );

        // Construct out virtual machine.
        let mut vm = Self {
            com_bus: CommunicationBus::new(mem),
            cpu: Cpu::default(),
        };

        // Build and load the boot ROM.
        // This will return a list of our default interrupt vector handlers
        // from the compiled boot ROM.
        vm.load_boot_rom();

        vm
    }

    /// Create and load the bootable ROM into memory.
    ///
    /// # Returns
    ///
    /// A hashmap, the key being the interrupt code and the value being the address in memory where the handler is located.
    fn load_boot_rom(&mut self) {
        // Create the bootable ROM.
        let boot_rom = BootRom::compile(&self.com_bus.mem);

        // Create a new memory segment tp hold the bootable ROM data.
        let boot_mem_id = self.com_bus.mem.add_mapped_memory_segment(
            BOOT_MEMORY_START,
            BOOT_MEMORY_LENGTH,
            true,
            false,
            BOOT_REGION_NAME,
        );

        // Load the contents of the boot into the newly created memory segment.
        self.com_bus
            .mem
            .get_mapped_segment_by_index_mut(boot_mem_id)
            .expect("failed to get memory segment")
            .set_contents(&boot_rom);
    }

    /// Reset the virtual machine back to a default configuration.
    pub fn reset(&mut self) {
        // Clear any stack type hints that may be present.
        self.com_bus.mem.reset_stack_configuration();

        // Completely clear the physical memory segment.
        self.com_bus.mem.clear();

        // Reset the registers to their startup configuration.
        self.cpu.registers = Registers::default();

        // Allow the CPU to run.
        self.cpu.is_halted = false;
    }

    /// Run the virtual machine until completion.
    pub fn run(&mut self) {
        self.cpu.run(&mut self.com_bus);
    }

    /// Run a sequence of [`Instruction`]s within the virtual machine.
    pub fn run_instructions(&mut self, instructions: &[Instruction]) {
        self.cpu.run_instructions(&mut self.com_bus, instructions);
    }
}
