use crate::{
    boot_rom::BootROM,
    cpu::Cpu,
    ins::instruction::Instruction,
    mem::{memory_handler::MemoryHandler, memory_interface::MemoryInterface},
};

pub const MIN_USER_SEGMENT_SIZE: usize = 1024 * 1024 * 32;
pub const MIN_BINARY_LOAD_ADDRESS: usize = 0x10000;
pub const U32_STACK_CAPACITY: usize = 1000;
pub const BYTES_IN_U32: usize = 4;

pub struct VirtualMachine<'a, T: MemoryInterface> {
    pub mem: MemoryHandler<'a, T>,
    pub cpu: Cpu,

    boot_rom: BootROM,
}

impl<'a, T: MemoryInterface> VirtualMachine<'a, T> {
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
            boot_rom: BootROM::new(),
        };

        // Add the mapped boot ROM memory region.
        //vm.mem.add_mapped_memory_region(BOOT_MEMORY_START, BOOT_MEMORY_LENGTH, true, false, BOOT_NAME);
        vm.mem.register_memory_mapped_device(&mut vm.boot_rom);

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
