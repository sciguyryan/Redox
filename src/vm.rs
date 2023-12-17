use crate::{cpu::Cpu, ins::instruction::Instruction, mem::memory::Memory};

pub const MIN_MEMORY_SIZE: usize = 1024 * 1024 * 32;
pub const MIN_BINARY_LOAD_ADDRESS: usize = 0x10000;
pub const U32_STACK_CAPACITY: usize = 1000;

pub struct VirtualMachine {
    pub ram: Memory,
    pub cpu: Cpu,
}

impl VirtualMachine {
    pub fn new(
        total_memory_size: usize,
        user_segment_size: usize,
        code_load_address: usize,
        code_segment_bytes: &[u8],
        data_segment_bytes: &[u8],
    ) -> Self {
        // We have a minimum memory condition for this VM to ensure that certain assumptions
        // around the placement of things in memory remain sound.
        assert!(total_memory_size >= MIN_MEMORY_SIZE);

        // We enforce a minimum binary load address to ensure that there is enough space for
        // important structures (such as the interrupt vector table) within memory.
        assert!(code_load_address >= MIN_BINARY_LOAD_ADDRESS);
        assert!(code_load_address < total_memory_size);

        // Construct out virtual machine.
        let mut vm = Self {
            ram: Memory::new(
                total_memory_size,
                user_segment_size,
                code_load_address,
                code_segment_bytes,
                data_segment_bytes,
                U32_STACK_CAPACITY,
            ),
            cpu: Cpu::default(),
        };

        // Update the segment registers, now that we know where the segments
        // are located in RAM.
        vm.cpu.update_segment_registers(&vm.ram);

        // Configure the CPU registers to account for the new stack pointer.
        vm.cpu.update_stack_pointer(vm.ram.stack_segment_end as u32);

        vm
    }

    pub fn run(&mut self) {
        self.cpu.run(&mut self.ram);
    }

    pub fn run_instructions(&mut self, instructions: &[Instruction]) {
        self.cpu.run_instructions(&mut self.ram, instructions);
    }
}
