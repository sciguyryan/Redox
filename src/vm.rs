use crate::{
    cpu::Cpu, ins::instruction::Instruction, mem::memory::Memory, reg::registers::RegisterId,
};

pub const MIN_MEMORY_SIZE: usize = 1024 * 1024 * 32;

pub struct VirtualMachine {
    pub ram: Memory,
    pub cpu: Cpu,
}

impl VirtualMachine {
    pub fn new(memory_size: usize) -> Self {
        assert!(memory_size >= MIN_MEMORY_SIZE);

        Self {
            ram: Memory::new(memory_size),
            cpu: Cpu::default(),
        }
    }

    pub fn run(&mut self) {
        self.cpu.run(&mut self.ram);
    }

    pub fn run_instructions(&mut self, instructions: &[Instruction]) {
        self.cpu.run_instructions(&mut self.ram, instructions);
    }

    pub fn setup(&mut self, binary_start: usize, bytes: &[u8], stack_capacity: usize) {
        let binary_len = bytes.len();
        assert!(binary_len < self.ram.len());

        // Load the bytecode raw data into RAM.
        self.ram.set_range(binary_start, bytes);

        // Now we can set up the stack, which will fall directly after
        // the binary data.
        let stack_start = binary_start + binary_len;
        let stack_end = self.ram.configure_stack(stack_start, stack_capacity) as u32;

        // Configure the CPU registers to account for the new stack pointer.
        self.cpu
            .registers
            .get_register_u32_mut(RegisterId::SP)
            .write_unchecked(stack_end);
    }
}
