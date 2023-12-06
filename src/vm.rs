use crate::{cpu::Cpu, ins::instruction::Instruction, mem::memory::Memory};

pub struct VirtualMachine {
    pub ram: Memory,
    pub cpu: Cpu,
}

impl VirtualMachine {
    pub fn new(memory_size: usize) -> Self {
        assert!(memory_size >= 1024 * 1024 * 32);

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

    pub fn load_code_block(&mut self, start: usize, bytes: &[u8]) {
        assert!(bytes.len() < self.ram.len());

        // Load the bytecode data into RAM.
        self.ram.set_range(start, bytes);
    }
}
