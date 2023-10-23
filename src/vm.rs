use crate::{
    cpu::Cpu,
    ins::instruction::Instruction,
    mem::memory::{Memory, MemoryPermission},
    security_context::SecurityContext,
};

pub struct VirtualMachine {
    pub ram: Memory,
    pub cpu: Cpu,
}

impl VirtualMachine {
    pub fn new(memory_size: usize) -> Self {
        Self {
            ram: Memory::new(memory_size),
            cpu: Cpu::new(),
        }
    }

    pub fn run(&mut self, mem_seq_id: usize) {
        self.cpu.run(&mut self.ram, mem_seq_id);
    }

    pub fn run_instructions(&mut self, instructions: &[Instruction]) {
        self.cpu.run_instructions(&mut self.ram, instructions);
    }

    pub fn load_code_block(&mut self, bytes: &[u8]) -> usize {
        assert!(bytes.len() < self.ram.len());

        // Allocate a new code block, starting from position 0 in RAM.
        let start = 0;
        let end = bytes.len();
        let access = MemoryPermission::R | MemoryPermission::W | MemoryPermission::EX;

        let seq_id = self
            .ram
            .add_memory_region(start, end, access, "Executable Code");

        // Next, load the data into RAM.
        self.ram.set_range(start, bytes, &SecurityContext::Machine);

        seq_id
    }
}
