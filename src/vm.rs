use crate::{cpu::Cpu, ins::instruction::Instruction, mem::memory::Memory};

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

    pub fn run(&mut self) {
        //self.cpu.run(self, 0);
    }

    pub fn run_instructions(&mut self, instructions: &[Instruction]) {
        self.cpu.run_instructions(&mut self.ram, instructions);
    }
}
