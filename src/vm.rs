use crate::{cpu::Cpu, mem::memory::Memory};

pub struct VirtualMachine {
    pub ram: Memory,
    pub cpu: Cpu,
}

impl VirtualMachine {
    pub fn new(memory: usize) -> Self {
        Self {
            ram: Memory::new(memory),
            cpu: Cpu::new(),
        }
    }

    pub fn run(&mut self) {
        self.cpu.run(&mut self.ram);
    }
}
