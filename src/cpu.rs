use crate::{mem::memory::Memory, reg::registers::Registers};

pub struct Cpu {
    pub registers: Registers,
}

impl Cpu {
    pub fn new() -> Self {
        Self {
            registers: Registers::new(),
        }
    }

    pub fn run(&mut self, vm: &mut Memory) {
        // Do something fun here.
    }
}

impl Default for Cpu {
    fn default() -> Self {
        Self::new()
    }
}
