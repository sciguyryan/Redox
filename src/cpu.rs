use crate::{mem::memory::Memory, reg::registers::Registers};

pub struct Cpu {
    pub registers_u32: Registers,
    pub is_halted: bool,
}

impl Cpu {
    pub fn new() -> Self {
        Self {
            registers_u32: Registers::default(),
            is_halted: false,
        }
    }

    pub fn run(&mut self, vm: &mut Memory, mem_seq_id: usize) {
        // Do something fun here.
    }
}

impl Default for Cpu {
    fn default() -> Self {
        Self::new()
    }
}
