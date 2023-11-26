use crate::{ins::instruction::Instruction, mem::memory::Memory};

pub struct Decompiler {}

impl Decompiler {
    pub fn decompile(bytes: &[u8]) -> Vec<Instruction> {
        let memory = Memory::from(bytes);
        memory.decompile_instructions(0, memory.len())
    }
}
