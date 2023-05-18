use crate::ram::Ram;

pub struct VirtualMachine {
    pub ram: Ram,
}

impl VirtualMachine {
    pub fn new(memory: usize) -> Self {
        Self {
            ram: Ram::new(memory),
        }
    }
}
