use crate::ram::RAM;

pub struct VirtualMachine {
    pub ram: RAM,
}

impl VirtualMachine {
    pub fn new(memory: usize) -> Self {
        Self {
            ram: RAM::new(memory),
        }
    }
}
