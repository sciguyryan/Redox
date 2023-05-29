use crate::{
    ins::instruction::Instruction,
    mem::memory::Memory,
    reg::registers::{RegisterId, Registers},
    security_context::SecurityContext,
    vm::VirtualMachine,
};

pub struct Cpu {
    pub registers: Registers,
    pub is_halted: bool,
}

impl Cpu {
    pub fn new() -> Self {
        Self {
            registers: Registers::default(),
            is_halted: false,
        }
    }

    pub fn run(&mut self, vm: &mut VirtualMachine, mem_seq_id: usize) {
        // Do something fun here.
    }

    pub fn run_tester(&mut self, mem: &mut Memory, instructions: &[Instruction]) {
        for ins in instructions {
            self.run_instruction(mem, ins);

            if self.is_halted {
                break;
            }
        }

        self.is_halted = true;
    }

    fn run_instruction(&mut self, mem: &mut Memory, instruction: &Instruction) {
        match instruction {
            Instruction::Nop() => {}
            Instruction::Hlt() => {
                self.is_halted = true;
            }
            Instruction::AddU32LitReg(v, r) => {
                self.registers
                    .get_register_u32_mut(*r)
                    .add(*v, &SecurityContext::User);
            }
        }

        // Increment the program counter (PC) register.
        self.registers
            .get_register_u32_mut(RegisterId::PC)
            .increment(&SecurityContext::System);
    }
}

impl Default for Cpu {
    fn default() -> Self {
        Self::new()
    }
}
