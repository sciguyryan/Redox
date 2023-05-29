use crate::{
    ins::instruction::Instruction,
    mem::memory::Memory,
    reg::registers::{RegisterId, Registers},
    security_context::SecurityContext,
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

    fn decode_next_instruction(&self) -> Instruction {
        let instruction = Instruction::Nop;

        instruction
    }

    pub fn run(&mut self, _mem: &mut Memory, _mem_seq_id: usize) {
        // Do something fun here.
    }

    pub fn run_instructions(&mut self, mem: &mut Memory, instructions: &[Instruction]) {
        for ins in instructions {
            self.run_instruction(mem, ins);

            if self.is_halted {
                break;
            }
        }

        self.is_halted = true;
    }

    fn run_instruction(&mut self, _mem: &mut Memory, instruction: &Instruction) {
        match instruction {
            Instruction::Nop => {}
            Instruction::Hlt => {
                self.is_halted = true;
            }
            Instruction::AddU32LitReg(v, r) => {
                let old_value = *self
                    .registers
                    .get_register_u32(*r)
                    .read(&SecurityContext::User);

                let new_value = match old_value.checked_add(*v) {
                    Some(nv) => {
                        self.set_overflow_flag_state(false);
                        nv
                    }
                    None => {
                        self.set_overflow_flag_state(true);
                        v + old_value
                    }
                };

                self.registers
                    .get_register_u32_mut(*r)
                    .write(new_value, &SecurityContext::User);
            }
        };

        self.update_program_counter_register();

        // Move the instruction pointer register forward by the number of
        // bytes used to build the instruction.
        self.update_instruction_pointer(instruction.get_instruction_len());
    }

    fn set_flag_state(&mut self, index: u32, set: bool) {
        let register = self.registers.get_register_u32_mut(RegisterId::FL);
        let mut flags = *register.read_unchecked() & !index;

        if set {
            flags |= index;
        }

        register.write_unchecked(flags);
    }

    fn set_overflow_flag_state(&mut self, set: bool) {
        self.set_flag_state(2, set);
    }

    #[inline(always)]
    fn update_instruction_pointer(&mut self, bytes: u32) {
        self.registers
            .get_register_u32_mut(RegisterId::IP)
            .add_unchecked(bytes);
    }

    #[inline(always)]
    fn update_program_counter_register(&mut self) {
        self.registers
            .get_register_u32_mut(RegisterId::PC)
            .increment_unchecked();
    }
}

impl Default for Cpu {
    fn default() -> Self {
        Self::new()
    }
}
