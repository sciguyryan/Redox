use core::fmt;
use std::slice::Iter;

use crate::{
    ins::instruction::Instruction,
    mem::memory::Memory,
    reg::registers::{RegisterId, Registers},
    security_context::SecurityContext,
    utils,
};

pub struct Cpu {
    pub registers: Registers,
    pub is_halted: bool,
    pub is_machine_mode: bool,
}

impl Cpu {
    pub fn new() -> Self {
        Self {
            registers: Registers::default(),
            is_halted: false,
            is_machine_mode: true,
        }
    }

    fn decode_next_instruction(&self) -> Instruction {
        let instruction = Instruction::Nop;

        instruction
    }

    /// Get the state of a given CPU flag.
    ///
    /// # Arguments
    ///
    /// * `flag` - The [`CpuFlag`] to be checked.
    fn get_flag_state(&self, flag: CpuFlag) -> bool {
        let register = self.registers.get_register_u32(RegisterId::FL);
        utils::is_bit_set(*register.read_unchecked(), flag as u8)
    }

    /// Get the current value of the program counter (PC) register.
    #[inline(always)]
    fn get_program_counter(&mut self) {
        self.registers
            .get_register_u32_mut(RegisterId::PC)
            .read_unchecked();
    }

    const U32_MAX: u64 = u32::MAX as u64;

    /// Perform a checked add of two u32 values.
    ///
    /// # Arguments
    ///
    /// * `value_1` - The first u32 value.
    /// * `value_2` - The second u32 value.
    ///
    /// # Returns
    ///
    /// The result of the operation, truncated in the case of an overflow.
    ///
    /// # Note
    ///
    /// This method sets and unsets the zero and overflow flags as required.
    #[inline(always)]
    fn perform_checked_add_u32(&mut self, value_1: u32, value_2: u32) -> u32 {
        let final_value = value_1 as u64 + value_2 as u64;
        self.set_flag_state(CpuFlag::V, final_value > Cpu::U32_MAX);

        let final_value = final_value as u32;
        self.set_flag_state(CpuFlag::Z, final_value == 0);

        final_value
    }

    /// Perform a hard reset on the CPU.
    pub fn reset(&mut self) {
        self.registers.reset();
        self.is_halted = false;
    }

    pub fn run(&mut self, _mem: &mut Memory, _mem_seq_id: usize) {
        // Do something fun here.
    }

    /// Execute a set of instructions on the CPU.
    ///
    /// # Arguments
    ///
    /// * `mem` - The [`Memory`] connected to this CPU instance.
    /// * `instructions` - A slice of [`Instruction`] instances to be executed.
    pub fn run_instructions(&mut self, mem: &mut Memory, instructions: &[Instruction]) {
        for ins in instructions {
            self.run_instruction(mem, ins);

            if self.is_halted {
                break;
            }
        }

        self.is_halted = true;
    }

    /// Execute a single instruction on the CPU.
    ///
    /// # Arguments
    ///
    /// * `mem` - The [`Memory`] connected to this CPU instance.
    /// * `instruction` - An [`Instruction`] instance to be executed.
    fn run_instruction(&mut self, mem: &mut Memory, instruction: &Instruction) {
        let privilege = match self.is_machine_mode {
            true => SecurityContext::Machine,
            false => SecurityContext::User,
        };

        match instruction {
            Instruction::Nop => {}
            Instruction::AddU32LitU32Reg(lit, reg) => {
                let old_value = *self.registers.get_register_u32(*reg).read(&privilege);
                let new_value = self.perform_checked_add_u32(old_value, *lit);

                self.registers
                    .get_register_u32_mut(RegisterId::AC)
                    .write(new_value, &privilege);
            }
            Instruction::AddU32RegU32Reg(reg1, reg2) => {
                let value1 = *self.registers.get_register_u32(*reg1).read(&privilege);
                let value2 = *self.registers.get_register_u32(*reg2).read(&privilege);
                let new_value = self.perform_checked_add_u32(value1, value2);

                self.registers
                    .get_register_u32_mut(RegisterId::AC)
                    .write(new_value, &privilege);
            }
            Instruction::MovU32LitU32Reg(literal, reg) => {
                self.registers
                    .get_register_u32_mut(*reg)
                    .write(*literal, &privilege);
            }
            Instruction::Ret => {
                todo!();
            }
            Instruction::Mret => {
                self.is_machine_mode = false;
            }
            Instruction::Hlt => {
                self.is_halted = true;
            }
        };

        self.update_program_counter_register();

        // Move the instruction pointer register forward by the number of
        // bytes used to build the instruction.
        self.update_instruction_pointer(instruction.get_instruction_size());
    }

    /// Set the state of the specified CPU flag.
    ///
    /// # Arguments
    ///
    /// * `flag` - The [`CpuFlag`] to be set or unset.
    /// * `state` - The new state of the flag.
    #[inline(always)]
    fn set_flag_state(&mut self, flag: CpuFlag, state: bool) {
        let register = self.registers.get_register_u32_mut(RegisterId::FL);
        let flags = utils::set_bit_state(*register.read_unchecked(), flag as u8, state);
        register.write_unchecked(flags);
    }

    /// Update the instruction pointer (IP) register.
    #[inline(always)]
    fn update_instruction_pointer(&mut self, bytes: u32) {
        self.registers
            .get_register_u32_mut(RegisterId::IP)
            .add_unchecked(bytes);
    }

    /// Update the program counter (PC) register.
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

#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum CpuFlag {
    /// The negative flag - set to true if the result of an operation is negative.
    N,
    /// The zero flag - set to true if the result of an operation is zero.
    Z,
    /// The overflow flag - set to true if the result of an operation overflowed.
    V,
    ///  The carry flag - set to true if the carry bit has been set by an operation.
    C,
}

impl fmt::Display for CpuFlag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CpuFlag::N => write!(f, "N"),
            CpuFlag::Z => write!(f, "Z"),
            CpuFlag::V => write!(f, "V"),
            CpuFlag::C => write!(f, "C"),
        }
    }
}

impl CpuFlag {
    pub fn iterator() -> Iter<'static, CpuFlag> {
        static FLAGS: [CpuFlag; 4] = [CpuFlag::N, CpuFlag::Z, CpuFlag::V, CpuFlag::C];
        FLAGS.iter()
    }
}

#[cfg(test)]
mod tests_cpu {
    use std::panic;

    use crate::{
        ins::instruction::Instruction,
        mem::memory::Memory,
        reg::registers::{RegisterId, Registers},
    };

    use super::Cpu;

    struct TestEntrySimple<'a> {
        pub instructions: &'a [Instruction],
        pub changed_registers: &'a [(RegisterId, u32)],
        pub should_panic: bool,
        pub fail_message: String,
    }

    impl<'a> TestEntrySimple<'a> {
        fn new(
            instructions: &'a [Instruction],
            changed_registers: &'a [(RegisterId, u32)],
            should_panic: bool,
            fail_message: &str,
        ) -> Self {
            Self {
                instructions,
                changed_registers,
                should_panic,
                fail_message: fail_message.to_string(),
            }
        }

        fn build_registers(
            &self,
            register_presets: &'a [(RegisterId, u32)],
            instructions: &'a [Instruction],
        ) -> Registers {
            let mut registers = Registers::new();

            // This should be done before the registers are set because there will be instances
            // there the number of executed instructions will be different than the total
            // instruction count, such as if we hit a halt or early return.
            let mut size = 0;
            for ins in instructions {
                size += ins.get_instruction_size();
            }

            registers
                .get_register_u32_mut(RegisterId::IP)
                .write_unchecked(size);
            registers
                .get_register_u32_mut(RegisterId::PC)
                .write_unchecked(instructions.len() as u32);

            for (reg, val) in register_presets {
                registers.get_register_u32_mut(*reg).write_unchecked(*val);
            }

            registers
        }

        /// Run this specific test entry.
        ///
        /// # Arguments
        ///
        /// * `id` - The ID of this test.
        pub fn run_test(&self, id: usize) {
            let result = panic::catch_unwind(|| {
                let (mut mem, mut cpu) = create_instance();
                cpu.run_instructions(&mut mem, self.instructions);

                (mem, cpu)
            });

            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                self.should_panic,
                "{}",
                self.fail_message(id, did_panic)
            );

            if let Ok((_, cpu)) = result {
                let registers = self.build_registers(self.changed_registers, self.instructions);
                assert_eq!(cpu.registers, registers, "{}", self.fail_message(id, false));
            }
        }

        /// Run a special test entry, returning a copy of the CPU instance for interrogation.
        ///
        /// # Arguments
        ///
        /// * `id` - The ID of this test.
        pub fn run_test_special(&self, id: usize) -> (Memory, Cpu) {
            let result = panic::catch_unwind(|| {
                let (mut mem, mut cpu) = create_instance();
                cpu.run_instructions(&mut mem, self.instructions);

                (mem, cpu)
            });

            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                self.should_panic,
                "{}",
                self.fail_message(id, true)
            );

            match result {
                Ok((mem, cpu)) => {
                    let registers = self.build_registers(self.changed_registers, self.instructions);
                    assert_eq!(cpu.registers, registers, "{}", self.fail_message(id, false));

                    (mem, cpu)
                }
                Err(_) => create_instance(),
            }
        }

        /// Run this specific test entry.
        ///
        /// # Arguments
        ///
        /// * `id` - The ID of this test.
        /// * `did_panic` - Did the test panic?
        pub fn fail_message(&self, id: usize, did_panic: bool) -> String {
            format!(
                "Test {id} Failed - Should Panic? {}, Panicked? {did_panic}. Message = {}",
                self.should_panic, self.fail_message
            )
        }
    }

    struct TestEntryU32Standard<'a> {
        pub instructions: &'a [Instruction],
        pub changed_registers: &'a [(RegisterId, u32)],
        pub expected_memory: Vec<u8>,
        pub should_panic: bool,
        pub fail_message: String,
    }

    impl<'a> TestEntryU32Standard<'a> {
        fn new(
            instructions: &'a [Instruction],
            changed_registers: &'a [(RegisterId, u32)],
            expected_memory: Vec<u8>,
            should_panic: bool,
            fail_message: &str,
        ) -> Self {
            Self {
                instructions,
                changed_registers,
                expected_memory,
                should_panic,
                fail_message: fail_message.to_string(),
            }
        }

        fn build_registers(
            &self,
            register_presets: &'a [(RegisterId, u32)],
            instructions: &'a [Instruction],
        ) -> Registers {
            let mut registers = Registers::new();

            // This should be done before the registers are set because there will be instances
            // there the number of executed instructions will be different than the total
            // instruction count, such as if we hit a halt or early return.
            let mut size = 0;
            for ins in instructions {
                size += ins.get_instruction_size();
            }

            registers
                .get_register_u32_mut(RegisterId::IP)
                .write_unchecked(size);
            registers
                .get_register_u32_mut(RegisterId::PC)
                .write_unchecked(instructions.len() as u32);

            for (reg, val) in register_presets {
                registers.get_register_u32_mut(*reg).write_unchecked(*val);
            }

            registers
        }

        /// Run this specific test entry.
        ///
        /// # Arguments
        ///
        /// * `id` - The ID of this test.
        pub fn run_test(&self, id: usize) {
            let result = panic::catch_unwind(|| {
                let (mut mem, mut cpu) = create_instance();
                cpu.run_instructions(&mut mem, self.instructions);

                (mem, cpu)
            });

            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                self.should_panic,
                "{}",
                self.fail_message(id, did_panic)
            );

            if let Ok((mem, cpu)) = result {
                let registers = self.build_registers(self.changed_registers, self.instructions);
                assert_eq!(cpu.registers, registers, "{}", self.fail_message(id, false));

                assert_eq!(
                    mem.get_storage(),
                    self.expected_memory,
                    "{}",
                    self.fail_message(id, false)
                );
            }
        }

        /// Run this specific test entry.
        ///
        /// # Arguments
        ///
        /// * `id` - The ID of this test.
        /// * `did_panic` - Did the test panic?
        pub fn fail_message(&self, id: usize, did_panic: bool) -> String {
            format!(
                "Test {id} Failed - Should Panic? {}, Panicked? {did_panic}. Message = {}",
                self.should_panic, self.fail_message
            )
        }
    }

    /// Run this specific test entry.
    ///
    /// # Returns
    ///
    /// A tuple containing a [`Memory`] instance and a [`Cpu`] instance.
    fn create_instance() -> (Memory, Cpu) {
        (Memory::new(100), Cpu::default())
    }

    /// Test the NOP instruction.
    #[test]
    fn test_nop() {
        let tests = [TestEntrySimple::new(
            &[Instruction::Nop],
            &[(RegisterId::IP, 2), (RegisterId::PC, 1)],
            false,
            "failed to execute NOP instruction",
        )];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the HLT instruction.
    #[test]
    fn test_hlt() {
        let tests = [
            TestEntrySimple::new(
                &[Instruction::Hlt],
                &[(RegisterId::IP, 2), (RegisterId::PC, 1)],
                false,
                "failed to execute HLT instruction",
            ),
            // The halt instruction should prevent any following instructions from executing, which
            // means that the program counter should also not increase.
            TestEntrySimple::new(
                &[Instruction::Hlt, Instruction::Nop],
                &[(RegisterId::IP, 2), (RegisterId::PC, 1)],
                false,
                "failed to correctly stop execution after a HLT instruction",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the MRET instruction.
    #[test]
    fn test_mret() {
        let tests = [TestEntrySimple::new(
            &[Instruction::Mret],
            &[],
            false,
            "failed to execute MRET instruction",
        )];

        for (id, test) in tests.iter().enumerate() {
            let (_, cpu) = test.run_test_special(id);
            assert!(
                !cpu.is_machine_mode,
                "Test {id} Failed - machine is still in machine mode after executing mret instruction!"
            );
        }
    }

    /// Test the AddU32LitU32Reg instruction.
    #[test]
    fn test_add_u32_lit_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32LitU32Reg(0x1, RegisterId::R1),
                    Instruction::AddU32LitU32Reg(0x2, RegisterId::R1)
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::AC, 0x3)],
                vec![0; 100],
                false,
                "failed to correctly execute ADD instruction"
            ),
            // This test should cause the CPU's overflow flag to be set.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32LitU32Reg(u32::MAX, RegisterId::R1),
                    Instruction::AddU32LitU32Reg(0x2, RegisterId::R1)
                ],
                &[
                    (RegisterId::R1, u32::MAX), (RegisterId::AC, 0x1),
                    (RegisterId::FL, 0b00000100)
                ],
                vec![0; 100],
                false,
                "failed to correctly execute ADD instruction: overflow CPU register flag was not correctly set"
            ),
            // This test should cause the CPU's zero flag to be set.
            TestEntryU32Standard::new(
                &[Instruction::AddU32LitU32Reg(0, RegisterId::R1)],
                &[(RegisterId::FL, 0b00000010)],
                vec![0; 100],
                false,
                "failed to correctly execute ADD instruction: zero CPU register flag was not correctly set"
            ),
            // This test should succeed in machine mode.
            TestEntryU32Standard::new(
                &[Instruction::AddU32LitU32Reg(0x1, RegisterId::TEST0)],
                &[(RegisterId::AC, 0x1)],
                vec![0; 100],
                false,
                "failed to correctly execute ADD instruction in machine mode"
            ),
            // This test should fail in user mode due to the register permissions.
            TestEntryU32Standard::new(
                &[
                    Instruction::Mret,
                    Instruction::AddU32LitU32Reg(0x2, RegisterId::TEST0)
                ],
                &[(RegisterId::AC, 2)],
                vec![0; 100],
                true,
                "succeeded in execute ADD instruction in user mode"
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the AddU32RegU32Reg instruction.
    #[test]
    fn test_add_u32_reg_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    // Setup register values.
                    Instruction::MovU32LitU32Reg(0xf, RegisterId::R1),
                    Instruction::MovU32LitU32Reg(0x1, RegisterId::R2),
                    // Add the register values.
                    Instruction::AddU32RegU32Reg(RegisterId::R1, RegisterId::R2)
                ],
                &[
                    (RegisterId::R1, 0xf), (RegisterId::R2, 0x1),
                    (RegisterId::AC, 0x10)
                ],
                vec![0; 100],
                false,
                "failed to correctly execute ADD instruction"
            ),
            // This test should cause the CPU's overflow flag to be set.
            TestEntryU32Standard::new(
                &[
                    // Setup register values.
                    Instruction::MovU32LitU32Reg(u32::MAX, RegisterId::R1),
                    Instruction::MovU32LitU32Reg(0x2, RegisterId::R2),
                    // Add the register values.
                    Instruction::AddU32RegU32Reg(RegisterId::R1, RegisterId::R2)
                ],
                &[
                    (RegisterId::R1, u32::MAX), (RegisterId::R2, 0x2),
                    (RegisterId::AC, 0x1), (RegisterId::FL, 0b00000100)
                ],
                vec![0; 100],
                false,
                "failed to correctly execute ADD instruction: overflow CPU register flag was not correctly set"
            ),
            // This test should cause the CPU's zero flag to be set.
            TestEntryU32Standard::new(
                &[
                    Instruction::AddU32RegU32Reg(RegisterId::R1, RegisterId::R2)
                ],
                &[(RegisterId::FL, 0b00000010)],
                vec![0; 100],
                false,
                "failed to correctly execute ADD instruction: zero CPU register flag was not correctly set"
            ),
            // This test succeed in machine mode.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32LitU32Reg(0x1, RegisterId::R1),
                    Instruction::AddU32RegU32Reg(RegisterId::R1, RegisterId::TEST0),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::AC, 0x1)],
                vec![0; 100],
                false,
                "failed to correctly execute ADD instruction in machine mode"
            ),
            // This test should fail in user mode due to the register permissions.
            TestEntryU32Standard::new(
                &[
                    Instruction::Mret,
                    Instruction::MovU32LitU32Reg(0x1, RegisterId::R1),
                    Instruction::AddU32RegU32Reg(RegisterId::R1, RegisterId::TEST0),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::AC, 0x1)],
                vec![0; 100],
                true,
                "succeeded in execute ADD instruction in user mode"
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the MoveU32LitU32Reg instruction.
    #[test]
    fn test_move_u32_lit_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32LitU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "failed to correctly execute MOV instruction"
            ),
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32LitU32Reg(0x1, RegisterId::R1),
                    Instruction::MovU32LitU32Reg(0x2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2)],
                vec![0; 100],
                false,
                "failed to correctly execute MOV instruction"
            ),
            // This test should succeed in machine mode.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32LitU32Reg(0x1, RegisterId::TEST0)
                ],
                &[(RegisterId::TEST0, 0x1)],
                vec![0; 100],
                false,
                "failed to correctly execute MOV instruction in machine mode"
            ),
            // This test should fail in user mode due to the register permissions.
            TestEntryU32Standard::new(
                &[
                    Instruction::Mret,
                    Instruction::MovU32LitU32Reg(0x1, RegisterId::TEST0)
                ],
                &[(RegisterId::TEST0, 0x1)],
                vec![0; 100],
                true,
                "succeeded in execute MOV instruction in user mode"
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }
}
