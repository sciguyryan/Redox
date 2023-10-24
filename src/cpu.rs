use core::fmt;
use std::{collections::HashMap, panic, slice::Iter};

use crate::{
    ins::{
        instruction::Instruction,
        move_expressions::{ExpressionArgs, ExpressionOperator, MoveExpressionHandler},
        op_codes::OpCode,
    },
    mem::memory::Memory,
    privilege_level::PrivilegeLevel,
    reg::registers::{RegisterId, Registers},
    utils,
};

pub struct Cpu {
    pub registers: Registers,
    pub is_halted: bool,
    pub is_machine_mode: bool,

    move_expression_cache: HashMap<u32, [ExpressionArgs; 3]>,
}

impl Cpu {
    pub fn new() -> Self {
        Self {
            registers: Registers::default(),
            is_halted: false,
            is_machine_mode: true,

            move_expression_cache: HashMap::new(),
        }
    }

    /// Execute a move instruction expression.
    ///
    /// # Arguments
    ///
    /// * `operators` - The operators that form the expression.
    /// * `privilege` - The [`PrivilegeLevel`] in which this expression should be executed.
    ///
    /// # Returns
    ///
    /// A u32 that is the calculated result of the expression.
    fn execute_u32_move_expression(
        &self,
        operators: &[ExpressionArgs; 3],
        privilege: &PrivilegeLevel,
    ) -> u32 {
        // Determine the first operand.
        let val_1 = match operators[0] {
            ExpressionArgs::Register(rid) => *self.registers.get_register_u32(rid).read(privilege),
            ExpressionArgs::Constant(val) => val as u32,
            _ => panic!(),
        };

        // Determine the second operand.
        let val_2 = match operators[2] {
            ExpressionArgs::Register(rid) => *self.registers.get_register_u32(rid).read(privilege),
            ExpressionArgs::Constant(val) => val as u32,
            _ => panic!(),
        };

        // Calculate the destination address by evaluating the expression.
        if let ExpressionArgs::Operator(op) = operators[1] {
            match op {
                ExpressionOperator::Add => val_1 + val_2,
                ExpressionOperator::Subtract => val_1 - val_2,
                ExpressionOperator::Multiply => val_1 * val_2,
            }
        } else {
            panic!();
        }
    }

    /// Get the current privilege level of the processor.
    ///
    /// # Returns
    ///
    /// A [`PrivilegeLevel`] giving the current privilege level of the processor.
    #[inline(always)]
    fn get_privilege(&self) -> PrivilegeLevel {
        if self.is_machine_mode {
            PrivilegeLevel::Machine
        } else {
            PrivilegeLevel::User
        }
    }

    /// Execute a move instruction expression.
    ///
    /// # Arguments
    ///
    /// * `mem` - A reference to the virtual machine [`Memory`] instance.
    ///
    /// # Returns
    ///
    /// The next [`Instruction`] to be executed.
    fn fetch_decode_next_instruction(&mut self, mem: &Memory) -> Instruction {
        // Get the current instruction pointer.
        let ip = *self
            .registers
            .get_register_u32(RegisterId::IP)
            .read(&PrivilegeLevel::Machine);

        // Update the program counter register.
        mem.get_instruction(ip as usize)
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

    /// Run the CPU from a specified starting point in memory.
    ///
    /// # Arguments
    ///
    /// * `mem` - A reference to the virtual machine [`Memory`] instance.
    pub fn run(&mut self, mem: &mut Memory) {
        loop {
            let ins = self.fetch_decode_next_instruction(mem);
            self.run_instruction(mem, &ins);

            if self.is_halted {
                break;
            }
        }
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
        let privilege = &self.get_privilege();

        match instruction {
            Instruction::Nop => {}

            /******** [Arithmetic Instructions] ********/
            Instruction::AddU32ImmU32Reg(imm, reg) => {
                let old_value = *self.registers.get_register_u32(*reg).read(privilege);
                let new_value = self.perform_checked_add_u32(old_value, *imm);

                self.registers
                    .get_register_u32_mut(RegisterId::AC)
                    .write(new_value, privilege);
            }
            Instruction::AddU32RegU32Reg(in_reg, out_reg) => {
                let value1 = *self.registers.get_register_u32(*in_reg).read(privilege);
                let value2 = *self.registers.get_register_u32(*out_reg).read(privilege);
                let new_value = self.perform_checked_add_u32(value1, value2);

                self.registers
                    .get_register_u32_mut(RegisterId::AC)
                    .write(new_value, privilege);
            }

            /******** [Simple Move Instructions - NO EXPRESSIONS] ********/
            Instruction::SwapU32RegU32Reg(reg1, reg2) => {
                let reg1_val = *self.registers.get_register_u32(*reg1).read(privilege);
                let reg2_val = *self.registers.get_register_u32(*reg2).read(privilege);

                self.registers
                    .get_register_u32_mut(*reg1)
                    .write(reg2_val, privilege);
                self.registers
                    .get_register_u32_mut(*reg2)
                    .write(reg1_val, privilege);
            }
            Instruction::MovU32ImmU32Reg(imm, reg) => {
                self.registers
                    .get_register_u32_mut(*reg)
                    .write(*imm, privilege);
            }
            Instruction::MovU32RegU32Reg(in_reg, out_reg) => {
                let value = *self.registers.get_register_u32(*in_reg).read(privilege);

                self.registers
                    .get_register_u32_mut(*out_reg)
                    .write(value, privilege);
            }
            Instruction::MovU32ImmMemRelSimple(imm, addr) => {
                mem.set_u32(*addr as usize, *imm);
            }
            Instruction::MovU32RegMemRelSimple(reg, addr) => {
                let value = *self.registers.get_register_u32(*reg).read(privilege);
                mem.set_u32(*addr as usize, value);
            }
            Instruction::MovMemU32RegRelSimple(addr, reg) => {
                let value = mem.get_u32(*addr as usize);
                self.registers
                    .get_register_u32_mut(*reg)
                    .write(value, privilege);
            }
            Instruction::MovU32RegPtrU32RegRelSimple(in_reg, out_reg) => {
                let address = self.registers.get_register_u32(*in_reg).read(privilege);
                let value = mem.get_u32(*address as usize);
                self.registers
                    .get_register_u32_mut(*out_reg)
                    .write(value, privilege);
            }

            /******** [Complex Move Instructions - WITH EXPRESSIONS] ********/
            Instruction::MovU32ImmMemRelExpr(imm, expr) => {
                // Cache the decoding result to speed up processing slightly.
                if !self.move_expression_cache.contains_key(expr) {
                    let mut expression_decoder = MoveExpressionHandler::new();
                    expression_decoder.decode(*expr);

                    self.move_expression_cache
                        .insert(*expr, expression_decoder.get_args_as_array());
                }

                let operators = self.move_expression_cache.get(expr).unwrap();
                let addr = self.execute_u32_move_expression(operators, privilege);

                mem.set_u32(addr as usize, *imm);
            }

            /******** [Special Instructions] ********/
            Instruction::Ret => {
                todo!();
            }
            Instruction::Mret => {
                self.set_machine_mode(false);
            }
            Instruction::Hlt => {
                self.is_halted = true;
            }
        };

        self.update_program_counter_register();

        // Move the instruction pointer register forward by the number of
        // bytes used to build the instruction.
        self.update_instruction_pointer(OpCode::from(*instruction).get_total_instruction_size());
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

    #[inline(always)]
    fn set_machine_mode(&mut self, state: bool) {
        self.is_machine_mode = state;
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
        ins::{
            instruction::Instruction,
            move_expressions::{ExpressionArgs, ExpressionOperator, MoveExpressionHandler},
            op_codes::OpCode,
        },
        mem::memory::Memory,
        reg::registers::{RegisterId, Registers},
    };

    use super::Cpu;

    struct TestEntryU32Standard {
        pub instructions: Vec<Instruction>,
        pub registers: Registers,
        pub expected_memory: Vec<u8>,
        pub should_panic: bool,
        pub fail_message: String,
    }

    impl TestEntryU32Standard {
        fn new(
            instructions: &[Instruction],
            changed_registers: &[(RegisterId, u32)],
            expected_memory: Vec<u8>,
            should_panic: bool,
            fail_message: &str,
        ) -> Self {
            // Ensure we always end with a halt instruction.
            let mut instructions_vec = instructions.to_vec();
            if let Some(ins) = instructions_vec.last() {
                if !matches!(ins, Instruction::Hlt) {
                    instructions_vec.push(Instruction::Hlt);
                }
            }

            let mut s = Self {
                instructions: instructions_vec,
                registers: Registers::default(),
                expected_memory,
                should_panic,
                fail_message: fail_message.to_string(),
            };

            s.build_registers(changed_registers);

            s
        }

        /// Build the expected [`Registers`] instance for our test.
        ///
        /// # Arguments
        ///
        /// * `register_presets` - A slice of tuples, the first entry being the [`RegisterId`] and the second being the expected value.
        fn build_registers(&mut self, register_presets: &[(RegisterId, u32)]) {
            // This should be done before the registers are set because there will be instances
            // there the number of executed instructions will be different than the total
            // instruction count, such as if we hit a halt or early return.
            let size: u32 = self
                .instructions
                .iter()
                .map(|i| OpCode::from(*i).get_total_instruction_size())
                .sum();

            self.registers
                .get_register_u32_mut(RegisterId::IP)
                .write_unchecked(size);
            self.registers
                .get_register_u32_mut(RegisterId::PC)
                .write_unchecked(self.instructions.len() as u32);

            for (reg, val) in register_presets {
                self.registers
                    .get_register_u32_mut(*reg)
                    .write_unchecked(*val);
            }
        }

        /// Run this specific test entry.
        ///
        /// # Arguments
        ///
        /// * `id` - The ID of this test.
        pub fn run_test(&self, id: usize) -> (Memory, Cpu) {
            let result = panic::catch_unwind(|| {
                let (mut mem, mut cpu) = create_instance();
                cpu.run_instructions(&mut mem, &self.instructions);

                (mem, cpu)
            });

            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                self.should_panic,
                "{}",
                self.fail_message(id, did_panic)
            );

            if did_panic {
                return create_instance();
            }

            let (mem, cpu) = result.unwrap();
            assert_eq!(
                cpu.registers,
                self.registers,
                "{}",
                self.fail_message(id, false)
            );

            assert_eq!(
                mem.get_storage(),
                self.expected_memory,
                "{}",
                self.fail_message(id, false)
            );

            (mem, cpu)
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
        let tests = [TestEntryU32Standard::new(
            &[Instruction::Nop],
            &[],
            vec![0; 100],
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
            TestEntryU32Standard::new(
                &[Instruction::Hlt],
                &[],
                vec![0; 100],
                false,
                "failed to execute HLT instruction",
            ),
            // The halt instruction should prevent any following instructions from executing, which
            // means that the program counter should also not increase.
            TestEntryU32Standard::new(
                &[Instruction::Hlt, Instruction::Nop],
                &[(RegisterId::IP, 4), (RegisterId::PC, 1)],
                vec![0; 100],
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
        let tests = [TestEntryU32Standard::new(
            &[Instruction::Mret],
            &[],
            vec![0; 100],
            false,
            "failed to execute MRET instruction",
        )];

        for (id, test) in tests.iter().enumerate() {
            let (_, cpu) = test.run_test(id);
            assert!(
                !cpu.is_machine_mode,
                "Test {id} Failed - machine is still in machine mode after executing mret instruction!"
            );
        }
    }

    /// Test the add u32 immediate to u32 register instruction.
    #[test]
    fn test_add_u32_imm_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::AddU32ImmU32Reg(0x2, RegisterId::R1)
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::AC, 0x3)],
                vec![0; 100],
                false,
                "failed to correctly execute ADD instruction"
            ),
            // This test should cause the CPU's overflow flag to be set.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    Instruction::AddU32ImmU32Reg(0x2, RegisterId::R1)
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
                &[Instruction::AddU32ImmU32Reg(0, RegisterId::R1)],
                &[(RegisterId::FL, 0b00000010)],
                vec![0; 100],
                false,
                "failed to correctly execute ADD instruction: zero CPU register flag was not correctly set"
            ),
            // This test should succeed in machine mode.
            TestEntryU32Standard::new(
                &[Instruction::AddU32ImmU32Reg(0x1, RegisterId::TEST0)],
                &[(RegisterId::AC, 0x1)],
                vec![0; 100],
                false,
                "failed to correctly execute ADD instruction in machine mode"
            ),
            // This test should fail in user mode due to the register permissions.
            TestEntryU32Standard::new(
                &[
                    Instruction::Mret,
                    Instruction::AddU32ImmU32Reg(0x2, RegisterId::TEST0)
                ],
                &[],
                vec![0; 100],
                true,
                "succeeded in execute ADD instruction in user mode"
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the add u32 register to u32 register instruction.
    #[test]
    fn test_add_u32_reg_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    // Setup register values.
                    Instruction::MovU32ImmU32Reg(0xf, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
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
                    Instruction::MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x2, RegisterId::R2),
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
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
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
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::AddU32RegU32Reg(RegisterId::R1, RegisterId::TEST0),
                ],
                &[],
                vec![0; 100],
                true,
                "succeeded in execute ADD instruction in user mode"
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the move u32 immediate to u32 register instruction.
    #[test]
    fn test_move_u32_imm_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1)],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "failed to correctly execute MOV instruction",
            ),
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2)],
                vec![0; 100],
                false,
                "failed to correctly execute MOV instruction",
            ),
            // This test should succeed in machine mode.
            TestEntryU32Standard::new(
                &[Instruction::MovU32ImmU32Reg(0x1, RegisterId::TEST0)],
                &[(RegisterId::TEST0, 0x1)],
                vec![0; 100],
                false,
                "failed to correctly execute MOV instruction in machine mode",
            ),
            // This test should fail in user mode due to the register permissions.
            TestEntryU32Standard::new(
                &[
                    Instruction::Mret,
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::TEST0),
                ],
                &[],
                vec![0; 100],
                true,
                "succeeded in execute MOV instruction in user mode",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the move u32 register to u32 register instruction.
    #[test]
    fn test_move_u32_reg_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::MovU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::R2, 0x1)],
                vec![0; 100],
                false,
                "failed to correctly execute MOV instruction",
            ),
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x2, RegisterId::R2),
                    Instruction::MovU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::R2, 0x1)],
                vec![0; 100],
                false,
                "failed to correctly execute MOV instruction",
            ),
            // This test should succeed in machine mode.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::MovU32RegU32Reg(RegisterId::R1, RegisterId::TEST0),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::TEST0, 0x1)],
                vec![0; 100],
                false,
                "failed to correctly execute MOV instruction in machine mode",
            ),
            // This test should fail in user mode due to the register permissions.
            TestEntryU32Standard::new(
                &[
                    Instruction::Mret,
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::MovU32RegU32Reg(RegisterId::R1, RegisterId::TEST0),
                ],
                &[],
                vec![0; 100],
                true,
                "succeeded in execute MOV instruction in user mode",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the move u32 literal to memory instruction.
    #[test]
    fn test_move_u32_lit_mem() {
        let tests = [TestEntryU32Standard::new(
            &[Instruction::MovU32ImmMemRelSimple(0x123, 0x0)],
            &[],
            vec![
                35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ],
            false,
            "failed to correctly execute MOV instruction",
        )];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the move u32 register to memory instruction.
    #[test]
    fn test_move_u32_reg_mem() {
        let tests = [TestEntryU32Standard::new(
            &[
                Instruction::MovU32ImmU32Reg(0x123, RegisterId::R1),
                Instruction::MovU32RegMemRelSimple(RegisterId::R1, 0x0),
            ],
            &[(RegisterId::R1, 0x123)],
            vec![
                35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ],
            false,
            "failed to correctly execute MOV instruction",
        )];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the move u32 register to memory instruction.
    #[test]
    fn test_move_mem_u32_reg() {
        let tests = [TestEntryU32Standard::new(
            &[
                Instruction::MovU32ImmU32Reg(0x123, RegisterId::R1),
                Instruction::MovU32RegMemRelSimple(RegisterId::R1, 0x0),
                Instruction::MovU32ImmU32Reg(0x0, RegisterId::R1),
                Instruction::MovMemU32RegRelSimple(0x0, RegisterId::R2),
            ],
            &[(RegisterId::R2, 0x123)],
            vec![
                35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ],
            false,
            "failed to correctly execute MOV instruction",
        )];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the move u32 to register from address provided by a different register.
    #[test]
    fn test_move_u32_reg_ptr_u32_reg() {
        let tests = [TestEntryU32Standard::new(
            &[
                // Store value in memory.
                Instruction::MovU32ImmU32Reg(0x123, RegisterId::R1),
                Instruction::MovU32RegMemRelSimple(RegisterId::R1, 0x0),
                Instruction::MovU32ImmU32Reg(0x0, RegisterId::R1),
                // Set the address pointer in R2.
                Instruction::MovU32ImmU32Reg(0x0, RegisterId::R2),
                // Read the value from the address of R2 into R1.
                Instruction::MovU32RegPtrU32RegRelSimple(RegisterId::R2, RegisterId::R1),
            ],
            &[(RegisterId::R1, 0x123)],
            vec![
                35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ],
            false,
            "failed to correctly execute MOV instruction",
        )];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the swap u32 register to u32 register instruction.
    #[test]
    fn test_swap_u32_reg_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x2, RegisterId::R2),
                    Instruction::SwapU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::R2, 0x1)],
                vec![0; 100],
                false,
                "failed to correctly execute SWAP instruction",
            ),
            // This test should succeed in machine mode.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x2, RegisterId::TEST0),
                    Instruction::SwapU32RegU32Reg(RegisterId::R1, RegisterId::TEST0),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::TEST0, 0x1)],
                vec![0; 100],
                false,
                "failed to correctly execute SWAP instruction in machine mode",
            ),
            // This test should fail in user mode due to the register permissions.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x2, RegisterId::TEST0),
                    Instruction::Mret,
                    Instruction::SwapU32RegU32Reg(RegisterId::R1, RegisterId::TEST0),
                ],
                &[],
                vec![0; 100],
                true,
                "succeeded in execute SWAP instruction in user mode",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the complex move value to expression-derived memory address.
    #[test]
    fn test_move_u32_imm_expr() {
        let mut handler = MoveExpressionHandler::new();

        let test_1_args = [
            Instruction::MovU32ImmMemRelExpr(
                0x123,
                handler.encode(&[
                    ExpressionArgs::Constant(0x8),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Constant(0x8),
                ]),
            ),
            Instruction::Hlt,
        ];

        let test_2_args = [
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R1),
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R2),
            Instruction::MovU32ImmMemRelExpr(
                0x123,
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Register(RegisterId::R2),
                ]),
            ),
            Instruction::Hlt,
        ];

        let test_3_args = [
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R1),
            Instruction::MovU32ImmMemRelExpr(
                0x123,
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Constant(0x8),
                ]),
            ),
            Instruction::Hlt,
        ];

        let test_4_args = [
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::TEST0),
            Instruction::MovU32ImmMemRelExpr(
                0x123,
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::TEST0),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Constant(0x8),
                ]),
            ),
            Instruction::Hlt,
        ];

        let test_5_args = [
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::TEST0),
            Instruction::Mret,
            Instruction::MovU32ImmMemRelExpr(
                0x123,
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::TEST0),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Constant(0x8),
                ]),
            ),
            Instruction::Hlt,
        ];

        let test_6_args = [
            Instruction::MovU32ImmU32Reg(0x2, RegisterId::R1),
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R2),
            Instruction::MovU32ImmMemRelExpr(
                0x123,
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::R2),
                ]),
            ),
            Instruction::Hlt,
        ];

        let test_7_args = [
            Instruction::MovU32ImmU32Reg(0x1A, RegisterId::R1),
            Instruction::MovU32ImmU32Reg(0x4, RegisterId::R2),
            Instruction::MovU32ImmMemRelExpr(
                0x123,
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Subtract),
                    ExpressionArgs::Register(RegisterId::R2),
                ]),
            ),
            Instruction::Hlt,
        ];

        let tests = [
            // Test with two constant values.
            TestEntryU32Standard::new(
                &test_1_args,
                &[],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "failed to correctly execute MOV instruction",
            ),
            // Test with two register values.
            TestEntryU32Standard::new(
                &test_2_args,
                &[(RegisterId::R1, 0x8), (RegisterId::R2, 0x8)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "failed to correctly execute MOV instruction",
            ),
            // Test with a constant and a register.
            TestEntryU32Standard::new(
                &test_3_args,
                &[(RegisterId::R1, 0x8)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "failed to correctly execute MOV instruction",
            ),
            // This test should succeed in machine mode.
            TestEntryU32Standard::new(
                &test_4_args,
                &[(RegisterId::TEST0, 0x8)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "failed to correctly execute MOV instruction in machine mode",
            ),
            // This test should fail in user mode due to the register permissions.
            TestEntryU32Standard::new(
                &test_5_args,
                &[],
                vec![],
                true,
                "succeeded in execute MOV instruction in user mode",
            ),
            // Test with multiplication.
            TestEntryU32Standard::new(
                &test_6_args,
                &[(RegisterId::R1, 0x2), (RegisterId::R2, 0x8)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "failed to correctly execute MOV instruction",
            ),
            // Test with subtraction.
            TestEntryU32Standard::new(
                &test_7_args,
                &[(RegisterId::R1, 0x1A), (RegisterId::R2, 0x4)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "failed to correctly execute MOV instruction",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }
}
