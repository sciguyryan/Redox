use core::fmt;
use std::{panic, slice::Iter};

use crate::{
    ins::{
        instruction::Instruction,
        move_expressions::{ExpressionArgs, MoveExpressionHandler},
    },
    mem::memory::Memory,
    privilege_level::PrivilegeLevel,
    reg::registers::{RegisterId, Registers},
    utils,
};

/// The mask to get only the lowest 8 bits of a u32 value.
const U32_LOW_BYTE_MASK: u32 = 0xff;

pub struct Cpu {
    pub registers: Registers,
    pub is_halted: bool,
    pub is_machine_mode: bool,
    pub is_in_interrupt_handler: bool,
}

impl Cpu {
    pub fn new() -> Self {
        Self {
            registers: Registers::default(),
            is_halted: false,
            is_machine_mode: true,
            is_in_interrupt_handler: false,
        }
    }

    /// Calculate the parity of the lowest byte of a u32 value.
    ///
    /// # Arguments
    ///
    /// * `value` - The input value.
    ///
    /// # Returns
    ///
    /// A boolean, true indicating that the parity bit should be 1, false indicating it should be 0.
    #[inline(always)]
    fn calculate_lowest_byte_parity(value: u32) -> bool {
        (value & U32_LOW_BYTE_MASK).count_ones() % 2 == 0
    }

    /// Decode and evaluate a move instruction expression.
    ///
    /// # Arguments
    ///
    /// * `expr` - The encoded move expression.
    /// * `privilege` - The [`PrivilegeLevel`] in which this expression should be executed.
    ///
    /// # Returns
    ///
    /// A u32 that is the calculated result of the expression.
    fn decode_evaluate_u32_move_expression(
        &mut self,
        expr: &u32,
        privilege: &PrivilegeLevel,
    ) -> u32 {
        let mut decoder = MoveExpressionHandler::new();
        decoder.unpack(*expr);

        // Determine the first and second operands.
        let value_1 = match decoder.operand_1 {
            ExpressionArgs::Register(id) => self.read_reg_u32(&id, privilege),
            ExpressionArgs::Immediate(val) => val as u32,
            _ => panic!(),
        };
        let value_2 = match decoder.operand_2 {
            ExpressionArgs::Register(id) => self.read_reg_u32(&id, privilege),
            ExpressionArgs::Immediate(val) => val as u32,
            _ => panic!(),
        };
        let value_3 = if decoder.is_extended {
            match decoder.operand_3 {
                ExpressionArgs::Register(id) => self.read_reg_u32(&id, privilege),
                ExpressionArgs::Immediate(val) => val as u32,
                _ => panic!(),
            }
        } else {
            0
        };

        decoder.evaluate(value_1, value_2, value_3)
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
    fn fetch_decode_next_instruction(&mut self, mem: &Memory) -> Option<Instruction> {
        // Get the current instruction pointer.
        let ip = *self
            .registers
            .get_register_u32(RegisterId::IP)
            .read_unchecked() as usize;

        // Are we within valid memory bounds?
        if ip < mem.len() {
            Some(mem.get_instruction(ip))
        } else {
            None
        }
    }

    /// Get the size of the current stack frame.
    #[inline(always)]
    pub fn get_stack_frame_size(&self) -> u32 {
        self.registers
            .get_register_u32(RegisterId::BP)
            .read_unchecked()
            - self
                .registers
                .get_register_u32(RegisterId::SP)
                .read_unchecked()
    }

    /// Get the value of the stack pointer (SP) register.
    #[inline(always)]
    pub fn get_stack_pointer(&self) -> u32 {
        *self
            .registers
            .get_register_u32(RegisterId::SP)
            .read_unchecked()
    }

    /// Update the instruction pointer (IP) register.
    ///
    /// # Arguments
    ///
    /// * `size` - The size of the instruction, in bytes.
    #[inline(always)]
    fn increment_instruction_pointer(&mut self, size: u32) {
        self.registers
            .get_register_u32_mut(RegisterId::IP)
            .add_unchecked(size);
    }

    /// Update the program counter (PC) register.
    #[inline(always)]
    fn increment_program_counter_register(&mut self) {
        self.registers
            .get_register_u32_mut(RegisterId::PC)
            .increment_unchecked();
    }

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
    /// This method affects the following flags: Sign (SF), Overflow (OF), Zero (ZF) and Parity (PF). Any other flags are undefined.
    #[inline(always)]
    fn perform_checked_add_u32(&mut self, value_1: u32, value_2: u32) -> u32 {
        let (final_value, overflow) = match value_1.checked_add(value_2) {
            Some(val) => (val, false),
            None => (value_1.wrapping_add(value_2), true),
        };

        self.set_standard_flags_by_value(final_value, overflow);

        final_value
    }

    /// Perform a checked subtraction of two u32 values.
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
    /// This method affects the following flags: Sign (SF), Overflow (OF), Zero (ZF) and Parity (PF). Any other flags are undefined.
    #[inline(always)]
    fn perform_checked_subtract_u32(&mut self, value_1: u32, value_2: u32) -> u32 {
        let (final_value, overflow) = match value_1.checked_sub(value_2) {
            Some(val) => (val, false),
            None => (value_1.wrapping_sub(value_2), true),
        };

        self.set_standard_flags_by_value(final_value, overflow);

        final_value
    }

    /// Perform a checked multiply of two u32 values.
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
    /// This method affects the following flags: Overflow (OF) and Carry (CF). Any other flags are undefined.
    #[inline(always)]
    fn perform_checked_mul_u32(&mut self, value_1: u32, value_2: u32) -> u32 {
        let final_value = value_1.wrapping_mul(value_2);

        // The overflow and carry flags will be true if the upper bits 16
        // bits of the value are not zero.
        let state = (final_value >> 16) != 0;
        self.set_flag_state(CpuFlag::OF, state);
        self.set_flag_state(CpuFlag::CF, state);

        final_value
    }

    /// Perform a checked integer division of two u32 values.
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
    /// This method affects the following flags: none. All flags are undefined.
    #[inline(always)]
    fn perform_checked_div_u32(&mut self, value_1: u32, value_2: u32) -> u32 {
        // TODO - when implementing exceptions and interrupts, ensure one is triggered with division by zero.

        value_1.wrapping_div(value_2)
    }

    /// Perform an integer modulo of two u32 values.
    ///
    /// # Arguments
    ///
    /// * `value_1` - The first u32 value.
    /// * `value_2` - The second u32 value.
    ///
    /// # Returns
    ///
    /// The result of the operation.
    ///
    /// # Note
    ///
    /// This method affects the following flags: none. All flags are undefined.
    #[inline(always)]
    fn perform_modulo_u32(&mut self, value_1: u32, value_2: u32) -> u32 {
        value_1 % value_2
    }

    /// Perform a bitwise and of two u32 values.
    ///
    /// # Arguments
    ///
    /// * `value_1` - The first u32 value.
    /// * `value_2` - The second u32 value.
    ///
    /// # Returns
    ///
    /// The result of the operation.
    ///
    /// # Note
    ///
    /// This method affects the following flags: Sign (SF), Zero (ZF) and Parity (PF).
    ///
    /// The Overflow (OF) and Carry (CF) flags will always be cleared. Any other flags are undefined.
    #[inline(always)]
    fn perform_bitwise_and_u32(&mut self, value_1: u32, value_2: u32) -> u32 {
        let new_value = value_1 & value_2;

        self.set_flag_state(CpuFlag::SF, utils::is_bit_set(new_value, 31));
        self.set_flag_state(CpuFlag::ZF, new_value == 0);
        self.set_flag_state(CpuFlag::OF, false);
        self.set_flag_state(CpuFlag::PF, Cpu::calculate_lowest_byte_parity(new_value));
        self.set_flag_state(CpuFlag::CF, false);

        new_value
    }

    /// Perform a checked left-shift of two u32 values.
    ///
    /// # Arguments
    ///
    /// * `value` - The first u32 value.
    /// * `shift_by` - The second u32 value.
    ///
    /// # Returns
    ///
    /// The result of the operation, truncated in the case of an overflow.
    ///
    /// # Note
    ///
    /// This method affects the following flags: Sign (SF), Overflow (OF), Zero (ZF), Carry (CF) and Parity (PF).
    ///
    /// The Overflow (OF) flag will only be affected by 1-bit shifts. Any other flags are undefined.
    #[inline(always)]
    fn perform_checked_left_shift_u32(&mut self, value: u32, shift_by: u32) -> u32 {
        if shift_by == 0 {
            return value;
        }

        assert!(shift_by <= 31);

        let final_value = value << shift_by;
        self.set_flag_state(CpuFlag::SF, utils::is_bit_set(final_value, 31));
        self.set_flag_state(CpuFlag::ZF, final_value == 0);
        if shift_by == 1 {
            self.set_flag_state(CpuFlag::OF, final_value < value);
        }
        self.set_flag_state(
            CpuFlag::CF,
            utils::is_bit_set_32(value, (32 - shift_by) & 1),
        );
        self.set_flag_state(CpuFlag::PF, Cpu::calculate_lowest_byte_parity(final_value));

        final_value
    }

    /// Perform an arithmetic left-shift of two u32 values.
    ///
    /// # Arguments
    ///
    /// * `value` - The first u32 value.
    /// * `shift_by` - The second u32 value.
    ///
    /// # Returns
    ///
    /// The result of the operation.
    ///
    /// # Note
    ///
    /// This method affects the following flags: Sign (SF), Zero (ZF) and Parity (PF).
    ///
    /// The Overflow (OF) and Carry (CF) flags will always be cleared. Any other flags are undefined.
    #[inline(always)]
    fn perform_arithmetic_left_shift_u32(&mut self, value: u32, shift_by: u32) -> u32 {
        let final_value = value.rotate_left(shift_by);
        self.set_flag_state(CpuFlag::SF, utils::is_bit_set(final_value, 31));
        self.set_flag_state(CpuFlag::ZF, final_value == 0);
        self.set_flag_state(CpuFlag::OF, false);
        self.set_flag_state(CpuFlag::CF, false);
        self.set_flag_state(CpuFlag::PF, Cpu::calculate_lowest_byte_parity(final_value));

        final_value
    }

    /// Perform a right-shift of two u32 values.
    ///
    /// # Arguments
    ///
    /// * `value` - The first u32 value.
    /// * `shift_by` - The second u32 value.
    ///
    /// # Returns
    ///
    /// The result of the operation.
    ///
    /// # Note
    ///
    /// This method affects the following flags: Sign (SF), Zero (ZF), Carry (CF) and Parity (PF).
    ///
    /// The Overflow (OF) flag will always be cleared. Any other flags are undefined.
    #[inline(always)]
    fn perform_right_shift_u32(&mut self, value: u32, shift_by: u32) -> u32 {
        if shift_by == 0 {
            return value;
        }

        assert!(shift_by <= 31);

        let final_value = value >> shift_by;
        self.set_flag_state(CpuFlag::SF, utils::is_bit_set(final_value, 31));
        self.set_flag_state(CpuFlag::ZF, final_value == 0);
        self.set_flag_state(CpuFlag::OF, false);
        // The carried forward flag should be the value of the least significant bit being
        // shifted out of the value and so MUST be calculated prior to the shifted value.
        // Since we are working in little-Endian the lowest bit has the lowest index.
        self.set_flag_state(CpuFlag::CF, utils::is_bit_set(value, 0));
        self.set_flag_state(CpuFlag::PF, Cpu::calculate_lowest_byte_parity(final_value));

        final_value
    }

    /// Perform an arithmetic right-shift of two u32 values.
    ///
    /// # Arguments
    ///
    /// * `value` - The first u32 value.
    /// * `shift_by` - The second u32 value.
    ///
    /// # Returns
    ///
    /// The result of the operation.
    ///
    /// # Note
    ///
    /// This method affects the following flags: Sign (SF), Zero (ZF) and Parity (PF).
    ///
    /// The Overflow (OF) and Carry (CF) flags will always be cleared. Any other flags are undefined.
    #[inline(always)]
    fn perform_arithmetic_right_shift_u32(&mut self, value: u32, shift_by: u32) -> u32 {
        let final_value = value.rotate_right(shift_by);
        self.set_flag_state(CpuFlag::SF, utils::is_bit_set(final_value, 31));
        self.set_flag_state(CpuFlag::ZF, final_value == 0);
        self.set_flag_state(CpuFlag::OF, false);
        self.set_flag_state(CpuFlag::CF, false);
        self.set_flag_state(CpuFlag::PF, Cpu::calculate_lowest_byte_parity(final_value));

        final_value
    }

    /// Perform a bit test on the specified value. The carry flag will be set to the value of the bit.
    ///
    /// # Arguments
    ///
    /// * `value` - The u32 value to be tested.
    /// * `bit` - The index of the bit to be tested.
    ///
    /// # Note
    ///
    /// This method affects the following flags: Carry (CF). Any other flags are undefined.
    ///
    /// This method will panic if the bit index is larger than the number of bytes in the value.
    #[inline(always)]
    fn perform_bit_test_with_carry_flag(&mut self, value: u32, bit: u8) {
        // Values must be in range of the number of bits in the type.
        assert!(bit <= 31);

        // Set the carry flag to the current state of the bit.
        self.set_flag_state(CpuFlag::CF, utils::is_bit_set(value, bit));
    }

    /// Perform a bit test on the specified value. The carry flag will be set to the value of the bit.
    ///
    /// # Arguments
    ///
    /// * `value` - The u32 value to be tested.
    /// * `bit` - The index of the bit to be tested.
    /// * `new_state` - The new state of the bit.
    ///
    /// # Note
    ///
    /// This method affects the following flags: Carry (CF). Any other flags are undefined.
    ///
    /// This method will panic if the bit index is larger than the number of bytes in the value.
    #[inline(always)]
    fn perform_bit_test_with_carry_flag_with_set(
        &mut self,
        value: &mut u32,
        bit: u8,
        new_state: bool,
    ) {
        self.perform_bit_test_with_carry_flag(*value, bit);

        // Set the bit state to the new state.
        utils::set_bit_state_inline(value, bit, new_state);
    }

    /// Perform a forward set bit search on the specified value.
    ///
    /// # Arguments
    ///
    /// * `value` - The u32 value to be tested.
    ///
    /// # Note
    ///
    /// This method affects the following flags: ZF (CF). Any other flags are undefined.
    ///
    /// The Zero flag (ZF) will be set if the value is zero, otherwise the flag will be cleared.
    #[inline(always)]
    fn perform_forward_bit_search(&mut self, value: u32) -> u32 {
        self.set_flag_state(CpuFlag::ZF, value == 0);

        value.trailing_zeros()
    }

    /// Perform a reverse set bit search on the specified value.
    ///
    /// # Arguments
    ///
    /// * `value` - The u32 value to be tested.
    ///
    /// # Note
    ///
    /// This method affects the following flags: ZF (CF). Any other flags are undefined.
    ///
    /// The Zero flag (ZF) will be set if the value is zero, otherwise the flag will be cleared.
    #[inline(always)]
    fn perform_reverse_bit_search(&mut self, value: u32) -> u32 {
        self.set_flag_state(CpuFlag::ZF, value == 0);

        value.leading_zeros()
    }

    /// Perform a zero of the high bits of the source value starting from a specified index.
    ///
    /// # Arguments
    ///
    /// * `source_reg` - The source u32 register.
    /// * `index` - The index of the bit to zero from.
    /// * `out_reg` - The destination u32 register.
    /// * `privilege` - The privilege level of the command.
    ///
    ///
    /// This method affects the following flags: Sign (SF), Zero (ZF), Carry (CF) and Parity (PF).
    ///
    /// The Overflow (OF) flag will always be cleared. Any other flags are undefined.
    #[inline(always)]
    fn perform_zero_high_bit_u32_reg(
        &mut self,
        source_reg: &RegisterId,
        index: u32,
        out_reg: &RegisterId,
        privilege: &PrivilegeLevel,
    ) {
        assert!(index <= 31);

        let value = self.read_reg_u32(source_reg, privilege);
        let final_value = if index > 0 {
            value & ((1 << (32 - index)) - 1)
        } else {
            value
        };

        self.set_flag_state(CpuFlag::SF, utils::is_bit_set(final_value, 31));
        self.set_flag_state(CpuFlag::ZF, final_value == 0);

        // The carry flag is set if the if the number contained in the
        // lowest 8 bits is greater than 31 (register size - 1).
        self.set_flag_state(CpuFlag::CF, (final_value & 0xff) > 31);

        // The overflow flag is always cleared.
        self.set_flag_state(CpuFlag::OF, false);

        // Write the value to the output register.
        self.write_reg_u32(out_reg, final_value, privilege);
    }

    /// Get the value of a specific u32 register.
    ///
    /// # Arguments
    ///
    /// * `reg` - Reference to the [`RegisterId`] for the register in question.
    /// * `privilege` - The [`PrivilegeLevel`] with which the read request should be processed.
    ///
    /// # Returns
    ///
    /// The u32 value of the register.
    #[inline(always)]
    fn read_reg_u32(&self, reg: &RegisterId, privilege: &PrivilegeLevel) -> u32 {
        *self.registers.get_register_u32(*reg).read(privilege)
    }

    /// Perform a hard reset on the CPU.
    pub fn reset(&mut self) {
        self.registers.reset();
        self.set_halted(false);
    }

    /// Run the CPU from a specified starting point in memory.
    ///
    /// # Arguments
    ///
    /// * `mem` - A reference to the virtual machine [`Memory`] instance.
    pub fn run(&mut self, mem: &mut Memory) {
        loop {
            if let Some(ins) = self.fetch_decode_next_instruction(mem) {
                self.run_instruction(mem, &ins);
            } else {
                self.is_halted = true;
            }

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

        self.set_halted(true);
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
                let old_value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_add_u32(old_value, *imm);

                self.set_u32_accumulator(new_value);
            }
            Instruction::AddU32RegU32Reg(in_reg, out_reg) => {
                let value1 = self.read_reg_u32(in_reg, privilege);
                let value2 = self.read_reg_u32(out_reg, privilege);
                let new_value = self.perform_checked_add_u32(value1, value2);

                self.set_u32_accumulator(new_value);
            }
            Instruction::SubU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_subtract_u32(old_value, *imm);

                self.set_u32_accumulator(new_value);
            }
            Instruction::SubU32RegU32Imm(reg, imm) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_subtract_u32(*imm, old_value);

                self.set_u32_accumulator(new_value);
            }
            Instruction::SubU32RegU32Reg(reg_1, reg_2) => {
                let reg_1_val = self.read_reg_u32(reg_1, privilege);
                let reg_2_val = self.read_reg_u32(reg_2, privilege);
                let new_value = self.perform_checked_subtract_u32(reg_2_val, reg_1_val);

                self.set_u32_accumulator(new_value);
            }
            Instruction::MulU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_mul_u32(old_value, *imm);

                self.set_u32_accumulator(new_value);
            }
            Instruction::MulU32RegU32Reg(reg_1, reg_2) => {
                let reg_1_val = self.read_reg_u32(reg_1, privilege);
                let reg_2_val = self.read_reg_u32(reg_2, privilege);
                let new_value = self.perform_checked_mul_u32(reg_1_val, reg_2_val);

                self.set_u32_accumulator(new_value);
            }
            Instruction::DivU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_div_u32(old_value, *imm);

                self.set_u32_accumulator(new_value);
            }
            Instruction::DivU32RegU32Imm(reg, imm) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_div_u32(*imm, old_value);

                self.set_u32_accumulator(new_value);
            }
            Instruction::DivU32RegU32Reg(reg_1, reg_2) => {
                let reg_1_val = self.read_reg_u32(reg_1, privilege);
                let reg_2_val = self.read_reg_u32(reg_2, privilege);
                let new_value = self.perform_checked_div_u32(reg_2_val, reg_1_val);

                self.set_u32_accumulator(new_value);
            }
            Instruction::ModU32ImmU32Reg(imm, reg) => {
                let reg_val = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_modulo_u32(reg_val, *imm);

                self.set_u32_accumulator(new_value);
            }
            Instruction::ModU32RegU32Imm(reg, imm) => {
                let reg_val = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_modulo_u32(*imm, reg_val);

                self.set_u32_accumulator(new_value);
            }
            Instruction::ModU32RegU32Reg(reg_1, reg_2) => {
                let reg_1_val = self.read_reg_u32(reg_1, privilege);
                let reg_2_val = self.read_reg_u32(reg_2, privilege);
                let new_value = self.perform_modulo_u32(reg_2_val, reg_1_val);

                self.set_u32_accumulator(new_value);
            }
            Instruction::IncU32Reg(reg) => {
                let value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_add_u32(value, 1);

                self.write_reg_u32(reg, new_value, privilege);
            }
            Instruction::DecU32Reg(reg) => {
                let value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_subtract_u32(value, 1);

                self.write_reg_u32(reg, new_value, privilege);
            }
            Instruction::AndU32ImmU32Reg(imm, reg) => {
                let value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_bitwise_and_u32(*imm, value);

                self.set_u32_accumulator(new_value);
            }

            /******** [Bit Operation Instructions] ********/
            Instruction::LeftShiftU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_checked_left_shift_u32(old_value, *imm);

                self.write_reg_u32(reg, shifted, privilege);
            }
            Instruction::LeftShiftU32RegU32Reg(shift_reg, reg) => {
                let shift_by = self.read_reg_u32(shift_reg, privilege);
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_checked_left_shift_u32(old_value, shift_by);

                self.write_reg_u32(reg, shifted, privilege);
            }
            Instruction::ArithLeftShiftU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_arithmetic_left_shift_u32(old_value, *imm);

                self.write_reg_u32(reg, shifted, privilege);
            }
            Instruction::ArithLeftShiftU32RegU32Reg(shift_reg, reg) => {
                let shift_by = self.read_reg_u32(shift_reg, privilege);
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_arithmetic_left_shift_u32(old_value, shift_by);

                self.write_reg_u32(reg, shifted, privilege);
            }
            Instruction::RightShiftU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_right_shift_u32(old_value, *imm);

                self.write_reg_u32(reg, shifted, privilege);
            }
            Instruction::RightShiftU32RegU32Reg(shift_reg, reg) => {
                let shift_by = self.read_reg_u32(shift_reg, privilege);
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_right_shift_u32(old_value, shift_by);

                self.write_reg_u32(reg, shifted, privilege);
            }
            Instruction::ArithRightShiftU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_arithmetic_right_shift_u32(old_value, *imm);

                self.write_reg_u32(reg, shifted, privilege);
            }
            Instruction::ArithRightShiftU32RegU32Reg(shift_reg, reg) => {
                let shift_by = self.read_reg_u32(shift_reg, privilege);
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_arithmetic_right_shift_u32(old_value, shift_by);

                self.write_reg_u32(reg, shifted, privilege);
            }

            /******** [Branching Instructions] ********/
            Instruction::Int(addr) => {
                todo!();
            }
            Instruction::IntRet => {
                todo!();
            }

            /******** [Data Instructions] ********/
            Instruction::SwapU32RegU32Reg(reg_1, reg_2) => {
                let reg_1_val = self.read_reg_u32(reg_1, privilege);
                let reg_2_val = self.read_reg_u32(reg_2, privilege);

                self.write_reg_u32(reg_1, reg_2_val, privilege);
                self.write_reg_u32(reg_2, reg_1_val, privilege);
            }
            Instruction::MovU32ImmU32Reg(imm, reg) => {
                self.write_reg_u32(reg, *imm, privilege);
            }
            Instruction::MovU32RegU32Reg(in_reg, out_reg) => {
                let value = self.read_reg_u32(in_reg, privilege);

                self.write_reg_u32(out_reg, value, privilege);
            }
            Instruction::MovU32ImmMemRelSimple(imm, addr) => {
                mem.set_u32(*addr as usize, *imm);
            }
            Instruction::MovU32RegMemRelSimple(reg, addr) => {
                let value = self.read_reg_u32(reg, privilege);

                mem.set_u32(*addr as usize, value);
            }
            Instruction::MovMemU32RegRelSimple(addr, reg) => {
                let value = mem.get_u32(*addr as usize);

                self.write_reg_u32(reg, value, privilege);
            }
            Instruction::MovU32RegPtrU32RegRelSimple(in_reg, out_reg) => {
                let address = self.read_reg_u32(in_reg, privilege) as usize;
                let value = mem.get_u32(address);

                self.write_reg_u32(out_reg, value, privilege);
            }
            Instruction::MovU32ImmMemExprRel(imm, expr) => {
                // mov imm, [addr] - move immediate to address.
                let addr = self.decode_evaluate_u32_move_expression(expr, privilege);

                mem.set_u32(addr as usize, *imm);
            }
            Instruction::MovMemExprU32RegRel(expr, reg) => {
                // mov [addr], register - move value at address to register.
                let addr = self.decode_evaluate_u32_move_expression(expr, privilege);
                let value = mem.get_u32(addr as usize);

                self.write_reg_u32(reg, value, privilege);
            }
            Instruction::MovU32RegMemExprRel(reg, expr) => {
                // mov reg, [addr] - move value of a register to an address.
                let addr = self.decode_evaluate_u32_move_expression(expr, privilege);
                let value = self.read_reg_u32(reg, privilege);

                mem.set_u32(addr as usize, value);
            }
            Instruction::ZeroHighBitsByIndexU32Reg(index_reg, source_reg, out_reg) => {
                // zhbi source_reg, index_reg, out_reg
                let index = self.read_reg_u32(index_reg, privilege);
                self.perform_zero_high_bit_u32_reg(source_reg, index, out_reg, privilege);
            }
            Instruction::ZeroHighBitsByIndexU32RegU32Imm(index, source_reg, out_reg) => {
                // zhbi source_reg, index, out_reg
                self.perform_zero_high_bit_u32_reg(source_reg, *index, out_reg, privilege);
            }
            Instruction::PushU32Imm(imm) => {
                // push imm
                mem.push_u32(*imm);

                // Update the stack pointer register. The
                self.registers
                    .get_register_u32_mut(RegisterId::SP)
                    .subtract_unchecked(4);
            }

            /******** [Logic Instructions] ********/
            Instruction::BitTestU32Reg(bit, reg) => {
                // bt bit, reg
                let value = self.read_reg_u32(reg, privilege);
                self.perform_bit_test_with_carry_flag(value, *bit);
            }
            Instruction::BitTestU32Mem(bit, addr) => {
                // bt bit, [addr]
                let value = mem.get_u32(*addr as usize);
                self.perform_bit_test_with_carry_flag(value, *bit);
            }
            Instruction::BitTestResetU32Reg(bit, reg) => {
                // btr bit, reg
                // Read the value and set the carry flag state, clear the bit.
                let mut value = self.read_reg_u32(reg, privilege);
                self.perform_bit_test_with_carry_flag_with_set(&mut value, *bit, false);

                // Write the value back to the register.
                self.write_reg_u32(reg, value, privilege);
            }
            Instruction::BitTestResetU32Mem(bit, addr) => {
                // btr bit, [addr]
                // Read the value and set the carry flag state, then clear the bit.
                let mut value = mem.get_u32(*addr as usize);
                self.perform_bit_test_with_carry_flag_with_set(&mut value, *bit, false);

                mem.set_u32(*addr as usize, value);
            }
            Instruction::BitTestSetU32Reg(bit, reg) => {
                // bts bit, reg
                // Read the value and set the carry flag state, set the bit.
                let mut value = self.read_reg_u32(reg, privilege);
                self.perform_bit_test_with_carry_flag_with_set(&mut value, *bit, true);

                // Write the value back to the register.
                self.write_reg_u32(reg, value, privilege);
            }
            Instruction::BitTestSetU32Mem(bit, addr) => {
                // bts bit, [addr]
                // Read the value and set the carry flag state, then set the bit.
                let mut value = mem.get_u32(*addr as usize);
                self.perform_bit_test_with_carry_flag_with_set(&mut value, *bit, true);

                mem.set_u32(*addr as usize, value);
            }
            Instruction::BitScanReverseU32RegU32Reg(in_reg, out_reg) => {
                // bsr in_reg, out_reg
                let value = self.read_reg_u32(in_reg, privilege);
                let index = self.perform_reverse_bit_search(value);

                self.write_reg_u32(out_reg, index, privilege);
            }
            Instruction::BitScanReverseU32MemU32Reg(addr, reg) => {
                // bsr [addr], reg
                let value = mem.get_u32(*addr as usize);
                let index = self.perform_reverse_bit_search(value);

                self.write_reg_u32(reg, index, privilege);
            }
            Instruction::BitScanReverseU32RegMemU32(reg, out_addr) => {
                // bsr reg, [out_addr]
                let value = self.read_reg_u32(reg, privilege);
                let index = self.perform_reverse_bit_search(value);

                mem.set_u32(*out_addr as usize, index);
            }
            Instruction::BitScanReverseU32MemU32Mem(in_addr, out_addr) => {
                // bsr [in_addr], [out_addr]
                let value = mem.get_u32(*in_addr as usize);
                let index = self.perform_reverse_bit_search(value);

                mem.set_u32(*out_addr as usize, index);
            }
            Instruction::BitScanForwardU32RegU32Reg(in_reg, out_reg) => {
                // bsf in_reg, out_reg
                let value = self.read_reg_u32(in_reg, privilege);
                let index = self.perform_forward_bit_search(value);

                self.write_reg_u32(out_reg, index, privilege);
            }
            Instruction::BitScanForwardU32MemU32Reg(addr, reg) => {
                // bsf [addr], reg
                let value = mem.get_u32(*addr as usize);
                let index = self.perform_forward_bit_search(value);

                self.write_reg_u32(reg, index, privilege);
            }
            Instruction::BitScanForwardU32RegMemU32(reg, out_addr) => {
                // bsf reg, [out_addr]
                let value = self.read_reg_u32(reg, privilege);
                let index = self.perform_forward_bit_search(value);

                mem.set_u32(*out_addr as usize, index);
            }
            Instruction::BitScanForwardU32MemU32Mem(in_addr, out_addr) => {
                // bsf [in_addr], [out_addr]
                let value = mem.get_u32(*in_addr as usize);
                let index = self.perform_forward_bit_search(value);

                mem.set_u32(*out_addr as usize, index);
            }
            Instruction::ByteSwapU32(reg) => {
                // bswap reg
                let value = self.read_reg_u32(reg, privilege).swap_bytes();

                self.write_reg_u32(reg, value, privilege);
            }

            /******** [Special Instructions] ********/
            Instruction::Ret => {
                todo!();
            }
            Instruction::Mret => {
                self.set_machine_mode(false);
            }
            Instruction::Hlt => {
                self.set_halted(true);
            }
        };

        self.increment_program_counter_register();

        // Move the instruction pointer register forward by the number of
        // bytes used to build the instruction.
        self.increment_instruction_pointer(instruction.get_total_instruction_size());
    }

    /// Set the state of the specified CPU flag.
    ///
    /// # Arguments
    ///
    /// * `flag` - The [`CpuFlag`] to be set or unset.
    /// * `state` - The new state of the flag.
    #[inline(always)]
    pub fn set_flag_state(&mut self, flag: CpuFlag, state: bool) {
        let register = self.registers.get_register_u32_mut(RegisterId::FL);
        let flags = utils::set_bit_state(*register.read_unchecked(), flag.into(), state);
        register.write_unchecked(flags);
    }

    /// Set the halted state of the processor.
    ///
    /// # Arguments
    ///
    /// * `state` - Whether the CPU should be halted or not.
    #[inline(always)]
    fn set_halted(&mut self, state: bool) {
        self.is_halted = state;
    }

    /// Update the instruction pointer (IP) register.
    ///
    /// # Arguments
    ///
    /// * `value` - The new value of the instruction pointer register.
    #[inline(always)]
    pub fn set_instruction_pointer(&mut self, value: u32) {
        self.registers
            .get_register_u32_mut(RegisterId::IP)
            .write_unchecked(value);
    }

    /// Set the machine mode privilege level of the processor.
    ///
    /// # Arguments
    ///
    /// * `state` - Whether the CPU should be running in machine mode or not.
    #[inline(always)]
    fn set_machine_mode(&mut self, state: bool) {
        self.is_machine_mode = state;
    }

    /// Update the various segment registers in the CPU.
    ///
    /// # Arguments
    ///
    /// * `mem` - A reference to the VM [`Memory`] instance.
    #[inline(always)]
    pub fn set_segment_registers(&mut self, mem: &Memory) {
        self.registers
            .get_register_u32_mut(RegisterId::SS)
            .write_unchecked(mem.stack_segment_start as u32);

        self.registers
            .get_register_u32_mut(RegisterId::CS)
            .write_unchecked(mem.code_segment_start as u32);

        self.registers
            .get_register_u32_mut(RegisterId::DS)
            .write_unchecked(mem.data_segment_start as u32);
    }

    /// Set the standard CPU flags based on the specified value and overflow.
    ///
    /// # Arguments
    ///
    /// * `value` - The input u32 value
    /// * `is_overflow` - Did the operation result in an overflow?
    ///
    /// # Note
    ///
    /// This method affects the following flags: Sign (SF), Overflow (OF), Zero (ZF) and Parity (PF).
    #[inline(always)]
    fn set_standard_flags_by_value(&mut self, value: u32, is_overflow: bool) {
        self.set_flag_state(CpuFlag::SF, utils::is_bit_set(value, 31));
        self.set_flag_state(CpuFlag::ZF, value == 0);
        self.set_flag_state(CpuFlag::OF, is_overflow);
        self.set_flag_state(CpuFlag::PF, Cpu::calculate_lowest_byte_parity(value));
    }

    /// Update the (stack frame) base pointer (BP) register.
    ///
    /// # Arguments
    ///
    /// * `value` - The new value of the base pointer register.
    #[inline(always)]
    pub fn set_stack_frame_base_pointer(&mut self, value: u32) {
        self.registers
            .get_register_u32_mut(RegisterId::BP)
            .write_unchecked(value);
    }

    /// Setup the registers that are required for running the CPU.
    ///
    /// # Arguments
    ///
    /// * `mem` - A reference to the VM [`Memory`] instance.
    #[inline(always)]
    pub fn synchronize_registers(&mut self, mem: &Memory) {
        // Update the segment registers, now that we know where the segments
        // are located in RAM.
        self.set_segment_registers(mem);

        // Configure the CPU instruction pointer.
        self.set_instruction_pointer(mem.code_segment_start as u32);

        // Configure the CPU registers to account for the new stack pointer.
        self.set_stack_frame_base_pointer(mem.stack_segment_end as u32);
        self.set_stack_pointer(mem.stack_segment_end as u32);
    }

    // Update the stack pointer (SP) register.
    ///
    /// # Arguments
    ///
    /// * `value` - The new value of the stack pointer register.
    #[inline(always)]
    pub fn set_stack_pointer(&mut self, value: u32) {
        self.registers
            .get_register_u32_mut(RegisterId::SP)
            .write_unchecked(value);
    }

    /// Update the u32 accumulator (AC) register.
    ///
    /// # Arguments
    ///
    /// * `value` - The new value of the accumulator register.
    #[inline(always)]
    fn set_u32_accumulator(&mut self, value: u32) {
        self.registers
            .get_register_u32_mut(RegisterId::AC)
            .write_unchecked(value);
    }

    /// Get the value of a specific u32 register.
    ///
    /// # Arguments
    ///
    /// * `reg` - Reference to the [`RegisterId`] for the register in question.
    /// * `new_val` - The new value of the register.
    /// * `privilege` - The [`PrivilegeLevel`] with which the read request should be processed.
    #[inline(always)]
    fn write_reg_u32(&mut self, reg: &RegisterId, new_val: u32, privilege: &PrivilegeLevel) {
        self.registers
            .get_register_u32_mut(*reg)
            .write(new_val, privilege);
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
    /// The signed (negative) flag - set to true if the result of an operation is negative.
    SF,
    /// The zero flag - set to true if the result of an operation is zero.
    ZF,
    /// The overflow flag - set to true if the result of an operation overflowed.
    OF,
    /// The carry flag - set to true if the carry bit has been set by an operation.
    CF,
    // The parity flag - set to true depending on the parity of the bits.
    PF,
    /// The interrupt disabled flag - set to true if interrupts are disabled.
    IF,
}

impl fmt::Display for CpuFlag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CpuFlag::SF => write!(f, "SF"),
            CpuFlag::ZF => write!(f, "ZF"),
            CpuFlag::OF => write!(f, "OF"),
            CpuFlag::CF => write!(f, "CF"),
            CpuFlag::PF => write!(f, "PF"),
            CpuFlag::IF => write!(f, "IF"),
        }
    }
}

impl CpuFlag {
    pub fn iterator() -> Iter<'static, CpuFlag> {
        static FLAGS: [CpuFlag; 6] = [
            CpuFlag::SF,
            CpuFlag::ZF,
            CpuFlag::OF,
            CpuFlag::CF,
            CpuFlag::PF,
            CpuFlag::IF,
        ];
        FLAGS.iter()
    }

    /// Build a flag value from the enabled CPU flags.
    ///
    /// # Arguments
    ///
    /// * `flags` - A slice of [`CpuFlag`]s to be enabled.
    #[inline(always)]
    pub fn compute_from(flags: &[CpuFlag]) -> u32 {
        let mut value = 0;
        for flag in flags {
            utils::set_bit_state_inline(&mut value, *flag as u8, true);
        }
        value
    }
}

impl From<CpuFlag> for u8 {
    fn from(m: CpuFlag) -> u8 {
        m as u8
    }
}

#[cfg(test)]
mod tests_cpu_version_2 {
    use std::{collections::HashMap, panic};

    use num_traits::ToBytes;
    use prettytable::{row, Table};

    use crate::{
        compiler::bytecode_compiler::Compiler,
        ins::{
            instruction::Instruction::{self, *},
            move_expressions::{ExpressionArgs, ExpressionOperator, MoveExpressionHandler},
        },
        mem::memory::Memory,
        reg::registers::{RegisterId, Registers},
        vm::{self, VirtualMachine},
    };

    use super::{Cpu, CpuFlag};

    struct TestEntryU32Standard {
        pub instructions: Vec<Instruction>,
        pub expected_changed_registers: HashMap<RegisterId, u32>,
        pub expected_user_seg_contents: Option<Vec<u8>>,
        pub user_seg_capacity_bytes: usize,
        pub stack_seg_capacity_u32: usize,
        pub should_panic: bool,
        pub fail_message: String,
    }

    impl TestEntryU32Standard {
        /// Create a new [`TestEntryU32Standard`] instance.
        ///
        /// # Arguments
        ///
        /// * `instructions` - A slice of [`Instruction`]s to be executed.
        /// * `expected_registers` - A slice of a tuple containing the [`RegisterId`] and the expected value of the register after execution.
        /// * `expected_user_seg_contents` - An option containing a vector of bytes representing the expected user segment memory contents after execution, if specified.
        /// * `user_seg_capacity_bytes` - The capacity of the user memory segment, in bytes.
        /// * `stack_seg_capacity_u32` - The capacity of the stack memory segment, in bytes.
        /// * `should_panic` - A boolean indicating whether the test should panic or not.
        /// * `fail_message` - A string slice that provides the message to be displayed if the test fails.
        fn new(
            instructions: &[Instruction],
            expected_registers: &[(RegisterId, u32)],
            expected_user_seg_contents: Option<Vec<u8>>,
            user_seg_capacity_bytes: usize,
            stack_seg_capacity_u32: usize,
            should_panic: bool,
            fail_message: &str,
        ) -> Self {
            // Ensure we always end with a halt instruction.
            let mut instructions_vec = instructions.to_vec();
            if let Some(ins) = instructions_vec.last() {
                if !matches!(ins, Hlt) {
                    instructions_vec.push(Hlt);
                }
            }

            // Build the list of registers we expect to change.
            let mut changed_registers = HashMap::new();
            for (id, value) in expected_registers {
                changed_registers.insert(*id, *value);
            }

            Self {
                instructions: instructions_vec,
                expected_changed_registers: changed_registers,
                user_seg_capacity_bytes,
                expected_user_seg_contents,
                stack_seg_capacity_u32,
                should_panic,
                fail_message: fail_message.to_string(),
            }
        }

        /// Run this specific test entry.
        ///
        /// # Arguments
        ///
        /// * `id` - The ID of this test.
        pub fn run_test(&self, id: usize) -> Option<VirtualMachine> {
            // Compile the test code.
            let mut compiler = Compiler::new();
            let compiled = compiler.compile(&self.instructions);

            // Attempt to execute the code.
            let result = panic::catch_unwind(|| {
                // Build the virtual machine instance.
                let mut vm = VirtualMachine::new(
                    self.user_seg_capacity_bytes,
                    compiled,
                    &[],
                    self.stack_seg_capacity_u32 * vm::BYTES_IN_U32,
                );

                // Execute the code.
                vm.run();

                // Return the completed VM instance.
                vm
            });

            // Confirm whether the test panicked, and whether that panic was expected or not.
            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                self.should_panic,
                "{}",
                self.fail_message(id, did_panic)
            );

            // We don't have a viable virtual machine instance to return here.
            if did_panic {
                return None;
            }

            let vm = result.unwrap();

            // This will be used to pretty print an output table in the event we
            // fail the test.
            let mut table = Table::new();
            table.add_row(row!["Register", "Expected Value", "Actual Value"]);

            // Did the test registers match their expected values?
            self.expected_changed_registers
                .iter()
                .for_each(|(id, expected_value)| {
                    let actual_value = *vm.cpu.registers.get_register_u32(*id).read_unchecked();
                    if *expected_value != actual_value {
                        table.add_row(row![
                            id,
                            format!("{expected_value}"),
                            format!("{actual_value}")
                        ]);
                    }
                });

            if table.len() > 1 {
                println!();
                println!("The following registers did not match their expected values:");
                println!("{table}");
                panic!("{}", self.fail_message(id, false));
            }

            // Check the user memory segment matches what we expect too, if
            // it has been specified in the test parameters.
            if let Some(contents) = &self.expected_user_seg_contents {
                assert_eq!(
                    vm.ram.get_user_segment_storage(),
                    contents,
                    "{}",
                    self.fail_message(id, false)
                );
            }

            Some(vm)
        }

        /// Generate a fail message for this test instance.
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

    /// Test the parity checking.
    #[test]
    fn test_parity() {
        let value_1 = 0b0000_0000_0000_0000_0000_0000_1111_1111;
        assert!(
            Cpu::calculate_lowest_byte_parity(value_1),
            "Test 1 - parity check failed: expected true, got false"
        );

        // We are only interested in the lowest byte, everything else should be ignored.
        let value_2 = 0b0000_0000_0000_0000_0000_0001_1111_1111;
        assert!(
            Cpu::calculate_lowest_byte_parity(value_2),
            "Test 2 - parity check failed: expected true, got false"
        );

        let value_3 = 0b0000_0000_0000_0000_0000_0000_0111_1111;
        assert!(
            !Cpu::calculate_lowest_byte_parity(value_3),
            "Test 3 - parity check failed: expected false, got true"
        );
    }

    /// Test the NOP instruction.
    #[test]
    fn test_nop() {
        let tests = [TestEntryU32Standard::new(
            &[Nop],
            &[],
            None,
            100,
            0,
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
                &[Hlt],
                &[],
                None,
                100,
                0,
                false,
                "failed to execute HLT instruction",
            ),
            // The halt instruction should prevent any following instructions from executing, which
            // means that the program counter should also not increase.
            TestEntryU32Standard::new(
                &[Hlt, Nop],
                &[(RegisterId::IP, 104), (RegisterId::PC, 1)],
                None,
                100,
                0,
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
            &[Mret],
            &[],
            None,
            100,
            0,
            false,
            "failed to execute MRET instruction",
        )];

        for (id, test) in tests.iter().enumerate() {
            if let Some(vm) = test.run_test(id) {
                assert!(
                    !vm.cpu.is_machine_mode,
                    "Test {id} Failed - machine is still in machine mode after executing mret instruction!"
                );
            } else {
                // We can't do anything here. The test asserted and so we didn't
                // yield a valid virtual machine instance to interrogate.
            }
        }
    }

    /// Test the add u32 immediate to u32 register instruction.
    #[test]
    fn test_add_u32_imm_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    AddU32ImmU32Reg(0x2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::AC, 0x3),
                    (RegisterId::FL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                100,
                0,
                false,
                "ADD - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    AddU32ImmU32Reg(0x2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, u32::MAX),
                    (RegisterId::AC, 0x1),
                    (RegisterId::FL, CpuFlag::compute_from(&[CpuFlag::OF])),
                ],
                None,
                100,
                0,
                false,
                "ADD - CPU flags not correctly set",
            ),
            TestEntryU32Standard::new(
                &[AddU32ImmU32Reg(0, RegisterId::R1)],
                &[(
                    RegisterId::FL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                None,
                100,
                0,
                false,
                "ADD - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    AddU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x2),
                    (RegisterId::AC, 0x3),
                    (RegisterId::FL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                100,
                0,
                false,
                "ADD - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    AddU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::AC, 0x2),
                    (RegisterId::FL, 0x0),
                ],
                None,
                100,
                0,
                false,
                "ADD - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    AddU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::AC, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                None,
                100,
                0,
                false,
                "ADD - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    AddU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::AC, 0x2),
                    (RegisterId::FL, 0x0),
                ],
                None,
                100,
                0,
                false,
                "ADD - CPU signed flag not correctly cleared",
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
                    MovU32ImmU32Reg(0xf, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    AddU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0xf),
                    (RegisterId::R2, 0x1),
                    (RegisterId::AC, 0x10),
                ],
                None,
                100,
                0,
                false,
                "ADD - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    MovU32ImmU32Reg(0x2, RegisterId::R2),
                    AddU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, u32::MAX),
                    (RegisterId::R2, 0x2),
                    (RegisterId::AC, 0x1),
                    (RegisterId::FL, CpuFlag::compute_from(&[CpuFlag::OF])),
                ],
                None,
                100,
                0,
                false,
                "ADD - CPU flags not correctly set",
            ),
            TestEntryU32Standard::new(
                &[AddU32RegU32Reg(RegisterId::R1, RegisterId::R2)],
                &[(
                    RegisterId::FL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                None,
                100,
                0,
                false,
                "ADD - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    AddU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0x2),
                    (RegisterId::R2, 0x1),
                    (RegisterId::AC, 0x3),
                    (RegisterId::FL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                100,
                0,
                false,
                "ADD - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    AddU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::AC, 0x2),
                    (RegisterId::FL, 0x0),
                ],
                None,
                100,
                0,
                false,
                "ADD - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    AddU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0x1),
                    (RegisterId::AC, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                None,
                100,
                0,
                false,
                "ADD - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    AddU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::R2, 0x1),
                    (RegisterId::AC, 0x2),
                    (RegisterId::FL, 0x0),
                ],
                None,
                100,
                0,
                false,
                "ADD - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test subtraction of u32 immediate from a u32 register instruction.
    #[test]
    fn test_sub_u32_imm_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    SubU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::AC, 0x1)],
                None,
                100,
                0,
                false,
                "SUB - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    SubU32ImmU32Reg(u32::MAX, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x2),
                    (RegisterId::AC, 0x3),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::PF]),
                    ),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU flags not correctly set",
            ),
            TestEntryU32Standard::new(
                &[SubU32ImmU32Reg(0, RegisterId::R1)],
                &[(
                    RegisterId::FL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                None,
                100,
                0,
                false,
                "SUB - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0x4, RegisterId::R1),
                    SubU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x4),
                    (RegisterId::AC, 0x3),
                    (RegisterId::FL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    SubU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x2),
                    (RegisterId::AC, 0x1),
                    (RegisterId::FL, 0x0),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    SubU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::AC, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::FL, CpuFlag::compute_from(&[CpuFlag::SF])),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    SubU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x2),
                    (RegisterId::AC, 0x1),
                    (RegisterId::FL, 0x0),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test subtraction of a u32 register from a u32 immediate instruction.
    #[test]
    fn test_sub_u32_reg_u32_imm() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    SubU32RegU32Imm(RegisterId::R1, 0x3),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::AC, 0x1)],
                None,
                100,
                0,
                false,
                "SUB - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    SubU32RegU32Imm(RegisterId::R1, 0x2),
                ],
                &[
                    (RegisterId::R1, u32::MAX),
                    (RegisterId::AC, 0x3),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::PF]),
                    ),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU flags not correctly set",
            ),
            TestEntryU32Standard::new(
                &[SubU32RegU32Imm(RegisterId::R1, 0)],
                &[(
                    RegisterId::FL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                None,
                100,
                0,
                false,
                "SUB - CPU flags not correctly set",
            ),
            // Test the parity flag gets enabled.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    SubU32RegU32Imm(RegisterId::R1, 0x4),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::AC, 0x3),
                    (RegisterId::FL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    SubU32RegU32Imm(RegisterId::R1, 0x2),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::AC, 0x1),
                    (RegisterId::FL, 0x0),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    SubU32RegU32Imm(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::AC, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::FL, CpuFlag::compute_from(&[CpuFlag::SF])),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    SubU32RegU32Imm(RegisterId::R1, 0x2),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::AC, 0x1),
                    (RegisterId::FL, 0x0),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the subtract u32 register from u32 register instruction.
    #[test]
    fn test_sub_u32_reg_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32ImmU32Reg(0xf, RegisterId::R2),
                    SubU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::R2, 0xf),
                    (RegisterId::AC, 0xe),
                ],
                None,
                100,
                0,
                false,
                "SUB - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    MovU32ImmU32Reg(0x2, RegisterId::R2),
                    SubU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, u32::MAX),
                    (RegisterId::R2, 0x2),
                    (RegisterId::AC, 0x3),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::PF]),
                    ),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU flags not correctly set",
            ),
            TestEntryU32Standard::new(
                &[SubU32RegU32Reg(RegisterId::R1, RegisterId::R2)],
                &[(
                    RegisterId::FL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                None,
                100,
                0,
                false,
                "SUB - CPU flags not correctly set",
            ),
            // Test the parity flag gets enabled.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32ImmU32Reg(0x4, RegisterId::R2),
                    SubU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::R2, 0x4),
                    (RegisterId::AC, 0x3),
                    (RegisterId::FL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32ImmU32Reg(0x2, RegisterId::R2),
                    SubU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::R2, 0x2),
                    (RegisterId::AC, 0x1),
                    (RegisterId::FL, 0x0),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R2),
                    SubU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::R2, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::AC, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::FL, CpuFlag::compute_from(&[CpuFlag::SF])),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32ImmU32Reg(0x2, RegisterId::R2),
                    SubU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::R2, 0x2),
                    (RegisterId::AC, 0x1),
                    (RegisterId::FL, 0x0),
                ],
                None,
                100,
                0,
                false,
                "SUB - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the multiply u32 immediate by a u32 register instruction.
    #[test]
    fn test_mul_u32_imm_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MulU32ImmU32Reg(0x2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::AC, 0x2)],
                None,
                100,
                0,
                false,
                "MUL - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    MulU32ImmU32Reg(0x2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, u32::MAX),
                    (RegisterId::AC, 4294967294),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::CF]),
                    ),
                ],
                None,
                100,
                0,
                false,
                "MUL - CPU flags not correctly set",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the multiply u32 immediate by a u32 register instruction.
    #[test]
    fn test_mul_u32_reg_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32ImmU32Reg(0x2, RegisterId::R2),
                    MulU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::R2, 0x2),
                    (RegisterId::AC, 0x2),
                ],
                None,
                100,
                0,
                false,
                "MUL - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    MovU32ImmU32Reg(0x2, RegisterId::R2),
                    MulU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, u32::MAX),
                    (RegisterId::R2, 0x2),
                    (RegisterId::AC, 4294967294),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::CF]),
                    ),
                ],
                None,
                100,
                0,
                false,
                "MUL - CPU flags not correctly set",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the division of a u32 register by a u32 immediate instruction.
    #[test]
    fn test_div_u32_imm_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    DivU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::AC, 0x2)],
                None,
                100,
                0,
                false,
                "DIV - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    DivU32ImmU32Reg(0x2, RegisterId::R1),
                ],
                &[(RegisterId::R1, u32::MAX), (RegisterId::AC, 2147483647)],
                None,
                100,
                0,
                false,
                "DIV - CPU flags not correctly set",
            ),
            TestEntryU32Standard::new(
                &[DivU32ImmU32Reg(0x0, RegisterId::R1)],
                &[],
                None,
                100,
                0,
                true,
                "DIV - failed to panic when attempting to divide by zero",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the division of a u32 immediate by a u32 register instruction.
    #[test]
    fn test_div_u32_reg_u32_imm() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    DivU32RegU32Imm(RegisterId::R1, 0x2),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::AC, 0x2)],
                None,
                100,
                0,
                false,
                "DIV - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    DivU32RegU32Imm(RegisterId::R1, u32::MAX),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::AC, 2147483647)],
                None,
                100,
                0,
                false,
                "DIV - CPU flags not correctly set",
            ),
            TestEntryU32Standard::new(
                &[DivU32RegU32Imm(RegisterId::R1, 0x0)],
                &[],
                None,
                100,
                0,
                true,
                "DIV - failed to panic when attempting to divide by zero",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the division of a u32 register by a u32 register instruction.
    #[test]
    fn test_div_u32_reg_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    DivU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x2),
                    (RegisterId::R2, 0x1),
                    (RegisterId::AC, 0x2),
                ],
                None,
                100,
                0,
                false,
                "DIV - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    MovU32ImmU32Reg(0x2, RegisterId::R2),
                    DivU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, u32::MAX),
                    (RegisterId::R2, 0x2),
                    (RegisterId::AC, 2147483647),
                ],
                None,
                100,
                0,
                false,
                "DIV - CPU flags not correctly set",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    DivU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[],
                None,
                100,
                0,
                true,
                "DIV - failed to panic when attempting to divide by zero",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the push u32 immediate instruction.
    #[test]
    fn test_push_u32_imm_multiple() {
        let test = TestEntryU32Standard::new(
            &[PushU32Imm(0x123), PushU32Imm(0x321)],
            &[],
            None,
            100,
            2,
            false,
            "failed to execute PUSH instruction",
        );

        let vm = test.run_test(0);
        if vm.is_none() {
            panic!("{}", test.fail_message(0, true));
        }

        let mut vm = vm.unwrap();
        assert_eq!(vm.ram.pop_u32(), 0x321);
        assert_eq!(vm.ram.pop_u32(), 0x123);
    }

    /// Test the push u32 immediate instruction.
    #[test]
    fn test_push_u32_imm_single() {
        let test = TestEntryU32Standard::new(
            &[PushU32Imm(0x123)],
            &[],
            None,
            100,
            1,
            false,
            "failed to execute PUSH instruction",
        );

        let vm = test.run_test(0);
        if vm.is_none() {
            panic!("{}", test.fail_message(0, true));
        }

        let mut vm = vm.unwrap();
        assert_eq!(vm.ram.pop_u32(), 0x123);
    }

    /// Test the push u32 immediate instruction.
    #[test]
    #[should_panic]
    fn test_push_u32_imm_invalid_pop() {
        let test = TestEntryU32Standard::new(
            &[Nop],
            &[],
            None,
            100,
            2,
            false,
            "failed to execute PUSH instruction",
        );

        let vm = test.run_test(0);
        if vm.is_none() {
            panic!("{}", test.fail_message(0, true));
        }

        // There is nothing on the stack, this should assert.
        let mut vm = vm.unwrap();
        _ = vm.ram.pop_u32();
    }
}

/*#[cfg(test)]
mod tests_cpu {
    /// Test the modulo of a u32 register by a u32 immediate instruction.
    #[test]
    fn test_mod_u32_imm_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    ModU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::AC, 0x0)],
                vec![0; 100],
                false,
                "MOD - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x3, RegisterId::R1),
                    ModU32ImmU32Reg(0x2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x3), (RegisterId::AC, 0x1)],
                vec![0; 100],
                false,
                "MOD - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[DivU32ImmU32Reg(0x0, RegisterId::R1)],
                &[],
                vec![0; 100],
                true,
                "MOD - failed to panic when attempting to divide by zero",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the modulo of a u32 immediate by a u32 register instruction.
    #[test]
    fn test_mod_u32_reg_u32_imm() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    ModU32RegU32Imm(RegisterId::R1, 0x2),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::AC, 0x0)],
                vec![0; 100],
                false,
                "MOD - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    ModU32RegU32Imm(RegisterId::R1, 0x3),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::AC, 0x1)],
                vec![0; 100],
                false,
                "MOD - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[ModU32RegU32Imm(RegisterId::R1, 0x0)],
                &[],
                vec![0; 100],
                true,
                "MOD - failed to panic when attempting to divide by zero",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the modulo of a u32 register by a u32 register instruction.
    #[test]
    fn test_mod_u32_reg_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32ImmU32Reg(0x2, RegisterId::R2),
                    ModU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::R2, 0x2),
                    (RegisterId::AC, 0x0),
                ],
                vec![0; 100],
                false,
                "MOD - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    MovU32ImmU32Reg(0x3, RegisterId::R2),
                    ModU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0x2),
                    (RegisterId::R2, 0x3),
                    (RegisterId::AC, 0x1),
                ],
                vec![0; 100],
                false,
                "MOD - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[ModU32RegU32Reg(RegisterId::R1, RegisterId::R2)],
                &[],
                vec![0; 100],
                true,
                "MOD - failed to panic when attempting to divide by zero",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the increment u32 register instruction.
    #[test]
    fn test_inc_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[IncU32Reg(RegisterId::R1)],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "INC - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    IncU32Reg(RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::OF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "INC - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    IncU32Reg(RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x3),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::PF])),
                ],
                vec![0; 100],
                false,
                "INC - CPU parity flag not correctly set",
            ),
            // Test the parity flag get cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::PF]), RegisterId::FL),
                    IncU32Reg(RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::FL, 0x0)],
                vec![0; 100],
                false,
                "INC - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set
            TestEntryU32Standard::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    IncU32Reg(RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "INC - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::SF]), RegisterId::FL),
                    IncU32Reg(RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::FL, 0x0)],
                vec![0; 100],
                false,
                "INC - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the decrement u32 register instruction.
    #[test]
    fn test_dec_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    DecU32Reg(RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x0),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "DEC - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[DecU32Reg(RegisterId::R1)],
                &[
                    (RegisterId::R1, u32::MAX),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::OF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "DEC - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0x4, RegisterId::R1),
                    DecU32Reg(RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x3),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::PF])),
                ],
                vec![0; 100],
                false,
                "DEC - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::PF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x3, RegisterId::R1),
                    DecU32Reg(RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::FL, 0x0)],
                vec![0; 100],
                false,
                "DEC - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    DecU32Reg(RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::SF])),
                ],
                vec![0; 100],
                false,
                "DEC - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    DecU32Reg(RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::FL, 0x0)],
                vec![0; 100],
                false,
                "DEC - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the bitwise and of a u32 immediate and a u32 register instruction.
    #[test]
    fn test_bitwise_and_u32_imm_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x3, RegisterId::R1),
                    AndU32ImmU32Reg(0x2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x3), (RegisterId::AC, 0x2)],
                vec![0; 100],
                false,
                "AND - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    AndU32ImmU32Reg(0x3, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::AC, 0x2)],
                vec![0; 100],
                false,
                "AND - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    AndU32ImmU32Reg(0x0, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x2),
                    (RegisterId::AC, 0x0),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "AND - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::R1),
                    AndU32ImmU32Reg(0x3, RegisterId::R1),
                ],
                &[(
                    RegisterId::FL,
                    CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                vec![0; 100],
                false,
                "AND - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0x3, RegisterId::R1),
                    AndU32ImmU32Reg(0x3, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x3),
                    (RegisterId::AC, 0x3),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::PF])),
                ],
                vec![0; 100],
                false,
                "AND - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::PF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x3, RegisterId::R1),
                    AndU32ImmU32Reg(0x2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x3),
                    (RegisterId::AC, 0x2),
                    (RegisterId::FL, 0x0),
                ],
                vec![0; 100],
                false,
                "AND - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    AndU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::AC, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "AND - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    AndU32ImmU32Reg(0x2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x2),
                    (RegisterId::AC, 0x2),
                    (RegisterId::FL, 0x0),
                ],
                vec![0; 100],
                false,
                "AND - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the left-shift u32 register by u32 immediate value.
    #[test]
    fn test_left_shift_u32_imm_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    LeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2)],
                vec![0; 100],
                false,
                "SHL - incorrect result value produced",
            ),
            // When shifted left by three places, this value will set the carry flag but not the
            // overflow flag. The overflow flag is only set when computing 1-bit shifts.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1011_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    LeftShiftU32ImmU32Reg(0x3, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1000),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::CF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHL - CPU flags not correctly set",
            ),
            // When shifted left by one place. This will set the carry and overflow flags.
            // The overflow flag is only set when computing 1-bit shifts.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    LeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::CF, CpuFlag::OF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHL - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::R1),
                    LeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(
                    RegisterId::FL,
                    CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                vec![0; 100],
                false,
                "SHL - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::OF]), RegisterId::FL),
                    // This will unset the overflow flag and set the zero flag instead.
                    MovU32ImmU32Reg(0x0, RegisterId::R1),
                    LeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(
                    RegisterId::FL,
                    CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                vec![0; 100],
                false,
                "SHL - parity or zero flags are not set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestEntryU32Standard::new(
                &[LeftShiftU32ImmU32Reg(0x20, RegisterId::R1)],
                &[],
                vec![],
                true,
                "SHL - successfully executed instruction with invalid shift value",
            ),
            // Test the parity flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::PF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    LeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::FL, 0x0)],
                vec![0; 100],
                false,
                "SHL - CPU parity flag not correctly cleared",
            ),
            // Test that a left-shift by zero leaves the value unchanged.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    LeftShiftU32ImmU32Reg(0x0, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "SHL - zero left-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0b0100_0000_0000_0000_0000_0000_0000_0000, RegisterId::R1),
                    LeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHL - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    LeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::FL, 0x0)],
                vec![0; 100],
                false,
                "SHL - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the left-shift u32 register by u32 register value.
    #[test]
    fn test_left_shift_u32_reg_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::R2, 0x1)],
                vec![0; 100],
                false,
                "SHL - incorrect result value produced",
            ),
            // When shifted left by three places, this value will set the carry flag but not the
            // overflow flag. The overflow flag is only set when computing 1-bit shifts.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1011_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    MovU32ImmU32Reg(0x3, RegisterId::R2),
                    LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1000),
                    (RegisterId::R2, 0x3),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::CF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHL - invalid CPU flag state",
            ),
            // When shifted left by one place. This will set the carry and overflow flags.
            // The overflow flag is only set when computing 1-bit shifts.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::R2, 0x1),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::OF, CpuFlag::CF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHL - CPU flags not correctly set",
            ),
            // When shifted left by two places, this value will set the overflow and carry flags.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::R2, 0x1),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::OF, CpuFlag::CF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHL - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R2, 0x1),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHL - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::OF]), RegisterId::FL),
                    // This will unset the overflow flag and set the zero flag instead.
                    MovU32ImmU32Reg(0x0, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R2, 0x1),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHL - CPU flags not correctly set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x20, RegisterId::R2),
                    LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[],
                vec![],
                true,
                "SHL - successfully executed instruction with invalid shift value",
            ),
            // Test that a left-shift by zero leaves the value unchanged.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "SHL - zero left-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0b0100_0000_0000_0000_0000_0000_0000_0000, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (RegisterId::R2, 0x1),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHL - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x2),
                    (RegisterId::R2, 0x1),
                    (RegisterId::FL, 0x0),
                ],
                vec![0; 100],
                false,
                "SHL - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the arithmetic left-shift u32 register by u32 immediate value.
    #[test]
    fn test_arith_left_shift_u32_imm_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    ArithLeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2)],
                vec![0; 100],
                false,
                "SAL - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1110, RegisterId::R1),
                    ArithLeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1101),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::SF])),
                ],
                vec![0; 100],
                false,
                "SAL - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::R1),
                    ArithLeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(
                    RegisterId::FL,
                    CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                vec![0; 100],
                false,
                "SAL - CPU flags not correctly set",
            ),
            // Test that a left-shift by zero leaves the value unchanged.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    ArithLeftShiftU32ImmU32Reg(0x0, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "SAL - zero left-shift did not leave the result unchanged",
            ),
            // Test the signed flag is set.
            TestEntryU32Standard::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0b0100_0000_0000_0000_0000_0000_0000_0000, RegisterId::R1),
                    ArithLeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SAL - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    ArithLeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::FL, 0x0)],
                vec![0; 100],
                false,
                "SAL - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the arithmetic left-shift u32 register by u32 register.
    #[test]
    fn test_arith_left_shift_u32_reg_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    ArithLeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::R2, 0x1)],
                vec![0; 100],
                false,
                "SAL - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1110, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    ArithLeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1101),
                    (RegisterId::R2, 0x1),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::SF])),
                ],
                vec![0; 100],
                false,
                "SAL - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    ArithLeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R2, 0x1),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SAL - CPU flags not correctly set",
            ),
            // Test that a left-shift by zero leaves the value unchanged.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    ArithLeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "SAL - zero left-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0b0100_0000_0000_0000_0000_0000_0000_0000, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    ArithLeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (RegisterId::R2, 0x1),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SAL - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    ArithLeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x2),
                    (RegisterId::R2, 0x1),
                    (RegisterId::FL, 0x0),
                ],
                vec![0; 100],
                false,
                "SAL - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the right-shift u32 register by u32 immediate value.
    #[test]
    fn test_right_shift_u32_imm_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    RightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "SHR - incorrect result value produced",
            ),
            // The SHR command should clear the overflow flag but set the
            // carry flag since the bit being shifted out is one.
            TestEntryU32Standard::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::OF]), RegisterId::FL),
                    // Execute the test instruction.
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    RightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::CF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHR - carry or parity CPU flags were not correctly set",
            ),
            // The SHR command should clear the overflow flag and the
            // carry flag in this case as the bit being shifted out is zero.
            TestEntryU32Standard::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::OF]), RegisterId::FL),
                    // Execute the test instruction.
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1110, RegisterId::R1),
                    RightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::PF])),
                ],
                vec![0; 100],
                false,
                "SHR - CPU flags were not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::R1),
                    RightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(
                    RegisterId::FL,
                    CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                vec![0; 100],
                false,
                "SHR - zero or parity CPU flags were not correctly set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestEntryU32Standard::new(
                &[RightShiftU32ImmU32Reg(0x20, RegisterId::R1)],
                &[],
                vec![],
                true,
                "SHR - successfully executed instruction with invalid shift value",
            ),
            // Test that a right-shift by zero leaves the value unchanged.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    RightShiftU32ImmU32Reg(0x0, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "SHR - zero right-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    RightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x0),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::CF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SAL - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the right-shift u32 register by u32 register.
    #[test]
    fn test_right_shift_u32_reg_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    RightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::R2, 0x1)],
                vec![0; 100],
                false,
                "SHR - incorrect result value produced",
            ),
            // The SHR command should clear the overflow flag but set the
            // carry flag since the bit being shifted out is one.
            TestEntryU32Standard::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::OF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    // Execute the test instruction.
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    RightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0x1),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::CF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHR - carry or parity CPU flags were not correctly set",
            ),
            // The SHR command should clear the overflow flag and the
            // carry flag in this case as the bit being shifted out is zero.
            TestEntryU32Standard::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::OF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    // Execute the test instruction.
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1110, RegisterId::R1),
                    RightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0x1),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::PF])),
                ],
                vec![0; 100],
                false,
                "SHR - CPU flags were not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    RightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R2, 0x1),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHR - zero or parity CPU flags were not correctly set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x20, RegisterId::R2),
                    RightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[],
                vec![],
                true,
                "SHR - successfully executed instruction with invalid shift value",
            ),
            // Test that a right-shift by zero leaves the value unchanged.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    RightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "SHR - zero right-shift did not leave the result unchanged",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the arithmetic right-shift u32 register by u32 immediate value.
    #[test]
    fn test_arith_right_shift_u32_imm_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    ArithRightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "SAR - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    ArithRightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1011_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SAR - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero right-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::R1),
                    ArithRightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(
                    RegisterId::FL,
                    CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                vec![0; 100],
                false,
                "SAR - zero or parity CPU flags were not correctly set",
            ),
            // Test that a right-shift by zero leaves the value unchanged.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    ArithRightShiftU32ImmU32Reg(0x0, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "SAR - zero right-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0b0000_0000_0000_0000_0000_0000_0000_0001, RegisterId::R1),
                    ArithRightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SAR - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    ArithRightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::FL, 0x0)],
                vec![0; 100],
                false,
                "SAR - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the arithmetic right-shift u32 register by u32 register.
    #[test]
    fn test_arith_right_shift_u32_reg_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    ArithRightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::R2, 0x1)],
                vec![0; 100],
                false,
                "SAR - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    ArithRightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0x1),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SAR - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    ArithRightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R2, 0x1),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SAR - zero or parity CPU flags were not correctly set",
            ),
            // Test that a right-shift by zero leaves the value unchanged.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    ArithRightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "SAR - zero right-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0b0000_0000_0000_0000_0000_0000_0000_0001, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    ArithRightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (RegisterId::R2, 0x1),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SAR - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    MovU32ImmU32Reg(0x1, RegisterId::R2),
                    ArithRightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0x1),
                    (RegisterId::R2, 0x1),
                    (RegisterId::FL, 0x0),
                ],
                vec![0; 100],
                false,
                "SAR - CPU signed flag not correctly cleared",
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
                &[MovU32ImmU32Reg(0x1, RegisterId::R1)],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "MOV - invalid value moved to register",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2)],
                vec![0; 100],
                false,
                "MOV - invalid value moved to register",
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
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::R2, 0x1)],
                vec![0; 100],
                false,
                "MOV - immediate value not moved to register",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::R1),
                    MovU32ImmU32Reg(0x2, RegisterId::R2),
                    MovU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::R2, 0x1)],
                vec![0; 100],
                false,
                "MOV - immediate value not moved to register",
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
            &[MovU32ImmMemRelSimple(0x123, 0x0)],
            &[],
            vec![
                35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ],
            false,
            "MOV - immediate value not moved to memory",
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
                MovU32ImmU32Reg(0x123, RegisterId::R1),
                MovU32RegMemRelSimple(RegisterId::R1, 0x0),
            ],
            &[(RegisterId::R1, 0x123)],
            vec![
                35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ],
            false,
            "MOV - u32 register value not moved to memory",
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
                // Move the value to memory.
                MovU32ImmU32Reg(0x123, RegisterId::R1),
                MovU32RegMemRelSimple(RegisterId::R1, 0x0),
                // Move the value from memory to a register.
                MovMemU32RegRelSimple(0x0, RegisterId::R2),
            ],
            &[(RegisterId::R1, 0x123), (RegisterId::R2, 0x123)],
            vec![
                35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ],
            false,
            "MOV - value not correctly moved from memory to register",
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
                MovU32ImmU32Reg(0x123, RegisterId::R1),
                MovU32RegMemRelSimple(RegisterId::R1, 0x0),
                // Set the address pointer in R2.
                MovU32ImmU32Reg(0x0, RegisterId::R2),
                // Read the value from the address of R2 into R1.
                MovU32RegPtrU32RegRelSimple(RegisterId::R2, RegisterId::R3),
            ],
            &[(RegisterId::R1, 0x123), (RegisterId::R3, 0x123)],
            vec![
                35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ],
            false,
            "MOV - value not correctly moved from memory to register via register pointer",
        )];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the swap u32 register to u32 register instruction.
    #[test]
    fn test_swap_u32_reg_u32_reg() {
        let tests = [TestEntryU32Standard::new(
            &[
                MovU32ImmU32Reg(0x1, RegisterId::R1),
                MovU32ImmU32Reg(0x2, RegisterId::R2),
                SwapU32RegU32Reg(RegisterId::R1, RegisterId::R2),
            ],
            &[(RegisterId::R1, 0x2), (RegisterId::R2, 0x1)],
            vec![0; 100],
            false,
            "SWAP - values of the two registers were not correctly swapped",
        )];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the complex move value to expression-derived memory address.
    #[test]
    fn test_move_u32_imm_expr() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemExprRel(
                    0x123,
                    MoveExpressionHandler::from(
                        &[
                            ExpressionArgs::Immediate(0x8),
                            ExpressionArgs::Operator(ExpressionOperator::Add),
                            ExpressionArgs::Immediate(0x8),
                        ][..],
                    )
                    .pack(),
                )],
                &[],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x8, RegisterId::R1),
                    MovU32ImmU32Reg(0x8, RegisterId::R2),
                    MovU32ImmMemExprRel(
                        0x123,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::R1),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Register(RegisterId::R2),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[(RegisterId::R1, 0x8), (RegisterId::R2, 0x8)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register with expression - two registers",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x8, RegisterId::R1),
                    MovU32ImmMemExprRel(
                        0x123,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::R1),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Immediate(0x8),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[(RegisterId::R1, 0x8)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register with expression - one constant and one register",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    MovU32ImmU32Reg(0x8, RegisterId::R2),
                    MovU32ImmMemExprRel(
                        0x123,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::R1),
                                ExpressionArgs::Operator(ExpressionOperator::Multiply),
                                ExpressionArgs::Register(RegisterId::R2),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::R2, 0x8)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register - using multiplication",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1a, RegisterId::R1),
                    MovU32ImmU32Reg(0x4, RegisterId::R2),
                    MovU32ImmMemExprRel(
                        0x123,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::R1),
                                ExpressionArgs::Operator(ExpressionOperator::Subtract),
                                ExpressionArgs::Register(RegisterId::R2),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[(RegisterId::R1, 0x1A), (RegisterId::R2, 0x4)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register - using subtraction",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemExprRel(
                    0x123,
                    MoveExpressionHandler::from(
                        &[
                            ExpressionArgs::Immediate(0x8),
                            ExpressionArgs::Operator(ExpressionOperator::Add),
                            ExpressionArgs::Immediate(0x4),
                            ExpressionArgs::Operator(ExpressionOperator::Multiply),
                            ExpressionArgs::Immediate(0x2),
                        ][..],
                    )
                    .pack(),
                )],
                &[],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register with expression - three constants",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the complex move from an expression-derived memory address to a register.
    #[test]
    fn test_move_mem_expr_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0x123, 0x10),
                    MovMemExprU32RegRel(
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Immediate(0x8),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Immediate(0x8),
                            ][..],
                        )
                        .pack(),
                        RegisterId::R8,
                    ),
                ],
                &[(RegisterId::R8, 0x123)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0x123, 0x10),
                    MovU32ImmU32Reg(0x8, RegisterId::R1),
                    MovU32ImmU32Reg(0x8, RegisterId::R2),
                    MovMemExprU32RegRel(
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::R1),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Register(RegisterId::R2),
                            ][..],
                        )
                        .pack(),
                        RegisterId::R8,
                    ),
                ],
                &[
                    (RegisterId::R1, 0x8),
                    (RegisterId::R2, 0x8),
                    (RegisterId::R8, 0x123),
                ],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register with expression - two registers",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0x123, 0x10),
                    MovU32ImmU32Reg(0x8, RegisterId::R1),
                    MovMemExprU32RegRel(
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::R1),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Immediate(0x8),
                            ][..],
                        )
                        .pack(),
                        RegisterId::R8,
                    ),
                ],
                &[(RegisterId::R1, 0x8), (RegisterId::R8, 0x123)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register with expression - one constant and one register",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0x123, 0x10),
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    MovU32ImmU32Reg(0x8, RegisterId::R2),
                    MovMemExprU32RegRel(
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::R1),
                                ExpressionArgs::Operator(ExpressionOperator::Multiply),
                                ExpressionArgs::Register(RegisterId::R2),
                            ][..],
                        )
                        .pack(),
                        RegisterId::R8,
                    ),
                ],
                &[
                    (RegisterId::R1, 0x2),
                    (RegisterId::R2, 0x8),
                    (RegisterId::R8, 0x123),
                ],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register - using multiplication",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0x123, 0x16),
                    MovU32ImmU32Reg(0x1A, RegisterId::R1),
                    MovU32ImmU32Reg(0x4, RegisterId::R2),
                    MovMemExprU32RegRel(
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::R1),
                                ExpressionArgs::Operator(ExpressionOperator::Subtract),
                                ExpressionArgs::Register(RegisterId::R2),
                            ][..],
                        )
                        .pack(),
                        RegisterId::R8,
                    ),
                ],
                &[
                    (RegisterId::R1, 0x1a),
                    (RegisterId::R2, 0x4),
                    (RegisterId::R8, 0x123),
                ],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register - using subtraction",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0x123, 0x10),
                    MovMemExprU32RegRel(
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Immediate(0x8),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Immediate(0x4),
                                ExpressionArgs::Operator(ExpressionOperator::Multiply),
                                ExpressionArgs::Immediate(0x2),
                            ][..],
                        )
                        .pack(),
                        RegisterId::R8,
                    ),
                ],
                &[(RegisterId::R8, 0x123)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the complex move from a register to an expression-derived memory address.
    #[test]
    fn test_move_u32_reg_mem_expr() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x123, RegisterId::R8),
                    MovU32RegMemExprRel(
                        RegisterId::R8,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Immediate(0x8),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Immediate(0x8),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[(RegisterId::R8, 0x123)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
            // Test with two register values.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x8, RegisterId::R1),
                    MovU32ImmU32Reg(0x8, RegisterId::R2),
                    MovU32ImmU32Reg(0x123, RegisterId::R8),
                    MovU32RegMemExprRel(
                        RegisterId::R8,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::R1),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Register(RegisterId::R2),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[
                    (RegisterId::R1, 0x8),
                    (RegisterId::R2, 0x8),
                    (RegisterId::R8, 0x123),
                ],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register with expression - two registers",
            ),
            // Test with a constant and a register.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x8, RegisterId::R1),
                    MovU32ImmU32Reg(0x123, RegisterId::R8),
                    MovU32RegMemExprRel(
                        RegisterId::R8,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::R1),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Immediate(0x8),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[(RegisterId::R1, 0x8), (RegisterId::R8, 0x123)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register with expression - one constant and one register",
            ),
            // Test with multiplication.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::R1),
                    MovU32ImmU32Reg(0x8, RegisterId::R2),
                    MovU32ImmU32Reg(0x123, RegisterId::R8),
                    MovU32RegMemExprRel(
                        RegisterId::R8,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::R1),
                                ExpressionArgs::Operator(ExpressionOperator::Multiply),
                                ExpressionArgs::Register(RegisterId::R2),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[
                    (RegisterId::R1, 0x2),
                    (RegisterId::R2, 0x8),
                    (RegisterId::R8, 0x123),
                ],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register - using multiplication",
            ),
            // Test with subtraction.
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x1A, RegisterId::R1),
                    MovU32ImmU32Reg(0x4, RegisterId::R2),
                    MovU32ImmU32Reg(0x123, RegisterId::R8),
                    MovU32RegMemExprRel(
                        RegisterId::R8,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::R1),
                                ExpressionArgs::Operator(ExpressionOperator::Subtract),
                                ExpressionArgs::Register(RegisterId::R2),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[
                    (RegisterId::R1, 0x1a),
                    (RegisterId::R2, 0x4),
                    (RegisterId::R8, 0x123),
                ],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register - using subtraction",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x123, RegisterId::R8),
                    MovU32RegMemExprRel(
                        RegisterId::R8,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Immediate(0x8),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Immediate(0x4),
                                ExpressionArgs::Operator(ExpressionOperator::Multiply),
                                ExpressionArgs::Immediate(0x2),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[(RegisterId::R8, 0x123)],
                vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the byte swap u32 register instruction.
    #[test]
    fn test_byte_swap_u32_reg() {
        let tests = [TestEntryU32Standard::new(
            &[
                MovU32ImmU32Reg(0xAABBCCDD, RegisterId::R1),
                ByteSwapU32(RegisterId::R1),
            ],
            &[(RegisterId::R1, 0xDDCCBBAA)],
            vec![0; 100],
            false,
            "BSWAP - the byte order of the register was not correctly swapped",
        )];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the zero high bit by index instruction.
    #[test]
    fn test_zhbi_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    MovU32ImmU32Reg(0x4, RegisterId::R2),
                    ZeroHighBitsByIndexU32Reg(RegisterId::R2, RegisterId::R1, RegisterId::R3),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0x4),
                    (RegisterId::R3, 0b0000_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    MovU32ImmU32Reg(0x18, RegisterId::R2),
                    ZeroHighBitsByIndexU32Reg(RegisterId::R2, RegisterId::R1, RegisterId::R3),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0x18),
                    (RegisterId::R3, 0b0000_0000_0000_0000_0000_0000_1111_1111),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    MovU32ImmU32Reg(0x19, RegisterId::R2),
                    ZeroHighBitsByIndexU32Reg(RegisterId::R2, RegisterId::R1, RegisterId::R3),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0x19),
                    (RegisterId::R3, 0b0000_0000_0000_0000_0000_0000_0111_1111),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    MovU32ImmU32Reg(0x1b, RegisterId::R2),
                    ZeroHighBitsByIndexU32Reg(RegisterId::R2, RegisterId::R1, RegisterId::R3),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0x1b),
                    (RegisterId::R3, 0b0000_0000_0000_0000_0000_0000_0001_1111),
                ],
                vec![0; 100],
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::R1),
                    MovU32ImmU32Reg(0x0, RegisterId::R2),
                    ZeroHighBitsByIndexU32Reg(RegisterId::R2, RegisterId::R1, RegisterId::R3),
                ],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF]))],
                vec![0; 100],
                false,
                "ZHBI - incorrect flag state - zero flag not set",
            ),
            TestEntryU32Standard::new(
                &[
                    // Manually set the overflow flag state.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::OF]), RegisterId::FL),
                    ZeroHighBitsByIndexU32Reg(
                        RegisterId::R1, // Already has the value of 0.
                        RegisterId::R2, // Already has the value of 0.
                        RegisterId::R3,
                    ),
                ],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF]))],
                vec![0; 100],
                false,
                "ZHBI - incorrect flag state - overflow flag not cleared",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x20, RegisterId::R2),
                    ZeroHighBitsByIndexU32Reg(
                        RegisterId::R2,
                        RegisterId::R1, // Already has the value of 0.
                        RegisterId::R3,
                    ),
                ],
                &[],
                vec![0; 100],
                true,
                "ZHBI - successfully executed instruction with invalid bit index",
            ),
            // Test the signed flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R3),
                    ZeroHighBitsByIndexU32Reg(
                        RegisterId::R2, // Already has the value of 0.
                        RegisterId::R1,
                        RegisterId::R3,
                    ),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R3, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::CF]),
                    ),
                ],
                vec![0; 100],
                false,
                "ZHBI - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R3),
                    ZeroHighBitsByIndexU32Reg(
                        RegisterId::R2, // Already has the value of 0.
                        RegisterId::R1,
                        RegisterId::R3,
                    ),
                ],
                &[
                    (RegisterId::R1, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R3, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "ZHBI - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the zero high bit by index instruction.
    #[test]
    fn test_zhbi_u32_reg_u32_imm() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    ZeroHighBitsByIndexU32RegU32Imm(0x4, RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0b0000_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    ZeroHighBitsByIndexU32RegU32Imm(0x18, RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0b0000_0000_0000_0000_0000_0000_1111_1111),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    ZeroHighBitsByIndexU32RegU32Imm(0x19, RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0b0000_0000_0000_0000_0000_0000_0111_1111),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    ZeroHighBitsByIndexU32RegU32Imm(0x1b, RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0b0000_0000_0000_0000_0000_0000_0001_1111),
                ],
                vec![0; 100],
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[ZeroHighBitsByIndexU32RegU32Imm(
                    0x0,
                    RegisterId::R1, // Already has the value of 0.
                    RegisterId::R2,
                )],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF]))],
                vec![0; 100],
                false,
                "ZHBI - incorrect flag state - zero flag not set",
            ),
            TestEntryU32Standard::new(
                &[
                    // Manually set the overflow flag state.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::OF]), RegisterId::FL),
                    ZeroHighBitsByIndexU32RegU32Imm(
                        0x0,
                        RegisterId::R1,
                        RegisterId::R2, // Already has the value of 0.
                    ),
                ],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF]))],
                vec![0; 100],
                false,
                "ZHBI - incorrect flag state - overflow flag not cleared",
            ),
            TestEntryU32Standard::new(
                &[ZeroHighBitsByIndexU32RegU32Imm(
                    0x20,
                    RegisterId::R1,
                    RegisterId::R2,
                )],
                &[],
                vec![0; 100],
                true,
                "ZHBI - successfully executed instruction with invalid bit index",
            ),
            // Test the signed flag gets set.
            TestEntryU32Standard::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::FL),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R2),
                    ZeroHighBitsByIndexU32RegU32Imm(0x0, RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::SF, CpuFlag::CF]),
                    ),
                ],
                vec![0; 100],
                false,
                "ZHBI - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestEntryU32Standard::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_for(&[CpuFlag::SF]), RegisterId::FL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R2),
                    ZeroHighBitsByIndexU32RegU32Imm(0x0, RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "ZHBI - CPU signed flag not correctly cleared",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the bit-test instruction with a u32 register.
    #[test]
    fn test_bit_test_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    BitTestU32Reg(0x0, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1110, RegisterId::R1),
                    BitTestU32Reg(0x0, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1110)],
                vec![0; 100],
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestEntryU32Standard::new(
                &[BitTestU32Reg(0x20, RegisterId::R1)],
                &[],
                vec![],
                true,
                "BT - successfully executed instruction with invalid bit index",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the bit-test instruction with a memory address.
    #[test]
    fn test_bit_test_mem() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitTestU32Mem(0x0, 0x0),
                ],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF]))],
                vec![
                    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0b1111_1111_1111_1111_1111_1111_1111_1110, 0x0),
                    BitTestU32Mem(0x0, 0x0),
                ],
                &[],
                vec![
                    254, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestEntryU32Standard::new(
                &[BitTestU32Mem(0x20, 0x0)],
                &[],
                vec![],
                true,
                "BT - successfully executed instruction with invalid bit index",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the bit-test and reset instruction with a u32 register.
    #[test]
    fn test_bit_test_reset_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    BitTestResetU32Reg(0x0, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "BTR - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        RegisterId::R1,
                    ),
                    BitTestResetU32Reg(0x0, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1110)],
                vec![0; 100],
                false,
                "BTR - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestEntryU32Standard::new(
                &[
                    BitTestResetU32Reg(0x20, RegisterId::R1),
                ],
                &[],
                vec![],
                true,
                "BTR - successfully executed instruction with invalid bit index",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the bit-test and reset instruction with a memory address.
    #[test]
    fn test_bit_test_reset_mem() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        0x0,
                    ),
                    BitTestResetU32Mem(0x0, 0x0),
                ],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF]))],
                vec![
                    254, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BTR - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        0x0,
                    ),
                    BitTestResetU32Mem(0x0, 0x0),
                ],
                &[],
                vec![
                    254, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BTR - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestEntryU32Standard::new(
                &[
                    BitTestResetU32Mem(0x20, 0x0),
                ],
                &[],
                vec![],
                true,
                "BTR - successfully executed instruction with invalid bit index",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the bit-test and set instruction with a u32 register.
    #[test]
    fn test_bit_test_set_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    BitTestSetU32Reg(0x0, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "BTS - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        RegisterId::R1,
                    ),
                    BitTestSetU32Reg(0x0, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                vec![0; 100],
                false,
                "BTS - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestEntryU32Standard::new(
                &[
                    BitTestSetU32Reg(0x20, RegisterId::R1),
                ],
                &[],
                vec![],
                true,
                "BTS - successfully executed instruction with invalid bit index",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the bit-test and set instruction with a memory address.
    #[test]
    fn test_bit_test_set_mem() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        0x0,
                    ),
                    BitTestSetU32Mem(0x0, 0x0),
                ],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF]))],
                vec![
                    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BTS - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        0x0,
                    ),
                    BitTestSetU32Mem(0x0, 0x0),
                ],
                &[],
                vec![
                    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BTS - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestEntryU32Standard::new(
                &[
                    BitTestSetU32Mem(0x20, 0x0),
                ],
                &[],
                vec![],
                true,
                "BTS - successfully executed instruction with invalid bit index",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the reverse bit scan with a source u32 register and a destination u32 register.
    #[test]
    fn test_bit_scan_reverse_u32_reg_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    BitScanReverseU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                vec![0; 100],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b0000_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    BitScanReverseU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0b0000_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0x4),
                ],
                vec![0; 100],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::R1),
                    BitScanReverseU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R2, 0x20),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF])),
                ],
                vec![0; 100],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the reverse bit scan with a source memory address and a destination u32 register.
    #[test]
    fn test_bit_scan_reverse_mem_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitScanReverseU32MemU32Reg(0x0, RegisterId::R1),
                ],
                &[],
                vec![
                    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0b0000_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitScanReverseU32MemU32Reg(0x0, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x4)],
                vec![
                    255, 255, 255, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[BitScanReverseU32MemU32Reg(0x0, RegisterId::R1)],
                &[
                    (RegisterId::R1, 0x20),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF])),
                ],
                vec![0; 100],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the reverse bit scan with a register and a destination memory address.
    #[test]
    fn test_bit_scan_reverse_u32_reg_mem() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    BitScanReverseU32RegMemU32(RegisterId::R1, 0x0),
                ],
                &[(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                vec![0; 100],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b0000_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    BitScanReverseU32RegMemU32(RegisterId::R1, 0x0),
                ],
                &[(RegisterId::R1, 0b0000_1111_1111_1111_1111_1111_1111_1111)],
                vec![
                    4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[BitScanReverseU32RegMemU32(RegisterId::R1, 0x0)],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF]))],
                vec![
                    32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the reverse bit scan with a source memory address and a destination memory address.
    #[test]
    fn test_bit_scan_reverse_mem_mem() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitScanReverseU32MemU32Mem(0x0, 0x4),
                ],
                &[],
                vec![
                    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0b0000_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitScanReverseU32MemU32Mem(0x0, 0x4),
                ],
                &[],
                vec![
                    255, 255, 255, 15, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[BitScanReverseU32MemU32Mem(0x0, 0x4)],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF]))],
                vec![
                    0, 0, 0, 0, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the forward bit scan with a source u32 register and a destination u32 register.
    #[test]
    fn test_bit_scan_forward_u32_reg_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    BitScanForwardU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                vec![0; 100],
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_0000, RegisterId::R1),
                    BitScanForwardU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_0000),
                    (RegisterId::R2, 0x4),
                ],
                vec![0; 100],
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::R1),
                    BitScanForwardU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R2, 0x20),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF])),
                ],
                vec![0; 100],
                false,
                "BSF - incorrect result produced from the bit search",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the forward bit scan with a source memory address and a destination u32 register.
    #[test]
    fn test_bit_scan_forward_mem_u32_reg() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitScanForwardU32MemU32Reg(0x0, RegisterId::R1),
                ],
                &[],
                vec![
                    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0b1111_1111_1111_1111_1111_1111_1111_0000, 0x0),
                    BitScanForwardU32MemU32Reg(0x0, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x4)],
                vec![
                    240, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[BitScanForwardU32MemU32Reg(0x0, RegisterId::R1)],
                &[
                    (RegisterId::R1, 0x20),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF])),
                ],
                vec![0; 100],
                false,
                "BSF - incorrect result produced from the bit search",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the forward bit scan with a register and a destination memory address.
    #[test]
    fn test_bit_scan_forward_u32_reg_mem() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::R1),
                    BitScanForwardU32RegMemU32(RegisterId::R1, 0x0),
                ],
                &[(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                vec![0; 100],
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_0000, RegisterId::R1),
                    BitScanForwardU32RegMemU32(RegisterId::R1, 0x0),
                ],
                &[(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_0000)],
                vec![
                    4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[BitScanForwardU32RegMemU32(RegisterId::R1, 0x0)],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF]))],
                vec![
                    32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BSF - incorrect result produced from the bit search",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the forward bit scan with a source memory address and a destination memory address.
    #[test]
    fn test_bit_scan_forward_mem_mem() {
        let tests = [
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitScanForwardU32MemU32Mem(0x0, 0x4),
                ],
                &[],
                vec![
                    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[
                    MovU32ImmMemRelSimple(0b1111_1111_1111_1111_1111_1111_1111_0000, 0x0),
                    BitScanForwardU32MemU32Mem(0x0, 0x4),
                ],
                &[],
                vec![
                    240, 255, 255, 255, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[BitScanForwardU32MemU32Mem(0x0, 0x4)],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF]))],
                vec![
                    0, 0, 0, 0, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /*/// Test push u32 immediate instruction.
    #[test]
    fn test_push_u32_imm() {
        /*let test_values = [
            vec![0x123],
            //vec![0x123, 0x321]
        ];

        let mut tests = vec![];

        for test_inputs in test_values {
            let mut input_bits = vec![];
            for input in test_inputs {
                input_bits.push(PushU32Imm(input));
            }

            tests.push(TestEntryU32Standard::new(
                &input_bits,
                &[],
                vec![],
                false,
                "PUSH - incorrect result produced from the push",
            ))
        }*/

        let tests = [
            TestEntryU32Standard::new(
                &[
                    PushU32Imm(0b1111_1111_1111_1111_1111_1111_1111_1111),
                ],
                &[],
                vec![],
                false,
                "PUSH - incorrect result produced from the push instruction",
            ),
        ];

        for (id, test) in tests.iter().enumerate() {
            let (cpu, mem) = test.run_test(id);
        }
    }*/
}
*/
