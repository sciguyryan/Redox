use core::fmt;
use std::{panic, slice::Iter};

use crate::{
    ins::{
        instruction::Instruction,
        move_expressions::{ExpressionArgs, MoveExpressionHandler},
    },
    mem::memory_handler::MemoryHandler,
    privilege_level::PrivilegeLevel,
    reg::registers::{RegisterId, Registers},
    utils,
};

/// A list of f32 registers to be preserved when entering a subroutine. Use when storing state.
const PRESERVE_REGISTERS_F32: [RegisterId; 1] = [RegisterId::FR1];

/// A reversed list of f32 registers to be preserved when entering a subroutine. Use when restoring state.
const PRESERVE_REGISTERS_F32_REV: [RegisterId; 1] = [RegisterId::FR1];

/// A list of u32 registers to be preserved when entering a subroutine. Use when storing state.
const PRESERVE_REGISTERS_U32: [RegisterId; 10] = [
    RegisterId::ER1,
    RegisterId::ER2,
    RegisterId::ER3,
    RegisterId::ER4,
    RegisterId::ER5,
    RegisterId::ER6,
    RegisterId::ER7,
    RegisterId::ER8,
    RegisterId::EIP,
    RegisterId::EBP,
];

/// A reversed list of u32 registers to be preserved when entering a subroutine. Use when restoring state.
const PRESERVE_REGISTERS_U32_REV: [RegisterId; 10] = [
    RegisterId::EBP,
    RegisterId::EIP,
    RegisterId::ER8,
    RegisterId::ER7,
    RegisterId::ER6,
    RegisterId::ER5,
    RegisterId::ER4,
    RegisterId::ER3,
    RegisterId::ER2,
    RegisterId::ER1,
];

/// The mask to get only the lowest 8 bits of a u32 value.
const U32_LOW_BYTE_MASK: u32 = 0xff;

pub struct Cpu {
    /// The registers associated with this CPU.
    pub registers: Registers,
    // Is the CPU currently halted?
    pub is_halted: bool,
    /// Is the CPU currently running in machine mode (superuser)?
    pub is_machine_mode: bool,
    /// Is the CPU currently in an interrupt handler?
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

    /// Get the instruction pointer (IP) register.
    #[inline(always)]
    pub fn get_instruction_pointer(&self) -> u32 {
        *self
            .registers
            .get_register_u32(RegisterId::EIP)
            .read_unchecked()
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

    /// Fetch and decode the next instruction to be executed.
    ///
    /// # Arguments
    ///
    /// * `mem` - A reference to the virtual machine [`Memory`] instance.
    ///
    /// # Returns
    ///
    /// The next [`Instruction`] to be executed.
    fn fetch_decode_next_instruction(&mut self, mem: &MemoryHandler) -> Instruction {
        // Get the current instruction pointer.
        let ip = *self
            .registers
            .get_register_u32(RegisterId::EIP)
            .read_unchecked() as usize;

        mem.get_instruction(ip)
    }

    /// Get the size of the current stack frame.
    #[inline(always)]
    pub fn get_stack_frame_size(&self) -> u32 {
        self.registers
            .get_register_u32(RegisterId::EBP)
            .read_unchecked()
            - self
                .registers
                .get_register_u32(RegisterId::ESP)
                .read_unchecked()
    }

    /// Get the value of the stack pointer (SP) register.
    #[inline(always)]
    pub fn get_stack_pointer(&self) -> u32 {
        *self
            .registers
            .get_register_u32(RegisterId::ESP)
            .read_unchecked()
    }

    /// Increment the instruction pointer (IP) register by a specified amount.
    #[inline(always)]
    fn increment_ip_register(&mut self, amount: usize) {
        self.registers
            .get_register_u32_mut(RegisterId::EIP)
            .add_unchecked(amount as u32);
    }

    /// Update the program counter (PC) register.
    #[inline(always)]
    fn increment_pc_register(&mut self) {
        self.registers
            .get_register_u32_mut(RegisterId::EPC)
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
        let (final_value, overflow) = value_1.overflowing_add(value_2);

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
        let (final_value, overflow) = value_1.overflowing_sub(value_2);

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

    /// Pop the relevant processor register states from the stack.
    ///
    /// # Arguments
    ///
    /// * `mem` - The [`Memory`] connected to this CPU instance.
    pub(crate) fn pop_state(&mut self, mem: &mut MemoryHandler) {
        let privilege = PrivilegeLevel::Machine;

        // NOTE - Remember that the states need to be popped in the reverse order!

        // Pop the f32 data registers from the stack.
        for reg in PRESERVE_REGISTERS_F32_REV {
            self.write_reg_f32(&reg, mem.pop_f32(), &privilege);
        }

        // Next, pop the the u32 data registers from the stack.
        for reg in PRESERVE_REGISTERS_U32_REV {
            self.write_reg_u32(&reg, mem.pop_u32(), &privilege);
        }
    }

    /// Push the relevant processor register states to the stack.
    ///
    /// # Arguments
    ///
    /// * `mem` - The [`Memory`] connected to this CPU instance.
    pub(crate) fn push_state(&mut self, mem: &mut MemoryHandler) {
        let privilege = PrivilegeLevel::Machine;

        // Push the u32 data registers to the stack.
        for reg in PRESERVE_REGISTERS_U32 {
            mem.push_u32(self.read_reg_u32(&reg, &privilege));
        }

        // Next, push the f32 data registers to the stack.
        for reg in PRESERVE_REGISTERS_F32 {
            mem.push_f32(self.read_reg_f32(&reg, &privilege));
        }
    }

    /// Get the value of a specific f32 register.
    ///
    /// # Arguments
    ///
    /// * `reg` - Reference to the [`RegisterId`] for the register in question.
    /// * `privilege` - The [`PrivilegeLevel`] with which the read request should be processed.
    ///
    /// # Returns
    ///
    /// The f32 value of the register.
    #[inline(always)]
    fn read_reg_f32(&self, reg: &RegisterId, privilege: &PrivilegeLevel) -> f32 {
        *self.registers.get_register_f32(*reg).read(privilege)
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
        self.registers = Registers::default();
        self.set_halted(false);
    }

    /// Run the CPU from a specified starting point in memory.
    ///
    /// # Arguments
    ///
    /// * `mem` - A reference to the virtual machine [`Memory`] instance.
    pub fn run(&mut self, mem: &mut MemoryHandler) {
        while !self.is_halted {
            let ins = self.fetch_decode_next_instruction(mem);
            self.run_instruction(mem, &ins);
        }
    }

    /// Execute a set of instructions on the CPU.
    ///
    /// # Arguments
    ///
    /// * `mem` - The [`Memory`] connected to this CPU instance.
    /// * `instructions` - A slice of [`Instruction`] instances to be executed.
    pub fn run_instructions(&mut self, mem: &mut MemoryHandler, instructions: &[Instruction]) {
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
    fn run_instruction(&mut self, mem: &mut MemoryHandler, instruction: &Instruction) {
        use Instruction::*;

        let privilege = &self.get_privilege();

        // Should we skip updating the instruction pointer register after this instruction?
        let mut skip_ip_update = false;

        match instruction {
            Nop => {}

            /******** [Arithmetic Instructions] ********/
            AddU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_add_u32(old_value, *imm);

                self.set_u32_accumulator(new_value);
            }
            AddU32RegU32Reg(in_reg, out_reg) => {
                let value1 = self.read_reg_u32(in_reg, privilege);
                let value2 = self.read_reg_u32(out_reg, privilege);
                let new_value = self.perform_checked_add_u32(value1, value2);

                self.set_u32_accumulator(new_value);
            }
            SubU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_subtract_u32(old_value, *imm);

                self.set_u32_accumulator(new_value);
            }
            SubU32RegU32Imm(reg, imm) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_subtract_u32(*imm, old_value);

                self.set_u32_accumulator(new_value);
            }
            SubU32RegU32Reg(reg_1, reg_2) => {
                let reg_1_val = self.read_reg_u32(reg_1, privilege);
                let reg_2_val = self.read_reg_u32(reg_2, privilege);
                let new_value = self.perform_checked_subtract_u32(reg_2_val, reg_1_val);

                self.set_u32_accumulator(new_value);
            }
            MulU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_mul_u32(old_value, *imm);

                self.set_u32_accumulator(new_value);
            }
            MulU32RegU32Reg(reg_1, reg_2) => {
                let reg_1_val = self.read_reg_u32(reg_1, privilege);
                let reg_2_val = self.read_reg_u32(reg_2, privilege);
                let new_value = self.perform_checked_mul_u32(reg_1_val, reg_2_val);

                self.set_u32_accumulator(new_value);
            }
            DivU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_div_u32(old_value, *imm);

                self.set_u32_accumulator(new_value);
            }
            DivU32RegU32Imm(reg, imm) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_div_u32(*imm, old_value);

                self.set_u32_accumulator(new_value);
            }
            DivU32RegU32Reg(reg_1, reg_2) => {
                let reg_1_val = self.read_reg_u32(reg_1, privilege);
                let reg_2_val = self.read_reg_u32(reg_2, privilege);
                let new_value = self.perform_checked_div_u32(reg_2_val, reg_1_val);

                self.set_u32_accumulator(new_value);
            }
            ModU32ImmU32Reg(imm, reg) => {
                let reg_val = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_modulo_u32(reg_val, *imm);

                self.set_u32_accumulator(new_value);
            }
            ModU32RegU32Imm(reg, imm) => {
                let reg_val = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_modulo_u32(*imm, reg_val);

                self.set_u32_accumulator(new_value);
            }
            ModU32RegU32Reg(reg_1, reg_2) => {
                let reg_1_val = self.read_reg_u32(reg_1, privilege);
                let reg_2_val = self.read_reg_u32(reg_2, privilege);
                let new_value = self.perform_modulo_u32(reg_2_val, reg_1_val);

                self.set_u32_accumulator(new_value);
            }
            IncU32Reg(reg) => {
                let value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_add_u32(value, 1);

                self.write_reg_u32(reg, new_value, privilege);
            }
            DecU32Reg(reg) => {
                let value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_checked_subtract_u32(value, 1);

                self.write_reg_u32(reg, new_value, privilege);
            }
            AndU32ImmU32Reg(imm, reg) => {
                let value = self.read_reg_u32(reg, privilege);
                let new_value = self.perform_bitwise_and_u32(*imm, value);

                self.set_u32_accumulator(new_value);
            }

            /******** [Bit Operation Instructions] ********/
            LeftShiftU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_checked_left_shift_u32(old_value, *imm);

                self.write_reg_u32(reg, shifted, privilege);
            }
            LeftShiftU32RegU32Reg(shift_reg, reg) => {
                let shift_by = self.read_reg_u32(shift_reg, privilege);
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_checked_left_shift_u32(old_value, shift_by);

                self.write_reg_u32(reg, shifted, privilege);
            }
            ArithLeftShiftU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_arithmetic_left_shift_u32(old_value, *imm);

                self.write_reg_u32(reg, shifted, privilege);
            }
            ArithLeftShiftU32RegU32Reg(shift_reg, reg) => {
                let shift_by = self.read_reg_u32(shift_reg, privilege);
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_arithmetic_left_shift_u32(old_value, shift_by);

                self.write_reg_u32(reg, shifted, privilege);
            }
            RightShiftU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_right_shift_u32(old_value, *imm);

                self.write_reg_u32(reg, shifted, privilege);
            }
            RightShiftU32RegU32Reg(shift_reg, reg) => {
                let shift_by = self.read_reg_u32(shift_reg, privilege);
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_right_shift_u32(old_value, shift_by);

                self.write_reg_u32(reg, shifted, privilege);
            }
            ArithRightShiftU32ImmU32Reg(imm, reg) => {
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_arithmetic_right_shift_u32(old_value, *imm);

                self.write_reg_u32(reg, shifted, privilege);
            }
            ArithRightShiftU32RegU32Reg(shift_reg, reg) => {
                let shift_by = self.read_reg_u32(shift_reg, privilege);
                let old_value = self.read_reg_u32(reg, privilege);
                let shifted = self.perform_arithmetic_right_shift_u32(old_value, shift_by);

                self.write_reg_u32(reg, shifted, privilege);
            }

            /******** [Branching Instructions] ********/
            Ret => {
                todo!();
            }
            Int(_addr) => {
                todo!();
            }
            IntRet => {
                todo!();
            }
            JumpAbsU32Imm(addr, _id) => {
                // jmp 0xaaaa
                // Set the instruction pointer to the jump address.
                self.set_instruction_pointer(*addr);

                skip_ip_update = true;
            }
            JumpAbsU32Reg(reg) => {
                // jmp %cs
                let shift_by = self.read_reg_u32(reg, privilege);

                // Set the instruction pointer to the jump address.
                self.set_instruction_pointer(shift_by);

                skip_ip_update = true;
            }

            /******** [Data Instructions] ********/
            SwapU32RegU32Reg(reg_1, reg_2) => {
                let reg_1_val = self.read_reg_u32(reg_1, privilege);
                let reg_2_val = self.read_reg_u32(reg_2, privilege);

                self.write_reg_u32(reg_1, reg_2_val, privilege);
                self.write_reg_u32(reg_2, reg_1_val, privilege);
            }
            MovU32ImmU32Reg(imm, reg) => {
                self.write_reg_u32(reg, *imm, privilege);
            }
            MovU32RegU32Reg(in_reg, out_reg) => {
                let value = self.read_reg_u32(in_reg, privilege);

                self.write_reg_u32(out_reg, value, privilege);
            }
            MovU32ImmMemSimple(imm, addr) => {
                mem.set_u32(*addr as usize, *imm);
            }
            MovU32RegMemSimple(reg, addr) => {
                let value = self.read_reg_u32(reg, privilege);

                mem.set_u32(*addr as usize, value);
            }
            MovMemU32RegSimple(addr, reg) => {
                let value = mem.get_u32(*addr as usize);

                self.write_reg_u32(reg, value, privilege);
            }
            MovU32RegPtrU32RegSimple(in_reg, out_reg) => {
                let address = self.read_reg_u32(in_reg, privilege) as usize;
                let value = mem.get_u32(address);

                self.write_reg_u32(out_reg, value, privilege);
            }
            MovU32ImmMemExpr(imm, expr) => {
                // mov imm, [addr] - move immediate to address.
                let addr = self.decode_evaluate_u32_move_expression(expr, privilege);

                mem.set_u32(addr as usize, *imm);
            }
            MovMemExprU32Reg(expr, reg) => {
                // mov [addr], register - move value at address to register.
                let addr = self.decode_evaluate_u32_move_expression(expr, privilege);
                let value = mem.get_u32(addr as usize);

                self.write_reg_u32(reg, value, privilege);
            }
            MovU32RegMemExpr(reg, expr) => {
                // mov reg, [addr] - move value of a register to an address.
                let addr = self.decode_evaluate_u32_move_expression(expr, privilege);
                let value = self.read_reg_u32(reg, privilege);

                mem.set_u32(addr as usize, value);
            }
            ZeroHighBitsByIndexU32Reg(index_reg, source_reg, out_reg) => {
                // zhbi source_reg, index_reg, out_reg
                let index = self.read_reg_u32(index_reg, privilege);
                self.perform_zero_high_bit_u32_reg(source_reg, index, out_reg, privilege);
            }
            ZeroHighBitsByIndexU32RegU32Imm(index, source_reg, out_reg) => {
                // zhbi source_reg, index, out_reg
                self.perform_zero_high_bit_u32_reg(source_reg, *index, out_reg, privilege);
            }
            PushU32Imm(imm) => {
                // push imm
                mem.push_u32(*imm);

                // Update the stack pointer register.
                self.registers
                    .get_register_u32_mut(RegisterId::ESP)
                    .subtract_unchecked(4);
            }
            PopU32ImmU32Reg(out_reg) => {
                // pop %out_reg
                self.registers
                    .get_register_u32_mut(*out_reg)
                    .write(mem.pop_u32(), privilege);
            }

            /******** [Logic Instructions] ********/
            BitTestU32Reg(bit, reg) => {
                // bt bit, reg
                let value = self.read_reg_u32(reg, privilege);
                self.perform_bit_test_with_carry_flag(value, *bit);
            }
            BitTestU32Mem(bit, addr) => {
                // bt bit, [addr]
                let value = mem.get_u32(*addr as usize);
                self.perform_bit_test_with_carry_flag(value, *bit);
            }
            BitTestResetU32Reg(bit, reg) => {
                // btr bit, reg
                // Read the value and set the carry flag state, clear the bit.
                let mut value = self.read_reg_u32(reg, privilege);
                self.perform_bit_test_with_carry_flag_with_set(&mut value, *bit, false);

                // Write the value back to the register.
                self.write_reg_u32(reg, value, privilege);
            }
            BitTestResetU32Mem(bit, addr) => {
                // btr bit, [addr]
                // Read the value and set the carry flag state, then clear the bit.
                let mut value = mem.get_u32(*addr as usize);
                self.perform_bit_test_with_carry_flag_with_set(&mut value, *bit, false);

                mem.set_u32(*addr as usize, value);
            }
            BitTestSetU32Reg(bit, reg) => {
                // bts bit, reg
                // Read the value and set the carry flag state, set the bit.
                let mut value = self.read_reg_u32(reg, privilege);
                self.perform_bit_test_with_carry_flag_with_set(&mut value, *bit, true);

                // Write the value back to the register.
                self.write_reg_u32(reg, value, privilege);
            }
            BitTestSetU32Mem(bit, addr) => {
                // bts bit, [addr]
                // Read the value and set the carry flag state, then set the bit.
                let mut value = mem.get_u32(*addr as usize);
                self.perform_bit_test_with_carry_flag_with_set(&mut value, *bit, true);

                mem.set_u32(*addr as usize, value);
            }
            BitScanReverseU32RegU32Reg(in_reg, out_reg) => {
                // bsr in_reg, out_reg
                let value = self.read_reg_u32(in_reg, privilege);
                let index = self.perform_reverse_bit_search(value);

                self.write_reg_u32(out_reg, index, privilege);
            }
            BitScanReverseU32MemU32Reg(addr, reg) => {
                // bsr [addr], reg
                let value = mem.get_u32(*addr as usize);
                let index = self.perform_reverse_bit_search(value);

                self.write_reg_u32(reg, index, privilege);
            }
            BitScanReverseU32RegMemU32(reg, out_addr) => {
                // bsr reg, [out_addr]
                let value = self.read_reg_u32(reg, privilege);
                let index = self.perform_reverse_bit_search(value);

                mem.set_u32(*out_addr as usize, index);
            }
            BitScanReverseU32MemU32Mem(in_addr, out_addr) => {
                // bsr [in_addr], [out_addr]
                let value = mem.get_u32(*in_addr as usize);
                let index = self.perform_reverse_bit_search(value);

                mem.set_u32(*out_addr as usize, index);
            }
            BitScanForwardU32RegU32Reg(in_reg, out_reg) => {
                // bsf in_reg, out_reg
                let value = self.read_reg_u32(in_reg, privilege);
                let index = self.perform_forward_bit_search(value);

                self.write_reg_u32(out_reg, index, privilege);
            }
            BitScanForwardU32MemU32Reg(addr, reg) => {
                // bsf [addr], reg
                let value = mem.get_u32(*addr as usize);
                let index = self.perform_forward_bit_search(value);

                self.write_reg_u32(reg, index, privilege);
            }
            BitScanForwardU32RegMemU32(reg, out_addr) => {
                // bsf reg, [out_addr]
                let value = self.read_reg_u32(reg, privilege);
                let index = self.perform_forward_bit_search(value);

                mem.set_u32(*out_addr as usize, index);
            }
            BitScanForwardU32MemU32Mem(in_addr, out_addr) => {
                // bsf [in_addr], [out_addr]
                let value = mem.get_u32(*in_addr as usize);
                let index = self.perform_forward_bit_search(value);

                mem.set_u32(*out_addr as usize, index);
            }
            ByteSwapU32(reg) => {
                // bswap reg
                let value = self.read_reg_u32(reg, privilege).swap_bytes();

                self.write_reg_u32(reg, value, privilege);
            }

            /******** [Special Instructions] ********/
            Mret => {
                self.set_machine_mode(false);
            }
            Hlt => {
                self.set_halted(true);
            }

            /******** [Reserved Instructions] ********/
            Reserved1 | Reserved2 | Reserved3 | Reserved4 | Reserved5 | Reserved6 | Reserved7
            | Reserved8 | Reserved9 => {
                unreachable!("attempted to use a reserved instruction");
            }

            /******** [Pseudo Instructions] ********/
            Label(_) => {
                unreachable!("attempted to use label pseudo instruction");
            }
            Unknown(id) => {
                panic!("attempted to run an unrecognized instruction: {id}");
            }
        };

        self.increment_pc_register();

        // Advance the instruction pointer by the number of bytes used for the instruction,
        // if we don't need to skip this step.
        if !skip_ip_update {
            self.increment_ip_register(instruction.get_total_instruction_size());
        }
    }

    /// Set the state of the specified CPU flag.
    ///
    /// # Arguments
    ///
    /// * `flag` - The [`CpuFlag`] to be set or unset.
    /// * `state` - The new state of the flag.
    #[inline(always)]
    pub fn set_flag_state(&mut self, flag: CpuFlag, state: bool) {
        let register = self.registers.get_register_u32_mut(RegisterId::EFL);
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
            .get_register_u32_mut(RegisterId::EIP)
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
    pub fn set_segment_registers(&mut self, mem: &MemoryHandler) {
        self.registers
            .get_register_u32_mut(RegisterId::ESS)
            .write_unchecked(mem.stack_segment_start as u32);

        self.registers
            .get_register_u32_mut(RegisterId::ECS)
            .write_unchecked(mem.code_segment_start as u32);

        self.registers
            .get_register_u32_mut(RegisterId::EDS)
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
            .get_register_u32_mut(RegisterId::EBP)
            .write_unchecked(value);
    }

    // Update the stack pointer (SP) register.
    ///
    /// # Arguments
    ///
    /// * `value` - The new value of the stack pointer register.
    #[inline(always)]
    pub fn set_stack_pointer(&mut self, value: u32) {
        self.registers
            .get_register_u32_mut(RegisterId::ESP)
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
            .get_register_u32_mut(RegisterId::EAC)
            .write_unchecked(value);
    }

    /// Set the value of a specific f32 register.
    ///
    /// # Arguments
    ///
    /// * `reg` - Reference to the [`RegisterId`] for the register in question.
    /// * `new_val` - The new value of the register.
    /// * `privilege` - The [`PrivilegeLevel`] with which the read request should be processed.
    #[inline(always)]
    fn write_reg_f32(&mut self, reg: &RegisterId, new_val: f32, privilege: &PrivilegeLevel) {
        self.registers
            .get_register_f32_mut(*reg)
            .write(new_val, privilege);
    }

    /// Set the value of a specific u32 register.
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
    /// The interrupt enabled flag - set to true if interrupts are enabled.
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
mod tests_cpu {
    use prettytable::{row, Table};
    use std::{collections::HashMap, panic};

    use crate::{
        compiler::bytecode_compiler::Compiler,
        ins::{
            instruction::Instruction::{self, *},
            move_expressions::{ExpressionArgs, ExpressionOperator, MoveExpressionHandler},
        },
        mem,
        reg::registers::RegisterId,
        vm::VirtualMachine,
    };

    use super::{Cpu, CpuFlag, PRESERVE_REGISTERS_F32, PRESERVE_REGISTERS_U32};

    struct TestsU32 {
        tests: Vec<TestU32>,
    }

    impl TestsU32 {
        pub fn new(tests: &[TestU32]) -> Self {
            Self {
                tests: tests.to_vec(),
            }
        }

        /// Run each unit test in the specified sequence.
        pub fn run_all(&self) {
            for (id, test) in self.tests.iter().enumerate() {
                test.run_test(id);
            }
        }

        /// Run each unit test in the specified sequence, then apply the closure function to the resulting virtual machine instance.
        ///
        /// # Arguments
        ///
        /// * `closure` - The closure to be executed on the completed virtual machine, to perform additional checks.
        pub fn run_all_special<F>(&self, closure: F)
        where
            F: Fn(usize, Option<VirtualMachine>),
        {
            for (id, test) in self.tests.iter().enumerate() {
                closure(id, test.run_test(id));
            }
        }
    }

    #[derive(Clone)]
    struct TestU32 {
        /// A vector of [`Instruction`]s to be executed.
        pub instructions: Vec<Instruction>,
        /// A [`HashMap`] containing [`RegisterId`] and the expected value of the register after execution.
        pub expected_changed_registers: HashMap<RegisterId, u32>,
        // An option containing a vector of bytes representing the expected user segment memory contents after execution, if specified.
        pub expected_user_seg_contents: Option<Vec<u8>>,
        /// The capacity of the user memory segment, in bytes.
        pub user_seg_capacity_bytes: usize,
        /// The capacity of the stack memory segment, in bytes.
        pub stack_seg_capacity_u32: usize,
        /// A boolean indicating whether the test should panic or not.
        pub should_panic: bool,
        /// A string slice that provides the message to be displayed if the test fails.
        pub fail_message: String,
    }

    impl TestU32 {
        /// Create a new [`TestEntryU32Standard`] instance.
        ///
        /// # Arguments
        ///
        /// * `instructions` - A slice of [`Instruction`]s to be executed.
        /// * `expected_registers` - A slice of a tuple containing the [`RegisterId`] and the expected value of the register after execution.
        /// * `expected_user_seg_contents` - An option containing a vector of bytes representing the expected user segment memory contents after execution, if specified.
        /// * `stack_seg_capacity_u32` - The capacity of the stack memory segment, in bytes.
        /// * `should_panic` - A boolean indicating whether the test should panic or not.
        /// * `fail_message` - A string slice that provides the message to be displayed if the test fails.
        ///
        /// # Note
        ///
        /// If the results need to check the user segment memory contents then the VM will automatically be
        /// created with a memory segment of the correct size. It doesn't need to be specified manually.
        fn new(
            instructions: &[Instruction],
            expected_registers: &[(RegisterId, u32)],
            expected_user_seg_contents: Option<Vec<u8>>,
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

            let (user_segment_memory_capacity, user_segment_contents) =
                expected_user_seg_contents.map_or_else(|| (100, None), |v| (v.len(), Some(v)));

            Self {
                instructions: instructions_vec,
                expected_changed_registers: changed_registers,
                user_seg_capacity_bytes: user_segment_memory_capacity,
                expected_user_seg_contents: user_segment_contents,
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
        ///
        /// # Returns
        ///
        /// An option containing the [`VirtualMachine`] instance if the execution did not panic, or None otherwise.
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
                    self.stack_seg_capacity_u32 * mem::memory_handler::BYTES_IN_U32,
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
                    vm.mem.get_user_segment_storage(),
                    *contents,
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

    /// Test the pop and push of the CPU state.
    #[test]
    fn test_push_pop_state() {
        // Build the virtual machine instance.
        let mut vm = VirtualMachine::new(100, &[], &[], 30 * mem::memory_handler::BYTES_IN_U32);

        for (id, reg) in PRESERVE_REGISTERS_U32.iter().enumerate() {
            vm.cpu
                .registers
                .get_register_u32_mut(*reg)
                .write_unchecked((id + 1) as u32);
        }
        for (id, reg) in PRESERVE_REGISTERS_F32.iter().enumerate() {
            vm.cpu
                .registers
                .get_register_f32_mut(*reg)
                .write_unchecked((id + 1) as f32);
        }

        // Keep a copy of the registers struct.
        let reference_registers = vm.cpu.registers.clone();

        // Store the state.
        vm.cpu.push_state(&mut vm.mem);

        // Reset the registers.
        for reg in PRESERVE_REGISTERS_U32 {
            vm.cpu
                .registers
                .get_register_u32_mut(reg)
                .write_unchecked(0);
        }
        for reg in PRESERVE_REGISTERS_F32 {
            vm.cpu
                .registers
                .get_register_f32_mut(reg)
                .write_unchecked(0f32);
        }

        // Restore the state.
        vm.cpu.pop_state(&mut vm.mem);

        // Our registers should now match the copy we made earlier.
        assert_eq!(reference_registers, vm.cpu.registers);
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

    /// Test the nop instruction.
    #[test]
    fn test_nop() {
        let tests = [TestU32::new(
            &[Nop],
            &[],
            None,
            0,
            false,
            "NOP - failed to execute NOP instruction",
        )];

        TestsU32::new(&tests).run_all();
    }

    /// Test the halt instruction.
    #[test]
    fn test_hlt() {
        let tests = [
            TestU32::new(
                &[Hlt],
                &[],
                None,
                0,
                false,
                "HLT - failed to execute HLT instruction",
            ),
            // The halt instruction should prevent any following instructions from executing, which
            // means that the register value will never change.
            TestU32::new(
                &[Hlt, MovU32ImmU32Reg(0xf, RegisterId::ER1)],
                &[(RegisterId::ER1, 0)],
                None,
                0,
                false,
                "HLT - failed to correctly stop execution after a HLT instruction",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the mret instruction.
    #[test]
    fn test_mret() {
        let tests = [TestU32::new(
            &[Mret],
            &[],
            None,
            0,
            false,
            "MRET - failed to execute MRET instruction",
        )];

        TestsU32::new(&tests).run_all_special(|id: usize, vm: Option<VirtualMachine>| {
            if let Some(v) = vm {
                assert!(
                    !v.cpu.is_machine_mode,
                    "Test {id} Failed - machine is still in machine mode after executing mret instruction!"
                );
            } else {
                // We can't do anything here. The test asserted and so we didn't
                // yield a valid virtual machine instance to interrogate.
            }
        });
    }

    /// Testing the use of unrecognized instruction.
    #[test]
    #[should_panic]
    fn test_invalid_instruction() {
        // The following is an invalid opcode.
        let compiled = (u32::MAX - 11).to_le_bytes();

        // Build a virtual machine instance.
        let mut vm = VirtualMachine::new(100, &compiled, &[], 0);

        vm.run();
    }

    /// Test the add u32 immediate to u32 register instruction.
    #[test]
    fn test_add_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    AddU32ImmU32Reg(0x2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::EAC, 0x3),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::PF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "ADD - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::ER1),
                    AddU32ImmU32Reg(0x2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, u32::MAX),
                    (RegisterId::EAC, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "ADD - CPU flags not correctly set",
            ),
            TestU32::new(
                &[AddU32ImmU32Reg(0, RegisterId::ER1)],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                )],
                None,
                0,
                false,
                "ADD - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    AddU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x2),
                    (RegisterId::EAC, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                0,
                false,
                "ADD - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    AddU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::EAC, 0x2),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "ADD - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    AddU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EAC, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                None,
                0,
                false,
                "ADD - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    AddU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::EAC, 0x2),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "ADD - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the add u32 register to u32 register instruction.
    #[test]
    fn test_add_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0xf, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    AddU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0xf),
                    (RegisterId::ER2, 0x1),
                    (RegisterId::EAC, 0x10),
                ],
                None,
                0,
                false,
                "ADD - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::ER1),
                    MovU32ImmU32Reg(0x2, RegisterId::ER2),
                    AddU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, u32::MAX),
                    (RegisterId::ER2, 0x2),
                    (RegisterId::EAC, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "ADD - CPU flags not correctly set",
            ),
            TestU32::new(
                &[AddU32RegU32Reg(RegisterId::ER1, RegisterId::ER2)],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                )],
                None,
                0,
                false,
                "ADD - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    AddU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0x2),
                    (RegisterId::ER2, 0x1),
                    (RegisterId::EAC, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                0,
                false,
                "ADD - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    AddU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::EAC, 0x2),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "ADD - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    AddU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0x1),
                    (RegisterId::EAC, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                None,
                0,
                false,
                "ADD - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    AddU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::ER2, 0x1),
                    (RegisterId::EAC, 0x2),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "ADD - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test subtraction of u32 immediate from a u32 register instruction.
    #[test]
    fn test_sub_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    SubU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x2), (RegisterId::EAC, 0x1)],
                None,
                0,
                false,
                "SUB - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    SubU32ImmU32Reg(u32::MAX, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x2),
                    (RegisterId::EAC, 0x3),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::PF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "SUB - CPU flags not correctly set",
            ),
            TestU32::new(
                &[SubU32ImmU32Reg(0, RegisterId::ER1)],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                )],
                None,
                0,
                false,
                "SUB - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x4, RegisterId::ER1),
                    SubU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x4),
                    (RegisterId::EAC, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                0,
                false,
                "SUB - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    SubU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x2),
                    (RegisterId::EAC, 0x1),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "SUB - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    SubU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EAC, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::SF])),
                ],
                None,
                0,
                false,
                "SUB - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    SubU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x2),
                    (RegisterId::EAC, 0x1),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "SUB - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test subtraction of a u32 register from a u32 immediate instruction.
    #[test]
    fn test_sub_u32_reg_u32_imm() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    SubU32RegU32Imm(RegisterId::ER1, 0x3),
                ],
                &[(RegisterId::ER1, 0x2), (RegisterId::EAC, 0x1)],
                None,
                0,
                false,
                "SUB - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::ER1),
                    SubU32RegU32Imm(RegisterId::ER1, 0x2),
                ],
                &[
                    (RegisterId::ER1, u32::MAX),
                    (RegisterId::EAC, 0x3),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::PF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "SUB - CPU flags not correctly set",
            ),
            TestU32::new(
                &[SubU32RegU32Imm(RegisterId::ER1, 0)],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                )],
                None,
                0,
                false,
                "SUB - CPU flags not correctly set",
            ),
            // Test the parity flag gets enabled.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    SubU32RegU32Imm(RegisterId::ER1, 0x4),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::EAC, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                0,
                false,
                "SUB - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    SubU32RegU32Imm(RegisterId::ER1, 0x2),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::EAC, 0x1),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "SUB - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    SubU32RegU32Imm(RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::EAC, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::SF])),
                ],
                None,
                0,
                false,
                "SUB - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    SubU32RegU32Imm(RegisterId::ER1, 0x2),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::EAC, 0x1),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "SUB - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the subtract u32 register from u32 register instruction.
    #[test]
    fn test_sub_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32ImmU32Reg(0xf, RegisterId::ER2),
                    SubU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::ER2, 0xf),
                    (RegisterId::EAC, 0xe),
                ],
                None,
                0,
                false,
                "SUB - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::ER1),
                    MovU32ImmU32Reg(0x2, RegisterId::ER2),
                    SubU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, u32::MAX),
                    (RegisterId::ER2, 0x2),
                    (RegisterId::EAC, 0x3),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::PF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "SUB - CPU flags not correctly set",
            ),
            TestU32::new(
                &[SubU32RegU32Reg(RegisterId::ER1, RegisterId::ER2)],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                )],
                None,
                0,
                false,
                "SUB - CPU flags not correctly set",
            ),
            // Test the parity flag gets enabled.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32ImmU32Reg(0x4, RegisterId::ER2),
                    SubU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::ER2, 0x4),
                    (RegisterId::EAC, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                0,
                false,
                "SUB - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32ImmU32Reg(0x2, RegisterId::ER2),
                    SubU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::ER2, 0x2),
                    (RegisterId::EAC, 0x1),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "SUB - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER2),
                    SubU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::ER2, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EAC, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::SF])),
                ],
                None,
                0,
                false,
                "SUB - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32ImmU32Reg(0x2, RegisterId::ER2),
                    SubU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::ER2, 0x2),
                    (RegisterId::EAC, 0x1),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "SUB - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the multiply u32 immediate by a u32 register instruction.
    #[test]
    fn test_mul_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MulU32ImmU32Reg(0x2, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1), (RegisterId::EAC, 0x2)],
                None,
                0,
                false,
                "MUL - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::ER1),
                    MulU32ImmU32Reg(0x2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, u32::MAX),
                    (RegisterId::EAC, 4294967294),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::CF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "MUL - CPU flags not correctly set",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the multiply u32 immediate by a u32 register instruction.
    #[test]
    fn test_mul_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32ImmU32Reg(0x2, RegisterId::ER2),
                    MulU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::ER2, 0x2),
                    (RegisterId::EAC, 0x2),
                ],
                None,
                0,
                false,
                "MUL - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::ER1),
                    MovU32ImmU32Reg(0x2, RegisterId::ER2),
                    MulU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, u32::MAX),
                    (RegisterId::ER2, 0x2),
                    (RegisterId::EAC, 4294967294),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::CF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "MUL - CPU flags not correctly set",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the division of a u32 register by a u32 immediate instruction.
    #[test]
    fn test_div_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    DivU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x2), (RegisterId::EAC, 0x2)],
                None,
                0,
                false,
                "DIV - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::ER1),
                    DivU32ImmU32Reg(0x2, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, u32::MAX), (RegisterId::EAC, 2147483647)],
                None,
                0,
                false,
                "DIV - CPU flags not correctly set",
            ),
            TestU32::new(
                &[DivU32ImmU32Reg(0x0, RegisterId::ER1)],
                &[],
                None,
                0,
                true,
                "DIV - failed to panic when attempting to divide by zero",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the division of a u32 immediate by a u32 register instruction.
    #[test]
    fn test_div_u32_reg_u32_imm() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    DivU32RegU32Imm(RegisterId::ER1, 0x2),
                ],
                &[(RegisterId::ER1, 0x1), (RegisterId::EAC, 0x2)],
                None,
                0,
                false,
                "DIV - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    DivU32RegU32Imm(RegisterId::ER1, u32::MAX),
                ],
                &[(RegisterId::ER1, 0x2), (RegisterId::EAC, 2147483647)],
                None,
                0,
                false,
                "DIV - CPU flags not correctly set",
            ),
            TestU32::new(
                &[DivU32RegU32Imm(RegisterId::ER1, 0x0)],
                &[],
                None,
                0,
                true,
                "DIV - failed to panic when attempting to divide by zero",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the division of a u32 register by a u32 register instruction.
    #[test]
    fn test_div_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    DivU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x2),
                    (RegisterId::ER2, 0x1),
                    (RegisterId::EAC, 0x2),
                ],
                None,
                0,
                false,
                "DIV - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::ER1),
                    MovU32ImmU32Reg(0x2, RegisterId::ER2),
                    DivU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, u32::MAX),
                    (RegisterId::ER2, 0x2),
                    (RegisterId::EAC, 2147483647),
                ],
                None,
                0,
                false,
                "DIV - CPU flags not correctly set",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    DivU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[],
                None,
                0,
                true,
                "DIV - failed to panic when attempting to divide by zero",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the modulo of a u32 register by a u32 immediate instruction.
    #[test]
    fn test_mod_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    ModU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x2), (RegisterId::EAC, 0x0)],
                None,
                0,
                false,
                "MOD - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x3, RegisterId::ER1),
                    ModU32ImmU32Reg(0x2, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x3), (RegisterId::EAC, 0x1)],
                None,
                0,
                false,
                "MOD - incorrect result value produced",
            ),
            TestU32::new(
                &[DivU32ImmU32Reg(0x0, RegisterId::ER1)],
                &[],
                None,
                0,
                true,
                "MOD - failed to panic when attempting to divide by zero",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the modulo of a u32 immediate by a u32 register instruction.
    #[test]
    fn test_mod_u32_reg_u32_imm() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    ModU32RegU32Imm(RegisterId::ER1, 0x2),
                ],
                &[(RegisterId::ER1, 0x1), (RegisterId::EAC, 0x0)],
                None,
                0,
                false,
                "MOD - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    ModU32RegU32Imm(RegisterId::ER1, 0x3),
                ],
                &[(RegisterId::ER1, 0x2), (RegisterId::EAC, 0x1)],
                None,
                0,
                false,
                "MOD - incorrect result value produced",
            ),
            TestU32::new(
                &[ModU32RegU32Imm(RegisterId::ER1, 0x0)],
                &[],
                None,
                0,
                true,
                "MOD - failed to panic when attempting to divide by zero",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the modulo of a u32 register by a u32 register instruction.
    #[test]
    fn test_mod_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32ImmU32Reg(0x2, RegisterId::ER2),
                    ModU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::ER2, 0x2),
                    (RegisterId::EAC, 0x0),
                ],
                None,
                0,
                false,
                "MOD - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    MovU32ImmU32Reg(0x3, RegisterId::ER2),
                    ModU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0x2),
                    (RegisterId::ER2, 0x3),
                    (RegisterId::EAC, 0x1),
                ],
                None,
                0,
                false,
                "MOD - incorrect result value produced",
            ),
            TestU32::new(
                &[ModU32RegU32Reg(RegisterId::ER1, RegisterId::ER2)],
                &[],
                None,
                0,
                true,
                "MOD - failed to panic when attempting to divide by zero",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the increment u32 register instruction.
    #[test]
    fn test_inc_u32_reg() {
        let tests = [
            TestU32::new(
                &[IncU32Reg(RegisterId::ER1)],
                &[(RegisterId::ER1, 0x1)],
                None,
                0,
                false,
                "INC - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::ER1),
                    IncU32Reg(RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[
                            CpuFlag::ZF,
                            CpuFlag::OF,
                            CpuFlag::PF,
                            CpuFlag::IF,
                        ]),
                    ),
                ],
                None,
                0,
                false,
                "INC - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    IncU32Reg(RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                0,
                false,
                "INC - CPU parity flag not correctly set",
            ),
            // Test the parity flag get cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    IncU32Reg(RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1), (RegisterId::EFL, 0x0)],
                None,
                0,
                false,
                "INC - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    IncU32Reg(RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                None,
                0,
                false,
                "INC - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    IncU32Reg(RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1), (RegisterId::EFL, 0x0)],
                None,
                0,
                false,
                "INC - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the decrement u32 register instruction.
    #[test]
    fn test_dec_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    DecU32Reg(RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x0),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "DEC - incorrect result value produced",
            ),
            TestU32::new(
                &[DecU32Reg(RegisterId::ER1)],
                &[
                    (RegisterId::ER1, u32::MAX),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[
                            CpuFlag::SF,
                            CpuFlag::OF,
                            CpuFlag::PF,
                            CpuFlag::IF,
                        ]),
                    ),
                ],
                None,
                0,
                false,
                "DEC - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x4, RegisterId::ER1),
                    DecU32Reg(RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                0,
                false,
                "DEC - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x3, RegisterId::ER1),
                    DecU32Reg(RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x2), (RegisterId::EFL, 0x0)],
                None,
                0,
                false,
                "DEC - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    DecU32Reg(RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::SF])),
                ],
                None,
                0,
                false,
                "DEC - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    DecU32Reg(RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1), (RegisterId::EFL, 0x0)],
                None,
                0,
                false,
                "DEC - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the bitwise and of a u32 immediate and a u32 register instruction.
    #[test]
    fn test_bitwise_and_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x3, RegisterId::ER1),
                    AndU32ImmU32Reg(0x2, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x3), (RegisterId::EAC, 0x2)],
                None,
                0,
                false,
                "AND - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    AndU32ImmU32Reg(0x3, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x2), (RegisterId::EAC, 0x2)],
                None,
                0,
                false,
                "AND - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    AndU32ImmU32Reg(0x0, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x2),
                    (RegisterId::EAC, 0x0),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "AND - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::ER1),
                    AndU32ImmU32Reg(0x3, RegisterId::ER1),
                ],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                )],
                None,
                0,
                false,
                "AND - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x3, RegisterId::ER1),
                    AndU32ImmU32Reg(0x3, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x3),
                    (RegisterId::EAC, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                0,
                false,
                "AND - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x3, RegisterId::ER1),
                    AndU32ImmU32Reg(0x2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x3),
                    (RegisterId::EAC, 0x2),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "AND - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    AndU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EAC, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                None,
                0,
                false,
                "AND - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    AndU32ImmU32Reg(0x2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x2),
                    (RegisterId::EAC, 0x2),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "AND - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the left-shift u32 register by u32 immediate value.
    #[test]
    fn test_left_shift_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    LeftShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x2)],
                None,
                0,
                false,
                "SHL - incorrect result value produced",
            ),
            // When shifted left by three places, this value will set the carry flag but not the
            // overflow flag. The overflow flag is only set when computing 1-bit shifts.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1011_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    LeftShiftU32ImmU32Reg(0x3, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1000),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::CF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "SHL - CPU flags not correctly set",
            ),
            // When shifted left by one place. This will set the carry and overflow flags.
            // The overflow flag is only set when computing 1-bit shifts.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    LeftShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[
                            CpuFlag::SF,
                            CpuFlag::CF,
                            CpuFlag::OF,
                            CpuFlag::IF,
                        ]),
                    ),
                ],
                None,
                0,
                false,
                "SHL - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::ER1),
                    LeftShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                )],
                None,
                0,
                false,
                "SHL - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::OF]), RegisterId::EFL),
                    // This will unset the overflow flag and set the zero flag instead.
                    MovU32ImmU32Reg(0x0, RegisterId::ER1),
                    LeftShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                None,
                0,
                false,
                "SHL - parity or zero flags are not set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestU32::new(
                &[LeftShiftU32ImmU32Reg(0x20, RegisterId::ER1)],
                &[],
                None,
                0,
                true,
                "SHL - successfully executed instruction with invalid shift value",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    LeftShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x2), (RegisterId::EFL, 0x0)],
                None,
                0,
                false,
                "SHL - CPU parity flag not correctly cleared",
            ),
            // Test that a left-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    LeftShiftU32ImmU32Reg(0x0, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1)],
                None,
                0,
                false,
                "SHL - zero left-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0100_0000_0000_0000_0000_0000_0000_0000, RegisterId::ER1),
                    LeftShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                None,
                0,
                false,
                "SHL - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    LeftShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x2), (RegisterId::EFL, 0x0)],
                None,
                0,
                false,
                "SHL - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the left-shift u32 register by u32 register value.
    #[test]
    fn test_left_shift_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    LeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x2), (RegisterId::ER2, 0x1)],
                None,
                0,
                false,
                "SHL - incorrect result value produced",
            ),
            // When shifted left by three places, this value will set the carry flag but not the
            // overflow flag. The overflow flag is only set when computing 1-bit shifts.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1011_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    MovU32ImmU32Reg(0x3, RegisterId::ER2),
                    LeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1000),
                    (RegisterId::ER2, 0x3),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::CF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "SHL - invalid CPU flag state",
            ),
            // When shifted left by one place. This will set the carry and overflow flags.
            // The overflow flag is only set when computing 1-bit shifts.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    LeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::ER2, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[
                            CpuFlag::SF,
                            CpuFlag::OF,
                            CpuFlag::CF,
                            CpuFlag::IF,
                        ]),
                    ),
                ],
                None,
                0,
                false,
                "SHL - CPU flags not correctly set",
            ),
            // When shifted left by two places, this value will set the overflow and carry flags.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    LeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::ER2, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[
                            CpuFlag::SF,
                            CpuFlag::OF,
                            CpuFlag::CF,
                            CpuFlag::IF,
                        ]),
                    ),
                ],
                None,
                0,
                false,
                "SHL - CPU flags not correctly set",
            ),
            // The zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    LeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER2, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "SHL - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::OF]), RegisterId::EFL),
                    // This will unset the overflow flag and set the zero flag instead.
                    MovU32ImmU32Reg(0x0, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    LeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER2, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                None,
                0,
                false,
                "SHL - CPU flags not correctly set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x20, RegisterId::ER2),
                    LeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[],
                None,
                0,
                true,
                "SHL - successfully executed instruction with invalid shift value",
            ),
            // Test that a left-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    LeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1)],
                None,
                0,
                false,
                "SHL - zero left-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0100_0000_0000_0000_0000_0000_0000_0000, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    LeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (RegisterId::ER2, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                None,
                0,
                false,
                "SHL - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    LeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x2),
                    (RegisterId::ER2, 0x1),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "SHL - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the arithmetic left-shift u32 register by u32 immediate value.
    #[test]
    fn test_arith_left_shift_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    ArithLeftShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x2)],
                None,
                0,
                false,
                "SAL - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1110, RegisterId::ER1),
                    ArithLeftShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1101),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "SAL - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::ER1),
                    ArithLeftShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                )],
                None,
                0,
                false,
                "SAL - CPU flags not correctly set",
            ),
            // Test that a left-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    ArithLeftShiftU32ImmU32Reg(0x0, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1)],
                None,
                0,
                false,
                "SAL - zero left-shift did not leave the result unchanged",
            ),
            // Test the signed flag is set.
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0100_0000_0000_0000_0000_0000_0000_0000, RegisterId::ER1),
                    ArithLeftShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                None,
                0,
                false,
                "SAL - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    ArithLeftShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x2), (RegisterId::EFL, 0x0)],
                None,
                0,
                false,
                "SAL - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the arithmetic left-shift u32 register by u32 register.
    #[test]
    fn test_arith_left_shift_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    ArithLeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x2), (RegisterId::ER2, 0x1)],
                None,
                0,
                false,
                "SAL - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1110, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    ArithLeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1101),
                    (RegisterId::ER2, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "SAL - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    ArithLeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER2, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "SAL - CPU flags not correctly set",
            ),
            // Test that a left-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    ArithLeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1)],
                None,
                0,
                false,
                "SAL - zero left-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0100_0000_0000_0000_0000_0000_0000_0000, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    ArithLeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (RegisterId::ER2, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                None,
                0,
                false,
                "SAL - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    ArithLeftShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x2),
                    (RegisterId::ER2, 0x1),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "SAL - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the right-shift u32 register by u32 immediate value.
    #[test]
    fn test_right_shift_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    RightShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1)],
                None,
                0,
                false,
                "SHR - incorrect result value produced",
            ),
            // The SHR command should clear the overflow flag but set the
            // carry flag since the bit being shifted out is one.
            TestU32::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::OF]), RegisterId::EFL),
                    // Execute the test instruction.
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    RightShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::PF]),
                    ),
                ],
                None,
                0,
                false,
                "SHR - carry or parity CPU flags were not correctly set",
            ),
            // The SHR command should clear the overflow flag and the
            // carry flag in this case as the bit being shifted out is zero.
            TestU32::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::OF]), RegisterId::EFL),
                    // Execute the test instruction.
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1110, RegisterId::ER1),
                    RightShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                0,
                false,
                "SHR - CPU flags were not correctly set",
            ),
            // The zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::ER1),
                    RightShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                )],
                None,
                0,
                false,
                "SHR - zero or parity CPU flags were not correctly set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestU32::new(
                &[RightShiftU32ImmU32Reg(0x20, RegisterId::ER1)],
                &[],
                None,
                0,
                true,
                "SHR - successfully executed instruction with invalid shift value",
            ),
            // Test that a right-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    RightShiftU32ImmU32Reg(0x0, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1)],
                None,
                0,
                false,
                "SHR - zero right-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    RightShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x0),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::CF, CpuFlag::PF]),
                    ),
                ],
                None,
                0,
                false,
                "SAL - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the right-shift u32 register by u32 register.
    #[test]
    fn test_right_shift_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    RightShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1), (RegisterId::ER2, 0x1)],
                None,
                0,
                false,
                "SHR - incorrect result value produced",
            ),
            // The SHR command should clear the overflow flag but set the
            // carry flag since the bit being shifted out is one.
            TestU32::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::OF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    // Execute the test instruction.
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    RightShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::PF]),
                    ),
                ],
                None,
                0,
                false,
                "SHR - carry or parity CPU flags were not correctly set",
            ),
            // The SHR command should clear the overflow flag and the
            // carry flag in this case as the bit being shifted out is zero.
            TestU32::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::OF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    // Execute the test instruction.
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1110, RegisterId::ER1),
                    RightShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0x1),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                None,
                0,
                false,
                "SHR - CPU flags were not correctly set",
            ),
            // The zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    RightShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER2, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "SHR - zero or parity CPU flags were not correctly set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x20, RegisterId::ER2),
                    RightShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[],
                None,
                0,
                true,
                "SHR - successfully executed instruction with invalid shift value",
            ),
            // Test that a right-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    RightShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1)],
                None,
                0,
                false,
                "SHR - zero right-shift did not leave the result unchanged",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the arithmetic right-shift u32 register by u32 immediate value.
    #[test]
    fn test_arith_right_shift_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    ArithRightShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1)],
                None,
                0,
                false,
                "SAR - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    ArithRightShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1011_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "SAR - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero right-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::ER1),
                    ArithRightShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                )],
                None,
                0,
                false,
                "SAR - zero or parity CPU flags were not correctly set",
            ),
            // Test that a right-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    ArithRightShiftU32ImmU32Reg(0x0, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1)],
                None,
                0,
                false,
                "SAR - zero right-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0000_0000_0000_0000_0000_0000_0000_0001, RegisterId::ER1),
                    ArithRightShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                None,
                0,
                false,
                "SAR - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    ArithRightShiftU32ImmU32Reg(0x1, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1), (RegisterId::EFL, 0x0)],
                None,
                0,
                false,
                "SAR - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the arithmetic right-shift u32 register by u32 register.
    #[test]
    fn test_arith_right_shift_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    ArithRightShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1), (RegisterId::ER2, 0x1)],
                None,
                0,
                false,
                "SAR - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    ArithRightShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "SAR - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    ArithRightShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER2, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "SAR - zero or parity CPU flags were not correctly set",
            ),
            // Test that a right-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    ArithRightShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x1)],
                None,
                0,
                false,
                "SAR - zero right-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0000_0000_0000_0000_0000_0000_0000_0001, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    ArithRightShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (RegisterId::ER2, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                None,
                0,
                false,
                "SAR - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1, RegisterId::ER2),
                    ArithRightShiftU32RegU32Reg(RegisterId::ER2, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0x1),
                    (RegisterId::ER2, 0x1),
                    (RegisterId::EFL, 0x0),
                ],
                None,
                0,
                false,
                "SAR - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the move u32 immediate to u32 register instruction.
    #[test]
    fn test_move_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[MovU32ImmU32Reg(0x1, RegisterId::ER1)],
                &[(RegisterId::ER1, 0x1)],
                None,
                0,
                false,
                "MOV - invalid value moved to register",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x2)],
                None,
                0,
                false,
                "MOV - invalid value moved to register",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the move u32 register to u32 register instruction.
    #[test]
    fn test_move_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[(RegisterId::ER1, 0x1), (RegisterId::ER2, 0x1)],
                None,
                0,
                false,
                "MOV - immediate value not moved to register",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::ER1),
                    MovU32ImmU32Reg(0x2, RegisterId::ER2),
                    MovU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[(RegisterId::ER1, 0x1), (RegisterId::ER2, 0x1)],
                None,
                0,
                false,
                "MOV - immediate value not moved to register",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the move u32 literal to memory instruction.
    #[test]
    fn test_move_u32_lit_mem() {
        let tests = [TestU32::new(
            &[MovU32ImmMemSimple(0x123, 0x0)],
            &[],
            Some(vec![
                35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ]),
            0,
            false,
            "MOV - immediate value not moved to memory",
        )];

        TestsU32::new(&tests).run_all();
    }

    /// Test the move u32 register to memory instruction.
    #[test]
    fn test_move_u32_reg_mem() {
        let tests = [TestU32::new(
            &[
                MovU32ImmU32Reg(0x123, RegisterId::ER1),
                MovU32RegMemSimple(RegisterId::ER1, 0x0),
            ],
            &[(RegisterId::ER1, 0x123)],
            Some(vec![
                35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ]),
            0,
            false,
            "MOV - u32 register value not moved to memory",
        )];

        TestsU32::new(&tests).run_all();
    }

    /// Test the move u32 register to memory instruction.
    #[test]
    fn test_move_mem_u32_reg() {
        let tests = [TestU32::new(
            &[
                // Move the value to memory.
                MovU32ImmU32Reg(0x123, RegisterId::ER1),
                MovU32RegMemSimple(RegisterId::ER1, 0x0),
                // Move the value from memory to a register.
                MovMemU32RegSimple(0x0, RegisterId::ER2),
            ],
            &[(RegisterId::ER1, 0x123), (RegisterId::ER2, 0x123)],
            Some(vec![
                35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ]),
            0,
            false,
            "MOV - value not correctly moved from memory to register",
        )];

        TestsU32::new(&tests).run_all();
    }

    /// Test the move u32 to register from address provided by a different register.
    #[test]
    fn test_move_u32_reg_ptr_u32_reg() {
        let tests = [TestU32::new(
            &[
                // Store value in memory.
                MovU32ImmU32Reg(0x123, RegisterId::ER1),
                MovU32RegMemSimple(RegisterId::ER1, 0x0),
                // Set the address pointer in R2.
                MovU32ImmU32Reg(0x0, RegisterId::ER2),
                // Read the value from the address of R2 into R1.
                MovU32RegPtrU32RegSimple(RegisterId::ER2, RegisterId::ER3),
            ],
            &[(RegisterId::ER1, 0x123), (RegisterId::ER3, 0x123)],
            Some(vec![
                35, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ]),
            0,
            false,
            "MOV - value not correctly moved from memory to register via register pointer",
        )];

        TestsU32::new(&tests).run_all();
    }

    /// Test the swap u32 register to u32 register instruction.
    #[test]
    fn test_swap_u32_reg_u32_reg() {
        let tests = [TestU32::new(
            &[
                MovU32ImmU32Reg(0x1, RegisterId::ER1),
                MovU32ImmU32Reg(0x2, RegisterId::ER2),
                SwapU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
            ],
            &[(RegisterId::ER1, 0x2), (RegisterId::ER2, 0x1)],
            None,
            0,
            false,
            "SWAP - values of the two registers were not correctly swapped",
        )];

        TestsU32::new(&tests).run_all();
    }

    /// Test the byte swap u32 register instruction.
    #[test]
    fn test_byte_swap_u32_reg() {
        let tests = [TestU32::new(
            &[
                MovU32ImmU32Reg(0xAABBCCDD, RegisterId::ER1),
                ByteSwapU32(RegisterId::ER1),
            ],
            &[(RegisterId::ER1, 0xDDCCBBAA)],
            None,
            0,
            false,
            "BSWAP - the byte order of the register was not correctly swapped",
        )];

        TestsU32::new(&tests).run_all();
    }

    /// Test the complex move value to expression-derived memory address.
    #[test]
    fn test_move_u32_imm_expr() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemExpr(
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
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x8, RegisterId::ER1),
                    MovU32ImmU32Reg(0x8, RegisterId::ER2),
                    MovU32ImmMemExpr(
                        0x123,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::ER1),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Register(RegisterId::ER2),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[(RegisterId::ER1, 0x8), (RegisterId::ER2, 0x8)],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register with expression - two registers",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x8, RegisterId::ER1),
                    MovU32ImmMemExpr(
                        0x123,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::ER1),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Immediate(0x8),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[(RegisterId::ER1, 0x8)],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register with expression - one constant and one register",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    MovU32ImmU32Reg(0x8, RegisterId::ER2),
                    MovU32ImmMemExpr(
                        0x123,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::ER1),
                                ExpressionArgs::Operator(ExpressionOperator::Multiply),
                                ExpressionArgs::Register(RegisterId::ER2),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[(RegisterId::ER1, 0x2), (RegisterId::ER2, 0x8)],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register - using multiplication",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1a, RegisterId::ER1),
                    MovU32ImmU32Reg(0x4, RegisterId::ER2),
                    MovU32ImmMemExpr(
                        0x123,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::ER1),
                                ExpressionArgs::Operator(ExpressionOperator::Subtract),
                                ExpressionArgs::Register(RegisterId::ER2),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[(RegisterId::ER1, 0x1A), (RegisterId::ER2, 0x4)],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register - using subtraction",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemExpr(
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
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register with expression - three constants",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the complex move from an expression-derived memory address to a register.
    #[test]
    fn test_move_mem_expr_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0x123, 0x10),
                    MovMemExprU32Reg(
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Immediate(0x8),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Immediate(0x8),
                            ][..],
                        )
                        .pack(),
                        RegisterId::ER8,
                    ),
                ],
                &[(RegisterId::ER8, 0x123)],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0x123, 0x10),
                    MovU32ImmU32Reg(0x8, RegisterId::ER1),
                    MovU32ImmU32Reg(0x8, RegisterId::ER2),
                    MovMemExprU32Reg(
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::ER1),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Register(RegisterId::ER2),
                            ][..],
                        )
                        .pack(),
                        RegisterId::ER8,
                    ),
                ],
                &[
                    (RegisterId::ER1, 0x8),
                    (RegisterId::ER2, 0x8),
                    (RegisterId::ER8, 0x123),
                ],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register with expression - two registers",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0x123, 0x10),
                    MovU32ImmU32Reg(0x8, RegisterId::ER1),
                    MovMemExprU32Reg(
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::ER1),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Immediate(0x8),
                            ][..],
                        )
                        .pack(),
                        RegisterId::ER8,
                    ),
                ],
                &[(RegisterId::ER1, 0x8), (RegisterId::ER8, 0x123)],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register with expression - one constant and one register",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0x123, 0x10),
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    MovU32ImmU32Reg(0x8, RegisterId::ER2),
                    MovMemExprU32Reg(
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::ER1),
                                ExpressionArgs::Operator(ExpressionOperator::Multiply),
                                ExpressionArgs::Register(RegisterId::ER2),
                            ][..],
                        )
                        .pack(),
                        RegisterId::ER8,
                    ),
                ],
                &[
                    (RegisterId::ER1, 0x2),
                    (RegisterId::ER2, 0x8),
                    (RegisterId::ER8, 0x123),
                ],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register - using multiplication",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0x123, 0x16),
                    MovU32ImmU32Reg(0x1A, RegisterId::ER1),
                    MovU32ImmU32Reg(0x4, RegisterId::ER2),
                    MovMemExprU32Reg(
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::ER1),
                                ExpressionArgs::Operator(ExpressionOperator::Subtract),
                                ExpressionArgs::Register(RegisterId::ER2),
                            ][..],
                        )
                        .pack(),
                        RegisterId::ER8,
                    ),
                ],
                &[
                    (RegisterId::ER1, 0x1a),
                    (RegisterId::ER2, 0x4),
                    (RegisterId::ER8, 0x123),
                ],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register - using subtraction",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0x123, 0x10),
                    MovMemExprU32Reg(
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
                        RegisterId::ER8,
                    ),
                ],
                &[(RegisterId::ER8, 0x123)],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the complex move from a register to an expression-derived memory address.
    #[test]
    fn test_move_u32_reg_mem_expr() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x123, RegisterId::ER8),
                    MovU32RegMemExpr(
                        RegisterId::ER8,
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
                &[(RegisterId::ER8, 0x123)],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
            // Test with two register values.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x8, RegisterId::ER1),
                    MovU32ImmU32Reg(0x8, RegisterId::ER2),
                    MovU32ImmU32Reg(0x123, RegisterId::ER8),
                    MovU32RegMemExpr(
                        RegisterId::ER8,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::ER1),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Register(RegisterId::ER2),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[
                    (RegisterId::ER1, 0x8),
                    (RegisterId::ER2, 0x8),
                    (RegisterId::ER8, 0x123),
                ],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register with expression - two registers",
            ),
            // Test with a constant and a register.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x8, RegisterId::ER1),
                    MovU32ImmU32Reg(0x123, RegisterId::ER8),
                    MovU32RegMemExpr(
                        RegisterId::ER8,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::ER1),
                                ExpressionArgs::Operator(ExpressionOperator::Add),
                                ExpressionArgs::Immediate(0x8),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[(RegisterId::ER1, 0x8), (RegisterId::ER8, 0x123)],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register with expression - one constant and one register",
            ),
            // Test with multiplication.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::ER1),
                    MovU32ImmU32Reg(0x8, RegisterId::ER2),
                    MovU32ImmU32Reg(0x123, RegisterId::ER8),
                    MovU32RegMemExpr(
                        RegisterId::ER8,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::ER1),
                                ExpressionArgs::Operator(ExpressionOperator::Multiply),
                                ExpressionArgs::Register(RegisterId::ER2),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[
                    (RegisterId::ER1, 0x2),
                    (RegisterId::ER2, 0x8),
                    (RegisterId::ER8, 0x123),
                ],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register - using multiplication",
            ),
            // Test with subtraction.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1A, RegisterId::ER1),
                    MovU32ImmU32Reg(0x4, RegisterId::ER2),
                    MovU32ImmU32Reg(0x123, RegisterId::ER8),
                    MovU32RegMemExpr(
                        RegisterId::ER8,
                        MoveExpressionHandler::from(
                            &[
                                ExpressionArgs::Register(RegisterId::ER1),
                                ExpressionArgs::Operator(ExpressionOperator::Subtract),
                                ExpressionArgs::Register(RegisterId::ER2),
                            ][..],
                        )
                        .pack(),
                    ),
                ],
                &[
                    (RegisterId::ER1, 0x1a),
                    (RegisterId::ER2, 0x4),
                    (RegisterId::ER8, 0x123),
                ],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register - using subtraction",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x123, RegisterId::ER8),
                    MovU32RegMemExpr(
                        RegisterId::ER8,
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
                &[(RegisterId::ER8, 0x123)],
                Some(vec![
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 35, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the zero high bit by index instruction.
    #[test]
    fn test_zhbi_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    MovU32ImmU32Reg(0x4, RegisterId::ER2),
                    ZeroHighBitsByIndexU32Reg(RegisterId::ER2, RegisterId::ER1, RegisterId::ER3),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0x4),
                    (RegisterId::ER3, 0b0000_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    MovU32ImmU32Reg(0x18, RegisterId::ER2),
                    ZeroHighBitsByIndexU32Reg(RegisterId::ER2, RegisterId::ER1, RegisterId::ER3),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0x18),
                    (RegisterId::ER3, 0b0000_0000_0000_0000_0000_0000_1111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    MovU32ImmU32Reg(0x19, RegisterId::ER2),
                    ZeroHighBitsByIndexU32Reg(RegisterId::ER2, RegisterId::ER1, RegisterId::ER3),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0x19),
                    (RegisterId::ER3, 0b0000_0000_0000_0000_0000_0000_0111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    MovU32ImmU32Reg(0x1b, RegisterId::ER2),
                    ZeroHighBitsByIndexU32Reg(RegisterId::ER2, RegisterId::ER1, RegisterId::ER3),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0x1b),
                    (RegisterId::ER3, 0b0000_0000_0000_0000_0000_0000_0001_1111),
                ],
                None,
                0,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::ER1),
                    MovU32ImmU32Reg(0x0, RegisterId::ER2),
                    ZeroHighBitsByIndexU32Reg(RegisterId::ER2, RegisterId::ER1, RegisterId::ER3),
                ],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::IF]),
                )],
                None,
                0,
                false,
                "ZHBI - incorrect flag state - zero flag not set",
            ),
            TestU32::new(
                &[
                    // Manually set the overflow flag state.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::OF]), RegisterId::EFL),
                    ZeroHighBitsByIndexU32Reg(
                        RegisterId::ER1, // Already has the value of 0.
                        RegisterId::ER2, // Already has the value of 0.
                        RegisterId::ER3,
                    ),
                ],
                &[(RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF]))],
                None,
                0,
                false,
                "ZHBI - incorrect flag state - overflow flag not cleared",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x20, RegisterId::ER2),
                    ZeroHighBitsByIndexU32Reg(
                        RegisterId::ER2,
                        RegisterId::ER1, // Already has the value of 0.
                        RegisterId::ER3,
                    ),
                ],
                &[],
                None,
                0,
                true,
                "ZHBI - successfully executed instruction with invalid bit index",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER3),
                    ZeroHighBitsByIndexU32Reg(
                        RegisterId::ER2, // Already has the value of 0.
                        RegisterId::ER1,
                        RegisterId::ER3,
                    ),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER3, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::CF]),
                    ),
                ],
                None,
                0,
                false,
                "ZHBI - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER3),
                    ZeroHighBitsByIndexU32Reg(
                        RegisterId::ER2, // Already has the value of 0.
                        RegisterId::ER1,
                        RegisterId::ER3,
                    ),
                ],
                &[
                    (RegisterId::ER1, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER3, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF])),
                ],
                None,
                0,
                false,
                "ZHBI - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the zero high bit by index instruction.
    #[test]
    fn test_zhbi_u32_reg_u32_imm() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    ZeroHighBitsByIndexU32RegU32Imm(0x4, RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0b0000_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    ZeroHighBitsByIndexU32RegU32Imm(0x18, RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0b0000_0000_0000_0000_0000_0000_1111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    ZeroHighBitsByIndexU32RegU32Imm(0x19, RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0b0000_0000_0000_0000_0000_0000_0111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    ZeroHighBitsByIndexU32RegU32Imm(0x1b, RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0b0000_0000_0000_0000_0000_0000_0001_1111),
                ],
                None,
                0,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[ZeroHighBitsByIndexU32RegU32Imm(
                    0x0,
                    RegisterId::ER1, // Already has the value of 0.
                    RegisterId::ER2,
                )],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::IF]),
                )],
                None,
                0,
                false,
                "ZHBI - incorrect flag state - zero flag not set",
            ),
            TestU32::new(
                &[
                    // Manually set the overflow flag state.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::OF]), RegisterId::EFL),
                    ZeroHighBitsByIndexU32RegU32Imm(
                        0x0,
                        RegisterId::ER1,
                        RegisterId::ER2, // Already has the value of 0.
                    ),
                ],
                &[(RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF]))],
                None,
                0,
                false,
                "ZHBI - incorrect flag state - overflow flag not cleared",
            ),
            TestU32::new(
                &[ZeroHighBitsByIndexU32RegU32Imm(
                    0x20,
                    RegisterId::ER1,
                    RegisterId::ER2,
                )],
                &[],
                None,
                0,
                true,
                "ZHBI - successfully executed instruction with invalid bit index",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER2),
                    ZeroHighBitsByIndexU32RegU32Imm(0x0, RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::CF]),
                    ),
                ],
                None,
                0,
                false,
                "ZHBI - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER2),
                    ZeroHighBitsByIndexU32RegU32Imm(0x0, RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF])),
                ],
                None,
                0,
                false,
                "ZHBI - CPU signed flag not correctly cleared",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the bit-test instruction with a u32 register.
    #[test]
    fn test_bit_test_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    BitTestU32Reg(0x0, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1110, RegisterId::ER1),
                    BitTestU32Reg(0x0, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1110)],
                None,
                0,
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestU32::new(
                &[BitTestU32Reg(0x20, RegisterId::ER1)],
                &[],
                None,
                0,
                true,
                "BT - successfully executed instruction with invalid bit index",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the bit-test instruction with a memory address.
    #[test]
    fn test_bit_test_mem() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitTestU32Mem(0x0, 0x0),
                ],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::IF]),
                )],
                Some(vec![
                    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_1110, 0x0),
                    BitTestU32Mem(0x0, 0x0),
                ],
                &[],
                Some(vec![
                    254, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestU32::new(
                &[BitTestU32Mem(0x20, 0x0)],
                &[],
                None,
                0,
                true,
                "BT - successfully executed instruction with invalid bit index",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the bit-test and reset instruction with a u32 register.
    #[test]
    fn test_bit_test_reset_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::ER1,
                    ),
                    BitTestResetU32Reg(0x0, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::IF])),
                ],
                None,
                0,
                false,
                "BTR - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        RegisterId::ER1,
                    ),
                    BitTestResetU32Reg(0x0, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1110)],
                None,
                0,
                false,
                "BTR - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestU32::new(
                &[
                    BitTestResetU32Reg(0x20, RegisterId::ER1),
                ],
                &[],
                None,
                0,
                true,
                "BTR - successfully executed instruction with invalid bit index",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the bit-test and reset instruction with a memory address.
    #[test]
    fn test_bit_test_reset_mem() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        0x0,
                    ),
                    BitTestResetU32Mem(0x0, 0x0),
                ],
                &[(RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::IF]))],
                Some(vec![
                    254, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BTR - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        0x0,
                    ),
                    BitTestResetU32Mem(0x0, 0x0),
                ],
                &[],
                Some(vec![
                    254, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BTR - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestU32::new(
                &[BitTestResetU32Mem(0x20, 0x0)],
                &[],
                None,
                0,
                true,
                "BTR - successfully executed instruction with invalid bit index",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the bit-test and set instruction with a u32 register.
    #[test]
    fn test_bit_test_set_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::ER1,
                    ),
                    BitTestSetU32Reg(0x0, RegisterId::ER1),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::IF])),
                ],
                None,
                0,
                false,
                "BTS - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        RegisterId::ER1,
                    ),
                    BitTestSetU32Reg(0x0, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                None,
                0,
                false,
                "BTS - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestU32::new(
                &[BitTestSetU32Reg(0x20, RegisterId::ER1)],
                &[],
                None,
                0,
                true,
                "BTS - successfully executed instruction with invalid bit index",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the bit-test and set instruction with a memory address.
    #[test]
    fn test_bit_test_set_mem() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        0x0,
                    ),
                    BitTestSetU32Mem(0x0, 0x0),
                ],
                &[(RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::IF]))],
                Some(vec![
                    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BTS - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        0x0,
                    ),
                    BitTestSetU32Mem(0x0, 0x0),
                ],
                &[],
                Some(vec![
                    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BTS - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestU32::new(
                &[BitTestSetU32Mem(0x20, 0x0)],
                &[],
                None,
                0,
                true,
                "BTS - successfully executed instruction with invalid bit index",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the reverse bit scan with a source u32 register and a destination u32 register.
    #[test]
    fn test_bit_scan_reverse_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    BitScanReverseU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[(RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                None,
                0,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b0000_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    BitScanReverseU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0b0000_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::ER2, 0x4),
                ],
                None,
                0,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::ER1),
                    BitScanReverseU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER2, 0x20),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the reverse bit scan with a source memory address and a destination u32 register.
    #[test]
    fn test_bit_scan_reverse_mem_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitScanReverseU32MemU32Reg(0x0, RegisterId::ER1),
                ],
                &[],
                Some(vec![
                    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b0000_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitScanReverseU32MemU32Reg(0x0, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x4)],
                Some(vec![
                    255, 255, 255, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[BitScanReverseU32MemU32Reg(0x0, RegisterId::ER1)],
                &[
                    (RegisterId::ER1, 0x20),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the reverse bit scan with a register and a destination memory address.
    #[test]
    fn test_bit_scan_reverse_u32_reg_mem() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    BitScanReverseU32RegMemU32(RegisterId::ER1, 0x0),
                ],
                &[(RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                None,
                0,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b0000_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    BitScanReverseU32RegMemU32(RegisterId::ER1, 0x0),
                ],
                &[(RegisterId::ER1, 0b0000_1111_1111_1111_1111_1111_1111_1111)],
                Some(vec![
                    4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[BitScanReverseU32RegMemU32(RegisterId::ER1, 0x0)],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::IF]),
                )],
                Some(vec![
                    32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the reverse bit scan with a source memory address and a destination memory address.
    #[test]
    fn test_bit_scan_reverse_mem_mem() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitScanReverseU32MemU32Mem(0x0, 0x4),
                ],
                &[],
                Some(vec![
                    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b0000_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitScanReverseU32MemU32Mem(0x0, 0x4),
                ],
                &[],
                Some(vec![
                    255, 255, 255, 15, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[BitScanReverseU32MemU32Mem(0x0, 0x4)],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::IF]),
                )],
                Some(vec![
                    0, 0, 0, 0, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the forward bit scan with a source u32 register and a destination u32 register.
    #[test]
    fn test_bit_scan_forward_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    BitScanForwardU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[(RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                None,
                0,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_0000, RegisterId::ER1),
                    BitScanForwardU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_0000),
                    (RegisterId::ER2, 0x4),
                ],
                None,
                0,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::ER1),
                    BitScanForwardU32RegU32Reg(RegisterId::ER1, RegisterId::ER2),
                ],
                &[
                    (RegisterId::ER2, 0x20),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the forward bit scan with a source memory address and a destination u32 register.
    #[test]
    fn test_bit_scan_forward_mem_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitScanForwardU32MemU32Reg(0x0, RegisterId::ER1),
                ],
                &[],
                Some(vec![
                    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_0000, 0x0),
                    BitScanForwardU32MemU32Reg(0x0, RegisterId::ER1),
                ],
                &[(RegisterId::ER1, 0x4)],
                Some(vec![
                    240, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[BitScanForwardU32MemU32Reg(0x0, RegisterId::ER1)],
                &[
                    (RegisterId::ER1, 0x20),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::IF]),
                    ),
                ],
                None,
                0,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the forward bit scan with a register and a destination memory address.
    #[test]
    fn test_bit_scan_forward_u32_reg_mem() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::ER1),
                    BitScanForwardU32RegMemU32(RegisterId::ER1, 0x0),
                ],
                &[(RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                None,
                0,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_0000, RegisterId::ER1),
                    BitScanForwardU32RegMemU32(RegisterId::ER1, 0x0),
                ],
                &[(RegisterId::ER1, 0b1111_1111_1111_1111_1111_1111_1111_0000)],
                Some(vec![
                    4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[BitScanForwardU32RegMemU32(RegisterId::ER1, 0x0)],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::IF]),
                )],
                Some(vec![
                    32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the forward bit scan with a source memory address and a destination memory address.
    #[test]
    fn test_bit_scan_forward_mem_mem() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitScanForwardU32MemU32Mem(0x0, 0x4),
                ],
                &[],
                Some(vec![
                    255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_0000, 0x0),
                    BitScanForwardU32MemU32Mem(0x0, 0x4),
                ],
                &[],
                Some(vec![
                    240, 255, 255, 255, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[BitScanForwardU32MemU32Mem(0x0, 0x4)],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::IF]),
                )],
                Some(vec![
                    0, 0, 0, 0, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                ]),
                0,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the push u32 immediate instruction.
    #[test]
    fn test_push_u32_imm_multiple() {
        let tests = [TestU32::new(
            &[PushU32Imm(0x123), PushU32Imm(0x321)],
            &[],
            None,
            2,
            false,
            "PUSH - failed to execute PUSH instruction",
        )];

        TestsU32::new(&tests).run_all_special(|_id: usize, vm: Option<VirtualMachine>| {
            let mut vm = vm.expect("failed to correctly execute test code");
            assert_eq!(vm.mem.pop_u32(), 0x321);
            assert_eq!(vm.mem.pop_u32(), 0x123);
        });
    }

    /// Test the push u32 immediate instruction.
    #[test]
    fn test_push_u32_imm_single() {
        let tests = [TestU32::new(
            &[PushU32Imm(0x123)],
            &[],
            None,
            1,
            false,
            "PUSH - failed to execute PUSH instruction",
        )];

        TestsU32::new(&tests).run_all_special(|_id: usize, vm: Option<VirtualMachine>| {
            let mut vm = vm.expect("failed to correctly execute test code");
            assert_eq!(vm.mem.pop_u32(), 0x123);
        });
    }

    /// Test the push u32 immediate instruction.
    #[test]
    #[should_panic]
    fn test_push_u32_imm_invalid_pop() {
        let tests = [TestU32::new(
            &[Nop],
            &[],
            None,
            2,
            false,
            "PUSH - failed to execute PUSH instruction",
        )];

        TestsU32::new(&tests).run_all_special(|_id: usize, vm: Option<VirtualMachine>| {
            let mut vm = vm.expect("failed to correctly execute test code");

            // There is nothing on the stack, this should assert.
            _ = vm.mem.pop_u32();
        });
    }

    /// Test the pop to u32 register instruction.
    #[test]
    fn test_pop_u32_reg() {
        let tests = [
            TestU32::new(
                &[PushU32Imm(0x123), PopU32ImmU32Reg(RegisterId::ER1)],
                &[(RegisterId::ER1, 0x123)],
                None,
                2,
                false,
                "POP - failed to execute POP instruction",
            ),
            TestU32::new(
                &[
                    PushU32Imm(0x123),
                    PushU32Imm(0x321),
                    PopU32ImmU32Reg(RegisterId::ER1),
                    PopU32ImmU32Reg(RegisterId::ER2),
                ],
                &[(RegisterId::ER1, 0x321), (RegisterId::ER1, 0x321)],
                None,
                2,
                false,
                "POP - failed to execute POP instruction",
            ),
            TestU32::new(
                &[PopU32ImmU32Reg(RegisterId::ER1)],
                &[],
                None,
                2,
                true,
                "POP - failed to execute POP instruction",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the absolute jump by immediate address instruction.
    #[test]
    fn test_jump_absolute_addr() {
        let tests = [
            TestU32::new(
                &[
                    // The address is calculated as follows:
                    //
                    // The start of the code segment is at byte index 100 since, by default,
                    // the testing user memory segment is 100 bytes in length.
                    //
                    // The jump instruction is _8 bytes_ in length
                    //      (4 for the instruction and 4 for the u32 argument).
                    // The first move instruction is _9 bytes_ in length
                    //      (4 for the instruction, 4 for the u32 argument and 1 for the register ID argument).
                    //
                    // This means that we need to move to index 100 + 8 + 9 = 117 to get to the
                    // start of the second move instruction.
                    //
                    // We expect that the first move instruction will be skipped entirely.
                    JumpAbsU32Imm(117, 0),
                    // This instruction should be skipped, so R1 should remain at the default value of 0.
                    MovU32ImmU32Reg(0xf, RegisterId::ER1),
                    // The jump should start execution here.
                    MovU32ImmU32Reg(0xa, RegisterId::ER2),
                ],
                &[(RegisterId::ER1, 0x0), (RegisterId::ER2, 0xa)],
                None,
                0,
                false,
                "JMP - failed to execute JMP instruction",
            ),
            TestU32::new(
                &[JumpAbsU32Imm(u32::MAX, 0)],
                &[],
                None,
                0,
                true,
                "JMP - successfully jumped outside of valid memory bounds.",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }

    /// Test the absolute jump by immediate address instruction.
    #[test]
    fn test_jump_absolute_u32_reg_ptr() {
        let tests = [
            TestU32::new(
                &[
                    // The address is calculated as follows:
                    // Move the value of the code segment register into R8.
                    //
                    // The instruction 2 is _9 bytes_ in length
                    //      (4 for the instruction, 4 for the u32 immediate argument and 1 for the register ID argument).
                    // The instruction 3 is _5 bytes_ in length
                    //      (4 for the instruction and 1 for the register ID argument).
                    // The instruction 4 is _9 bytes_ in length
                    //      (4 for the instruction, 4 for the u32 argument and 1 for the register ID argument).
                    //
                    // This means that we need to add 23 to the value of the CS register to point
                    // to the start of the second move instruction.
                    //
                    // We expect that the first move instruction will be skipped entirely.
                    AddU32ImmU32Reg(23, RegisterId::ECS),
                    JumpAbsU32Reg(RegisterId::EAC),
                    // This instruction should be skipped, so R1 should remain at the default value of 0.
                    MovU32ImmU32Reg(0xf, RegisterId::ER1),
                    // The jump should start execution here.
                    MovU32ImmU32Reg(0xa, RegisterId::ER2),
                ],
                &[(RegisterId::ER1, 0x0), (RegisterId::ER2, 0xa)],
                None,
                0,
                false,
                "JMP - failed to execute JMP instruction",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::ER1),
                    JumpAbsU32Reg(RegisterId::ER1),
                ],
                &[],
                None,
                0,
                true,
                "JMP - successfully jumped outside of valid memory bounds.",
            ),
        ];

        TestsU32::new(&tests).run_all();
    }
}
