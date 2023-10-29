use core::fmt;
use std::{collections::BTreeMap, panic, slice::Iter};

use crate::{
    ins::{
        instruction::Instruction,
        move_expressions::{ExpressionArgs, ExpressionOperator, MoveExpressionHandler},
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

    move_expression_cache: BTreeMap<u32, [ExpressionArgs; 3]>,
}

impl Cpu {
    pub fn new() -> Self {
        Self {
            registers: Registers::default(),
            is_halted: false,
            is_machine_mode: true,

            move_expression_cache: BTreeMap::new(),
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
            ExpressionArgs::Register(rid) => self.read_reg_u32(&rid, privilege),
            ExpressionArgs::Constant(val) => val as u32,
            _ => panic!(),
        };

        // Determine the second operand.
        let val_2 = match operators[2] {
            ExpressionArgs::Register(rid) => self.read_reg_u32(&rid, privilege),
            ExpressionArgs::Constant(val) => val as u32,
            _ => panic!(),
        };

        // Calculate the destination address by evaluating the expression.
        if let ExpressionArgs::Operator(op) = operators[1] {
            match op {
                ExpressionOperator::Add => val_1 + val_2,
                ExpressionOperator::Subtract => val_1 - val_2,
                ExpressionOperator::Multiply => val_1 * val_2,
                ExpressionOperator::Divide => panic!("operation not supported in this instance"),
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
            .read_unchecked();

        // Update the program counter register.
        mem.get_instruction(ip as usize)
    }

    /// Get a cached move expression decode result, or compute and cache it if not present.
    ///
    /// # Arguments
    ///
    /// * `expr` - The encoded expression to be decoded.
    ///
    /// # Returns
    ///
    /// An array containing the three decoded [`ExpressionArgs`] objects.
    #[inline(always)]
    fn get_cache_move_expression_decode(&mut self, expr: &u32) -> [ExpressionArgs; 3] {
        if !self.move_expression_cache.contains_key(expr) {
            // Cache the decoding result to speed up processing slightly.
            let mut expression_decoder = MoveExpressionHandler::new();
            expression_decoder.decode(*expr);

            let result = expression_decoder.into_array();
            self.move_expression_cache.insert(*expr, result);

            result
        } else {
            *self.move_expression_cache.get(expr).unwrap()
        }
    }

    /// Get the state of a given CPU flag.
    ///
    /// # Arguments
    ///
    /// * `flag` - The [`CpuFlag`] to be checked.
    #[inline(always)]
    fn get_flag_state(&self, flag: CpuFlag) -> bool {
        let register = self.registers.get_register_u32(RegisterId::FL);
        utils::is_bit_set(*register.read_unchecked(), flag.into())
    }

    /// Get the current value of the program counter (PC) register.
    #[inline(always)]
    fn get_program_counter(&mut self) {
        self.registers
            .get_register_u32_mut(RegisterId::PC)
            .read_unchecked();
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
    /// This method sets and unsets the zero and overflow flags as required.
    #[inline(always)]
    fn perform_checked_add_u32(&mut self, value_1: u32, value_2: u32) -> u32 {
        let (final_value, overflow) = match value_1.checked_add(value_2) {
            Some(val) => (val, false),
            None => ((value_1 as u64 + value_2 as u64) as u32, true),
        };
        self.set_flag_state(CpuFlag::OF, overflow);
        self.set_flag_state(CpuFlag::ZF, final_value == 0);

        final_value
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
    /// This method sets and unsets the zero, overflow and carry flags as required.
    /// The overflow (OF) flag will only be affected by 1-bit shifts.
    #[inline(always)]
    fn perform_checked_left_shift_u32(&mut self, value: u32, shift_by: u32) -> u32 {
        assert!(shift_by <= 31);
        if shift_by == 0 {
            return value;
        }

        let final_value = value << shift_by;
        if shift_by == 1 {
            self.set_flag_state(CpuFlag::OF, final_value < value);
        }
        self.set_flag_state(
            CpuFlag::CF,
            utils::is_bit_set_32(value, (32 - shift_by) & 1),
        );
        self.set_flag_state(CpuFlag::ZF, final_value == 0);

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
    /// This method sets and unsets the zero flag as required and always clears the overflow
    /// and carry flags.
    #[inline(always)]
    fn perform_arithmetic_left_shift_u32(&mut self, value: u32, shift_by: u32) -> u32 {
        self.set_flag_state(CpuFlag::OF, false);
        self.set_flag_state(CpuFlag::CF, false);

        let final_value = value.rotate_left(shift_by);
        self.set_flag_state(CpuFlag::ZF, final_value == 0);

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
    /// This method sets and unsets the zero and carry flags as required,
    /// and always clears the overflow flag.
    #[inline(always)]
    fn perform_right_shift_u32(&mut self, value: u32, shift_by: u32) -> u32 {
        assert!(shift_by <= 31);
        if shift_by == 0 {
            return value;
        }

        self.set_flag_state(CpuFlag::OF, false);

        // The carried forward flag should be the value of the least significant bit being
        // shifted out of the value and so MUST be calculated prior to the shifted value.
        // Since we are working in little-Endian the lowest bit has the lowest index.
        self.set_flag_state(CpuFlag::CF, utils::is_bit_set(value, 0));

        let final_value = value >> shift_by;
        self.set_flag_state(CpuFlag::ZF, final_value == 0);

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
    /// This method sets and unsets the zero flag as required and always clears the overflow
    /// and carry flags.
    #[inline(always)]
    fn perform_arithmetic_right_shift_u32(&mut self, value: u32, shift_by: u32) -> u32 {
        self.set_flag_state(CpuFlag::OF, false);
        self.set_flag_state(CpuFlag::CF, false);

        let final_value = value.rotate_right(shift_by);
        self.set_flag_state(CpuFlag::ZF, final_value == 0);

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

    /// Perform a reverse set bit search on the specified value.
    ///
    /// # Arguments
    ///
    /// * `value` - The u32 value to be tested.
    ///
    /// # Note
    ///
    /// This method will set the zero flag if the value is zero, otherwise the flag will be cleared.
    #[inline(always)]
    fn perform_reverse_bit_search(&mut self, value: u32) -> u32 {
        self.set_flag_state(CpuFlag::ZF, value == 0);

        // TODO - if we end up only supporting little Endian, this an be removed.
        if cfg!(target_endian = "little") {
            value.leading_zeros()
        } else {
            value.trailing_zeros()
        }
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

                self.write_reg_u32(&RegisterId::AC, new_value, privilege);
            }
            Instruction::AddU32RegU32Reg(in_reg, out_reg) => {
                let value1 = self.read_reg_u32(in_reg, privilege);
                let value2 = self.read_reg_u32(out_reg, privilege);
                let new_value = self.perform_checked_add_u32(value1, value2);

                self.write_reg_u32(&RegisterId::AC, new_value, privilege);
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

            /******** [Data Instructions] ********/
            /******** [Move Instructions - NO EXPRESSIONS] ********/
            Instruction::SwapU32RegU32Reg(reg1, reg2) => {
                let reg1_val = self.read_reg_u32(reg1, privilege);
                let reg2_val = self.read_reg_u32(reg2, privilege);

                self.write_reg_u32(reg1, reg2_val, privilege);
                self.write_reg_u32(reg2, reg1_val, privilege);
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

            /******** [Move Instructions - WITH EXPRESSIONS] ********/
            Instruction::MovU32ImmMemExprRel(imm, expr) => {
                // mov imm, [addr] - move immediate to address.
                let args = self.get_cache_move_expression_decode(expr);
                let addr = self.execute_u32_move_expression(&args, privilege);

                mem.set_u32(addr as usize, *imm);
            }
            Instruction::MovMemExprU32RegRel(expr, reg) => {
                // mov [addr], register - move value at address to register.
                let args = self.get_cache_move_expression_decode(expr);
                let addr = self.execute_u32_move_expression(&args, privilege);
                let value = mem.get_u32(addr as usize);

                self.write_reg_u32(reg, value, privilege);
            }
            Instruction::MovU32RegMemExprRel(reg, expr) => {
                // mov reg, [addr] - move value of a register to an address.
                let args = self.get_cache_move_expression_decode(expr);
                let addr = self.execute_u32_move_expression(&args, privilege);
                let value = self.read_reg_u32(reg, privilege);

                mem.set_u32(addr as usize, value);
            }

            /******** [Logic Instructions] ********/
            Instruction::BitTestU32Reg(bit, reg) => {
                // bt bit, reg
                let value = self.read_reg_u32(reg, privilege);
                self.perform_bit_test_with_carry_flag(value, *bit);
            }
            Instruction::BitTestMem(bit, addr) => {
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
            Instruction::BitTestResetMem(bit, addr) => {
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
            Instruction::BitTestSetMem(bit, addr) => {
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
            Instruction::BitScanReverseMemU32Reg(addr, reg) => {
                // bsr [addr], reg
                let value = mem.get_u32(*addr as usize);
                let index = self.perform_reverse_bit_search(value);

                self.write_reg_u32(reg, index, privilege);
            }
            Instruction::BitScanReverseU32RegMem(reg, out_addr) => {
                // bsr reg, [out_addr]
                let value = self.read_reg_u32(reg, privilege);
                let index = self.perform_reverse_bit_search(value);

                mem.set_u32(*out_addr as usize, index);
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

        self.update_program_counter_register();

        // Move the instruction pointer register forward by the number of
        // bytes used to build the instruction.
        self.update_instruction_pointer(instruction.get_total_instruction_size());
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

    /// Set the machine mode privilege level of the processor.
    ///
    /// # Arguments
    ///
    /// * `state` - Whether the CPU should be running in machine mode or not.
    #[inline(always)]
    fn set_machine_mode(&mut self, state: bool) {
        self.is_machine_mode = state;
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
    /// * `size` - The size of the instruction, in bytes.
    #[inline(always)]
    fn update_instruction_pointer(&mut self, size: u32) {
        self.registers
            .get_register_u32_mut(RegisterId::IP)
            .add_unchecked(size);
    }

    /// Update the program counter (PC) register.
    #[inline(always)]
    fn update_program_counter_register(&mut self) {
        self.registers
            .get_register_u32_mut(RegisterId::PC)
            .increment_unchecked();
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
    /// The negative flag - set to true if the result of an operation is negative.
    SF,
    /// The zero flag - set to true if the result of an operation is zero.
    ZF,
    /// The overflow flag - set to true if the result of an operation overflowed.
    OF,
    /// The carry flag - set to true if the carry bit has been set by an operation.
    CF,
}

impl fmt::Display for CpuFlag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CpuFlag::SF => write!(f, "SF"),
            CpuFlag::ZF => write!(f, "ZF"),
            CpuFlag::OF => write!(f, "OF"),
            CpuFlag::CF => write!(f, "CF"),
        }
    }
}

impl CpuFlag {
    pub fn iterator() -> Iter<'static, CpuFlag> {
        static FLAGS: [CpuFlag; 4] = [CpuFlag::SF, CpuFlag::ZF, CpuFlag::OF, CpuFlag::CF];
        FLAGS.iter()
    }

    /// Build a flag value from the enabled CPU flags.
    ///
    /// # Arguments
    ///
    /// * `flags` - A slice of [`CpuFlag`]s to be enabled.
    #[inline(always)]
    pub fn compute_for(flags: &[CpuFlag]) -> u32 {
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
    use std::panic;

    use crate::{
        ins::{
            instruction::Instruction,
            move_expressions::{ExpressionArgs, ExpressionOperator, MoveExpressionHandler},
        },
        mem::memory::Memory,
        reg::registers::{RegisterId, Registers},
    };

    use super::{Cpu, CpuFlag};

    struct TestEntryU32Standard {
        pub instructions: Vec<Instruction>,
        pub expected_registers: Registers,
        pub expected_memory: Vec<u8>,
        pub should_panic: bool,
        pub fail_message: String,
    }

    impl TestEntryU32Standard {
        fn new(
            instructions: &[Instruction],
            expected_registers: &[(RegisterId, u32)],
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
                expected_registers: Registers::default(),
                expected_memory,
                should_panic,
                fail_message: fail_message.to_string(),
            };

            s.build_registers(expected_registers);

            s
        }

        /// Build the expected [`Registers`] instance for our test.
        ///
        /// # Arguments
        ///
        /// * `expected_registers` - A slice of tuples, the first entry being the [`RegisterId`] and the second being the expected value.
        fn build_registers(&mut self, expected_registers: &[(RegisterId, u32)]) {
            // This should be done before the registers are set because there will be instances
            // there the number of executed instructions will be different than the total
            // instruction count, such as if we hit a halt or early return.
            let size: u32 = self
                .instructions
                .iter()
                .map(|i| i.get_total_instruction_size())
                .sum();

            self.expected_registers
                .get_register_u32_mut(RegisterId::IP)
                .write_unchecked(size);
            self.expected_registers
                .get_register_u32_mut(RegisterId::PC)
                .write_unchecked(self.instructions.len() as u32);

            for (reg, val) in expected_registers {
                self.expected_registers
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
                self.expected_registers,
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
                    Instruction::AddU32ImmU32Reg(0x2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::AC, 0x3)],
                vec![0; 100],
                false,
                "ADD - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    Instruction::AddU32ImmU32Reg(0x2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, u32::MAX),
                    (RegisterId::AC, 0x1),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::OF])),
                ],
                vec![0; 100],
                false,
                "ADD - CPU flags not correctly set",
            ),
            TestEntryU32Standard::new(
                &[Instruction::AddU32ImmU32Reg(0, RegisterId::R1)],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF]))],
                vec![0; 100],
                false,
                "ADD - CPU flags not correctly set",
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
                    Instruction::AddU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, 0xf),
                    (RegisterId::R2, 0x1),
                    (RegisterId::AC, 0x10),
                ],
                vec![0; 100],
                false,
                "ADD - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    // Setup register values.
                    Instruction::MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x2, RegisterId::R2),
                    // Add the register values.
                    Instruction::AddU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R1, u32::MAX),
                    (RegisterId::R2, 0x2),
                    (RegisterId::AC, 0x1),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::OF])),
                ],
                vec![0; 100],
                false,
                "ADD - CPU flags not correctly set",
            ),
            TestEntryU32Standard::new(
                &[Instruction::AddU32RegU32Reg(RegisterId::R1, RegisterId::R2)],
                &[(RegisterId::FL, 0b0000_0010)],
                vec![0; 100],
                false,
                "ADD - CPU flags not correctly set",
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
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::LeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
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
                    Instruction::MovU32ImmU32Reg(
                        0b1011_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::LeftShiftU32ImmU32Reg(0x3, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1000),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "SHL - CPU flags not correctly set",
            ),
            // When shifted left by one place. This will set the carry and overflow flags.
            // The overflow flag is only set when computing 1-bit shifts.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::LeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::CF, CpuFlag::OF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHL - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x0, RegisterId::R1),
                    Instruction::LeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF]))],
                vec![0; 100],
                false,
                "SHL - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    // Manually set the overflow flag.
                    Instruction::MovU32ImmU32Reg(
                        CpuFlag::compute_for(&[CpuFlag::OF]),
                        RegisterId::FL,
                    ),
                    // This will unset the overflow flag and set the zero flag instead.
                    Instruction::MovU32ImmU32Reg(0x0, RegisterId::R1),
                    Instruction::LeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF]))],
                vec![0; 100],
                false,
                "SHL - correct flags not set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestEntryU32Standard::new(
                &[Instruction::LeftShiftU32ImmU32Reg(0x20, RegisterId::R1)],
                &[],
                vec![],
                true,
                "SHL - successfully executed instruction with invalid shift value",
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
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    Instruction::LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
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
                    Instruction::MovU32ImmU32Reg(
                        0b1011_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::MovU32ImmU32Reg(0x3, RegisterId::R2),
                    Instruction::LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1000),
                    (RegisterId::R2, 0x3),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "SHL - invalid CPU flag state",
            ),
            // When shifted left by one place. This will set the carry and overflow flags.
            // The overflow flag is only set when computing 1-bit shifts.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    Instruction::LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::R2, 0x1),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::OF, CpuFlag::CF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHL - CPU flags not correctly set",
            ),
            // When shifted left by two places, this value will set the overflow and carry flags.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(u32::MAX, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    Instruction::LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::R2, 0x1),
                    (
                        RegisterId::FL,
                        CpuFlag::compute_for(&[CpuFlag::OF, CpuFlag::CF]),
                    ),
                ],
                vec![0; 100],
                false,
                "SHL - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x0, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    Instruction::LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R2, 0x1),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF])),
                ],
                vec![0; 100],
                false,
                "SHL - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    // Manually set the overflow flag.
                    Instruction::MovU32ImmU32Reg(
                        CpuFlag::compute_for(&[CpuFlag::OF]),
                        RegisterId::FL,
                    ),
                    // This will unset the overflow flag and set the zero flag instead.
                    Instruction::MovU32ImmU32Reg(0x0, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    Instruction::LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R2, 0x1),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF])),
                ],
                vec![0; 100],
                false,
                "SHL - CPU flags not correctly set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x20, RegisterId::R2),
                    Instruction::LeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[],
                vec![],
                true,
                "SHL - successfully executed instruction with invalid shift value",
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
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::ArithLeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2)],
                vec![0; 100],
                false,
                "SAL - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        RegisterId::R1,
                    ),
                    Instruction::ArithLeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1101)],
                vec![0; 100],
                false,
                "SAL - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x0, RegisterId::R1),
                    Instruction::LeftShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF]))],
                vec![0; 100],
                false,
                "SAL - CPU flags not correctly set",
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
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    Instruction::ArithLeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x2), (RegisterId::R2, 0x1)],
                vec![0; 100],
                false,
                "SAL - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        RegisterId::R1,
                    ),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    Instruction::ArithLeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1101),
                    (RegisterId::R2, 0x1),
                ],
                vec![0; 100],
                false,
                "SAL - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x0, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    Instruction::ArithLeftShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R2, 0x1),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF])),
                ],
                vec![0; 100],
                false,
                "SAL - CPU flags not correctly set",
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
                    Instruction::MovU32ImmU32Reg(0x2, RegisterId::R1),
                    Instruction::RightShiftU32ImmU32Reg(0x1, RegisterId::R1),
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
                    Instruction::MovU32ImmU32Reg(
                        CpuFlag::compute_for(&[CpuFlag::OF]),
                        RegisterId::FL,
                    ),
                    // Execute the test instruction.
                    Instruction::MovU32ImmU32Reg(
                        0b0111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::RightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "SHR - CPU flags not correctly set",
            ),
            // The SHR command should clear the overflow flag and the
            // carry flag in this case as the bit being shifted out is zero.
            TestEntryU32Standard::new(
                &[
                    // Manually set the overflow flag.
                    Instruction::MovU32ImmU32Reg(
                        CpuFlag::compute_for(&[CpuFlag::OF]),
                        RegisterId::FL,
                    ),
                    // Execute the test instruction.
                    Instruction::MovU32ImmU32Reg(
                        0b0111_1111_1111_1111_1111_1111_1111_1110,
                        RegisterId::R1,
                    ),
                    Instruction::RightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0b0011_1111_1111_1111_1111_1111_1111_1111)],
                vec![0; 100],
                false,
                "SHR - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x0, RegisterId::R1),
                    Instruction::RightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF]))],
                vec![0; 100],
                false,
                "SHR - CPU flags not correctly set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestEntryU32Standard::new(
                &[Instruction::RightShiftU32ImmU32Reg(0x20, RegisterId::R1)],
                &[],
                vec![],
                true,
                "SHR - successfully executed instruction with invalid shift value",
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
                    Instruction::MovU32ImmU32Reg(0x2, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    Instruction::RightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
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
                    Instruction::MovU32ImmU32Reg(
                        CpuFlag::compute_for(&[CpuFlag::OF]),
                        RegisterId::FL,
                    ),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    // Execute the test instruction.
                    Instruction::MovU32ImmU32Reg(
                        0b0111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::RightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0x1),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::CF])),
                ],
                vec![0; 100],
                false,
                "SHR - CPU flags not correctly set",
            ),
            // The SHR command should clear the overflow flag and the
            // carry flag in this case as the bit being shifted out is zero.
            TestEntryU32Standard::new(
                &[
                    // Manually set the overflow flag.
                    Instruction::MovU32ImmU32Reg(
                        CpuFlag::compute_for(&[CpuFlag::OF]),
                        RegisterId::FL,
                    ),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    // Execute the test instruction.
                    Instruction::MovU32ImmU32Reg(
                        0b0111_1111_1111_1111_1111_1111_1111_1110,
                        RegisterId::R1,
                    ),
                    Instruction::RightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0x1),
                ],
                vec![0; 100],
                false,
                "SHR - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x0, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    Instruction::RightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R2, 0x1),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF])),
                ],
                vec![0; 100],
                false,
                "SHR - CPU flags not correctly set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x20, RegisterId::R2),
                    Instruction::RightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[],
                vec![],
                true,
                "SHR - successfully executed instruction with invalid shift value",
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
                    Instruction::MovU32ImmU32Reg(0x2, RegisterId::R1),
                    Instruction::ArithRightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1)],
                vec![0; 100],
                false,
                "SAR - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(
                        0b0111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::ArithRightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0b1011_1111_1111_1111_1111_1111_1111_1111)],
                vec![0; 100],
                false,
                "SAR - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero right-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x0, RegisterId::R1),
                    Instruction::ArithRightShiftU32ImmU32Reg(0x1, RegisterId::R1),
                ],
                &[(RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF]))],
                vec![0; 100],
                false,
                "SAR - CPU flags not correctly set",
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
                    Instruction::MovU32ImmU32Reg(0x2, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    Instruction::ArithRightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::R2, 0x1)],
                vec![0; 100],
                false,
                "SAR - incorrect result value produced",
            ),
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(
                        0b0111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    Instruction::ArithRightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R1, 0b1011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::R2, 0x1),
                ],
                vec![0; 100],
                false,
                "SAR - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x0, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R2),
                    Instruction::ArithRightShiftU32RegU32Reg(RegisterId::R2, RegisterId::R1),
                ],
                &[
                    (RegisterId::R2, 0x1),
                    (RegisterId::FL, CpuFlag::compute_for(&[CpuFlag::ZF])),
                ],
                vec![0; 100],
                false,
                "SAR - CPU flags not correctly set",
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
                "MOV - invalid value moved to register",
            ),
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::MovU32ImmU32Reg(0x2, RegisterId::R1),
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
                    Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                    Instruction::MovU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[(RegisterId::R1, 0x1), (RegisterId::R2, 0x1)],
                vec![0; 100],
                false,
                "MOV - immediate value not moved to register",
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
            &[Instruction::MovU32ImmMemRelSimple(0x123, 0x0)],
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
                Instruction::MovU32ImmU32Reg(0x123, RegisterId::R1),
                Instruction::MovU32RegMemRelSimple(RegisterId::R1, 0x0),
                // Move the value from memory to a register.
                Instruction::MovMemU32RegRelSimple(0x0, RegisterId::R2),
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
                Instruction::MovU32ImmU32Reg(0x123, RegisterId::R1),
                Instruction::MovU32RegMemRelSimple(RegisterId::R1, 0x0),
                // Set the address pointer in R2.
                Instruction::MovU32ImmU32Reg(0x0, RegisterId::R2),
                // Read the value from the address of R2 into R1.
                Instruction::MovU32RegPtrU32RegRelSimple(RegisterId::R2, RegisterId::R3),
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
                Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
                Instruction::MovU32ImmU32Reg(0x2, RegisterId::R2),
                Instruction::SwapU32RegU32Reg(RegisterId::R1, RegisterId::R2),
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
        let mut handler = MoveExpressionHandler::new();

        let test_1_args = [Instruction::MovU32ImmMemExprRel(
            0x123,
            handler.encode(&[
                ExpressionArgs::Constant(0x8),
                ExpressionArgs::Operator(ExpressionOperator::Add),
                ExpressionArgs::Constant(0x8),
            ]),
        )];

        let test_2_args = [
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R1),
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R2),
            Instruction::MovU32ImmMemExprRel(
                0x123,
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Register(RegisterId::R2),
                ]),
            ),
        ];

        let test_3_args = [
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R1),
            Instruction::MovU32ImmMemExprRel(
                0x123,
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Constant(0x8),
                ]),
            ),
        ];

        let test_4_args = [
            Instruction::MovU32ImmU32Reg(0x2, RegisterId::R1),
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R2),
            Instruction::MovU32ImmMemExprRel(
                0x123,
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::R2),
                ]),
            ),
        ];

        let test_5_args = [
            Instruction::MovU32ImmU32Reg(0x1a, RegisterId::R1),
            Instruction::MovU32ImmU32Reg(0x4, RegisterId::R2),
            Instruction::MovU32ImmMemExprRel(
                0x123,
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Subtract),
                    ExpressionArgs::Register(RegisterId::R2),
                ]),
            ),
        ];

        let tests = [
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
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
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
                "MOV - value not correctly moved from memory to register with expression - two registers",
            ),
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
                "MOV - value not correctly moved from memory to register with expression - one constant and one register",
            ),
            TestEntryU32Standard::new(
                &test_4_args,
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
                &test_5_args,
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
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the complex move from an expression-derived memory address to a register.
    #[test]
    fn test_move_mem_expr_u32_reg() {
        let mut handler = MoveExpressionHandler::new();

        let test_1_args = [
            Instruction::MovU32ImmMemRelSimple(0x123, 0x10),
            Instruction::MovMemExprU32RegRel(
                handler.encode(&[
                    ExpressionArgs::Constant(0x8),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Constant(0x8),
                ]),
                RegisterId::R8,
            ),
        ];

        let test_2_args = [
            Instruction::MovU32ImmMemRelSimple(0x123, 0x10),
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R1),
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R2),
            Instruction::MovMemExprU32RegRel(
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Register(RegisterId::R2),
                ]),
                RegisterId::R8,
            ),
        ];

        let test_3_args = [
            Instruction::MovU32ImmMemRelSimple(0x123, 0x10),
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R1),
            Instruction::MovMemExprU32RegRel(
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Constant(0x8),
                ]),
                RegisterId::R8,
            ),
        ];

        let test_4_args = [
            Instruction::MovU32ImmMemRelSimple(0x123, 0x10),
            Instruction::MovU32ImmU32Reg(0x2, RegisterId::R1),
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R2),
            Instruction::MovMemExprU32RegRel(
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::R2),
                ]),
                RegisterId::R8,
            ),
        ];

        let test_5_args = [
            Instruction::MovU32ImmMemRelSimple(0x123, 0x16),
            Instruction::MovU32ImmU32Reg(0x1A, RegisterId::R1),
            Instruction::MovU32ImmU32Reg(0x4, RegisterId::R2),
            Instruction::MovMemExprU32RegRel(
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Subtract),
                    ExpressionArgs::Register(RegisterId::R2),
                ]),
                RegisterId::R8,
            ),
        ];

        let tests = [
            TestEntryU32Standard::new(
                &test_1_args,
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
                &test_2_args,
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
                &test_3_args,
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
                &test_4_args,
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
                &test_5_args,
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
        ];

        for (id, test) in tests.iter().enumerate() {
            test.run_test(id);
        }
    }

    /// Test the complex move from a register to an expression-derived memory address.
    #[test]
    fn test_move_u32_reg_mem_expr() {
        let mut handler = MoveExpressionHandler::new();

        let test_1_args = [
            Instruction::MovU32ImmU32Reg(0x123, RegisterId::R8),
            Instruction::MovU32RegMemExprRel(
                RegisterId::R8,
                handler.encode(&[
                    ExpressionArgs::Constant(0x8),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Constant(0x8),
                ]),
            ),
        ];

        let test_2_args = [
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R1),
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R2),
            Instruction::MovU32ImmU32Reg(0x123, RegisterId::R8),
            Instruction::MovU32RegMemExprRel(
                RegisterId::R8,
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Register(RegisterId::R2),
                ]),
            ),
        ];

        let test_3_args = [
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R1),
            Instruction::MovU32ImmU32Reg(0x123, RegisterId::R8),
            Instruction::MovU32RegMemExprRel(
                RegisterId::R8,
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Add),
                    ExpressionArgs::Constant(0x8),
                ]),
            ),
        ];

        let test_4_args = [
            Instruction::MovU32ImmU32Reg(0x2, RegisterId::R1),
            Instruction::MovU32ImmU32Reg(0x8, RegisterId::R2),
            Instruction::MovU32ImmU32Reg(0x123, RegisterId::R8),
            Instruction::MovU32RegMemExprRel(
                RegisterId::R8,
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Multiply),
                    ExpressionArgs::Register(RegisterId::R2),
                ]),
            ),
        ];

        let test_5_args = [
            Instruction::MovU32ImmU32Reg(0x1A, RegisterId::R1),
            Instruction::MovU32ImmU32Reg(0x4, RegisterId::R2),
            Instruction::MovU32ImmU32Reg(0x123, RegisterId::R8),
            Instruction::MovU32RegMemExprRel(
                RegisterId::R8,
                handler.encode(&[
                    ExpressionArgs::Register(RegisterId::R1),
                    ExpressionArgs::Operator(ExpressionOperator::Subtract),
                    ExpressionArgs::Register(RegisterId::R2),
                ]),
            ),
        ];

        let tests = [
            TestEntryU32Standard::new(
                &test_1_args,
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
                &test_2_args,
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
                &test_3_args,
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
                &test_4_args,
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
                &test_5_args,
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
                    Instruction::MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::BitTestU32Reg(0x0, RegisterId::R1),
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
                    Instruction::MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        RegisterId::R1,
                    ),
                    Instruction::BitTestU32Reg(0x0, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1110)],
                vec![0; 100],
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestEntryU32Standard::new(
                &[Instruction::BitTestU32Reg(32, RegisterId::R1)],
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
                    Instruction::MovU32ImmMemRelSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        0x0,
                    ),
                    Instruction::BitTestMem(0x0, 0x0),
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
                    Instruction::MovU32ImmMemRelSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        0x0,
                    ),
                    Instruction::BitTestMem(0x0, 0x0),
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
                &[Instruction::BitTestMem(32, 0x0)],
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
                    Instruction::MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::BitTestResetU32Reg(0x0, RegisterId::R1),
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
                    Instruction::MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        RegisterId::R1,
                    ),
                    Instruction::BitTestResetU32Reg(0x0, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1110)],
                vec![0; 100],
                false,
                "BTR - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestEntryU32Standard::new(
                &[
                    Instruction::BitTestResetU32Reg(32, RegisterId::R1),
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
                    Instruction::MovU32ImmMemRelSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        0x0,
                    ),
                    Instruction::BitTestResetMem(0x0, 0x0),
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
                    Instruction::MovU32ImmMemRelSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        0x0,
                    ),
                    Instruction::BitTestResetMem(0x0, 0x0),
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
                    Instruction::BitTestResetMem(32, 0x0),
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
                    Instruction::MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::BitTestSetU32Reg(0x0, RegisterId::R1),
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
                    Instruction::MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        RegisterId::R1,
                    ),
                    Instruction::BitTestSetU32Reg(0x0, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                vec![0; 100],
                false,
                "BTS - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestEntryU32Standard::new(
                &[
                    Instruction::BitTestSetU32Reg(32, RegisterId::R1),
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
                    Instruction::MovU32ImmMemRelSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        0x0,
                    ),
                    Instruction::BitTestSetMem(0x0, 0x0),
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
                    Instruction::MovU32ImmMemRelSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        0x0,
                    ),
                    Instruction::BitTestSetMem(0x0, 0x0),
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
                    Instruction::BitTestSetMem(32, 0x0),
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
                    Instruction::MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::BitScanReverseU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                vec![0; 100],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(
                        0b0000_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::BitScanReverseU32RegU32Reg(RegisterId::R1, RegisterId::R2),
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
                    Instruction::MovU32ImmU32Reg(0x0, RegisterId::R1),
                    Instruction::BitScanReverseU32RegU32Reg(RegisterId::R1, RegisterId::R2),
                ],
                &[
                    (RegisterId::R2, 32),
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
                    Instruction::MovU32ImmMemRelSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        0x0,
                    ),
                    Instruction::BitScanReverseMemU32Reg(0x0, RegisterId::R1),
                ],
                &[(RegisterId::R1, 0x0)],
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
                    Instruction::MovU32ImmMemRelSimple(
                        0b0000_1111_1111_1111_1111_1111_1111_1111,
                        0x0,
                    ),
                    Instruction::BitScanReverseMemU32Reg(0x0, RegisterId::R1),
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
                &[Instruction::BitScanReverseMemU32Reg(0x0, RegisterId::R1)],
                &[
                    (RegisterId::R1, 32),
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
                    Instruction::MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::BitScanReverseU32RegMem(RegisterId::R1, 0x0),
                ],
                &[(RegisterId::R1, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                vec![0; 100],
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestEntryU32Standard::new(
                &[
                    Instruction::MovU32ImmU32Reg(
                        0b0000_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::R1,
                    ),
                    Instruction::BitScanReverseU32RegMem(RegisterId::R1, 0x0),
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
                &[Instruction::BitScanReverseU32RegMem(RegisterId::R1, 0x0)],
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
}
