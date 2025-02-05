use core::fmt;
use hashbrown::HashMap;
use std::{arch::asm, panic, slice::Iter};

use crate::{
    com_bus::communication_bus::CommunicationBus,
    ins::{
        expression::{Expression, ExpressionArgs},
        instruction::Instruction,
    },
    mem::memory_handler::{MemoryHandler, StackArgType, SIZE_OF_F32, SIZE_OF_U32},
    privilege_level::PrivilegeLevel,
    reg::registers::{RegisterId, Registers},
    utils,
};

// I have taken some of the common ones from the ones that are generally universal.
/// Division by zero interrupt - default handler implemented.
pub const DIVIDE_BY_ZERO_INT: u8 = 0x0;
/// Single-step interrupt - not yet implemented.
pub const SINGLE_STEP_INT: u8 = 0x1;
/// NMI (non-maskable interrupt) - default handler implemented.
pub const NON_MASKABLE_INT: u8 = 0x2;
/// The invalid opcode interrupt - not yet implemented.
pub const INVALID_OPCODE_INT: u8 = 0x6;
/// A general protection fault - default handler implemented.
pub const GENERAL_PROTECTION_FAULT_INT: u8 = 0x0d;
/// A general device error interrupt - default handler implemented.
pub const DEVICE_ERROR_INT: u8 = 0x16;

/// By default, all interrupts below 32 (0x20) are used for exceptions and are thus not maskable.
const EXCEPTION_INT_CUTOFF: u8 = 0x20;

/// A list of f32 registers to be preserved when entering a subroutine. Use when storing state.
///
/// # Note
///
/// By convention FR1 isn't preserved as that will be the register that can hold the return value from the subroutine.
const PRESERVE_REGISTERS_F32: [RegisterId; 1] = [RegisterId::FR2];

/// A reversed list of f32 registers to be preserved when entering a subroutine. Use when restoring state.
const PRESERVE_REGISTERS_F32_REV: [RegisterId; 1] = [RegisterId::FR2];

/// A list of u32 registers to be preserved when entering a subroutine. Use when storing state
///
/// # Note
///
/// By convention ER1 isn't preserved as that will be the register that can hold the return value from the subroutine.
const PRESERVE_REGISTERS_U32: [RegisterId; 8] = [
    RegisterId::EBX,
    RegisterId::ECX,
    RegisterId::EDX,
    RegisterId::E5X,
    RegisterId::E6X,
    RegisterId::E7X,
    RegisterId::E8X,
    RegisterId::EIP,
];

/// A reversed list of u32 registers to be preserved when entering a subroutine. Use when restoring state.
const PRESERVE_REGISTERS_U32_REV: [RegisterId; 8] = [
    RegisterId::EIP,
    RegisterId::E8X,
    RegisterId::E7X,
    RegisterId::E6X,
    RegisterId::E5X,
    RegisterId::EDX,
    RegisterId::ECX,
    RegisterId::EBX,
];

/// The mask to get only the lowest 8 bits of a u32 value.
const U32_LOW_BYTE_MASK: u32 = 0xff;
/// The mask to get everything except the lowest 8 bits of a u32 value.
const NOT_U32_LOW_BYTE_MASK: u32 = !U32_LOW_BYTE_MASK;

/// The size of a u32 value, in bytes.
const SIZE_OF_U32_LOCAL: u32 = SIZE_OF_U32 as u32;
/// The size of a f32 value, in bytes.
const SIZE_OF_F32_LOCAL: u32 = SIZE_OF_F32 as u32;

pub struct Cpu {
    /// The registers associated with this CPU.
    pub registers: Registers,
    // Is the CPU currently halted?
    is_halted: bool,
    /// The current machine execution mode.
    privilege_level: PrivilegeLevel,
    /// Is the CPU currently in an interrupt handler?
    is_in_interrupt_handler: bool,
    /// The last interrupt handler that was "active" (i.e. was not terminated with intret).
    last_interrupt_code: Option<u8>,
    /// The decoded expressions cache.
    decode_expression_cache: HashMap<u32, Expression>,
}

impl Cpu {
    pub fn new() -> Self {
        Self {
            registers: Registers::default(),
            is_halted: false,
            privilege_level: PrivilegeLevel::Machine,
            is_in_interrupt_handler: false,
            last_interrupt_code: None,
            decode_expression_cache: HashMap::new(),
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
    ///
    /// # Returns
    ///
    /// A u32 that is the calculated result of the expression.
    #[inline]
    fn decode_evaluate_u32_expression(&mut self, expr: &u32) -> u32 {
        use ExpressionArgs as EA;

        let privilege = &self.privilege_level;

        // Cache the decoded expression so we don't need to do this multiple times
        // for the same expression.
        let decoder = self
            .decode_expression_cache
            .entry(*expr)
            .or_insert_with(|| {
                let mut expression = Expression::new();
                expression.unpack(*expr);
                expression
            });

        // Determine the first and second operands.
        let value_1 = match decoder.operand_1 {
            EA::Register(id) => self.registers.read_reg_u32(&id, privilege),
            EA::Immediate(val) => val as u32,
            _ => panic!(),
        };
        let value_2 = match decoder.operand_2 {
            EA::Register(id) => self.registers.read_reg_u32(&id, privilege),
            EA::Immediate(val) => val as u32,
            _ => panic!(),
        };
        let value_3 = if decoder.is_extended {
            match decoder.operand_3 {
                EA::Register(id) => self.registers.read_reg_u32(&id, privilege),
                EA::Immediate(val) => val as u32,
                _ => panic!(),
            }
        } else {
            0
        };

        decoder.evaluate(value_1, value_2, value_3)
    }

    /// Decrease the stack pointer (ESP) register by a specified amount.
    #[inline(always)]
    fn decrease_sp_register_by(&mut self, amount: u32) {
        self.registers
            .get_register_u32_mut(RegisterId::ESP)
            .subtract_unchecked(amount);
    }

    /// Get the flags register.
    #[inline(always)]
    pub fn get_flags(&self) -> u32 {
        self.registers.read_reg_u32_unchecked(&RegisterId::EFL)
    }

    /// Get the instruction pointer (IP) register.
    #[inline(always)]
    pub fn get_instruction_pointer(&self) -> u32 {
        self.registers.read_reg_u32_unchecked(&RegisterId::EIP)
    }

    /// Get the current privilege level of the processor.
    ///
    /// # Returns
    ///
    /// A [`PrivilegeLevel`] giving the current privilege level of the processor.
    #[allow(unused)]
    #[inline(always)]
    fn get_privilege(&self) -> PrivilegeLevel {
        self.privilege_level
    }

    /// Fetch and decode the next instruction to be executed.
    ///
    /// # Arguments
    ///
    /// * `com_bus` - A reference to the [`CommunicationBus`] for the virtual machine.
    ///
    /// # Returns
    ///
    /// The next [`Instruction`] to be executed.
    fn fetch_decode_next_instruction(&mut self, com_bus: &CommunicationBus) -> Instruction {
        // Get the current instruction pointer.
        let ip = self.registers.read_reg_u32_unchecked(&RegisterId::EIP) as usize;

        // Decode the next instruction.
        com_bus.mem.get_instruction(ip)
    }

    /// Get the value of the stack pointer (ESP) register.
    #[inline(always)]
    pub fn get_stack_pointer(&self) -> u32 {
        self.registers.read_reg_u32_unchecked(&RegisterId::ESP)
    }

    /// Increase the instruction pointer (EIP) register by a specified amount.
    #[inline(always)]
    fn increase_ip_register_by(&mut self, amount: usize) {
        self.registers
            .get_register_u32_mut(RegisterId::EIP)
            .add_unchecked(amount as u32);
    }

    /// Increase the stack pointer (ESP) register by a specified amount.
    #[inline(always)]
    fn increase_sp_register_by(&mut self, amount: u32) {
        self.registers
            .get_register_u32_mut(RegisterId::ESP)
            .add_unchecked(amount);
    }

    /// Check whether a specific CPU flag is set.
    ///
    /// # Arguments
    ///
    /// * `flag` - The [`CpuFlag`] to check.
    #[allow(unused)]
    #[inline(always)]
    fn is_flag_set(&self, flag: &CpuFlag) -> bool {
        // Get the current CPU flags.
        let flags = self.registers.read_reg_u32_unchecked(&RegisterId::EFL);
        utils::is_bit_set(flags, *flag as u8)
    }

    /// Check whether the CPU is currently in machine mode.
    #[inline(always)]
    pub fn is_in_machine_mode(&self) -> bool {
        self.privilege_level == PrivilegeLevel::Machine
    }

    /// Perform a checked add of two u32 values.
    ///
    /// # Arguments
    ///
    /// * `value_1` - The first u32 value.
    /// * `value_2` - The second u32 value.
    /// * `out_reg` - The destination u32 register.
    ///
    /// # Returns
    ///
    /// The result of the operation, truncated in the case of an overflow.
    ///
    /// # Note
    ///
    /// This method affects the following flags: Sign (SF), Overflow (OF), Zero (ZF) and Parity (PF). Any other flags are undefined.
    #[inline]
    fn perform_checked_add_u32(&mut self, value_1: u32, value_2: u32, out_reg: &RegisterId) {
        let (new_value, overflow) = value_1.overflowing_add(value_2);

        self.set_standard_flags_by_value(new_value, overflow);

        self.write_reg_u32(out_reg, new_value);
    }

    /// Perform a checked subtraction of two u32 values.
    ///
    /// # Arguments
    ///
    /// * `target_value` - The first u32 value.
    /// * `sub_value` - The second u32 value.
    /// * `out_reg` - The destination u32 register.
    ///
    /// # Returns
    ///
    /// The result of the operation, truncated in the case of an overflow.
    ///
    /// # Note
    ///
    /// This method affects the following flags: Sign (SF), Overflow (OF), Zero (ZF) and Parity (PF). Any other flags are undefined.
    #[inline]
    fn perform_checked_subtract_u32(
        &mut self,
        target_value: u32,
        sub_value: u32,
        out_reg: &RegisterId,
    ) {
        let (new_value, overflow) = target_value.overflowing_sub(sub_value);

        self.set_standard_flags_by_value(new_value, overflow);

        self.write_reg_u32(out_reg, new_value);
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
    #[inline]
    fn perform_checked_mul_u32(&mut self, value_1: u32, value_2: u32) {
        let new_value = value_1.wrapping_mul(value_2);

        // The overflow and carry flags will be true if the upper bits 16
        // bits of the value are not zero.
        let state = (new_value >> 16) != 0;
        self.set_flag_state(CpuFlag::OF, state);
        self.set_flag_state(CpuFlag::CF, state);

        self.write_reg_u32_unchecked(&RegisterId::EAX, new_value);
    }

    /// Perform a bitwise and of two u32 values.
    ///
    /// # Arguments
    ///
    /// * `value_1` - The first u32 value.
    /// * `value_2` - The second u32 value.
    /// * `out_reg` - The destination u32 register.
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
    #[inline]
    fn perform_bitwise_and_u32(&mut self, value_1: u32, value_2: u32, out_reg: &RegisterId) {
        let new_value = value_1 & value_2;

        self.set_flag_state(CpuFlag::SF, utils::is_bit_set(new_value, 31));
        self.set_flag_state(CpuFlag::ZF, new_value == 0);
        self.set_flag_state(CpuFlag::OF, false);
        self.set_flag_state(CpuFlag::PF, Cpu::calculate_lowest_byte_parity(new_value));
        self.set_flag_state(CpuFlag::CF, false);

        self.write_reg_u32(out_reg, new_value);
    }

    /// Perform a checked left-shift a u32 value by a u8 value.
    ///
    /// # Arguments
    ///
    /// * `value` - The first u32 value.
    /// * `shift_by` - The value giving the number of places to shift.
    /// * `out_reg` - The destination u32 register.
    ///
    /// # Returns
    ///
    /// The result of the operation.
    ///
    /// # Note
    ///
    /// This method affects the following flags: Sign (SF), Overflow (OF), Zero (ZF), Carry (CF) and Parity (PF).
    ///
    /// The Overflow (OF) flag will only be affected by 1-bit shifts. Any other flags are undefined.
    #[inline]
    fn perform_checked_left_shift_u32(&mut self, value: u32, shift_by: u8, out_reg: &RegisterId) {
        if shift_by == 0 {
            return;
        }

        assert!(shift_by <= 31);

        let new_value = value << shift_by;
        self.set_flag_state(CpuFlag::SF, utils::is_bit_set(new_value, 31));
        self.set_flag_state(CpuFlag::ZF, new_value == 0);
        if shift_by == 1 {
            self.set_flag_state(CpuFlag::OF, new_value < value);
        }
        self.set_flag_state(CpuFlag::CF, utils::is_bit_set(value, (32 - shift_by) & 1));
        self.set_flag_state(CpuFlag::PF, Cpu::calculate_lowest_byte_parity(new_value));

        self.write_reg_u32(out_reg, new_value);
    }

    /// Perform an unsigned u32 division and modulo between the two arguments.
    /// ER1 will be set to the quotient and ER4 will be set to modulo.
    ///
    /// # Arguments
    ///
    /// * `dividend` - The dividend (the number to be divided).
    /// * `divisor` - The divisor (the number to be divided by).
    ///
    /// # Note
    ///
    /// This method affects the following flags: none. All flags are undefined.
    /// This method will trigger a DIVISION_BY_ZERO interrupt if the divisor is zero.
    #[inline]
    pub fn perform_division_u32(&mut self, dividend: u32, divisor: u32, mem: &mut MemoryHandler) {
        if divisor == 0 {
            self.trigger_interrupt(DIVIDE_BY_ZERO_INT, mem);
            return;
        }

        let quotient: u32;
        let modulo: u32;

        // This allows us to compute the quotient and modulo at once, avoiding doing two
        // separate operations here.
        unsafe {
            asm!(
                "xor edx, edx", // Clear the EDX register before attempting to load a new value into it.
                "div ecx",
                inout("eax") dividend => quotient, // The quotient is always stored in EAX.
                in("ecx") divisor,
                lateout("edx") modulo // The remainder (modulo) is always stored in EDX.
            );
        }

        self.write_reg_u32_unchecked(&RegisterId::EAX, quotient);
        self.write_reg_u32_unchecked(&RegisterId::EDX, modulo);
    }

    /// Perform an arithmetic left-shift of two u32 values.
    ///
    /// # Arguments
    ///
    /// * `value` - The first u32 value.
    /// * `shift_by` - The value giving the number of places to shift.
    /// * `out_reg` - The destination u32 register.
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
    #[inline]
    fn perform_arithmetic_left_shift_u32(
        &mut self,
        value: u32,
        shift_by: u32,
        out_reg: &RegisterId,
    ) {
        let new_value = value.rotate_left(shift_by);
        self.set_flag_state(CpuFlag::SF, utils::is_bit_set(new_value, 31));
        self.set_flag_state(CpuFlag::ZF, new_value == 0);
        self.set_flag_state(CpuFlag::OF, false);
        self.set_flag_state(CpuFlag::CF, false);
        self.set_flag_state(CpuFlag::PF, Cpu::calculate_lowest_byte_parity(new_value));

        self.write_reg_u32(out_reg, new_value);
    }

    /// Perform a call jump.
    ///
    /// # Arguments
    ///
    /// * `mem` - A mutable reference to the [`MemoryHandler`] connected to this CPU instance.
    /// * `addr` - The call jump address.
    #[inline]
    fn perform_call_jump(&mut self, mem: &mut MemoryHandler, addr: u32) {
        // Store the current stack frame state.
        self.push_state(mem);

        // Jump to the subroutine by moving the instruction pointer to the
        // specified location.
        self.set_instruction_pointer(addr);
    }

    /// Perform an call jump from a register value with an offset.
    ///
    /// # Arguments
    ///
    /// * `mem` - A mutable reference to the [`MemoryHandler`] connected to this CPU instance.
    /// * `base_reg` - The register ID that will be used to obtain the base address.
    /// * `offset` - The offset from the base address.
    fn perform_call_offset_jump(
        &mut self,
        mem: &mut MemoryHandler,
        base_reg: &RegisterId,
        offset: u32,
    ) {
        let base_addr = self.read_reg_u32(base_reg);

        self.perform_call_jump(mem, base_addr + offset);
    }

    /// Perform a right-shift of a u32 value by a u8 value.
    ///
    /// # Arguments
    ///
    /// * `value` - The first u32 value.
    /// * `shift_by` - The value giving the number of places to shift.
    /// * `out_reg` - The destination u32 register.
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
    #[inline]
    fn perform_right_shift_u32(&mut self, value: u32, shift_by: u8, out_reg: &RegisterId) {
        if shift_by == 0 {
            return;
        }

        assert!(shift_by <= 31);

        let new_value = value >> shift_by;
        self.set_flag_state(CpuFlag::SF, utils::is_bit_set(new_value, 31));
        self.set_flag_state(CpuFlag::ZF, new_value == 0);
        self.set_flag_state(CpuFlag::OF, false);
        // The carried forward flag should be the value of the least significant bit being
        // shifted out of the value and so MUST be calculated prior to the shifted value.
        // Since we are working in little-Endian the lowest bit has the lowest index.
        self.set_flag_state(CpuFlag::CF, utils::is_bit_set(value, 0));
        self.set_flag_state(CpuFlag::PF, Cpu::calculate_lowest_byte_parity(new_value));

        self.write_reg_u32(out_reg, new_value);
    }

    /// Perform an arithmetic right-shift of two u32 values.
    ///
    /// # Arguments
    ///
    /// * `value` - The first u32 value.
    /// * `shift_by` - The value giving the number of places to shift.
    /// * `out_reg` - The destination u32 register.
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
    #[inline]
    fn perform_arithmetic_right_shift_u32(
        &mut self,
        value: u32,
        shift_by: u32,
        out_reg: &RegisterId,
    ) {
        let new_value = value.rotate_right(shift_by);
        self.set_flag_state(CpuFlag::SF, utils::is_bit_set(new_value, 31));
        self.set_flag_state(CpuFlag::ZF, new_value == 0);
        self.set_flag_state(CpuFlag::OF, false);
        self.set_flag_state(CpuFlag::CF, false);
        self.set_flag_state(CpuFlag::PF, Cpu::calculate_lowest_byte_parity(new_value));

        self.write_reg_u32(out_reg, new_value);
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
    #[inline]
    fn perform_bit_test_with_carry_flag(&mut self, value: u32, bit: u8) {
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
    #[inline]
    fn perform_bit_test_with_carry_flag_with_set(
        &mut self,
        value: &mut u32,
        bit: u8,
        new_state: bool,
    ) {
        self.perform_bit_test_with_carry_flag(*value, bit);

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
    #[inline]
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
    #[inline]
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
    ///
    ///
    /// This method affects the following flags: Sign (SF), Zero (ZF), Carry (CF) and Parity (PF).
    ///
    /// The Overflow (OF) flag will always be cleared. Any other flags are undefined.
    #[inline]
    fn perform_zero_high_bit_u32_reg(
        &mut self,
        source_reg: &RegisterId,
        index: u32,
        out_reg: &RegisterId,
    ) {
        assert!(index <= 31);

        let value = self
            .registers
            .read_reg_u32(source_reg, &self.privilege_level);

        let mut final_value: u32 = 0;
        if index > 0 {
            if is_x86_feature_detected!("bmi2") {
                // We can use the inbuilt assembly instruction here for added performance gains.
                unsafe {
                    asm!(
                        "bzhi {0:e}, {1:e}, {2:e}",
                        out(reg) final_value,
                        in(reg) value,
                        in(reg) index,
                    );
                }
            } else {
                final_value = value & ((1 << index) - 1)
            }
        }

        self.set_flag_state(CpuFlag::SF, utils::is_bit_set(final_value, 31));
        self.set_flag_state(CpuFlag::ZF, final_value == 0);

        // The carry flag is set if the if the number contained in the
        // lowest 8 bits is greater than 31 (register size - 1).
        self.set_flag_state(CpuFlag::CF, (final_value & 0xff) > 31);

        // The overflow flag is always cleared.
        self.set_flag_state(CpuFlag::OF, false);

        // Write the value to the output register.
        self.write_reg_u32(out_reg, final_value);
    }

    /// Pop the relevant processor register states from the stack.
    ///
    /// # Arguments
    ///
    /// * `mem` - The [`Memory`] connected to this CPU instance.
    /// * `arg_types` - The type of arguments passed via the stack.
    #[inline]
    pub fn pop_state(&mut self, mem: &mut MemoryHandler, arg_types: StackArgType) {
        // NOTE - remember that the states need to be popped in the reverse order!

        // First, get the current frame pointer.
        let base_pointer = self.registers.read_reg_u32_unchecked(&RegisterId::EBP);

        // Next, set the stack pointer to the current base pointer location.
        // Any stack entries higher in the stack can be disregarded as they are out of scope
        // from this point forward.
        self.write_reg_u32_unchecked(&RegisterId::ESP, base_pointer);

        // Next, pop the old stack frame size.
        let old_base_pointer = mem.pop_u32();
        mem.stack_base_pointer = old_base_pointer as usize;

        // Next, pop the f32 data registers from the stack.
        for reg in PRESERVE_REGISTERS_F32_REV {
            self.write_reg_f32_unchecked(&reg, mem.pop_f32());
        }

        // Next, pop the the u32 data registers from the stack.
        for reg in PRESERVE_REGISTERS_U32_REV {
            self.write_reg_u32_unchecked(&reg, mem.pop_u32());
        }

        // Next, clear the arguments from the stack.
        let arg_count = mem.pop_u32() as usize;
        match arg_types {
            StackArgType::F32 => mem.pop_top_f32(arg_count),
            StackArgType::U32 => mem.pop_top_u32(arg_count),
        }

        // Finally, reset the stack base pointer position to the original state.
        self.write_reg_u32_unchecked(&RegisterId::EBP, old_base_pointer);
    }

    /// Push the relevant processor register states to the stack.
    ///
    /// # Arguments
    ///
    /// * `mem` - The [`Memory`] connected to this CPU instance.
    #[inline]
    pub fn push_state(&mut self, mem: &mut MemoryHandler) {
        // First, push the u32 data registers to the stack.
        for reg in PRESERVE_REGISTERS_U32 {
            mem.push_u32(self.registers.read_reg_u32_unchecked(&reg));
        }

        // Next, push the f32 data registers to the stack.
        for reg in PRESERVE_REGISTERS_F32 {
            mem.push_f32(self.registers.read_reg_f32_unchecked(&reg));
        }

        // Next, push the current stack frame base pointer to the stack.
        mem.push_u32(mem.stack_base_pointer as u32);

        // Finally, update the stack base pointer to be the current stack pointer.
        self.write_reg_u32_unchecked(&RegisterId::EBP, self.get_stack_pointer());
    }

    /// Read the value of a specific u32 register.
    ///
    /// # Arguments
    ///
    /// * `reg` - Reference to the [`RegisterId`] for the register in question.
    #[inline(always)]
    fn read_reg_u32(&self, reg: &RegisterId) -> u32 {
        self.registers
            .get_register_u32(*reg)
            .read(&self.privilege_level)
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
    /// * `com_bus` - A mutable reference to the [`CommunicationBus`] for the virtual machine.
    pub fn run(&mut self, com_bus: &mut CommunicationBus) {
        while !self.is_halted {
            let ins = self.fetch_decode_next_instruction(com_bus);
            self.run_instruction(com_bus, &ins);
        }
    }

    /// Execute a set of instructions on the CPU.
    ///
    /// # Arguments
    ///
    /// * `com_bus` - A mutable reference to the [`CommunicationBus`] for the virtual machine.
    /// * `instructions` - A slice of [`Instruction`] instances to be executed.
    pub fn run_instructions(
        &mut self,
        com_bus: &mut CommunicationBus,
        instructions: &[Instruction],
    ) {
        for ins in instructions {
            self.run_instruction(com_bus, ins);

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
    /// * `com_bus` - A mutable reference to the [`CommunicationBus`] for the virtual machine.
    /// * `instruction` - An [`Instruction`] instance to be executed.
    fn run_instruction(&mut self, com_bus: &mut CommunicationBus, instruction: &Instruction) {
        use Instruction as I;

        // Advance the instruction pointer by the number of bytes used for the instruction.
        self.increase_ip_register_by(instruction.get_total_instruction_size());

        match instruction {
            I::Nop => {}

            /******** [Arithmetic Instructions] ********/
            I::AddU32ImmU32Reg(imm, out_reg) => {
                let old_value = self.read_reg_u32(out_reg);

                self.perform_checked_add_u32(old_value, *imm, out_reg);
            }
            I::AddU32RegU32Reg(in_reg, out_reg) => {
                let value1 = self.read_reg_u32(in_reg);
                let value2 = self.read_reg_u32(out_reg);

                self.perform_checked_add_u32(value1, value2, out_reg);
            }
            I::SubU32ImmU32Reg(imm, out_reg) => {
                let target_val = self.read_reg_u32(out_reg);

                self.perform_checked_subtract_u32(target_val, *imm, out_reg);
            }
            I::SubU32RegU32Reg(target_reg, out_reg) => {
                let sub_val = self.read_reg_u32(target_reg);
                let target_val = self.read_reg_u32(out_reg);

                self.perform_checked_subtract_u32(target_val, sub_val, out_reg);
            }
            I::MulU32Imm(imm) => {
                let old_value = self.registers.read_reg_u32_unchecked(&RegisterId::EAX);

                self.perform_checked_mul_u32(old_value, *imm);
            }
            I::MulU32Reg(reg) => {
                let reg_er1_val = self.registers.read_reg_u32_unchecked(&RegisterId::EAX);
                let reg_other_val = self.read_reg_u32(reg);

                self.perform_checked_mul_u32(reg_er1_val, reg_other_val);
            }
            I::DivU32Imm(divisor_imm) => {
                let dividend = self.registers.read_reg_u32_unchecked(&RegisterId::EAX);

                self.perform_division_u32(dividend, *divisor_imm, &mut com_bus.mem);
            }
            I::DivU32Reg(divisor_reg) => {
                let divisor = self.read_reg_u32(divisor_reg);
                let dividend = self.read_reg_u32(&RegisterId::EAX);

                self.perform_division_u32(dividend, divisor, &mut com_bus.mem);
            }
            I::IncU32Reg(reg) => {
                let value = self.read_reg_u32(reg);

                self.perform_checked_add_u32(value, 1, reg);
            }
            I::DecU32Reg(reg) => {
                let value = self.read_reg_u32(reg);

                self.perform_checked_subtract_u32(value, 1, reg);
            }
            I::AndU32ImmU32Reg(imm, reg) => {
                let value = self.read_reg_u32(reg);

                self.perform_bitwise_and_u32(*imm, value, reg);
            }

            /******** [Bit Operation Instructions] ********/
            I::LeftShiftU8ImmU32Reg(imm, reg) => {
                let value = self.read_reg_u32(reg);

                self.perform_checked_left_shift_u32(value, *imm, reg);
            }
            I::LeftShiftU32RegU32Reg(shift_reg, out_reg) => {
                let shift_by = self.read_reg_u32(shift_reg);
                let value = self.read_reg_u32(out_reg);
                self.perform_checked_left_shift_u32(value, shift_by as u8, out_reg);
            }
            I::ArithLeftShiftU8ImmU32Reg(imm, out_reg) => {
                let value = self.read_reg_u32(out_reg);

                self.perform_arithmetic_left_shift_u32(value, *imm as u32, out_reg);
            }
            I::ArithLeftShiftU32RegU32Reg(shift_reg, out_reg) => {
                let shift_by = self.read_reg_u32(shift_reg);
                let value = self.read_reg_u32(out_reg);

                self.perform_arithmetic_left_shift_u32(value, shift_by, out_reg);
            }
            I::RightShiftU8ImmU32Reg(imm, out_reg) => {
                let value = self.read_reg_u32(out_reg);

                self.perform_right_shift_u32(value, *imm, out_reg);
            }
            I::RightShiftU32RegU32Reg(shift_reg, out_reg) => {
                let shift_by = self.read_reg_u32(shift_reg);
                let value = self.read_reg_u32(out_reg);

                self.perform_right_shift_u32(value, shift_by as u8, out_reg);
            }
            I::ArithRightShiftU8ImmU32Reg(imm, out_reg) => {
                let value = self.read_reg_u32(out_reg);

                self.perform_arithmetic_right_shift_u32(value, *imm as u32, out_reg);
            }
            I::ArithRightShiftU32RegU32Reg(shift_reg, out_reg) => {
                let shift_by = self.read_reg_u32(shift_reg);
                let value = self.read_reg_u32(out_reg);

                self.perform_arithmetic_right_shift_u32(value, shift_by, out_reg);
            }

            /******** [Branching Instructions] ********/
            I::CallAbsU32Imm(addr, _) => {
                // call 0xdeafbeef
                self.perform_call_jump(&mut com_bus.mem, *addr);
            }
            I::CallAbsU32Reg(reg) => {
                // call reg
                // Get the value of the register.
                let addr = self.read_reg_u32(reg);

                self.perform_call_jump(&mut com_bus.mem, addr);
            }
            I::CallRelU32RegU32Offset(offset, base_reg) => {
                // callr 0xdeadbeef, reg
                // callr &[reg + 0xdeadbeef]
                self.perform_call_offset_jump(&mut com_bus.mem, base_reg, *offset);
            }
            I::CallRelCSU32Offset(offset, _) => {
                // call :label - calls to labels within the bounds of the program code block will automatically be converted.
                // callr 0xdeadbeef
                self.perform_call_offset_jump(&mut com_bus.mem, &RegisterId::ECS, *offset);
            }
            I::RetArgsU32 => {
                // iret
                // Restore the state of the last stack frame.
                self.pop_state(&mut com_bus.mem, StackArgType::U32);
            }
            I::Int(int_type) => {
                self.trigger_interrupt(*int_type, &mut com_bus.mem);
            }
            I::IntRet => {
                self.is_in_interrupt_handler = false;
                self.last_interrupt_code = None;

                self.set_instruction_pointer(com_bus.mem.pop_u32());
                self.set_flags(com_bus.mem.pop_u32());
            }
            I::JumpAbsU32Imm(addr) => {
                // jmp 0xaaaa
                // jmp :label
                // Set the instruction pointer to the jump address.
                self.set_instruction_pointer(*addr);
            }
            I::JumpAbsU32Reg(reg) => {
                // jmp cs
                let addr = self.read_reg_u32(reg);

                // Set the instruction pointer to the jump address.
                self.set_instruction_pointer(addr);
            }

            /******** [Data Instructions] ********/
            I::SwapU32RegU32Reg(reg_1, reg_2) => {
                let reg_1_val = self.read_reg_u32(reg_1);
                let reg_2_val = self.read_reg_u32(reg_2);

                self.write_reg_u32(reg_1, reg_2_val);
                self.write_reg_u32(reg_2, reg_1_val);
            }
            I::MovU32ImmU32Reg(imm, reg) => {
                self.write_reg_u32(reg, *imm);
            }
            I::MovU32RegU32Reg(in_reg, out_reg) => {
                let value = self.read_reg_u32(in_reg);

                self.write_reg_u32(out_reg, value);
            }
            I::MovU32ImmMemSimple(imm, addr) => {
                com_bus.mem.set_u32(*addr as usize, *imm);
            }
            I::MovU32RegMemSimple(reg, addr) => {
                let value = self.read_reg_u32(reg);

                com_bus.mem.set_u32(*addr as usize, value);
            }
            I::MovMemU32RegSimple(addr, reg) => {
                let value = com_bus.mem.get_u32(*addr as usize);

                self.write_reg_u32(reg, value);
            }
            I::MovU32RegPtrU32RegSimple(in_reg, out_reg) => {
                let address = self.read_reg_u32(in_reg) as usize;
                let value = com_bus.mem.get_u32(address);

                self.write_reg_u32(out_reg, value);
            }
            I::MovU32ImmMemExpr(imm, expr) => {
                // mov imm, &[addr] - move immediate to address.
                let addr = self.decode_evaluate_u32_expression(expr);

                com_bus.mem.set_u32(addr as usize, *imm);
            }
            I::MovMemExprU32Reg(expr, reg) => {
                // mov &[addr], register - move value at address to register.
                let addr = self.decode_evaluate_u32_expression(expr);
                let value = com_bus.mem.get_u32(addr as usize);

                self.write_reg_u32(reg, value);
            }
            I::MovU32RegMemExpr(reg, expr) => {
                // mov &reg, &[addr] - move value of a register to an address.
                let addr = self.decode_evaluate_u32_expression(expr);
                let value = self.read_reg_u32(reg);

                com_bus.mem.set_u32(addr as usize, value);
            }
            I::ZeroHighBitsByIndexU32Reg(index_reg, source_reg, out_reg) => {
                // zhbi source_reg, index_reg, out_reg
                let index = self.read_reg_u32(index_reg);
                self.perform_zero_high_bit_u32_reg(source_reg, index, out_reg);
            }
            I::ZeroHighBitsByIndexU32RegU32Imm(index, source_reg, out_reg) => {
                // zhbi source_reg, index, out_reg
                self.perform_zero_high_bit_u32_reg(source_reg, *index, out_reg);
            }
            I::PushF32Imm(imm) => {
                // push imm
                com_bus.mem.push_f32(*imm);

                self.decrease_sp_register_by(SIZE_OF_F32_LOCAL);
            }
            I::PushU32Imm(imm) => {
                // push imm
                com_bus.mem.push_u32(*imm);

                self.decrease_sp_register_by(SIZE_OF_U32_LOCAL);
            }
            I::PushU32Reg(reg) => {
                // push reg
                let value = self.read_reg_u32(reg);
                com_bus.mem.push_u32(value);

                self.decrease_sp_register_by(SIZE_OF_U32_LOCAL);
            }
            I::PopF32ToF32Reg(reg) => {
                // pop reg
                self.write_reg_f32(reg, com_bus.mem.pop_f32());

                self.increase_sp_register_by(SIZE_OF_F32_LOCAL);
            }
            I::PopU32ToU32Reg(reg) => {
                // pop reg
                self.write_reg_u32(reg, com_bus.mem.pop_u32());

                self.increase_sp_register_by(SIZE_OF_U32_LOCAL);
            }

            /******** [IO Instructions] ********/
            I::OutF32Imm(value, port) => {
                // out 0.1, 0xff
                if com_bus.write_f32(*value, *port).is_err() {
                    self.trigger_interrupt(DEVICE_ERROR_INT, &mut com_bus.mem);
                }
            }
            I::OutU32Imm(value, port) => {
                // out 0xdeadbeef, 0xff
                if com_bus.write_u32(*value, *port).is_err() {
                    self.trigger_interrupt(DEVICE_ERROR_INT, &mut com_bus.mem);
                }
            }
            I::OutU32Reg(reg, port) => {
                // out reg, 0xff
                let value = self.read_reg_u32(reg);
                if com_bus.write_u32(value, *port).is_err() {
                    self.trigger_interrupt(DEVICE_ERROR_INT, &mut com_bus.mem);
                }
            }
            I::OutU8Imm(value, port) => {
                // out 0xff, 0xff
                if com_bus.write_u8(*value, *port).is_err() {
                    self.trigger_interrupt(DEVICE_ERROR_INT, &mut com_bus.mem);
                }
            }
            I::InU8Reg(port, reg) => {
                // in 0xff, reg
                if let Ok(val) = com_bus.read_u8(*port) {
                    self.write_reg_u32(reg, val as u32);
                } else {
                    self.trigger_interrupt(DEVICE_ERROR_INT, &mut com_bus.mem);
                }
            }
            I::InU8Mem(port, addr) => {
                // in 0xff, &[addr]
                if let Ok(val) = com_bus.read_u8(*port) {
                    com_bus.mem.set(*addr as usize, val);
                } else {
                    self.trigger_interrupt(DEVICE_ERROR_INT, &mut com_bus.mem);
                }
            }
            I::InU32Reg(port, reg) => {
                // in 0xff, reg
                if let Ok(val) = com_bus.read_u32(*port) {
                    self.write_reg_u32(reg, val);
                } else {
                    self.trigger_interrupt(DEVICE_ERROR_INT, &mut com_bus.mem);
                }
            }
            I::InU32Mem(port, addr) => {
                // in 0xff, &[addr]
                if let Ok(val) = com_bus.read_u32(*port) {
                    com_bus.mem.set_u32(*addr as usize, val);
                } else {
                    self.trigger_interrupt(DEVICE_ERROR_INT, &mut com_bus.mem);
                }
            }
            I::InF32Reg(port, reg) => {
                // in 0xff, reg
                if let Ok(val) = com_bus.read_f32(*port) {
                    self.write_reg_f32(reg, val);
                } else {
                    self.trigger_interrupt(DEVICE_ERROR_INT, &mut com_bus.mem);
                }
            }
            I::InF32Mem(port, addr) => {
                // in 0xff, &[addr]
                if let Ok(val) = com_bus.read_f32(*port) {
                    com_bus.mem.set_f32(*addr as usize, val);
                } else {
                    self.trigger_interrupt(DEVICE_ERROR_INT, &mut com_bus.mem);
                }
            }

            /******** [Logic Instructions] ********/
            I::BitTestU32Reg(bit, reg) => {
                // bt bit, reg
                let value = self.read_reg_u32(reg);
                self.perform_bit_test_with_carry_flag(value, *bit % 32);
            }
            I::BitTestU32Mem(bit, addr) => {
                // bt bit, &[addr]
                let value = com_bus.mem.get_u32(*addr as usize);
                self.perform_bit_test_with_carry_flag(value, *bit % 32);
            }
            I::BitTestResetU32Reg(bit, reg) => {
                // btr bit, reg
                // Read the value and set the carry flag state, clear the bit.
                let mut value = self.read_reg_u32(reg);
                self.perform_bit_test_with_carry_flag_with_set(&mut value, *bit % 32, false);

                // Write the value back to the register.
                self.write_reg_u32(reg, value);
            }
            I::BitTestResetU32Mem(bit, addr) => {
                // btr bit, &[addr]
                // Read the value and set the carry flag state, then clear the bit.
                let mut value = com_bus.mem.get_u32(*addr as usize);
                self.perform_bit_test_with_carry_flag_with_set(&mut value, *bit % 32, false);

                com_bus.mem.set_u32(*addr as usize, value);
            }
            I::BitTestSetU32Reg(bit, reg) => {
                // bts bit, reg
                // Read the value and set the carry flag state, set the bit.
                let mut value = self.read_reg_u32(reg);
                self.perform_bit_test_with_carry_flag_with_set(&mut value, *bit % 32, true);

                // Write the value back to the register.
                self.write_reg_u32(reg, value);
            }
            I::BitTestSetU32Mem(bit, addr) => {
                // bts bit, &[addr]
                // Read the value and set the carry flag state, then set the bit.
                let mut value = com_bus.mem.get_u32(*addr as usize);
                self.perform_bit_test_with_carry_flag_with_set(&mut value, *bit % 32, true);

                com_bus.mem.set_u32(*addr as usize, value);
            }
            I::BitScanReverseU32RegU32Reg(in_reg, out_reg) => {
                // bsr in_reg, out_reg
                let value = self.read_reg_u32(in_reg);
                let index = self.perform_reverse_bit_search(value);

                self.write_reg_u32(out_reg, index);
            }
            I::BitScanReverseU32MemU32Reg(addr, reg) => {
                // bsr &[addr], reg
                let value = com_bus.mem.get_u32(*addr as usize);
                let index = self.perform_reverse_bit_search(value);

                self.write_reg_u32(reg, index);
            }
            I::BitScanReverseU32RegMemU32(reg, out_addr) => {
                // bsr reg, &[out_addr]
                let value = self.read_reg_u32(reg);
                let index = self.perform_reverse_bit_search(value);

                com_bus.mem.set_u32(*out_addr as usize, index);
            }
            I::BitScanReverseU32MemU32Mem(in_addr, out_addr) => {
                // bsr &[in_addr], &[out_addr]
                let value = com_bus.mem.get_u32(*in_addr as usize);
                let index = self.perform_reverse_bit_search(value);

                com_bus.mem.set_u32(*out_addr as usize, index);
            }
            I::BitScanForwardU32RegU32Reg(in_reg, out_reg) => {
                // bsf in_reg, out_reg
                let value = self.read_reg_u32(in_reg);
                let index = self.perform_forward_bit_search(value);

                self.write_reg_u32(out_reg, index);
            }
            I::BitScanForwardU32MemU32Reg(addr, reg) => {
                // bsf &[addr], reg
                let value = com_bus.mem.get_u32(*addr as usize);
                let index = self.perform_forward_bit_search(value);

                self.write_reg_u32(reg, index);
            }
            I::BitScanForwardU32RegMemU32(reg, out_addr) => {
                // bsf reg, &[out_addr]
                let value = self.read_reg_u32(reg);
                let index = self.perform_forward_bit_search(value);

                com_bus.mem.set_u32(*out_addr as usize, index);
            }
            I::BitScanForwardU32MemU32Mem(in_addr, out_addr) => {
                // bsf &[in_addr], &[out_addr]
                let value = com_bus.mem.get_u32(*in_addr as usize);
                let index = self.perform_forward_bit_search(value);

                com_bus.mem.set_u32(*out_addr as usize, index);
            }
            I::ByteSwapU32(reg) => {
                // bswap reg
                let value = self.read_reg_u32(reg).swap_bytes();

                self.write_reg_u32(reg, value);
            }

            /******** [Special Instructions] ********/
            I::MaskInterrupt(int_code) => {
                let im = self.registers.get_register_u32_mut(RegisterId::EIM);
                let new_mask = im.read_unchecked() & !*int_code as u32;
                im.write_unchecked(new_mask);
            }
            I::UnmaskInterrupt(int_code) => {
                let im = self.registers.get_register_u32_mut(RegisterId::EIM);
                let new_mask = (im.read_unchecked() & NOT_U32_LOW_BYTE_MASK) | *int_code as u32;
                im.write_unchecked(new_mask);
            }
            I::LoadIVTAddrU32Imm(addr) => {
                // livt &addr
                if !self.is_in_machine_mode() {
                    self.trigger_interrupt(GENERAL_PROTECTION_FAULT_INT, &mut com_bus.mem);
                    return;
                }

                self.write_reg_u32_unchecked(&RegisterId::IDTR, *addr);
            }
            I::MachineReturn => {
                self.set_privilege_level(PrivilegeLevel::User);
            }
            I::Halt => {
                self.set_halted(true);
            }

            /******** [Pseudo Instructions] ********/
            I::Label(_) => {
                self.trigger_interrupt(INVALID_OPCODE_INT, &mut com_bus.mem);
            }
            I::Unknown(_id) => {
                self.trigger_interrupt(INVALID_OPCODE_INT, &mut com_bus.mem);
            }
        };
    }

    /// Update the flags register.
    ///
    /// # Arguments
    ///
    /// * `value` - The new value of the flags register.
    #[inline(always)]
    pub fn set_flags(&mut self, value: u32) {
        self.write_reg_u32_unchecked(&RegisterId::EFL, value);
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
        let flags = utils::set_bit_state(register.read_unchecked(), flag.into(), state);
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
        self.write_reg_u32_unchecked(&RegisterId::EIP, value);
    }

    /// Set the privilege level of the CPU.
    ///
    /// # Arguments
    ///
    /// * `level` - The new privilege level of the CPU.
    #[inline(always)]
    fn set_privilege_level(&mut self, level: PrivilegeLevel) {
        self.privilege_level = level;
    }

    /// Update the various segment registers in the CPU.
    ///
    /// # Arguments
    ///
    /// * `mem` - A reference to the [`MemoryHandler`] connected to this CPU instance.
    #[inline(always)]
    pub fn set_segment_registers(&mut self, mem: &MemoryHandler) {
        self.write_reg_u32_unchecked(&RegisterId::ESS, mem.stack_segment_start as u32);
        self.write_reg_u32_unchecked(&RegisterId::ECS, mem.code_segment_start as u32);
        self.write_reg_u32_unchecked(&RegisterId::EDS, mem.data_segment_start as u32);
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

    // Update the stack pointer (SP) register.
    ///
    /// # Arguments
    ///
    /// * `value` - The new value of the stack pointer register.
    #[inline(always)]
    pub fn set_stack_pointer(&mut self, value: u32) {
        self.write_reg_u32_unchecked(&RegisterId::ESP, value);
    }

    /// Trigger and handle an interrupt.
    ///
    /// # Arguments
    ///
    /// * `int_code` - The type of interrupt to be triggered.
    /// * `mem` - A mutable reference to the [`MemoryHandler`] connected to this CPU instance.
    #[inline]
    fn trigger_interrupt(&mut self, int_code: u8, mem: &mut MemoryHandler) {
        let is_masked =
            (self.registers.read_reg_u32_unchecked(&RegisterId::EIM) & (int_code as u32)) == 0;

        // Should we handle this interrupt?
        // Exception interrupts have an interrupt code below 0x20 and are
        // considered important enough that they can't be masked.
        if is_masked && int_code >= EXCEPTION_INT_CUTOFF {
            return;
        }

        // Get the base IVT address.
        let ivt_base = self.registers.read_reg_u32_unchecked(&RegisterId::IDTR);

        // Calculate the place within the interrupt vector we need to look.
        let iv_addr = ivt_base + (int_code as u32 * 4);

        // Get the handler address from the interrupt vector.
        let iv_handler_addr = mem.get_u32(iv_addr as usize);
        if iv_handler_addr == 0 {
            // TODO - there might be something else we want to do here instead.
            panic!("no handler has been specified for this interrupt code!");
        }

        // We will only save state when we are not already in an interrupt handler.
        if !self.is_in_interrupt_handler {
            // Want to preserve the state of the flags register.
            mem.push_u32(self.get_flags());

            // Once the interrupt handler has completed then we will jump back to the
            // instruction directly following the one that triggered the exception.
            // The return address will be pushed to the stack.
            mem.push_u32(self.get_instruction_pointer());
        }

        // Indicate that we are currently within an interrupt handler.
        self.is_in_interrupt_handler = true;

        // Indicate that we have an unfinished interrupt.
        self.last_interrupt_code = Some(int_code);

        // Jump to the interrupt vector handler.
        self.set_instruction_pointer(iv_handler_addr);
    }

    /// Set the value of a specific f32 register.
    ///
    /// # Arguments
    ///
    /// * `reg` - Reference to the [`RegisterId`] for the register in question.
    /// * `new_val` - The new value of the register.
    #[inline(always)]
    fn write_reg_f32(&mut self, reg: &RegisterId, new_val: f32) {
        self.registers
            .get_register_f32_mut(*reg)
            .write(new_val, &self.privilege_level);
    }

    /// Set the value of a specific f32 register, without privilege checking.
    ///
    /// # Arguments
    ///
    /// * `reg` - Reference to the [`RegisterId`] for the register in question.
    /// * `new_val` - The new value of the register.
    #[inline(always)]
    fn write_reg_f32_unchecked(&mut self, reg: &RegisterId, new_val: f32) {
        self.registers
            .get_register_f32_mut(*reg)
            .write_unchecked(new_val);
    }

    /// Set the value of a specific u32 register.
    ///
    /// # Arguments
    ///
    /// * `reg` - Reference to the [`RegisterId`] for the register in question.
    /// * `new_val` - The new value of the register.
    #[inline(always)]
    fn write_reg_u32(&mut self, reg: &RegisterId, new_val: u32) {
        self.registers
            .get_register_u32_mut(*reg)
            .write(new_val, &self.privilege_level);
    }

    /// Set the value of a specific u32 register, without privilege checking.
    ///
    /// # Arguments
    ///
    /// * `reg` - Reference to the [`RegisterId`] for the register in question.
    /// * `new_val` - The new value of the register.
    #[inline(always)]
    fn write_reg_u32_unchecked(&mut self, reg: &RegisterId, new_val: u32) {
        self.registers
            .get_register_u32_mut(*reg)
            .write_unchecked(new_val);
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
}

impl fmt::Display for CpuFlag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CpuFlag::SF => write!(f, "SF"),
            CpuFlag::ZF => write!(f, "ZF"),
            CpuFlag::OF => write!(f, "OF"),
            CpuFlag::CF => write!(f, "CF"),
            CpuFlag::PF => write!(f, "PF"),
        }
    }
}

impl CpuFlag {
    pub fn iterator() -> Iter<'static, CpuFlag> {
        static FLAGS: [CpuFlag; 5] = [
            CpuFlag::SF,
            CpuFlag::ZF,
            CpuFlag::OF,
            CpuFlag::CF,
            CpuFlag::PF,
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
    use hashbrown::HashMap;
    use prettytable::{row, Table};
    use std::panic;

    use crate::{
        com_bus::communication_bus::TEST_DEBUG_DEVICE_ID,
        compiling::compiler::Compiler,
        cpu::{DEVICE_ERROR_INT, DIVIDE_BY_ZERO_INT, GENERAL_PROTECTION_FAULT_INT},
        ins::{
            expression::{Expression, ExpressionArgs, ExpressionOperator},
            instruction::Instruction::{self, *},
        },
        mem,
        reg::registers::RegisterId,
        vm::{self, VirtualMachine},
    };

    use super::{Cpu, CpuFlag};

    /// The default stack capacity (in terms of u32 values) for tests.
    const DEFAULT_U32_STACK_CAPACITY: usize = 30;

    trait Test {
        /// Run this specific test entry.
        ///
        /// # Arguments
        ///
        /// * `id` - The ID of this test.
        ///
        /// # Returns
        ///
        /// An option containing the [`VirtualMachine`] instance if the execution did not panic, or None otherwise.
        fn run_test(&self, id: usize) -> Option<VirtualMachine>;
    }

    struct CpuTests<T: Test + Clone> {
        tests: Vec<T>,
    }

    impl<T: Test + Clone> CpuTests<T> {
        pub fn new(tests: &[T]) -> Self {
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
    struct TestF32 {
        /// A vector of [`Instruction`]s to be executed.
        pub instructions: Vec<Instruction>,
        /// A [`HashMap`] containing [`RegisterId`] and the expected value of the register after execution.
        pub expected_changed_registers: HashMap<RegisterId, f32>,
        /// The capacity of the stack memory segment, in bytes.
        pub stack_seg_capacity_u32: usize,
        /// A boolean indicating whether the test should panic or not.
        pub should_panic: bool,
        /// A string slice that provides the message to be displayed if the test fails.
        pub fail_message: String,
    }

    impl TestF32 {
        /// Create a new [`TestF32`] instance.
        ///
        /// # Arguments
        ///
        /// * `instructions` - A slice of [`Instruction`]s to be executed.
        /// * `expected_registers` - A slice of a tuple containing the [`RegisterId`] and the expected value of the register after execution.
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
            expected_registers: &[(RegisterId, f32)],
            stack_seg_capacity_u32: usize,
            should_panic: bool,
            fail_message: &str,
        ) -> Self {
            // Ensure we always end with a halt instruction.
            let mut instructions_vec = instructions.to_vec();
            if let Some(ins) = instructions_vec.last() {
                if !matches!(ins, Halt) {
                    instructions_vec.push(Halt);
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
                stack_seg_capacity_u32,
                should_panic,
                fail_message: fail_message.to_string(),
            }
        }

        fn print_register_differences(&self, id: usize, vm: &VirtualMachine) {
            let mut table = Table::new();
            table.add_row(row!["Register", "Expected Value", "Actual Value"]);

            // Did the test registers match their expected values?
            self.expected_changed_registers
                .iter()
                .for_each(|(id, &expected_value)| {
                    let actual_value = vm.cpu.registers.read_reg_f32_unchecked(id);
                    if expected_value != actual_value {
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

    impl Test for TestF32 {
        fn run_test(&self, id: usize) -> Option<VirtualMachine> {
            // Compile the test code.
            let mut compiler = Compiler::new();
            let compiled = compiler.compile(&self.instructions);

            // Attempt to execute the code.
            let result = panic::catch_unwind(|| {
                // Build the virtual machine instance.
                let mut vm = VirtualMachine::new(
                    vm::MIN_USER_SEGMENT_SIZE,
                    compiled,
                    &[],
                    self.stack_seg_capacity_u32 * mem::memory_handler::SIZE_OF_U32,
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
            self.print_register_differences(id, &vm);

            Some(vm)
        }
    }

    #[derive(Clone)]
    struct TestU32 {
        /// A vector of [`Instruction`]s to be executed.
        pub instructions: Vec<Instruction>,
        /// A [`HashMap`] containing [`RegisterId`] and the expected value of the register after execution.
        pub expected_changed_registers: HashMap<RegisterId, u32>,
        /// The capacity of the stack memory segment, in bytes.
        pub stack_seg_capacity_u32: usize,
        /// A boolean indicating whether the test should panic or not.
        pub should_panic: bool,
        /// A string slice that provides the message to be displayed if the test fails.
        pub fail_message: String,
    }

    impl TestU32 {
        /// Create a new [`TestU32`] instance.
        ///
        /// # Arguments
        ///
        /// * `instructions` - A slice of [`Instruction`]s to be executed.
        /// * `expected_registers` - A slice of a tuple containing the [`RegisterId`] and the expected value of the register after execution.
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
            stack_seg_capacity_u32: usize,
            should_panic: bool,
            fail_message: &str,
        ) -> Self {
            // Ensure we always end with a halt instruction.
            let mut instructions_vec = instructions.to_vec();
            if let Some(ins) = instructions_vec.last() {
                if !matches!(ins, Halt) {
                    instructions_vec.push(Halt);
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
                stack_seg_capacity_u32,
                should_panic,
                fail_message: fail_message.to_string(),
            }
        }

        fn print_register_differences(&self, id: usize, vm: &VirtualMachine) {
            let mut table = Table::new();
            table.add_row(row!["Register", "Expected Value", "Actual Value"]);

            // Did the test registers match their expected values?
            self.expected_changed_registers
                .iter()
                .for_each(|(id, &expected_value)| {
                    let actual_value = vm.cpu.registers.read_reg_u32_unchecked(id);
                    if expected_value != actual_value {
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

    impl Test for TestU32 {
        fn run_test(&self, id: usize) -> Option<VirtualMachine> {
            // Compile the test code.
            let mut compiler = Compiler::new();
            let compiled = compiler.compile(&self.instructions);

            // Attempt to execute the code.
            let result = panic::catch_unwind(|| {
                // Build the virtual machine instance.
                let mut vm = VirtualMachine::new(
                    vm::MIN_USER_SEGMENT_SIZE,
                    compiled,
                    &[],
                    self.stack_seg_capacity_u32 * mem::memory_handler::SIZE_OF_U32,
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
            self.print_register_differences(id, &vm);

            Some(vm)
        }
    }

    /// Test writing to a port that is unused.
    #[test]
    fn test_writing_to_unused_port() {
        let tests = [TestU32::new(
            &[OutU8Imm(0x0, 0xfd)],
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "OUT - successfully executed OUT instruction with an unused port",
        )];

        CpuTests::new(&tests).run_all_special(|id, vm| {
            if let Some(v) = vm {
                // The CPU should still be within an interrupt handler.
                assert!(v.cpu.is_in_interrupt_handler);
                assert_eq!(v.cpu.last_interrupt_code, Some(DEVICE_ERROR_INT));
            } else {
                panic!("failed to correctly execute test id {id}");
            }
        });
    }

    /// Test writing to a valid port with a valid data types.
    #[test]
    fn test_writing_port_valid_data_types() {
        let tests = [TestU32::new(
            &[OutU8Imm(0x0, TEST_DEBUG_DEVICE_ID), OutU32Imm(0x0, TEST_DEBUG_DEVICE_ID)],
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "OUT - failed to successfully execute OUT instruction with a valid port and data type",
        ),
        TestU32::new(
            &[OutU32Reg(RegisterId::EAX, TEST_DEBUG_DEVICE_ID)],
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "OUT - failed to successfully execute OUT instruction with a valid port and data type",
        )];

        CpuTests::new(&tests).run_all();
    }

    /// Test writing to a valid port with an unsupported data type.
    #[test]
    fn test_writing_to_port_invalid_data_types() {
        let tests = [TestU32::new(
            &[OutF32Imm(0.1, TEST_DEBUG_DEVICE_ID)],
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "OUT - successfully executed OUT instruction with unsupported data type",
        )];

        CpuTests::new(&tests).run_all_special(|id, vm| {
            if let Some(v) = vm {
                // The CPU should still be within an interrupt handler.
                assert!(v.cpu.is_in_interrupt_handler);
                assert_eq!(v.cpu.last_interrupt_code, Some(DEVICE_ERROR_INT));
            } else {
                panic!("failed to correctly execute test id {id}");
            }
        });
    }

    /// Test load interrupt vector table instruction fails in user mode.
    #[test]
    fn test_livt_fails_user_mode() {
        let tests = [TestU32::new(
            &[MachineReturn, LoadIVTAddrU32Imm(0xdeafbeef)],
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "LIVT - successfully executed instruction in user mode",
        )];

        CpuTests::new(&tests).run_all_special(|id, vm| {
            if let Some(v) = vm {
                // The CPU should still be within an interrupt handler.
                assert!(v.cpu.is_in_interrupt_handler);
                assert_eq!(
                    v.cpu.last_interrupt_code,
                    Some(GENERAL_PROTECTION_FAULT_INT)
                );
            } else {
                panic!("failed to correctly execute test id {id}");
            }
        });
    }

    /// Test load interrupt vector table instruction successfully changes the interrupt vector table address.
    #[test]
    fn test_livt_successful_update() {
        let instructions = &[
            // Move the address of the division by zero interrupt handler
            // to the new IVT location. We don't care about the rest here.
            MovMemU32RegSimple(0x0, RegisterId::EAX),
            MovU32RegMemSimple(RegisterId::EAX, 0x1000),
            // Erase the old handler address to ensure it can't be used.
            MovU32ImmMemSimple(0x0, 0x0),
            // Load the new IVT address.
            Instruction::LoadIVTAddrU32Imm(0x1000),
            // Trigger the division by zero interrupt.
            Int(0),
        ];

        let tests = [TestU32::new(
            instructions,
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "LIVT - failed to successfully execute LIVT instruction",
        )];

        CpuTests::new(&tests).run_all_special(|id, vm| {
            if let Some(v) = vm {
                // The CPU should still be within an interrupt handler.
                assert!(v.cpu.is_in_interrupt_handler);
                assert_eq!(v.cpu.last_interrupt_code, Some(DIVIDE_BY_ZERO_INT));
            } else {
                panic!("failed to correctly execute test id {id}");
            }
        });
    }

    /// Test whether a non-masked interrupt executes correctly.
    #[test]
    fn test_non_masked_interrupt_no_execution() {
        let base_offset = vm::MIN_USER_SEGMENT_SIZE as u32;

        let instructions = &[
            // Write the handler address into the IVT.
            Instruction::MovU32ImmMemSimple(base_offset + 35, 255 * 4), // Starts at [base]. Length = 12.
            // Mask the interrupt.
            Instruction::MaskInterrupt(0xff), // Starts at [base + 12]. Length = 5.
            Instruction::Int(0xff),           // Starts at [base + 17]. Length = 5.
            Instruction::AddU32ImmU32Reg(0x5, RegisterId::EAX), // Starts at [base + 22]. Length = 9.
            Instruction::Halt, // Starts at [base + 31]. Length = 4.
            /***** INT_FF - Interrupt handler for interrupt 0xff starts here. *****/
            Instruction::MovU32ImmU32Reg(0x64, RegisterId::EAX), // Starts at [base + 35]. Length = 9.
            Instruction::IntRet,
        ];

        let tests = [TestU32::new(
            instructions,
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "INT - failed to successfully execute INT instruction",
        )];

        CpuTests::new(&tests).run_all_special(|id, vm| {
            if let Some(v) = vm {
                // The interrupt handler should not be executed and so the value of 5 should be added
                // to the value of the register ER1 (which is zero). Therefore, 5 should be moved to
                // accumulator register.
                assert_eq!(
                    v.cpu
                        .registers
                        .get_register_u32(RegisterId::EAX)
                        .read_unchecked(),
                    0x5
                );

                // The CPU should not still be within an interrupt handler.
                assert!(!v.cpu.is_in_interrupt_handler);

                // There should be no indication of the last interrupt handler, since it
                // should have completed.
                assert_eq!(v.cpu.last_interrupt_code, None);
            } else {
                panic!("failed to correctly execute test id {id}");
            }
        });
    }

    /// Test whether a non-masked interrupt executes correctly.
    #[test]
    fn test_unmasked_interrupt_executes() {
        let base_offset = vm::MIN_USER_SEGMENT_SIZE as u32;

        let instructions = &[
            // Write the handler address into the IVT.
            Instruction::MovU32ImmMemSimple(base_offset + 33, 255 * 4), // Starts at [base]. Length = 10.
            // Mask every interrupt.
            Instruction::MovU32ImmU32Reg(0, RegisterId::EIM), // Starts at [base + 10]. Length = 7.
            // And now unmask the specific interrupt we want to enable.
            Instruction::UnmaskInterrupt(0xff), // Starts at [base + 17]. Length = 4.
            Instruction::Int(0xff),             // Starts at [base + 21]. Length = 3.
            Instruction::AddU32ImmU32Reg(0x5, RegisterId::EAX), // Starts at [base + 24]. Length = 7.
            Instruction::Halt, // Starts at [base + 31]. Length = 2.
            /***** INT_FF - Interrupt handler for interrupt 0xff starts here. *****/
            Instruction::MovU32ImmU32Reg(0x64, RegisterId::EAX), // Starts at [base + 33].
            Instruction::IntRet,
        ];

        let tests = [TestU32::new(
            instructions,
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "INT - failed to successfully execute INT instruction",
        )];

        CpuTests::new(&tests).run_all_special(|id, vm| {
            if let Some(v) = vm {
                // The interrupt handler should not be executed and so the value of 5 should be added
                // to the value of the register ER1 (which is zero). Therefore, 5 should be moved to
                // accumulator register.
                assert_eq!(
                    v.cpu
                        .registers
                        .get_register_u32(RegisterId::EAX)
                        .read_unchecked(),
                    0x69
                );

                // The CPU should not still be within an interrupt handler.
                assert!(!v.cpu.is_in_interrupt_handler);

                // There should be no indication of the last interrupt handler, since it
                // should have completed.
                assert_eq!(v.cpu.last_interrupt_code, None);
            } else {
                panic!("failed to correctly execute test id {id}");
            }
        });
    }

    /// Test whether a masking an interrupt ensures it doesn't execute.
    #[test]
    fn test_masked_interrupt_non_execute() {
        let base_offset = vm::MIN_USER_SEGMENT_SIZE as u32;

        let instructions = &[
            // Write the handler address into the IVT.
            Instruction::MovU32ImmMemSimple(base_offset + 35, 255 * 4), // Starts at [base]. Length = 12.
            Instruction::MaskInterrupt(0xff), // Starts at [base + 12]. Length = 5.
            Instruction::Int(0xff),           // Starts at [base + 17]. Length = 5.
            Instruction::AddU32ImmU32Reg(0x5, RegisterId::EAX), // Starts at [base + 22]. Length = 9.
            Instruction::Halt, // Starts at [base + 31]. Length = 4.
            /***** INT_FF - Interrupt handler for interrupt 0xff starts here. *****/
            Instruction::MovU32ImmU32Reg(0x64, RegisterId::EAX), // Starts at [base + 35]. Length = 9.
            Instruction::IntRet,
        ];

        let tests = [TestU32::new(
            instructions,
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "INT - failed to successfully execute INT instruction",
        )];

        CpuTests::new(&tests).run_all_special(|id, vm| {
            if let Some(v) = vm {
                // The interrupt handler should set the value of the ER1 register to 0x64.
                // After returning, 0x5 should be added to the value of ER1, the result moved to the accumulator.
                assert_eq!(
                    v.cpu
                        .registers
                        .get_register_u32(RegisterId::EAX)
                        .read_unchecked(),
                    0x5
                );

                // The CPU should not still be within an interrupt handler.
                assert!(!v.cpu.is_in_interrupt_handler);

                // There should be no indication of the last interrupt handler, since it
                // should have completed.
                assert_eq!(v.cpu.last_interrupt_code, None);
            } else {
                panic!("failed to correctly execute test id {id}");
            }
        });
    }

    /// Test the call and return from ECS register with offset instruction.
    #[test]
    fn test_call_return_from_cs_with_offset() {
        let instructions = &[
            Instruction::PushU32Imm(3), // Starts at [ECS]. Length = 5. This should remain in place.
            Instruction::PushU32Imm(2), // Starts at [ECS + 5]. Length = 5. Subroutine argument 1.
            Instruction::PushU32Imm(1), // Starts at [ECS + 10]. Length = 5. The number of arguments.
            Instruction::CallRelCSU32Offset(30, String::default()), // Starts at [ECS + 15]. Length = 6.
            Instruction::AddU32ImmU32Reg(100, RegisterId::EAX), // Starts at [ECS + 21]. Length = 7.
            Instruction::Halt,                                  // Starts at [ECS + 28]. Length = 2.
            /***** FUNC_AAAA - Subroutine starts here. *****/
            Instruction::PushU32Imm(5), // Starts at [ECS + 30].
            Instruction::PopU32ToU32Reg(RegisterId::EAX),
            Instruction::RetArgsU32,
        ];

        let tests = [TestU32::new(
            instructions,
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "CALLCS - failed to successfully execute CALLCS instruction",
        )];

        CpuTests::new(&tests).run_all_special(|id, vm| {
            if let Some(mut v) = vm {
                // The subroutine should set the value of the ER1 register to 5.
                // After returning, 100 should be added to the value of ER1, the result moved to the accumulator.
                assert_eq!(
                    v.cpu
                        .registers
                        .get_register_u32(RegisterId::EAX)
                        .read_unchecked(),
                    105
                );

                // There should be one entry on the stack.
                assert_eq!(v.com_bus.mem.pop_u32(), 3);

                // The stack should now be empty.
                assert_eq!(v.com_bus.mem.get_stack_frame_size(), 0);
            } else {
                panic!("failed to correctly execute test id {id}");
            }
        });
    }

    /// Test the call and return with offset instruction.
    #[test]
    fn test_call_return_with_offset_reg() {
        let instructions = &[
            Instruction::PushU32Imm(3), // Starts at [ECS]. Length = 5. This should remain in place.
            Instruction::PushU32Imm(2), // Starts at [ECS + 5]. Length = 5. Subroutine argument 1.
            Instruction::PushU32Imm(1), // Starts at [ECS + 10]. Length = 5. The number of arguments.
            Instruction::CallRelU32RegU32Offset(31, RegisterId::ECS), // Starts at [ECS + 15]. Length = 7.
            Instruction::AddU32ImmU32Reg(100, RegisterId::EAX), // Starts at [ECS + 22]. Length = 7.
            Instruction::Halt,                                  // Starts at [ECS + 29]. Length = 2.
            /***** FUNC_AAAA - Subroutine starts here. *****/
            Instruction::PushU32Imm(5), // Starts at [ECS + 31].
            Instruction::PopU32ToU32Reg(RegisterId::EAX),
            Instruction::RetArgsU32,
        ];

        let tests = [TestU32::new(
            instructions,
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "CALL - failed to successfully execute CALL instruction",
        )];

        CpuTests::new(&tests).run_all_special(|id, vm| {
            if let Some(mut v) = vm {
                // The subroutine should set the value of the ER1 register to 5.
                // After returning, 100 should be added to the value of ER1, the result moved to the accumulator.
                assert_eq!(
                    v.cpu
                        .registers
                        .get_register_u32(RegisterId::EAX)
                        .read_unchecked(),
                    105
                );

                // There should be one entry on the stack.
                assert_eq!(v.com_bus.mem.pop_u32(), 3);

                // The stack should now be empty.
                assert_eq!(v.com_bus.mem.get_stack_frame_size(), 0);
            } else {
                panic!("failed to correctly execute test id {id}");
            }
        });
    }

    /// Test the call and return instructions by register pointer instruction.
    #[test]
    fn test_call_return_u32_reg() {
        let base_offset = vm::MIN_USER_SEGMENT_SIZE as u32;

        let instructions = &[
            Instruction::MovU32ImmU32Reg(base_offset + 34, RegisterId::E8X), // Starts at [base]. Length = 7. This should remain in place.
            Instruction::PushU32Imm(3), // Starts at [base + 7]. Length = 5. This should remain in place.
            Instruction::PushU32Imm(2), // Starts at [base + 12]. Length = 5. Subroutine argument 1.
            Instruction::PushU32Imm(1), // Starts at [base + 17]. Length = 5. The number of arguments.
            Instruction::CallAbsU32Reg(RegisterId::E8X), // Starts at [base + 22]. Length = 3.
            Instruction::AddU32ImmU32Reg(100, RegisterId::EAX), // Starts at [base + 25]. Length = 7.
            Instruction::Halt, // Starts at [base + 32]. Length = 2.
            /***** FUNC_AAAA - Subroutine starts here. *****/
            Instruction::PushU32Imm(5), // Starts at [base + 34]. Length = 5.
            Instruction::PopU32ToU32Reg(RegisterId::EAX), // Starts at [base + 39]. Length = 2.
            Instruction::RetArgsU32,    // Starts at [base + 41]. Length = 2.
        ];

        let tests = [TestU32::new(
            instructions,
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "CALL - failed to successfully execute CALL instruction",
        )];

        CpuTests::new(&tests).run_all_special(|id, vm| {
            if let Some(mut v) = vm {
                // The subroutine should set the value of the ER1 register to 5.
                // After returning, 100 should be added to the value of ER1, the result moved to the accumulator.
                assert_eq!(
                    v.cpu
                        .registers
                        .get_register_u32(RegisterId::EAX)
                        .read_unchecked(),
                    105
                );

                // There should be one entry on the stack.
                assert_eq!(v.com_bus.mem.pop_u32(), 3);

                // The stack should now be empty.
                assert_eq!(v.com_bus.mem.get_stack_frame_size(), 0);
            } else {
                panic!("failed to correctly execute test id {id}");
            }
        });
    }

    /// Test the call and return instructions, simple non-nested test.
    #[test]
    fn test_call_return_u32_simple() {
        let base_offset = vm::MIN_USER_SEGMENT_SIZE as u32;

        let test_registers = [
            (RegisterId::EBX, 2),
            (RegisterId::ECX, 3),
            (RegisterId::EDX, 4),
            (RegisterId::E5X, 5),
            (RegisterId::E6X, 6),
            (RegisterId::E7X, 7),
            (RegisterId::E8X, 8),
            // This register is preserved across frames and is used as a
            // subroutine return value register.
            (RegisterId::EAX, 0xdeafbeef),
        ];

        let instructions = &[
            Instruction::MovU32ImmU32Reg(test_registers[0].1, test_registers[0].0), // Starts at [base]. Length = 7.
            Instruction::MovU32ImmU32Reg(test_registers[1].1, test_registers[1].0), // Starts at [base + 7]. Length = 7.
            Instruction::MovU32ImmU32Reg(test_registers[2].1, test_registers[2].0), // Starts at [base + 14]. Length = 7.
            Instruction::MovU32ImmU32Reg(test_registers[3].1, test_registers[3].0), // Starts at [base + 21]. Length = 7.
            Instruction::MovU32ImmU32Reg(test_registers[4].1, test_registers[4].0), // Starts at [base + 28]. Length = 7.
            Instruction::MovU32ImmU32Reg(test_registers[5].1, test_registers[5].0), // Starts at [base + 35]. Length = 7.
            Instruction::MovU32ImmU32Reg(test_registers[6].1, test_registers[6].0), // Starts at [base + 42]. Length = 7.
            // The number of arguments for the subroutine.
            Instruction::PushU32Imm(0), // Starts at [base + 49]. Length = 5. The number of arguments.
            Instruction::CallAbsU32Imm(base_offset + 62, String::default()), // Starts at [base + 54]. Length = 6.
            Instruction::Halt, // Starts at [base + 60]. Length = 2.
            /***** FUNC_AAAA - Subroutine starts here. *****/
            Instruction::MovU32ImmU32Reg(0, RegisterId::EBX), // Starts at [base + 62].
            Instruction::MovU32ImmU32Reg(0, RegisterId::ECX),
            Instruction::MovU32ImmU32Reg(0, RegisterId::EDX),
            Instruction::MovU32ImmU32Reg(0, RegisterId::E5X),
            Instruction::MovU32ImmU32Reg(0, RegisterId::E6X),
            Instruction::MovU32ImmU32Reg(0, RegisterId::E7X),
            Instruction::MovU32ImmU32Reg(0, RegisterId::E8X),
            // This will ensure the subroutine was executed since this register is preserved
            // across stack frames.
            Instruction::MovU32ImmU32Reg(test_registers[7].1, test_registers[7].0),
            Instruction::RetArgsU32,
        ];

        let tests = [TestU32::new(
            instructions,
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "CALL - failed to successfully execute CALl instruction",
        )];

        CpuTests::new(&tests).run_all_special(|id, vm| {
            if let Some(v) = vm {
                // Check the registers were correctly set and restored.
                for (id, value) in test_registers {
                    assert_eq!(v.cpu.registers.read_reg_u32_unchecked(&id), value);
                }
            } else {
                panic!("failed to correctly execute test id {id}");
            }
        });
    }

    /// Test the call and return instructions, nested test.
    #[test]
    fn test_call_return_u32_nested() {
        let base_offset = vm::MIN_USER_SEGMENT_SIZE as u32;

        let instructions = &[
            Instruction::PushU32Imm(3), // Starts at [base]. Length = 5. This should remain in place.
            Instruction::PushU32Imm(2), // Starts at [base + 5]. Length = 5. Subroutine argument 1.
            Instruction::PushU32Imm(1), // Starts at [base + 10]. Length = 5. The number of arguments.
            Instruction::CallAbsU32Imm(base_offset + 24, String::default()), // Starts at [base + 15]. Length = 6.
            Instruction::Halt, // Starts at [base + 21]. Length = 2.
            /***** FUNC_AAAA - Subroutine 1 starts here. *****/
            Instruction::PushU32Imm(0), // Starts at [base + 23]. Length = 5. The number of arguments.
            Instruction::CallAbsU32Imm(base_offset + 43, String::default()), // Starts at [base + 28]. Length = 6.
            Instruction::AddU32ImmU32Reg(5, RegisterId::EAX), // Starts at [base + 34]. Length = 7.
            Instruction::RetArgsU32,                          // Starts at [base + 41]. Length = 2.
            /***** FUNC_BBBB - Subroutine 2 starts here. *****/
            Instruction::PushU32Imm(100), // Starts at [base + 43]. Length = 5. This should NOT be preserved.
            Instruction::PopU32ToU32Reg(RegisterId::EAX),
            Instruction::RetArgsU32,
        ];

        let tests = [TestU32::new(
            instructions,
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "CALL - failed to successfully execute CALl instruction",
        )];

        CpuTests::new(&tests).run_all_special(|id, vm| {
            if let Some(mut v) = vm {
                // The first subroutine should set the value of the EAC register to 100.
                // The second subroutine should add 5 to the value of EAC and set the accumulator
                // to the resulting value.
                assert_eq!(
                    v.cpu
                        .registers
                        .get_register_u32(RegisterId::EAX)
                        .read_unchecked(),
                    105
                );

                // There should be one entry on the stack.
                assert_eq!(v.com_bus.mem.pop_u32(), 3);

                // The stack should now be empty.
                assert_eq!(v.com_bus.mem.get_stack_frame_size(), 0);
            } else {
                panic!("failed to correctly execute test id {id}");
            }
        });
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
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "NOP - failed to execute NOP instruction",
        )];

        CpuTests::new(&tests).run_all();
    }

    /// Test the halt instruction.
    #[test]
    fn test_hlt() {
        let tests = [
            TestU32::new(
                &[Halt],
                &[],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "HLT - failed to execute HLT instruction",
            ),
            // The halt instruction should prevent any following instructions from executing, which
            // means that the register value will never change.
            TestU32::new(
                &[Halt, MovU32ImmU32Reg(0xf, RegisterId::E5X)],
                &[(RegisterId::E5X, 0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "HLT - failed to correctly stop execution after a HLT instruction",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the mret instruction.
    #[test]
    fn test_mret() {
        let tests = [TestU32::new(
            &[MachineReturn],
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "MRET - failed to execute MRET instruction",
        )];

        CpuTests::new(&tests).run_all_special(|id, vm| {
            if let Some(v) = vm {
                assert!(
                    !v.cpu.is_in_machine_mode(),
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
        let mut vm = VirtualMachine::new(vm::MIN_USER_SEGMENT_SIZE, &compiled, &[], 0);

        vm.run();
    }

    /// Test the add u32 immediate to u32 register instruction.
    #[test]
    fn test_add_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    AddU32ImmU32Reg(0x2, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ADD - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::EAX),
                    AddU32ImmU32Reg(0x2, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0x1),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::OF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ADD - CPU flags not correctly set",
            ),
            TestU32::new(
                &[AddU32ImmU32Reg(0, RegisterId::EAX)],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ADD - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    AddU32ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ADD - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    AddU32ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ADD - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                    AddU32ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ADD - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    AddU32ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ADD - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the add u32 register to u32 register instruction.
    #[test]
    fn test_add_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0xf, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    AddU32RegU32Reg(RegisterId::EAX, RegisterId::E8X),
                ],
                &[(RegisterId::EAX, 0xf), (RegisterId::E8X, 0x10)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ADD - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::EAX),
                    MovU32ImmU32Reg(0x2, RegisterId::E8X),
                    AddU32RegU32Reg(RegisterId::EAX, RegisterId::E8X),
                ],
                &[
                    (RegisterId::EAX, u32::MAX),
                    (RegisterId::E8X, 0x1),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::OF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ADD - CPU flags not correctly set",
            ),
            TestU32::new(
                &[AddU32RegU32Reg(RegisterId::EAX, RegisterId::E8X)],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ADD - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    AddU32RegU32Reg(RegisterId::EAX, RegisterId::E8X),
                ],
                &[
                    (RegisterId::EAX, 0x2),
                    (RegisterId::E8X, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ADD - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    AddU32ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ADD - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    AddU32RegU32Reg(RegisterId::EAX, RegisterId::E8X),
                ],
                &[
                    (RegisterId::EAX, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E8X, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ADD - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    AddU32RegU32Reg(RegisterId::EAX, RegisterId::E8X),
                ],
                &[
                    (RegisterId::EAX, 0x1),
                    (RegisterId::E8X, 0x2),
                    (RegisterId::EFL, 0x0),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ADD - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test subtraction of u32 immediate from a u32 register instruction.
    #[test]
    fn test_sub_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    SubU32ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SUB - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    SubU32ImmU32Reg(u32::MAX, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0x3),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SUB - CPU flags not correctly set",
            ),
            TestU32::new(
                &[SubU32ImmU32Reg(0, RegisterId::EAX)],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SUB - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x4, RegisterId::EAX),
                    SubU32ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SUB - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    SubU32ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SUB - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                    SubU32ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::SF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SUB - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    SubU32ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SUB - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the subtract u32 register from u32 register instruction.
    #[test]
    fn test_sub_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    MovU32ImmU32Reg(0xf, RegisterId::E8X),
                    SubU32RegU32Reg(RegisterId::EAX, RegisterId::E8X),
                ],
                &[(RegisterId::EAX, 0x1), (RegisterId::E8X, 0xe)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SUB - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::EAX),
                    MovU32ImmU32Reg(0x2, RegisterId::E8X),
                    SubU32RegU32Reg(RegisterId::EAX, RegisterId::E8X),
                ],
                &[
                    (RegisterId::EAX, u32::MAX),
                    (RegisterId::E8X, 0x3),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SUB - CPU flags not correctly set",
            ),
            TestU32::new(
                &[SubU32RegU32Reg(RegisterId::EAX, RegisterId::E8X)],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SUB - CPU flags not correctly set",
            ),
            // Test the parity flag gets enabled.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    MovU32ImmU32Reg(0x4, RegisterId::E8X),
                    SubU32RegU32Reg(RegisterId::EAX, RegisterId::E8X),
                ],
                &[
                    (RegisterId::EAX, 0x1),
                    (RegisterId::E8X, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SUB - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    MovU32ImmU32Reg(0x2, RegisterId::E8X),
                    SubU32RegU32Reg(RegisterId::EAX, RegisterId::E8X),
                ],
                &[
                    (RegisterId::EAX, 0x1),
                    (RegisterId::E8X, 0x1),
                    (RegisterId::EFL, 0x0),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SUB - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E8X),
                    SubU32RegU32Reg(RegisterId::EAX, RegisterId::E8X),
                ],
                &[
                    (RegisterId::EAX, 0x1),
                    (RegisterId::E8X, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::SF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SUB - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    MovU32ImmU32Reg(0x2, RegisterId::E8X),
                    SubU32RegU32Reg(RegisterId::EAX, RegisterId::E8X),
                ],
                &[
                    (RegisterId::EAX, 0x1),
                    (RegisterId::E8X, 0x1),
                    (RegisterId::EFL, 0x0),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SUB - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the multiplication of the value of the ER1 register by a u32 immediate instruction.
    #[test]
    fn test_mul_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[MovU32ImmU32Reg(0x1, RegisterId::EAX), MulU32Imm(0x2)],
                &[(RegisterId::EAX, 0x2)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MUL - incorrect result value produced",
            ),
            TestU32::new(
                &[MovU32ImmU32Reg(u32::MAX, RegisterId::EAX), MulU32Imm(0x2)],
                &[
                    (RegisterId::EAX, 4294967294),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::CF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MUL - CPU flags not correctly set",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the multiplication of the value of the ER1 register by a u32 register instruction.
    #[test]
    fn test_mul_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    MovU32ImmU32Reg(0x2, RegisterId::E8X),
                    MulU32Reg(RegisterId::E8X),
                ],
                &[(RegisterId::EAX, 0x2), (RegisterId::E8X, 0x2)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MUL - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::EAX),
                    MovU32ImmU32Reg(0x2, RegisterId::E8X),
                    MulU32Reg(RegisterId::E8X),
                ],
                &[
                    (RegisterId::EAX, 4294967294),
                    (RegisterId::E8X, 0x2),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::OF, CpuFlag::CF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MUL - CPU flags not correctly set",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the division of the value of the ER1 register by a u32 register instruction.
    #[test]
    fn test_div_u32_imm() {
        let tests = [
            TestU32::new(
                &[MovU32ImmU32Reg(0x3, RegisterId::EAX), DivU32Imm(0x2)],
                &[(RegisterId::EAX, 0x1), (RegisterId::EDX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "DIV - incorrect result value produced",
            ),
            TestU32::new(
                &[MovU32ImmU32Reg(u32::MAX, RegisterId::EAX), DivU32Imm(0x2)],
                &[(RegisterId::EAX, 2147483647), (RegisterId::EDX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "DIV - CPU flags not correctly set",
            ),
        ];

        CpuTests::new(&tests).run_all();

        // Additional division by zero exception tests.
        let div_zero_tests = [TestU32::new(
            &[DivU32Imm(0x0)],
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "DIV - failed to panic when attempting to divide by zero",
        )];

        CpuTests::new(&div_zero_tests).run_all_special(|id: usize, vm: Option<VirtualMachine>| {
            if let Some(v) = vm {
                assert!(
                    v.cpu.is_in_interrupt_handler,
                    "Test {id} Failed - machine is not in an interrupt handler!"
                );
                assert_eq!(
                    v.cpu.last_interrupt_code,
                    Some(DIVIDE_BY_ZERO_INT),
                    "Test {id} Failed - incorrect interrupt handler code!"
                );
            } else {
                // We can't do anything here. The test asserted and so we didn't
                // yield a valid virtual machine instance to interrogate.
            }
        });
    }

    /// Test the division of the value of the ER1 register by a u32 register instruction.
    #[test]
    fn test_div_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x3, RegisterId::EAX),
                    MovU32ImmU32Reg(0x2, RegisterId::E8X),
                    DivU32Reg(RegisterId::E8X),
                ],
                &[
                    (RegisterId::EAX, 0x1),
                    (RegisterId::E8X, 0x2),
                    (RegisterId::EDX, 0x1),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "DIV - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::EAX),
                    MovU32ImmU32Reg(0x2, RegisterId::E8X),
                    DivU32Reg(RegisterId::E8X),
                ],
                &[
                    (RegisterId::EAX, 2147483647),
                    (RegisterId::E8X, 0x2),
                    (RegisterId::EDX, 0x1),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "DIV - CPU flags not correctly set",
            ),
        ];

        CpuTests::new(&tests).run_all();

        // Additional division by zero exception tests.
        let div_zero_tests = [TestU32::new(
            &[DivU32Reg(RegisterId::E8X)],
            &[],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "DIV - failed to panic when attempting to divide by zero",
        )];

        CpuTests::new(&div_zero_tests).run_all_special(|id: usize, vm: Option<VirtualMachine>| {
            if let Some(v) = vm {
                assert!(
                    v.cpu.is_in_interrupt_handler,
                    "Test {id} Failed - machine is not in an interrupt handler!"
                );
                assert_eq!(
                    v.cpu.last_interrupt_code,
                    Some(DIVIDE_BY_ZERO_INT),
                    "Test {id} Failed - incorrect interrupt handler code!"
                );
            } else {
                // We can't do anything here. The test asserted and so we didn't
                // yield a valid virtual machine instance to interrogate.
            }
        });
    }

    /// Test the increment u32 register instruction.
    #[test]
    fn test_inc_u32_reg() {
        let tests = [
            TestU32::new(
                &[IncU32Reg(RegisterId::EAX)],
                &[(RegisterId::EAX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "INC - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::EAX),
                    IncU32Reg(RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::OF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "INC - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    IncU32Reg(RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "INC - CPU parity flag not correctly set",
            ),
            // Test the parity flag get cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    IncU32Reg(RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "INC - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                    IncU32Reg(RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "INC - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    IncU32Reg(RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "INC - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the decrement u32 register instruction.
    #[test]
    fn test_dec_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    DecU32Reg(RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0x0),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "DEC - incorrect result value produced",
            ),
            TestU32::new(
                &[DecU32Reg(RegisterId::EAX)],
                &[
                    (RegisterId::EAX, u32::MAX),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::OF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "DEC - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x4, RegisterId::EAX),
                    DecU32Reg(RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "DEC - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x3, RegisterId::EAX),
                    DecU32Reg(RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "DEC - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                    DecU32Reg(RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::SF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "DEC - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    DecU32Reg(RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "DEC - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the bitwise and of a u32 immediate and a u32 register instruction.
    #[test]
    fn test_bitwise_and_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x3, RegisterId::EAX),
                    AndU32ImmU32Reg(0x2, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "AND - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    AndU32ImmU32Reg(0x3, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "AND - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    AndU32ImmU32Reg(0x0, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0x0),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "AND - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::EAX),
                    AndU32ImmU32Reg(0x3, RegisterId::EAX),
                ],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "AND - CPU flags not correctly set",
            ),
            // Test the parity flag gets set.
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0x3, RegisterId::EAX),
                    AndU32ImmU32Reg(0x3, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0x3),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "AND - CPU parity flag not correctly set",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x3, RegisterId::EAX),
                    AndU32ImmU32Reg(0x2, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "AND - CPU parity flag not correctly cleared",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                    AndU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "AND - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    AndU32ImmU32Reg(0x2, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "AND - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the left-shift u32 register by u32 immediate value.
    #[test]
    fn test_left_shift_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    LeftShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - incorrect result value produced",
            ),
            // When shifted left by three places, this value will set the carry flag but not the
            // overflow flag. The overflow flag is only set when computing 1-bit shifts.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1011_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                    LeftShiftU8ImmU32Reg(0x3, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1111_1111_1111_1111_1111_1111_1111_1000),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::CF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - CPU flags not correctly set",
            ),
            // When shifted left by one place. This will set the carry and overflow flags.
            // The overflow flag is only set when computing 1-bit shifts.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                    LeftShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::CF, CpuFlag::OF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::EAX),
                    LeftShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::OF]), RegisterId::EFL),
                    // This will unset the overflow flag and set the zero flag instead.
                    MovU32ImmU32Reg(0x0, RegisterId::EAX),
                    LeftShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - parity or zero flags are not set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestU32::new(
                &[LeftShiftU8ImmU32Reg(0x20, RegisterId::EAX)],
                &[],
                DEFAULT_U32_STACK_CAPACITY,
                true,
                "SHL - successfully executed instruction with invalid shift value",
            ),
            // Test the parity flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the parity flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::PF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    LeftShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - CPU parity flag not correctly cleared",
            ),
            // Test that a left-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    LeftShiftU8ImmU32Reg(0x0, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - zero left-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0100_0000_0000_0000_0000_0000_0000_0000, RegisterId::EAX),
                    LeftShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    LeftShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the left-shift u32 register by u32 register value.
    #[test]
    fn test_left_shift_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    LeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2), (RegisterId::E8X, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - incorrect result value produced",
            ),
            // When shifted left by three places, this value will set the carry flag but not the
            // overflow flag. The overflow flag is only set when computing 1-bit shifts.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1011_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                    MovU32ImmU32Reg(0x3, RegisterId::E8X),
                    LeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1111_1111_1111_1111_1111_1111_1111_1000),
                    (RegisterId::E8X, 0x3),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::CF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - invalid CPU flag state",
            ),
            // When shifted left by one place. This will set the carry and overflow flags.
            // The overflow flag is only set when computing 1-bit shifts.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    LeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::E8X, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::OF, CpuFlag::CF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - CPU flags not correctly set",
            ),
            // When shifted left by two places, this value will set the overflow and carry flags.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    LeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::E8X, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::OF, CpuFlag::CF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - CPU flags not correctly set",
            ),
            // The zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    LeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::E8X, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - CPU flags not correctly set",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::OF]), RegisterId::EFL),
                    // This will unset the overflow flag and set the zero flag instead.
                    MovU32ImmU32Reg(0x0, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    LeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::E8X, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - CPU flags not correctly set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x20, RegisterId::E8X),
                    LeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[],
                DEFAULT_U32_STACK_CAPACITY,
                true,
                "SHL - successfully executed instruction with invalid shift value",
            ),
            // Test that a left-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    LeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - zero left-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0100_0000_0000_0000_0000_0000_0000_0000, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    LeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (RegisterId::E8X, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    LeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0x2),
                    (RegisterId::E8X, 0x1),
                    (RegisterId::EFL, 0x0),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHL - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the arithmetic left-shift u32 register by u32 immediate value.
    #[test]
    fn test_arith_left_shift_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    ArithLeftShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAL - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1110, RegisterId::EAX),
                    ArithLeftShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1111_1111_1111_1111_1111_1111_1111_1101),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::SF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAL - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::EAX),
                    ArithLeftShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAL - CPU flags not correctly set",
            ),
            // Test that a left-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    ArithLeftShiftU8ImmU32Reg(0x0, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAL - zero left-shift did not leave the result unchanged",
            ),
            // Test the signed flag is set.
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0100_0000_0000_0000_0000_0000_0000_0000, RegisterId::EAX),
                    ArithLeftShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAL - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    ArithLeftShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAL - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the arithmetic left-shift u32 register by u32 register.
    #[test]
    fn test_arith_left_shift_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    ArithLeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2), (RegisterId::E8X, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAL - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1110, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    ArithLeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1111_1111_1111_1111_1111_1111_1111_1101),
                    (RegisterId::E8X, 0x1),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::SF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAL - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    ArithLeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::E8X, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAL - CPU flags not correctly set",
            ),
            // Test that a left-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    ArithLeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAL - zero left-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear any set flags.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0100_0000_0000_0000_0000_0000_0000_0000, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    ArithLeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (RegisterId::E8X, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAL - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    ArithLeftShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0x2),
                    (RegisterId::E8X, 0x1),
                    (RegisterId::EFL, 0x0),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAL - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the right-shift u32 register by u32 immediate value.
    #[test]
    fn test_right_shift_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    RightShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
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
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                    RightShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
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
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1110, RegisterId::EAX),
                    RightShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHR - CPU flags were not correctly set",
            ),
            // The zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::EAX),
                    RightShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHR - zero or parity CPU flags were not correctly set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestU32::new(
                &[RightShiftU8ImmU32Reg(0x20, RegisterId::EAX)],
                &[],
                DEFAULT_U32_STACK_CAPACITY,
                true,
                "SHR - successfully executed instruction with invalid shift value",
            ),
            // Test that a right-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    RightShiftU8ImmU32Reg(0x0, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHR - zero right-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    RightShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0x0),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::CF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAL - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the right-shift u32 register by u32 register.
    #[test]
    fn test_right_shift_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    RightShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1), (RegisterId::E8X, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHR - incorrect result value produced",
            ),
            // The SHR command should clear the overflow flag but set the
            // carry flag since the bit being shifted out is one.
            TestU32::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::OF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    // Execute the test instruction.
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                    RightShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E8X, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::CF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHR - carry or parity CPU flags were not correctly set",
            ),
            // The SHR command should clear the overflow flag and the
            // carry flag in this case as the bit being shifted out is zero.
            TestU32::new(
                &[
                    // Manually set the overflow flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::OF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    // Execute the test instruction.
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1110, RegisterId::EAX),
                    RightShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b0011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E8X, 0x1),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::PF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHR - CPU flags were not correctly set",
            ),
            // The zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    RightShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::E8X, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHR - zero or parity CPU flags were not correctly set",
            ),
            // This should assert as a shift of value higher than 31 is unsupported.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x20, RegisterId::E8X),
                    RightShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[],
                DEFAULT_U32_STACK_CAPACITY,
                true,
                "SHR - successfully executed instruction with invalid shift value",
            ),
            // Test that a right-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    RightShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SHR - zero right-shift did not leave the result unchanged",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the arithmetic right-shift u32 register by u32 immediate value.
    #[test]
    fn test_arith_right_shift_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    ArithRightShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAR - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                    ArithRightShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1011_1111_1111_1111_1111_1111_1111_1111),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAR - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero right-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::EAX),
                    ArithRightShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(
                    RegisterId::EFL,
                    CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                )],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAR - zero or parity CPU flags were not correctly set",
            ),
            // Test that a right-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    ArithRightShiftU8ImmU32Reg(0x0, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAR - zero right-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0000_0000_0000_0000_0000_0000_0000_0001, RegisterId::EAX),
                    ArithRightShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAR - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    ArithRightShiftU8ImmU32Reg(0x1, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1), (RegisterId::EFL, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAR - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the arithmetic right-shift u32 register by u32 register.
    #[test]
    fn test_arith_right_shift_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    ArithRightShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1), (RegisterId::E8X, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAR - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    ArithRightShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1011_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E8X, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAR - incorrect result value produced",
            ),
            // Just zero flag should be set here since zero left-shifted by anything will be zero.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    ArithRightShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::E8X, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::ZF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAR - zero or parity CPU flags were not correctly set",
            ),
            // Test that a right-shift by zero leaves the value unchanged.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    ArithRightShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAR - zero right-shift did not leave the result unchanged",
            ),
            // Test the signed flag gets set.
            TestU32::new(
                &[
                    // Clear every flag.
                    MovU32ImmU32Reg(0x0, RegisterId::EFL),
                    MovU32ImmU32Reg(0b0000_0000_0000_0000_0000_0000_0000_0001, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    ArithRightShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1000_0000_0000_0000_0000_0000_0000_0000),
                    (RegisterId::E8X, 0x1),
                    (
                        RegisterId::EFL,
                        CpuFlag::compute_from(&[CpuFlag::SF, CpuFlag::PF]),
                    ),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAR - CPU signed flag not correctly set",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                    MovU32ImmU32Reg(0x1, RegisterId::E8X),
                    ArithRightShiftU32RegU32Reg(RegisterId::E8X, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0x1),
                    (RegisterId::E8X, 0x1),
                    (RegisterId::EFL, 0x0),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "SAR - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the move u32 immediate to u32 register instruction.
    #[test]
    fn test_move_u32_imm_u32_reg() {
        let tests = [
            TestU32::new(
                &[MovU32ImmU32Reg(0x1, RegisterId::EAX)],
                &[(RegisterId::EAX, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - invalid value moved to register",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    MovU32ImmU32Reg(0x2, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0x2)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - invalid value moved to register",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the move u32 register to u32 register instruction.
    #[test]
    fn test_move_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    MovU32RegU32Reg(RegisterId::EAX, RegisterId::E8X),
                ],
                &[(RegisterId::EAX, 0x1), (RegisterId::E8X, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - immediate value not moved to register",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1, RegisterId::EAX),
                    MovU32ImmU32Reg(0x2, RegisterId::E8X),
                    MovU32RegU32Reg(RegisterId::EAX, RegisterId::E8X),
                ],
                &[(RegisterId::EAX, 0x1), (RegisterId::E8X, 0x1)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - immediate value not moved to register",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the move u32 literal to memory instruction.
    #[test]
    fn test_move_u32_lit_mem() {
        let tests = [TestU32::new(
            &[
                MovU32ImmMemSimple(0x123, 0x0),
                // Check that the value was written by reading it back into a register.
                MovMemU32RegSimple(0x0, RegisterId::EAX),
            ],
            &[(RegisterId::EAX, 0x123)],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "MOV - immediate value not moved to memory",
        )];

        CpuTests::new(&tests).run_all();
    }

    /// Test the move u32 register to memory instruction.
    #[test]
    fn test_move_u32_reg_mem() {
        let tests = [TestU32::new(
            &[
                MovU32ImmU32Reg(0x123, RegisterId::EAX),
                MovU32RegMemSimple(RegisterId::EAX, 0x0),
                // Check that the value was written by reading it back into a register.
                MovMemU32RegSimple(0x0, RegisterId::E8X),
            ],
            &[(RegisterId::EAX, 0x123), (RegisterId::E8X, 0x123)],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "MOV - u32 register value not moved to memory",
        )];

        CpuTests::new(&tests).run_all();
    }

    /// Test the move u32 register to memory instruction.
    #[test]
    fn test_move_mem_u32_reg() {
        let tests = [TestU32::new(
            &[
                // Move the value to memory.
                MovU32ImmU32Reg(0x123, RegisterId::EAX),
                MovU32RegMemSimple(RegisterId::EAX, 0x0),
                // Check that the value was written by reading it back into a register.
                MovMemU32RegSimple(0x0, RegisterId::E8X),
            ],
            &[(RegisterId::EAX, 0x123), (RegisterId::E8X, 0x123)],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "MOV - value not correctly moved from memory to register",
        )];

        CpuTests::new(&tests).run_all();
    }

    /// Test the move u32 to register from address provided by a different register.
    #[test]
    fn test_move_u32_reg_ptr_u32_reg() {
        let tests = [TestU32::new(
            &[
                MovU32ImmU32Reg(0x123, RegisterId::EAX),
                MovU32RegMemSimple(RegisterId::EAX, 0x0),
                MovU32ImmU32Reg(0x0, RegisterId::E8X),
                MovU32RegPtrU32RegSimple(RegisterId::E8X, RegisterId::ECX),
            ],
            &[(RegisterId::EAX, 0x123), (RegisterId::ECX, 0x123)],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "MOV - value not correctly moved from memory to register via register pointer",
        )];

        CpuTests::new(&tests).run_all();
    }

    /// Test the swap u32 register to u32 register instruction.
    #[test]
    fn test_swap_u32_reg_u32_reg() {
        let tests = [TestU32::new(
            &[
                MovU32ImmU32Reg(0x1, RegisterId::EAX),
                MovU32ImmU32Reg(0x2, RegisterId::E8X),
                SwapU32RegU32Reg(RegisterId::EAX, RegisterId::E8X),
            ],
            &[(RegisterId::EAX, 0x2), (RegisterId::E8X, 0x1)],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "SWAP - values of the two registers were not correctly swapped",
        )];

        CpuTests::new(&tests).run_all();
    }

    /// Test the byte swap u32 register instruction.
    #[test]
    fn test_byte_swap_u32_reg() {
        let tests = [TestU32::new(
            &[
                MovU32ImmU32Reg(0xAABBCCDD, RegisterId::EAX),
                ByteSwapU32(RegisterId::EAX),
            ],
            &[(RegisterId::EAX, 0xDDCCBBAA)],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "BSWAP - the byte order of the register was not correctly swapped",
        )];

        CpuTests::new(&tests).run_all();
    }

    /// Test the complex move value to expression-derived memory address.
    #[test]
    fn test_move_u32_imm_expr() {
        use ExpressionArgs as EA;
        use ExpressionOperator as EO;

        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemExpr(
                        0x123,
                        Expression::try_from(
                            &[
                                EA::Immediate(0x8),
                                EA::Operator(EO::Add),
                                EA::Immediate(0x8),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                    ),
                    // Check that the value was written by reading it back into a register.
                    MovMemU32RegSimple(0x8 + 0x8, RegisterId::E8X),
                ],
                &[(RegisterId::E8X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x8, RegisterId::E5X),
                    MovU32ImmU32Reg(0x8, RegisterId::E6X),
                    MovU32ImmMemExpr(
                        0x123,
                        Expression::try_from(
                            &[
                                EA::Register(RegisterId::E5X),
                                EA::Operator(EO::Add),
                                EA::Register(RegisterId::E6X),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                    ),
                    // Check that the value was written by reading it back into a register.
                    MovMemU32RegSimple(0x8 + 0x8, RegisterId::E8X),
                ],
                &[(RegisterId::E5X, 0x8), (RegisterId::E6X, 0x8), (RegisterId::E8X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register with expression - two registers",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x8, RegisterId::E5X),
                    MovU32ImmMemExpr(
                        0x123,
                        Expression::try_from(
                            &[
                                EA::Register(RegisterId::E5X),
                                EA::Operator(EO::Add),
                                EA::Immediate(0x8),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                    ),
                    // Check that the value was written by reading it back into a register.
                    MovMemU32RegSimple(0x8 + 0x8, RegisterId::E8X),
                ],
                &[(RegisterId::E5X, 0x8), (RegisterId::E8X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register with expression - one constant and one register",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::E5X),
                    MovU32ImmU32Reg(0x8, RegisterId::E6X),
                    MovU32ImmMemExpr(
                        0x123,
                        Expression::try_from(
                            &[
                                EA::Register(RegisterId::E5X),
                                EA::Operator(EO::Multiply),
                                EA::Register(RegisterId::E6X),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                    ),
                    // Check that the value was written by reading it back into a register.
                    MovMemU32RegSimple(0x2 * 0x8, RegisterId::E8X),
                ],
                &[(RegisterId::E5X, 0x2), (RegisterId::E6X, 0x8), (RegisterId::E8X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register - using multiplication",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1a, RegisterId::E5X),
                    MovU32ImmU32Reg(0x4, RegisterId::E6X),
                    MovU32ImmMemExpr(
                        0x123,
                        Expression::try_from(
                            &[
                                EA::Register(RegisterId::E5X),
                                EA::Operator(EO::Subtract),
                                EA::Register(RegisterId::E6X),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                    ),
                    // Check that the value was written by reading it back into a register.
                    MovMemU32RegSimple(0x1a - 0x4, RegisterId::E8X),
                ],
                &[(RegisterId::E5X, 0x1a), (RegisterId::E6X, 0x4), (RegisterId::E8X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register - using subtraction",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemExpr(
                        0x123,
                        Expression::try_from(
                            &[
                                EA::Immediate(0x8),
                                EA::Operator(EO::Add),
                                EA::Immediate(0x4),
                                EA::Operator(EO::Multiply),
                                EA::Immediate(0x2),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                    ),
                    // Check that the value was written by reading it back into a register.
                    MovMemU32RegSimple(8 + (0x4 * 0x2), RegisterId::E8X),
                ],
                &[(RegisterId::E8X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register with expression - three constants",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the complex move from an expression-derived memory address to a register.
    #[test]
    fn test_move_mem_expr_u32_reg() {
        use ExpressionArgs as EA;
        use ExpressionOperator as EO;

        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0x123, 0x10),
                    MovMemExprU32Reg(
                        Expression::try_from(
                            &[
                                EA::Immediate(0x8),
                                EA::Operator(EO::Add),
                                EA::Immediate(0x8),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                        RegisterId::E8X,
                    ),
                ],
                &[(RegisterId::E8X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0x123, 0x10),
                    MovU32ImmU32Reg(0x8, RegisterId::E5X),
                    MovU32ImmU32Reg(0x8, RegisterId::E6X),
                    MovMemExprU32Reg(
                        Expression::try_from(
                            &[
                                EA::Register(RegisterId::E5X),
                                EA::Operator(EO::Add),
                                EA::Register(RegisterId::E6X),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                        RegisterId::E8X,
                    ),
                ],
                &[
                    (RegisterId::E5X, 0x8),
                    (RegisterId::E6X, 0x8),
                    (RegisterId::E8X, 0x123),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register with expression - two registers",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0x123, 0x10),
                    MovU32ImmU32Reg(0x8, RegisterId::E5X),
                    MovMemExprU32Reg(
                        Expression::try_from(
                            &[
                                EA::Register(RegisterId::E5X),
                                EA::Operator(EO::Add),
                                EA::Immediate(0x8),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                        RegisterId::E8X,
                    ),
                ],
                &[(RegisterId::E5X, 0x8), (RegisterId::E8X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register with expression - one constant and one register",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0x123, 0x10),
                    MovU32ImmU32Reg(0x2, RegisterId::E5X),
                    MovU32ImmU32Reg(0x8, RegisterId::E6X),
                    MovMemExprU32Reg(
                        Expression::try_from(
                            &[
                                EA::Register(RegisterId::E5X),
                                EA::Operator(EO::Multiply),
                                EA::Register(RegisterId::E6X),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                        RegisterId::E8X,
                    ),
                ],
                &[
                    (RegisterId::E5X, 0x2),
                    (RegisterId::E6X, 0x8),
                    (RegisterId::E8X, 0x123),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register - using multiplication",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0x123, 0x16),
                    MovU32ImmU32Reg(0x1A, RegisterId::E5X),
                    MovU32ImmU32Reg(0x4, RegisterId::E6X),
                    MovMemExprU32Reg(
                        Expression::try_from(
                            &[
                                EA::Register(RegisterId::E5X),
                                EA::Operator(EO::Subtract),
                                EA::Register(RegisterId::E6X),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                        RegisterId::E8X,
                    ),
                ],
                &[
                    (RegisterId::E5X, 0x1a),
                    (RegisterId::E6X, 0x4),
                    (RegisterId::E8X, 0x123),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register - using subtraction",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0x123, 0x10),
                    MovMemExprU32Reg(
                        Expression::try_from(
                            &[
                                EA::Immediate(0x8),
                                EA::Operator(EO::Add),
                                EA::Immediate(0x4),
                                EA::Operator(EO::Multiply),
                                EA::Immediate(0x2),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                        RegisterId::E8X,
                    ),
                ],
                &[(RegisterId::E8X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the complex move from a register to an expression-derived memory address.
    #[test]
    fn test_move_u32_reg_mem_expr() {
        use ExpressionArgs as EA;
        use ExpressionOperator as EO;

        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x123, RegisterId::E8X),
                    MovU32RegMemExpr(
                        RegisterId::E8X,
                        Expression::try_from(
                            &[
                                EA::Immediate(0x8),
                                EA::Operator(EO::Add),
                                EA::Immediate(0x8),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                    ),
                ],
                &[(RegisterId::E8X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
            // Test with two register values.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x8, RegisterId::E5X),
                    MovU32ImmU32Reg(0x8, RegisterId::E6X),
                    MovU32ImmU32Reg(0x123, RegisterId::E8X),
                    MovU32RegMemExpr(
                        RegisterId::E8X,
                        Expression::try_from(
                            &[
                                EA::Register(RegisterId::E5X),
                                EA::Operator(EO::Add),
                                EA::Register(RegisterId::E6X),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                    ),
                ],
                &[
                    (RegisterId::E5X, 0x8),
                    (RegisterId::E6X, 0x8),
                    (RegisterId::E8X, 0x123),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register with expression - two registers",
            ),
            // Test with a constant and a register.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x8, RegisterId::E5X),
                    MovU32ImmU32Reg(0x123, RegisterId::E8X),
                    MovU32RegMemExpr(
                        RegisterId::E8X,
                        Expression::try_from(
                            &[
                                EA::Register(RegisterId::E5X),
                                EA::Operator(EO::Add),
                                EA::Immediate(0x8),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                    ),
                ],
                &[(RegisterId::E5X, 0x8), (RegisterId::E8X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register with expression - one constant and one register",
            ),
            // Test with multiplication.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x2, RegisterId::E5X),
                    MovU32ImmU32Reg(0x8, RegisterId::E6X),
                    MovU32ImmU32Reg(0x123, RegisterId::E8X),
                    MovU32RegMemExpr(
                        RegisterId::E8X,
                        Expression::try_from(
                            &[
                                EA::Register(RegisterId::E5X),
                                EA::Operator(EO::Multiply),
                                EA::Register(RegisterId::E6X),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                    ),
                ],
                &[
                    (RegisterId::E5X, 0x2),
                    (RegisterId::E6X, 0x8),
                    (RegisterId::E8X, 0x123),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register - using multiplication",
            ),
            // Test with subtraction.
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x1A, RegisterId::E5X),
                    MovU32ImmU32Reg(0x4, RegisterId::E6X),
                    MovU32ImmU32Reg(0x123, RegisterId::E8X),
                    MovU32RegMemExpr(
                        RegisterId::E8X,
                        Expression::try_from(
                            &[
                                EA::Register(RegisterId::E5X),
                                EA::Operator(EO::Subtract),
                                EA::Register(RegisterId::E6X),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                    ),
                ],
                &[
                    (RegisterId::E5X, 0x1a),
                    (RegisterId::E6X, 0x4),
                    (RegisterId::E8X, 0x123),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register - using subtraction",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x123, RegisterId::E8X),
                    MovU32RegMemExpr(
                        RegisterId::E8X,
                        Expression::try_from(
                            &[
                                EA::Immediate(0x8),
                                EA::Operator(EO::Add),
                                EA::Immediate(0x4),
                                EA::Operator(EO::Multiply),
                                EA::Immediate(0x2),
                            ][..],
                        )
                        .expect("")
                        .pack(),
                    ),
                ],
                &[(RegisterId::E8X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "MOV - value not correctly moved from memory to register with expression - two constants",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the zero high bit by index instruction.
    #[test]
    fn test_zhbi_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    MovU32ImmU32Reg(4, RegisterId::E6X),
                    ZeroHighBitsByIndexU32Reg(RegisterId::E6X, RegisterId::E5X, RegisterId::E7X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E6X, 4),
                    (RegisterId::E7X, 0b0000_0000_0000_0000_0000_0000_0000_1111),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    MovU32ImmU32Reg(24, RegisterId::E6X),
                    ZeroHighBitsByIndexU32Reg(RegisterId::E6X, RegisterId::E5X, RegisterId::E7X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E6X, 24),
                    (RegisterId::E7X, 0b0000_0000_1111_1111_1111_1111_1111_1111),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    MovU32ImmU32Reg(25, RegisterId::E6X),
                    ZeroHighBitsByIndexU32Reg(RegisterId::E6X, RegisterId::E5X, RegisterId::E7X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E6X, 25),
                    (RegisterId::E7X, 0b0000_0001_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    MovU32ImmU32Reg(27, RegisterId::E6X),
                    ZeroHighBitsByIndexU32Reg(RegisterId::E6X, RegisterId::E5X, RegisterId::E7X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E6X, 27),
                    (RegisterId::E7X, 0b0000_0111_1111_1111_1111_1111_1111_1111),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0, RegisterId::E5X),
                    MovU32ImmU32Reg(0, RegisterId::E6X),
                    ZeroHighBitsByIndexU32Reg(RegisterId::E6X, RegisterId::E5X, RegisterId::E7X),
                ],
                &[(RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF]))],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ZHBI - incorrect flag state - zero flag not set",
            ),
            TestU32::new(
                &[
                    // Manually set the overflow flag state.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::OF]), RegisterId::EFL),
                    ZeroHighBitsByIndexU32Reg(
                        RegisterId::E6X, // Already has the value of 0.
                        RegisterId::E5X, // Already has the value of 0.
                        RegisterId::E7X,
                    ),
                ],
                &[(RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF]))],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ZHBI - incorrect flag state - overflow flag not cleared",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(32, RegisterId::E6X),
                    ZeroHighBitsByIndexU32Reg(
                        RegisterId::E6X,
                        RegisterId::E5X, // Already has the value of 0.
                        RegisterId::E7X,
                    ),
                ],
                &[],
                DEFAULT_U32_STACK_CAPACITY,
                true,
                "ZHBI - successfully executed instruction with invalid bit index",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    ZeroHighBitsByIndexU32Reg(
                        RegisterId::E6X, // Already has the value of 0.
                        RegisterId::E5X,
                        RegisterId::E7X,
                    ),
                ],
                &[
                    (RegisterId::E5X, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E7X, 0),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ZHBI - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the zero high bit by index instruction.
    #[test]
    fn test_zhbi_u32_reg_u32_imm() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    ZeroHighBitsByIndexU32RegU32Imm(0x4, RegisterId::E5X, RegisterId::E6X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E6X, 0b0000_0000_0000_0000_0000_0000_0000_1111),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    ZeroHighBitsByIndexU32RegU32Imm(0x18, RegisterId::E5X, RegisterId::E6X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E6X, 0b0000_0000_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    ZeroHighBitsByIndexU32RegU32Imm(0x19, RegisterId::E5X, RegisterId::E6X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E6X, 0b0000_0001_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    ZeroHighBitsByIndexU32RegU32Imm(0x1b, RegisterId::E5X, RegisterId::E6X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E6X, 0b0000_0111_1111_1111_1111_1111_1111_1111),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ZHBI - incorrect result value produced",
            ),
            TestU32::new(
                &[ZeroHighBitsByIndexU32RegU32Imm(
                    0,
                    RegisterId::E5X, // Already has the value of 0.
                    RegisterId::E6X,
                )],
                &[(RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF]))],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ZHBI - incorrect flag state - zero flag not set",
            ),
            TestU32::new(
                &[
                    // Manually set the overflow flag state.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::OF]), RegisterId::EFL),
                    ZeroHighBitsByIndexU32RegU32Imm(
                        0,
                        RegisterId::E5X,
                        RegisterId::E6X, // Already has the value of 0.
                    ),
                ],
                &[(RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF]))],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ZHBI - incorrect flag state - overflow flag not cleared",
            ),
            TestU32::new(
                &[ZeroHighBitsByIndexU32RegU32Imm(
                    32,
                    RegisterId::E5X,
                    RegisterId::E6X,
                )],
                &[],
                DEFAULT_U32_STACK_CAPACITY,
                true,
                "ZHBI - successfully executed instruction with invalid bit index",
            ),
            // Test the signed flag gets cleared.
            TestU32::new(
                &[
                    // Manually set the signed flag.
                    MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::SF]), RegisterId::EFL),
                    MovU32ImmU32Reg(0b0111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    ZeroHighBitsByIndexU32RegU32Imm(0x0, RegisterId::E5X, RegisterId::E6X),
                ],
                &[
                    (RegisterId::E5X, 0b0111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E6X, 0b0),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "ZHBI - CPU signed flag not correctly cleared",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the bit-test instruction with a u32 register.
    #[test]
    fn test_bit_test_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    BitTestU32Reg(0x0, RegisterId::E5X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1110, RegisterId::E5X),
                    BitTestU32Reg(0x0, RegisterId::E5X),
                ],
                &[(RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1110)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    // The modulo of the bit should be taken, meaning the 0th bit should be tested.
                    BitTestU32Reg(0x20, RegisterId::E5X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
        ];

        CpuTests::new(&tests).run_all();
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
                &[(RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF]))],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_1110, 0x0),
                    BitTestU32Mem(0x0, 0x0),
                ],
                &[],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestU32::new(
                &[
                    // The modulo of the bit should be taken, meaning the 0th bit should be tested.
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x0),
                    BitTestU32Mem(0x20, 0x0),
                ],
                &[(RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF]))],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BT - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the bit-test and reset instruction with a u32 register.
    #[test]
    fn test_bit_test_reset_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::E5X,
                    ),
                    BitTestResetU32Reg(0x0, RegisterId::E5X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BTR - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        RegisterId::E5X,
                    ),
                    BitTestResetU32Reg(0x0, RegisterId::E5X),
                ],
                &[(RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1110)],

                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BTR - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::E5X,
                    ),
                    // The modulo of the bit should be taken, meaning the 0th bit should be tested.
                    BitTestResetU32Reg(0x20, RegisterId::E5X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1110),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BTR - successfully executed instruction with invalid bit index",
            ),
        ];

        CpuTests::new(&tests).run_all();
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
                    MovMemU32RegSimple(0x0, RegisterId::E8X),
                ],
                &[(RegisterId::E8X, 0b1111_1111_1111_1111_1111_1111_1111_1110), (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF]))],
                DEFAULT_U32_STACK_CAPACITY,
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
                    MovMemU32RegSimple(0x0, RegisterId::E8X),
                ],
                &[(RegisterId::E8X, 0b1111_1111_1111_1111_1111_1111_1111_1110)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BTR - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        0x0,
                    ),
                    // The modulo of the bit should be taken, meaning the 0th bit should be tested.
                    BitTestResetU32Mem(0x20, 0x0),
                    MovMemU32RegSimple(0x0, RegisterId::E8X),
                ],
                &[(RegisterId::E8X, 0b1111_1111_1111_1111_1111_1111_1111_1110), (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF]))],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BTR - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the bit-test and set instruction with a u32 register.
    #[test]
    fn test_bit_test_set_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::EAX,
                    ),
                    BitTestSetU32Reg(0x0, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BTS - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1110,
                        RegisterId::EAX,
                    ),
                    BitTestSetU32Reg(0x0, RegisterId::EAX),
                ],
                &[(RegisterId::EAX, 0b1111_1111_1111_1111_1111_1111_1111_1111)],

                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BTS - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        RegisterId::EAX,
                    ),
                    // The modulo of the bit should be taken, meaning the 0th bit should be tested.
                    BitTestSetU32Reg(0x20, RegisterId::EAX),
                ],
                &[
                    (RegisterId::EAX, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BTS - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
        ];

        CpuTests::new(&tests).run_all();
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
                    MovMemU32RegSimple(0x0, RegisterId::E8X),
                ],
                &[
                    (RegisterId::E8X, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF]))
                ],
                DEFAULT_U32_STACK_CAPACITY,
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
                    MovMemU32RegSimple(0x0, RegisterId::E8X),
                ],
                &[(RegisterId::E8X, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BTS - incorrect result produced from the bit-test instruction - carry flag set",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(
                        0b1111_1111_1111_1111_1111_1111_1111_1111,
                        0x0,
                    ),
                    // The modulo of the bit should be taken, meaning the 0th bit should be tested.
                    BitTestSetU32Mem(0x20, 0x0),
                    MovMemU32RegSimple(0x0, RegisterId::E8X),
                ],
                &[
                    (RegisterId::E8X, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::CF]))
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BTS - incorrect result produced from the bit-test instruction - carry flag not set",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the reverse bit scan with a source u32 register and a destination u32 register.
    #[test]
    fn test_bit_scan_reverse_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    BitScanReverseU32RegU32Reg(RegisterId::E5X, RegisterId::E6X),
                ],
                &[(RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b0000_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    BitScanReverseU32RegU32Reg(RegisterId::E5X, RegisterId::E6X),
                ],
                &[
                    (RegisterId::E5X, 0b0000_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E6X, 0x4),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::E5X),
                    BitScanReverseU32RegU32Reg(RegisterId::E5X, RegisterId::E6X),
                ],
                &[
                    // The modulo of the bit should be taken, meaning the last bit should be tested.
                    (RegisterId::E6X, 0x20),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the reverse bit scan with a source memory address and a destination u32 register.
    #[test]
    fn test_bit_scan_reverse_mem_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x1000),
                    BitScanReverseU32MemU32Reg(0x1000, RegisterId::E5X),
                ],
                &[(RegisterId::E5X, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b0000_1111_1111_1111_1111_1111_1111_1111, 0x1000),
                    BitScanReverseU32MemU32Reg(0x1000, RegisterId::E5X),
                ],
                &[(RegisterId::E5X, 0x4)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[BitScanReverseU32MemU32Reg(0x1000, RegisterId::E5X)],
                &[
                    (RegisterId::E5X, 0x20),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the reverse bit scan with a register and a destination memory address.
    #[test]
    fn test_bit_scan_reverse_u32_reg_mem() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    BitScanReverseU32RegMemU32(RegisterId::E5X, 0x0),
                    MovMemU32RegSimple(0x0, RegisterId::E8X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E8X, 0x0),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b0000_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    BitScanReverseU32RegMemU32(RegisterId::E5X, 0x0),
                    MovMemU32RegSimple(0x0, RegisterId::E8X),
                ],
                &[
                    (RegisterId::E5X, 0b0000_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E8X, 0x4),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    BitScanReverseU32RegMemU32(RegisterId::E5X, 0x0),
                    MovMemU32RegSimple(0x0, RegisterId::E8X),
                ],
                &[
                    (RegisterId::E8X, 0x20),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the reverse bit scan with a source memory address and a destination memory address.
    #[test]
    fn test_bit_scan_reverse_mem_mem() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x1000),
                    BitScanReverseU32MemU32Mem(0x1000, 0x1004),
                    // Check that the value was written by reading it back into a register.
                    MovMemU32RegSimple(0x1004, RegisterId::E8X),
                ],
                &[(RegisterId::E8X, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b0000_1111_1111_1111_1111_1111_1111_1111, 0x1000),
                    BitScanReverseU32MemU32Mem(0x1000, 0x1004),
                    // Check that the value was written by reading it back into a register.
                    MovMemU32RegSimple(0x1004, RegisterId::E8X),
                ],
                &[(RegisterId::E8X, 0x4)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    BitScanReverseU32MemU32Mem(0x1000, 0x1004),
                    // Check that the value was written by reading it back into a register.
                    MovMemU32RegSimple(0x1004, RegisterId::E8X),
                ],
                &[
                    (RegisterId::E8X, 0x20),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the forward bit scan with a source u32 register and a destination u32 register.
    #[test]
    fn test_bit_scan_forward_u32_reg_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    BitScanForwardU32RegU32Reg(RegisterId::E5X, RegisterId::E6X),
                ],
                &[(RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1111)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_0000, RegisterId::E5X),
                    BitScanForwardU32RegU32Reg(RegisterId::E5X, RegisterId::E6X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_0000),
                    (RegisterId::E6X, 0x4),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x0, RegisterId::E5X),
                    BitScanForwardU32RegU32Reg(RegisterId::E5X, RegisterId::E6X),
                ],
                &[
                    (RegisterId::E6X, 0x20),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the forward bit scan with a source memory address and a destination u32 register.
    #[test]
    fn test_bit_scan_forward_mem_u32_reg() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x1000),
                    BitScanForwardU32MemU32Reg(0x1000, RegisterId::E5X),
                ],
                &[],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_0000, 0x1000),
                    BitScanForwardU32MemU32Reg(0x1000, RegisterId::E5X),
                ],
                &[(RegisterId::E5X, 0x4)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[BitScanForwardU32MemU32Reg(0x1000, RegisterId::E5X)],
                &[
                    (RegisterId::E5X, 0x20),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the forward bit scan with a register and a destination memory address.
    #[test]
    fn test_bit_scan_forward_u32_reg_mem() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_1111, RegisterId::E5X),
                    BitScanForwardU32RegMemU32(RegisterId::E5X, 0x0),
                    MovMemU32RegSimple(0x0, RegisterId::E8X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_1111),
                    (RegisterId::E8X, 0x0),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0b1111_1111_1111_1111_1111_1111_1111_0000, RegisterId::E5X),
                    BitScanForwardU32RegMemU32(RegisterId::E5X, 0x0),
                    MovMemU32RegSimple(0x0, RegisterId::E8X),
                ],
                &[
                    (RegisterId::E5X, 0b1111_1111_1111_1111_1111_1111_1111_0000),
                    (RegisterId::E8X, 0x4),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    BitScanForwardU32RegMemU32(RegisterId::E5X, 0x0),
                    MovMemU32RegSimple(0x0, RegisterId::E8X),
                ],
                &[
                    (RegisterId::E8X, 0x20),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the forward bit scan with a source memory address and a destination memory address.
    #[test]
    fn test_bit_scan_forward_mem_mem() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_1111, 0x1000),
                    BitScanForwardU32MemU32Mem(0x1000, 0x1004),
                    MovMemU32RegSimple(0x1004, RegisterId::E8X),
                ],
                &[(RegisterId::E8X, 0x0)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSF - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    MovU32ImmMemSimple(0b1111_1111_1111_1111_1111_1111_1111_0000, 0x1000),
                    BitScanForwardU32MemU32Mem(0x1000, 0x1004),
                    MovMemU32RegSimple(0x1004, RegisterId::E8X),
                ],
                &[(RegisterId::E8X, 0x4)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
            TestU32::new(
                &[
                    BitScanForwardU32MemU32Mem(0x1000, 0x1004),
                    MovMemU32RegSimple(0x1004, RegisterId::E8X),
                ],
                &[
                    (RegisterId::E8X, 0x20),
                    (RegisterId::EFL, CpuFlag::compute_from(&[CpuFlag::ZF])),
                ],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "BSR - incorrect result produced from the bit search",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test push-pop a f32 immediate.
    #[test]
    fn test_push_f32_imm_f32_reg() {
        let tests = [TestF32::new(
            &[PushF32Imm(0.1), PopF32ToF32Reg(RegisterId::FR1)],
            &[(RegisterId::FR1, 0.1)],
            DEFAULT_U32_STACK_CAPACITY,
            false,
            "PUSH - failed to execute PUSH instruction",
        )];

        CpuTests::new(&tests).run_all();
    }

    /// Test the push u32 register instruction.
    #[test]
    fn test_push_u32_reg_() {
        let tests = [
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x123, RegisterId::E5X),
                    MovU32ImmU32Reg(0x321, RegisterId::E6X),
                    PushU32Reg(RegisterId::E5X),
                    PushU32Reg(RegisterId::E6X),
                    PopU32ToU32Reg(RegisterId::E7X),
                    PopU32ToU32Reg(RegisterId::E8X),
                ],
                &[(RegisterId::E7X, 0x321), (RegisterId::E8X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "POP - failed to execute POP instruction",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(0x123, RegisterId::E5X),
                    PushU32Reg(RegisterId::E5X),
                    PopU32ToU32Reg(RegisterId::E6X),
                ],
                &[(RegisterId::E6X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "PUSH - failed to execute PUSH instruction",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the push u32 immediate instruction.
    #[test]
    fn test_push_u32_imm_multiple() {
        let tests = [
            TestU32::new(
                &[
                    PushU32Imm(0x123),
                    PushU32Imm(0x321),
                    PopU32ToU32Reg(RegisterId::E5X),
                    PopU32ToU32Reg(RegisterId::E6X),
                ],
                &[(RegisterId::E5X, 0x321), (RegisterId::E6X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "PUSH - failed to execute PUSH instruction",
            ),
            TestU32::new(
                &[PushU32Imm(0x123), PopU32ToU32Reg(RegisterId::E5X)],
                &[(RegisterId::E5X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "PUSH - failed to execute PUSH instruction",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the pop to u32 register instruction.
    #[test]
    fn test_pop_u32_reg() {
        let tests = [
            TestU32::new(
                &[PushU32Imm(0x123), PopU32ToU32Reg(RegisterId::E5X)],
                &[(RegisterId::E5X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "POP - failed to execute POP instruction",
            ),
            TestU32::new(
                &[
                    PushU32Imm(0x123),
                    PushU32Imm(0x321),
                    PopU32ToU32Reg(RegisterId::E5X),
                    PopU32ToU32Reg(RegisterId::E6X),
                ],
                &[(RegisterId::E5X, 0x321), (RegisterId::E6X, 0x123)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "POP - failed to execute POP instruction",
            ),
            TestU32::new(
                &[PopU32ToU32Reg(RegisterId::E5X)],
                &[],
                DEFAULT_U32_STACK_CAPACITY,
                true,
                "POP - failed to execute POP instruction",
            ),
            TestU32::new(
                &[PopU32ToU32Reg(RegisterId::E5X)],
                &[],
                DEFAULT_U32_STACK_CAPACITY,
                true,
                "POP - succeeded in executing POP instruction with empty stack",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }

    /// Test the absolute jump by immediate address instruction.
    #[test]
    fn test_jump_absolute_addr() {
        let base_offset = vm::MIN_USER_SEGMENT_SIZE as u32;

        let tests = [
            TestU32::new(
                &[
                    // The address is calculated as follows:
                    //
                    // The start of the code segment is at byte index [base].
                    //
                    // The jump instruction is _8 bytes_ in length
                    //      (4 for the instruction and 4 for the u32 argument).
                    // The first move instruction is _9 bytes_ in length
                    //      (4 for the instruction, 4 for the u32 argument and 1 for the register ID argument).
                    //
                    // This means that we need to move to index [base + 8 + 9] to get to the
                    // start of the second move instruction.
                    //
                    // We expect that the first move instruction will be skipped entirely.
                    JumpAbsU32Imm(base_offset + 13),
                    // This instruction should be skipped, so R1 should remain at the default value of 0.
                    MovU32ImmU32Reg(0xf, RegisterId::E5X),
                    // The jump should start execution here.
                    MovU32ImmU32Reg(0xa, RegisterId::E6X),
                ],
                &[(RegisterId::E5X, 0x0), (RegisterId::E6X, 0xa)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "JMP - failed to execute JMP instruction",
            ),
            TestU32::new(
                &[JumpAbsU32Imm(u32::MAX)],
                &[],
                DEFAULT_U32_STACK_CAPACITY,
                true,
                "JMP - successfully jumped outside of valid memory bounds.",
            ),
        ];

        CpuTests::new(&tests).run_all();
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
                    // The instruction 2 is _7 bytes_ in length
                    //      (2 for the instruction, 4 for the u32 immediate argument and 1 for the register ID argument).
                    // The instruction 3 is _4 bytes_ in length
                    //      (2 for the instruction, 1 for the register ID argument, 1 for the register ID argument).
                    // The instruction 4 is _3 bytes_ in length
                    //      (2 for the instruction and 1 for the register ID argument).
                    // The instruction 5 is _7 bytes_ in length
                    //      (2 for the instruction, 4 for the u32 argument and 1 for the register ID argument).
                    //
                    // This means that we need to add 21 to the value of the CS register to point
                    // to the start of the second move instruction.
                    //
                    // We expect that the first move instruction will be skipped entirely.
                    MovU32ImmU32Reg(21, RegisterId::E5X),
                    AddU32RegU32Reg(RegisterId::ECS, RegisterId::E5X),
                    JumpAbsU32Reg(RegisterId::E5X),
                    // This instruction should be skipped, so R1 should remain at the default value of 0.
                    MovU32ImmU32Reg(0xf, RegisterId::E6X),
                    // The jump should start execution here.
                    MovU32ImmU32Reg(0xa, RegisterId::E7X),
                ],
                &[(RegisterId::E6X, 0x0), (RegisterId::E7X, 0xa)],
                DEFAULT_U32_STACK_CAPACITY,
                false,
                "JMP - failed to execute JMP instruction",
            ),
            TestU32::new(
                &[
                    MovU32ImmU32Reg(u32::MAX, RegisterId::E5X),
                    JumpAbsU32Reg(RegisterId::E5X),
                ],
                &[],
                DEFAULT_U32_STACK_CAPACITY,
                true,
                "JMP - successfully jumped outside of valid memory bounds.",
            ),
        ];

        CpuTests::new(&tests).run_all();
    }
}
