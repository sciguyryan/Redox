use crate::{
    ins::{instruction::Instruction, op_codes::OpCode},
    reg::registers::RegisterId,
};

pub struct Compiler {
    /// A vector containing the compiled bytecode.
    bytes: Vec<u8>,
}

impl Compiler {
    pub fn new() -> Self {
        Self { bytes: Vec::new() }
    }

    /// Compile a sequence of instructions into bytecode.
    ///
    /// # Arguments
    ///
    /// * `instructions` - A slice of [`Instruction`] objects to be compiled.
    ///
    /// # Returns
    ///
    /// A slice of bytes containing the compiled bytecode.
    pub fn compile(&mut self, instructions: &[Instruction]) -> &[u8] {
        for ins in instructions {
            self.compile_instruction(ins);
        }

        &self.bytes
    }

    /// Compile a single instruction into bytecode.
    ///
    /// # Arguments
    ///
    /// * `instruction` - The [`Instruction`] object to be compiled.
    ///
    /// # Returns
    ///
    /// A vector of u8 bytes containing the compiled data in byte form.
    fn compile_instruction(&mut self, instruction: &Instruction) {
        use Instruction as I;

        // First, we push the bytes for the opcode.
        self.write_opcode(OpCode::from(instruction));

        // Now we need to encode the instruction argument bytes.
        // This is done based on the type and order of the arguments.
        match *instruction {
            /******** [u32 immediate and u32 register] ********/
            I::AddU32ImmU32Reg(imm, reg)
            | I::MovU32ImmU32Reg(imm, reg)
            | I::MovMemU32RegSimple(imm, reg)
            | I::MovMemExprU32Reg(imm, reg)
            | I::BitScanReverseU32MemU32Reg(imm, reg)
            | I::BitScanForwardU32MemU32Reg(imm, reg)
            | I::SubU32ImmU32Reg(imm, reg)
            | I::AndU32ImmU32Reg(imm, reg) => {
                self.write_u32(imm);
                self.write_register_id(&reg);
            }

            /******** [u32 register and u32 register] ********/
            I::AddU32RegU32Reg(reg_1, reg_2)
            | I::LeftShiftU32RegU32Reg(reg_1, reg_2)
            | I::ArithLeftShiftU32RegU32Reg(reg_1, reg_2)
            | I::RightShiftU32RegU32Reg(reg_1, reg_2)
            | I::ArithRightShiftU32RegU32Reg(reg_1, reg_2)
            | I::SwapU32RegU32Reg(reg_1, reg_2)
            | I::MovU32RegU32Reg(reg_1, reg_2)
            | I::MovU32RegPtrU32RegSimple(reg_1, reg_2)
            | I::BitScanReverseU32RegU32Reg(reg_1, reg_2)
            | I::BitScanForwardU32RegU32Reg(reg_1, reg_2)
            | I::SubU32RegU32Reg(reg_1, reg_2) => {
                self.write_register_id(&reg_1);
                self.write_register_id(&reg_2);
            }

            /******** [u32 immediate and u32 immediate] ********/
            I::MovU32ImmMemSimple(imm_1, imm_2)
            | I::MovU32ImmMemExpr(imm_1, imm_2)
            | I::BitScanReverseU32MemU32Mem(imm_1, imm_2)
            | I::BitScanForwardU32MemU32Mem(imm_1, imm_2) => {
                self.write_u32(imm_1);
                self.write_u32(imm_2);
            }

            /******** [u32 register and u32 immediate] ********/
            I::MovU32RegMemSimple(reg, imm)
            | I::MovU32RegMemExpr(reg, imm)
            | I::BitScanReverseU32RegMemU32(reg, imm)
            | I::BitScanForwardU32RegMemU32(reg, imm) => {
                self.write_register_id(&reg);
                self.write_u32(imm);
            }

            /******** [u32 immediate] ********/
            I::MulU32Imm(imm)
            | I::DivU32Imm(imm)
            | I::PushU32Imm(imm)
            | I::CallU32Imm(imm)
            | I::JumpAbsU32Imm(imm)
            | I::LoadIVTAddrU32Imm(imm) => {
                self.write_u32(imm);
            }

            /******** [u8 immediate and u32 register] ********/
            I::LeftShiftU8ImmU32Reg(imm, reg)
            | I::RightShiftU8ImmU32Reg(imm, reg)
            | I::ArithLeftShiftU8ImmU32Reg(imm, reg)
            | I::ArithRightShiftU8ImmU32Reg(imm, reg)
            | I::BitTestU32Reg(imm, reg)
            | I::BitTestResetU32Reg(imm, reg)
            | I::BitTestSetU32Reg(imm, reg) => {
                self.write_u8(imm);
                self.write_register_id(&reg);
            }

            /******** [u8 immediate and u32 immediate] ********/
            I::BitTestU32Mem(imm_1, imm_2)
            | I::BitTestResetU32Mem(imm_1, imm_2)
            | I::BitTestSetU32Mem(imm_1, imm_2) => {
                self.write_u8(imm_1);
                self.write_u32(imm_2);
            }

            /******** [u32 register] ********/
            I::ByteSwapU32(reg)
            | I::MulU32Reg(reg)
            | I::DivU32Reg(reg)
            | I::IncU32Reg(reg)
            | I::DecU32Reg(reg)
            | I::JumpAbsU32Reg(reg)
            | I::PushU32Reg(reg)
            | I::PopU32ToU32Reg(reg)
            | I::CallU32Reg(reg)
            | I::PopF32ToF32Reg(reg) => {
                self.write_register_id(&reg);
            }

            /******** [u32 register, u32 register and u32 register] ********/
            I::ZeroHighBitsByIndexU32Reg(reg_1, reg_2, reg_3) => {
                self.write_register_id(&reg_1);
                self.write_register_id(&reg_2);
                self.write_register_id(&reg_3);
            }

            /******** [u32 immediate, u32 register, and u32 register] ********/
            I::ZeroHighBitsByIndexU32RegU32Imm(imm, reg_1, reg_2) => {
                self.write_u32(imm);
                self.write_register_id(&reg_1);
                self.write_register_id(&reg_2);
            }

            /******** [u8 immediate] ********/
            I::Int(imm) | I::MaskInterrupt(imm) | I::UnmaskInterrupt(imm) => {
                self.write_u8(imm);
            }

            /******** [u32 immediate and u8 immediate] ********/
            I::OutU32Imm(imm1, imm2) => {
                self.write_u32(imm1);
                self.write_u8(imm2);
            }

            /******** [u8 immediate and u8 immediate] ********/
            I::OutU8Imm(imm1, imm2) => {
                self.write_u8(imm1);
                self.write_u8(imm2)
            }

            /******** [f32 immediate and u8 immediate] ********/
            I::OutF32Imm(imm1, imm2) => {
                self.write_f32(imm1);
                self.write_u8(imm2);
            }

            /******** [register ID and u8 immediate] ********/
            I::OutU32Reg(reg, imm) => {
                self.write_register_id(&reg);
                self.write_u8(imm);
            }

            /******** [u8 immediate and register ID] ********/
            I::InU8Reg(imm, reg) | I::InU32Reg(imm, reg) | I::InF32Reg(imm, reg) => {
                self.write_u8(imm);
                self.write_register_id(&reg);
            }

            /******** [u8 immediate and u32 immediate] ********/
            I::InU8Mem(imm1, imm2) | I::InU32Mem(imm1, imm2) | I::InF32Mem(imm1, imm2) => {
                self.write_u8(imm1);
                self.write_u32(imm2);
            }

            /******** [f32 immediate] ********/
            I::PushF32Imm(imm) => self.write_f32(imm),

            /******** [No Arguments] ********/
            I::Nop | I::IntRet | I::RetArgsU32 | I::MachineReturn | I::Halt => {}

            /* These instructions are reserved for future use and shouldn't be constructed. */
            I::Reserved1
            | I::Reserved2
            | I::Reserved3
            | I::Reserved4
            | I::Reserved5
            | I::Reserved6
            | I::Reserved7
            | I::Reserved8
            | I::Reserved9 => {
                unreachable!();
            }

            /* This pseudo-instruction shouldn't be constructed and exists as a compiler hint. */
            I::Label(_) => {
                unreachable!();
            }

            /* This pseudo-instruction shouldn't be constructed and exists to preserve the provided opcode when debugging. */
            I::Unknown(imm) => {
                unreachable!("invalid opcode id {imm}");
            }
        }
    }

    /// Write an opcode into the byte sequence.
    ///
    /// # Arguments
    ///
    /// * `opcode` - The [`OpCode`] to be written.
    fn write_opcode(&mut self, opcode: OpCode) {
        self.write_u32(opcode as u32);
    }

    /// Write a register ID into the byte sequence.
    ///
    /// # Arguments
    ///
    /// * `reg_id` - The [`RegisterId`] to be written.
    fn write_register_id(&mut self, reg_id: &RegisterId) {
        self.bytes.push((*reg_id).into());
    }

    /// Write a f32 into the byte sequence.
    ///
    /// # Arguments
    ///
    /// * `value` - The value to be written.
    fn write_f32(&mut self, value: f32) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    /// Write a u32 into the byte sequence.
    ///
    /// # Arguments
    ///
    /// * `value` - The value to be written.
    fn write_u32(&mut self, value: u32) {
        self.bytes.extend_from_slice(&value.to_le_bytes());
    }

    /// Write a u8 into the byte sequence.
    ///
    /// # Arguments
    ///
    /// * `value` - The value to be written.
    fn write_u8(&mut self, value: u8) {
        self.bytes.push(value);
    }
}

impl Default for Compiler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests_compiler {
    use crate::{
        compiler::bytecode_decompiler::Decompiler,
        ins::{instruction::Instruction, op_codes::OpCode},
        reg::registers::RegisterId,
    };

    use super::Compiler;

    use strum::IntoEnumIterator;

    /// Test a round trip instruction compile and decompile.
    #[test]
    fn test_compile_roundtrip() {
        use Instruction as I;
        use OpCode as O;
        use RegisterId::*;

        let mut instructions_in = Vec::new();

        // This might seem a little long-winded, but it's done this way to ensure
        // that each time a new instruction is added that a corresponding entry
        // is added here.
        for opcode in OpCode::iter() {
            let ins = match opcode {
                O::Nop => I::Nop,
                O::AddU32ImmU32Reg => I::AddU32ImmU32Reg(0x123, EBX),
                O::AddU32RegU32Reg => I::AddU32RegU32Reg(EBX, ECX),
                O::SubU32ImmU32Reg => I::SubU32ImmU32Reg(0x123, EBX),
                O::SubU32RegU32Reg => I::SubU32RegU32Reg(EBX, ECX),
                O::MulU32Imm => I::MulU32Imm(0x123),
                O::MulU32Reg => I::MulU32Reg(EBX),
                O::DivU32Imm => I::DivU32Imm(0x123),
                O::DivU32Reg => I::DivU32Reg(EBX),
                O::IncU32Reg => I::IncU32Reg(EBX),
                O::DecU32Reg => I::DecU32Reg(EBX),
                O::AndU32ImmU32Reg => I::AndU32ImmU32Reg(0x123, EBX),
                O::LeftShiftU8ImmU32Reg => I::LeftShiftU8ImmU32Reg(31, EBX),
                O::LeftShiftU32RegU32Reg => I::LeftShiftU32RegU32Reg(EBX, ECX),
                O::ArithLeftShiftU8ImmU32Reg => I::ArithLeftShiftU8ImmU32Reg(31, EBX),
                O::ArithLeftShiftU32RegU32Reg => I::ArithLeftShiftU32RegU32Reg(EBX, ECX),
                O::RightShiftU8ImmU32Reg => I::RightShiftU8ImmU32Reg(31, EBX),
                O::RightShiftU32RegU32Reg => I::RightShiftU32RegU32Reg(EBX, ECX),
                O::ArithRightShiftU8ImmU32Reg => I::ArithRightShiftU8ImmU32Reg(31, EBX),
                O::ArithRightShiftU32RegU32Reg => I::ArithRightShiftU32RegU32Reg(EBX, ECX),
                O::CallU32Imm => I::CallU32Imm(0xdeafbeed),
                O::CallU32Reg => I::CallU32Reg(RegisterId::EBX),
                O::RetArgsU32 => I::RetArgsU32,
                O::Int => I::Int(0xff),
                O::IntRet => I::IntRet,
                O::JumpAbsU32Imm => I::JumpAbsU32Imm(0xdeadbeef),
                O::JumpAbsU32Reg => I::JumpAbsU32Reg(EAX),
                O::SwapU32RegU32Reg => I::SwapU32RegU32Reg(EBX, ECX),
                O::MovU32ImmU32Reg => I::MovU32ImmU32Reg(0x123, EBX),
                O::MovU32RegU32Reg => I::MovU32RegU32Reg(EBX, ECX),
                O::MovU32ImmMemSimple => I::MovU32ImmMemSimple(0x123, 0x321),
                O::MovU32RegMemSimple => I::MovU32RegMemSimple(EBX, 0x123),
                O::MovMemU32RegSimple => I::MovMemU32RegSimple(0x123, EBX),
                O::MovU32RegPtrU32RegSimple => I::MovU32RegPtrU32RegSimple(EBX, ECX),
                O::MovU32ImmMemExpr => I::MovU32ImmMemExpr(0x123, 0x321),
                O::MovMemExprU32Reg => I::MovMemExprU32Reg(0x123, EBX),
                O::MovU32RegMemExpr => I::MovU32RegMemExpr(EBX, 0x123),
                O::ByteSwapU32 => I::ByteSwapU32(EBX),
                O::ZeroHighBitsByIndexU32Reg => I::ZeroHighBitsByIndexU32Reg(EBX, ECX, EDX),
                O::ZeroHighBitsByIndexU32RegU32Imm => {
                    I::ZeroHighBitsByIndexU32RegU32Imm(0x123, EBX, ECX)
                }
                O::PushU32Imm => I::PushU32Imm(0x123),
                O::PushF32Imm => I::PushF32Imm(0.1),
                O::PushU32Reg => I::PushU32Reg(EBX),
                O::PopF32ToF32Reg => I::PopF32ToF32Reg(FR2),
                O::PopU32ToU32Reg => I::PopU32ToU32Reg(EBX),
                O::OutF32Imm => I::OutF32Imm(1.0, 0xab),
                O::OutU32Imm => I::OutU32Imm(0xdeadbeef, 0xab),
                O::OutU32Reg => I::OutU32Reg(EBX, 0xab),
                O::OutU8Imm => I::OutU8Imm(0xba, 0xab),
                O::InU8Reg => I::InU8Reg(0xab, EBX),
                O::InU8Mem => I::InU8Mem(0xab, 0xdeadbeef),
                O::InU32Reg => I::InU32Reg(0xab, EBX),
                O::InU32Mem => I::InU32Mem(0xab, 0xdeadbeef),
                O::InF32Reg => I::InF32Reg(0xab, FR2),
                O::InF32Mem => I::InF32Mem(0xab, 0xdeadbeef),
                O::BitTestU32Reg => I::BitTestU32Reg(0x40, EBX),
                O::BitTestU32Mem => I::BitTestU32Mem(0x40, 0x123),
                O::BitTestResetU32Reg => I::BitTestResetU32Reg(0x40, EBX),
                O::BitTestResetU32Mem => I::BitTestResetU32Mem(0x40, 0x123),
                O::BitTestSetU32Reg => I::BitTestSetU32Reg(0x40, EBX),
                O::BitTestSetU32Mem => I::BitTestSetU32Mem(0x40, 0x123),
                O::BitScanReverseU32RegU32Reg => I::BitScanReverseU32RegU32Reg(EBX, ECX),
                O::BitScanReverseU32MemU32Reg => I::BitScanReverseU32MemU32Reg(0x123, EBX),
                O::BitScanReverseU32RegMemU32 => I::BitScanReverseU32RegMemU32(EBX, 0x123),
                O::BitScanReverseU32MemU32Mem => I::BitScanReverseU32MemU32Mem(0x123, 0x321),
                O::BitScanForwardU32RegU32Reg => I::BitScanForwardU32RegU32Reg(EBX, ECX),
                O::BitScanForwardU32MemU32Reg => I::BitScanForwardU32MemU32Reg(0x123, EBX),
                O::BitScanForwardU32RegMemU32 => I::BitScanForwardU32RegMemU32(EBX, 0x123),
                O::BitScanForwardU32MemU32Mem => I::BitScanForwardU32MemU32Mem(0x123, 0x321),
                O::MaskInterrupt => I::MaskInterrupt(0xff),
                O::UnmaskInterrupt => I::UnmaskInterrupt(0xff),
                O::LoadIVTAddrU32Imm => I::LoadIVTAddrU32Imm(0xdeadbeef),
                O::MachineReturn => I::MachineReturn,
                O::Halt => I::Halt,

                // We don't want to test constructing these instructions.
                O::Reserved1
                | O::Reserved2
                | O::Reserved3
                | O::Reserved4
                | O::Reserved5
                | O::Reserved6
                | O::Reserved7
                | O::Reserved8
                | O::Reserved9
                | O::Label
                | O::Unknown => continue,
            };

            instructions_in.push(ins);
        }

        // Now we can compile the instructions into bytecode.
        let mut compiler = Compiler::new();
        compiler.compile(&instructions_in);

        // Finally we can decompile the code.
        // We should end up with the original instructions.
        let decompiled_instructions = Decompiler::decompile(&compiler.bytes);

        let mut success = true;
        for (i, (decompiled, original)) in decompiled_instructions
            .iter()
            .zip(&instructions_in)
            .enumerate()
        {
            if original != decompiled {
                eprintln!("instruction {i} did not correctly round-trip. Expected = {original}, Actual = {decompiled}");
                success = false;
            }
        }
        assert!(
            success,
            "failed to successfully compile and decompile one or more instructions"
        );
    }
}
