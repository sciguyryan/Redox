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
        // First, we push the bytes for the opcode.
        self.write_opcode(OpCode::from(instruction));

        // Now we need to encode the instruction argument bytes.
        // This is done based on the type and order of the arguments.
        match *instruction {
            /******** [u32 immediate and u32 register] ********/
            Instruction::AddU32ImmU32Reg(imm, reg)
            | Instruction::LeftShiftU32ImmU32Reg(imm, reg)
            | Instruction::ArithLeftShiftU32ImmU32Reg(imm, reg)
            | Instruction::RightShiftU32ImmU32Reg(imm, reg)
            | Instruction::ArithRightShiftU32ImmU32Reg(imm, reg)
            | Instruction::MovU32ImmU32Reg(imm, reg)
            | Instruction::MovMemU32RegSimple(imm, reg)
            | Instruction::MovMemExprU32Reg(imm, reg)
            | Instruction::BitScanReverseU32MemU32Reg(imm, reg)
            | Instruction::BitScanForwardU32MemU32Reg(imm, reg)
            | Instruction::SubU32ImmU32Reg(imm, reg)
            | Instruction::MulU32ImmU32Reg(imm, reg)
            | Instruction::DivU32ImmU32Reg(imm, reg)
            | Instruction::ModU32ImmU32Reg(imm, reg)
            | Instruction::AndU32ImmU32Reg(imm, reg) => {
                self.write_u32(imm);
                self.write_register_id(&reg);
            }

            /******** [u32 register and u32 register] ********/
            Instruction::AddU32RegU32Reg(reg_1, reg_2)
            | Instruction::LeftShiftU32RegU32Reg(reg_1, reg_2)
            | Instruction::ArithLeftShiftU32RegU32Reg(reg_1, reg_2)
            | Instruction::RightShiftU32RegU32Reg(reg_1, reg_2)
            | Instruction::ArithRightShiftU32RegU32Reg(reg_1, reg_2)
            | Instruction::SwapU32RegU32Reg(reg_1, reg_2)
            | Instruction::MovU32RegU32Reg(reg_1, reg_2)
            | Instruction::MovU32RegPtrU32RegSimple(reg_1, reg_2)
            | Instruction::BitScanReverseU32RegU32Reg(reg_1, reg_2)
            | Instruction::BitScanForwardU32RegU32Reg(reg_1, reg_2)
            | Instruction::SubU32RegU32Reg(reg_1, reg_2)
            | Instruction::MulU32RegU32Reg(reg_1, reg_2)
            | Instruction::DivU32RegU32Reg(reg_1, reg_2)
            | Instruction::ModU32RegU32Reg(reg_1, reg_2) => {
                self.write_register_id(&reg_1);
                self.write_register_id(&reg_2);
            }

            /******** [u32 immediate and u32 immediate] ********/
            Instruction::MovU32ImmMemSimple(imm_1, imm_2)
            | Instruction::MovU32ImmMemExpr(imm_1, imm_2)
            | Instruction::BitScanReverseU32MemU32Mem(imm_1, imm_2)
            | Instruction::BitScanForwardU32MemU32Mem(imm_1, imm_2) => {
                self.write_u32(imm_1);
                self.write_u32(imm_2);
            }

            /******** [u32 register and u32 immediate] ********/
            Instruction::MovU32RegMemSimple(reg, imm)
            | Instruction::MovU32RegMemExpr(reg, imm)
            | Instruction::BitScanReverseU32RegMemU32(reg, imm)
            | Instruction::BitScanForwardU32RegMemU32(reg, imm)
            | Instruction::SubU32RegU32Imm(reg, imm)
            | Instruction::DivU32RegU32Imm(reg, imm)
            | Instruction::ModU32RegU32Imm(reg, imm) => {
                self.write_register_id(&reg);
                self.write_u32(imm);
            }

            /******** [u32 immediate] ********/
            Instruction::PushU32Imm(imm)
            | Instruction::CallU32Imm(imm)
            | Instruction::JumpAbsU32Imm(imm)
            | Instruction::LoadIVTAddrU32Imm(imm) => {
                self.write_u32(imm);
            }

            /******** [u8 immediate and u32 register] ********/
            Instruction::BitTestU32Reg(imm, reg)
            | Instruction::BitTestResetU32Reg(imm, reg)
            | Instruction::BitTestSetU32Reg(imm, reg) => {
                self.write_u8(imm);
                self.write_register_id(&reg);
            }

            /******** [u8 immediate and u32 immediate] ********/
            Instruction::BitTestU32Mem(imm_1, imm_2)
            | Instruction::BitTestResetU32Mem(imm_1, imm_2)
            | Instruction::BitTestSetU32Mem(imm_1, imm_2) => {
                self.write_u8(imm_1);
                self.write_u32(imm_2);
            }

            /******** [u32 register] ********/
            Instruction::ByteSwapU32(reg)
            | Instruction::IncU32Reg(reg)
            | Instruction::DecU32Reg(reg)
            | Instruction::JumpAbsU32Reg(reg)
            | Instruction::PushU32Reg(reg)
            | Instruction::PopU32ImmU32Reg(reg)
            | Instruction::CallU32Reg(reg) => {
                self.write_register_id(&reg);
            }

            /******** [u32 register, u32 register and u32 register] ********/
            Instruction::ZeroHighBitsByIndexU32Reg(reg_1, reg_2, reg_3) => {
                self.write_register_id(&reg_1);
                self.write_register_id(&reg_2);
                self.write_register_id(&reg_3);
            }

            /******** [u32 immediate, u32 register, and u32 register] ********/
            Instruction::ZeroHighBitsByIndexU32RegU32Imm(imm, reg_1, reg_2) => {
                self.write_u32(imm);
                self.write_register_id(&reg_1);
                self.write_register_id(&reg_2);
            }

            /******** [u8 immediate] ********/
            Instruction::Int(imm)
            | Instruction::MaskInterrupt(imm)
            | Instruction::UnmaskInterrupt(imm) => {
                self.write_u8(imm);
            }

            /******** [No Arguments] ********/
            Instruction::Nop
            | Instruction::IntRet
            | Instruction::RetArgsU32
            | Instruction::ClearInterruptFlag
            | Instruction::SetInterruptFlag
            | Instruction::MachineReturn
            | Instruction::Halt => {}

            /* These instructions are reserved for future use and shouldn't be constructed. */
            Instruction::Reserved1
            | Instruction::Reserved2
            | Instruction::Reserved3
            | Instruction::Reserved4
            | Instruction::Reserved5
            | Instruction::Reserved6
            | Instruction::Reserved7
            | Instruction::Reserved8
            | Instruction::Reserved9 => {
                unreachable!();
            }

            /* This pseudo-instruction shouldn't be constructed and exists as a compiler hint. */
            Instruction::Label(_) => {
                unreachable!();
            }

            /* This pseudo-instruction shouldn't be constructed and exists to preserve the provided opcode when debugging. */
            Instruction::Unknown(imm) => {
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
        use {OpCode::*, RegisterId::*};

        let mut instructions_in = Vec::new();

        // This might seem a little long-winded, but it's done this way to ensure
        // that each time a new instruction is added that a corresponding entry
        // is added here.
        for opcode in OpCode::iter() {
            let ins = match opcode {
                Nop => Instruction::Nop,
                AddU32ImmU32Reg => Instruction::AddU32ImmU32Reg(0x123, ER2),
                AddU32RegU32Reg => Instruction::AddU32RegU32Reg(ER2, ER3),
                SubU32ImmU32Reg => Instruction::SubU32ImmU32Reg(0x123, ER2),
                SubU32RegU32Imm => Instruction::SubU32RegU32Imm(ER2, 0x123),
                SubU32RegU32Reg => Instruction::SubU32RegU32Reg(ER2, ER3),
                MulU32ImmU32Reg => Instruction::MulU32ImmU32Reg(0x123, ER2),
                MulU32RegU32Reg => Instruction::MulU32RegU32Reg(ER2, ER3),
                DivU32ImmU32Reg => Instruction::DivU32ImmU32Reg(0x123, ER2),
                DivU32RegU32Imm => Instruction::DivU32RegU32Imm(ER2, 0x123),
                DivU32RegU32Reg => Instruction::DivU32RegU32Reg(ER2, ER3),
                ModU32ImmU32Reg => Instruction::ModU32ImmU32Reg(0x123, ER2),
                ModU32RegU32Imm => Instruction::ModU32RegU32Imm(ER2, 0x123),
                ModU32RegU32Reg => Instruction::ModU32RegU32Reg(ER2, ER3),
                IncU32Reg => Instruction::IncU32Reg(ER2),
                DecU32Reg => Instruction::DecU32Reg(ER2),
                AndU32ImmU32Reg => Instruction::AndU32ImmU32Reg(0x123, ER2),
                LeftShiftU32ImmU32Reg => Instruction::LeftShiftU32ImmU32Reg(31, ER2),
                LeftShiftU32RegU32Reg => Instruction::LeftShiftU32RegU32Reg(ER2, ER3),
                ArithLeftShiftU32ImmU32Reg => Instruction::ArithLeftShiftU32ImmU32Reg(31, ER2),
                ArithLeftShiftU32RegU32Reg => Instruction::ArithLeftShiftU32RegU32Reg(ER2, ER3),
                RightShiftU32ImmU32Reg => Instruction::RightShiftU32ImmU32Reg(31, ER2),
                RightShiftU32RegU32Reg => Instruction::RightShiftU32RegU32Reg(ER2, ER3),
                ArithRightShiftU32ImmU32Reg => Instruction::ArithRightShiftU32ImmU32Reg(31, ER2),
                ArithRightShiftU32RegU32Reg => Instruction::ArithRightShiftU32RegU32Reg(ER2, ER3),
                CallU32Imm => Instruction::CallU32Imm(0xdeafbeed),
                CallU32Reg => Instruction::CallU32Reg(RegisterId::ER2),
                RetArgsU32 => Instruction::RetArgsU32,
                Int => Instruction::Int(0xff),
                IntRet => Instruction::IntRet,
                JumpAbsU32Imm => Instruction::JumpAbsU32Imm(0xdeadbeef),
                JumpAbsU32Reg => Instruction::JumpAbsU32Reg(ER1),
                SwapU32RegU32Reg => Instruction::SwapU32RegU32Reg(ER2, ER3),
                MovU32ImmU32Reg => Instruction::MovU32ImmU32Reg(0x123, ER2),
                MovU32RegU32Reg => Instruction::MovU32RegU32Reg(ER2, ER3),
                MovU32ImmMemSimple => Instruction::MovU32ImmMemSimple(0x123, 0x321),
                MovU32RegMemSimple => Instruction::MovU32RegMemSimple(ER2, 0x123),
                MovMemU32RegSimple => Instruction::MovMemU32RegSimple(0x123, ER2),
                MovU32RegPtrU32RegSimple => Instruction::MovU32RegPtrU32RegSimple(ER2, ER3),
                MovU32ImmMemExpr => Instruction::MovU32ImmMemExpr(0x123, 0x321),
                MovMemExprU32Reg => Instruction::MovMemExprU32Reg(0x123, ER2),
                MovU32RegMemExpr => Instruction::MovU32RegMemExpr(ER2, 0x123),
                ByteSwapU32 => Instruction::ByteSwapU32(ER2),
                ZeroHighBitsByIndexU32Reg => Instruction::ZeroHighBitsByIndexU32Reg(ER2, ER3, ER4),
                ZeroHighBitsByIndexU32RegU32Imm => {
                    Instruction::ZeroHighBitsByIndexU32RegU32Imm(0x123, ER2, ER3)
                }
                PushU32Imm => Instruction::PushU32Imm(0x123),
                PushU32Reg => Instruction::PushU32Reg(ER2),
                PopU32ImmU32Reg => Instruction::PopU32ImmU32Reg(ER2),
                BitTestU32Reg => Instruction::BitTestU32Reg(0x40, ER2),
                BitTestU32Mem => Instruction::BitTestU32Mem(0x40, 0x123),
                BitTestResetU32Reg => Instruction::BitTestResetU32Reg(0x40, ER2),
                BitTestResetU32Mem => Instruction::BitTestResetU32Mem(0x40, 0x123),
                BitTestSetU32Reg => Instruction::BitTestSetU32Reg(0x40, ER2),
                BitTestSetU32Mem => Instruction::BitTestSetU32Mem(0x40, 0x123),
                BitScanReverseU32RegU32Reg => Instruction::BitScanReverseU32RegU32Reg(ER2, ER3),
                BitScanReverseU32MemU32Reg => Instruction::BitScanReverseU32MemU32Reg(0x123, ER2),
                BitScanReverseU32RegMemU32 => Instruction::BitScanReverseU32RegMemU32(ER2, 0x123),
                BitScanReverseU32MemU32Mem => Instruction::BitScanReverseU32MemU32Mem(0x123, 0x321),
                BitScanForwardU32RegU32Reg => Instruction::BitScanForwardU32RegU32Reg(ER2, ER3),
                BitScanForwardU32MemU32Reg => Instruction::BitScanForwardU32MemU32Reg(0x123, ER2),
                BitScanForwardU32RegMemU32 => Instruction::BitScanForwardU32RegMemU32(ER2, 0x123),
                BitScanForwardU32MemU32Mem => Instruction::BitScanForwardU32MemU32Mem(0x123, 0x321),
                ClearInterruptFlag => Instruction::ClearInterruptFlag,
                SetInterruptFlag => Instruction::SetInterruptFlag,
                MaskInterrupt => Instruction::MaskInterrupt(0xff),
                UnmaskInterrupt => Instruction::UnmaskInterrupt(0xff),
                LoadIVTAddrU32Imm => Instruction::LoadIVTAddrU32Imm(0xdeadbeef),
                MachineReturn => Instruction::MachineReturn,
                Halt => Instruction::Halt,

                // We don't want to test constructing these instructions.
                Reserved1 | Reserved2 | Reserved3 | Reserved4 | Reserved5 | Reserved6
                | Reserved7 | Reserved8 | Reserved9 | Label | Unknown => continue,
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
