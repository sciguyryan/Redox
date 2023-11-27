use crate::{
    ins::{instruction::Instruction, op_codes::OpCode},
    reg::registers::RegisterId,
};

pub struct Compiler {
    /// A vector containing all of the compiled bytecode.
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
        self.write_opcode(OpCode::from(*instruction));

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
            | Instruction::MovMemU32RegRelSimple(imm, reg)
            | Instruction::MovMemExprU32RegRel(imm, reg)
            | Instruction::BitScanReverseU32MemU32Reg(imm, reg)
            | Instruction::BitScanForwardU32MemU32Reg(imm, reg)
            | Instruction::SubU32ImmU32Reg(imm, reg)
            | Instruction::MulU32ImmU32Reg(imm, reg) => {
                self.write_u32(imm);
                self.write_register_id(&reg);
            }

            /******** [u32 register and u32 register] ********/
            Instruction::AddU32RegU32Reg(reg1, reg2)
            | Instruction::LeftShiftU32RegU32Reg(reg1, reg2)
            | Instruction::ArithLeftShiftU32RegU32Reg(reg1, reg2)
            | Instruction::RightShiftU32RegU32Reg(reg1, reg2)
            | Instruction::ArithRightShiftU32RegU32Reg(reg1, reg2)
            | Instruction::SwapU32RegU32Reg(reg1, reg2)
            | Instruction::MovU32RegU32Reg(reg1, reg2)
            | Instruction::MovU32RegPtrU32RegRelSimple(reg1, reg2)
            | Instruction::BitScanReverseU32RegU32Reg(reg1, reg2)
            | Instruction::BitScanForwardU32RegU32Reg(reg1, reg2)
            | Instruction::SubU32RegU32Reg(reg1, reg2) => {
                self.write_register_id(&reg1);
                self.write_register_id(&reg2);
            }

            /******** [u32 immediate and u32 immediate] ********/
            Instruction::MovU32ImmMemRelSimple(imm1, imm2)
            | Instruction::MovU32ImmMemExprRel(imm1, imm2)
            | Instruction::BitScanReverseU32MemU32Mem(imm1, imm2)
            | Instruction::BitScanForwardU32MemU32Mem(imm1, imm2) => {
                self.write_u32(imm1);
                self.write_u32(imm2);
            }

            /******** [u32 register and u32 immediate] ********/
            Instruction::MovU32RegMemRelSimple(reg, imm)
            | Instruction::MovU32RegMemExprRel(reg, imm)
            | Instruction::BitScanReverseU32RegMemU32(reg, imm)
            | Instruction::BitScanForwardU32RegMemU32(reg, imm)
            | Instruction::SubU32RegU32Imm(reg, imm) => {
                self.write_register_id(&reg);
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
            Instruction::BitTestU32Mem(imm1, imm2)
            | Instruction::BitTestResetU32Mem(imm1, imm2)
            | Instruction::BitTestSetU32Mem(imm1, imm2) => {
                self.write_u8(imm1);
                self.write_u32(imm2);
            }

            /******** [u32 register] ********/
            Instruction::ByteSwapU32(reg)
            | Instruction::IncU32Reg(reg)
            | Instruction::DecU32Reg(reg) => {
                self.write_register_id(&reg);
            }

            /******** [u32 register, u32 register and u32 register] ********/
            Instruction::ZeroHighBitsByIndexU32Reg(reg1, reg2, reg3) => {
                self.write_register_id(&reg1);
                self.write_register_id(&reg2);
                self.write_register_id(&reg3);
            }

            /******** [u32 immediate, u32 register, and u32 register] ********/
            Instruction::ZeroHighBitsByIndexU32RegU32Imm(imm, reg1, reg2) => {
                self.write_u32(imm);
                self.write_register_id(&reg1);
                self.write_register_id(&reg2);
            }

            /******** [No Arguments] ********/
            Instruction::Nop | Instruction::Ret | Instruction::Mret | Instruction::Hlt => {}
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
        use RegisterId::*;

        let mut instructions_in = Vec::new();

        // This might seem a little long-winded, but it's done this way to ensure
        // that each time a new instruction is added that a corresponding entry
        // is added here.
        for opcode in OpCode::iter() {
            let ins = match opcode {
                OpCode::Nop => Instruction::Nop,
                OpCode::AddU32ImmU32Reg => Instruction::AddU32ImmU32Reg(0x123, R2),
                OpCode::AddU32RegU32Reg => Instruction::AddU32RegU32Reg(R2, R3),
                OpCode::SubU32ImmU32Reg => Instruction::SubU32ImmU32Reg(0x123, R2),
                OpCode::SubU32RegU32Imm => Instruction::SubU32RegU32Imm(R2, 0x123),
                OpCode::SubU32RegU32Reg => Instruction::SubU32RegU32Reg(R2, R3),
                OpCode::MulU32ImmU32Reg => Instruction::MulU32ImmU32Reg(0x123, R2),
                OpCode::IncU32Reg => Instruction::IncU32Reg(R2),
                OpCode::DecU32Reg => Instruction::DecU32Reg(R2),
                OpCode::LeftShiftU32ImmU32Reg => Instruction::LeftShiftU32ImmU32Reg(31, R2),
                OpCode::LeftShiftU32RegU32Reg => Instruction::LeftShiftU32RegU32Reg(R2, R3),
                OpCode::ArithLeftShiftU32ImmU32Reg => {
                    Instruction::ArithLeftShiftU32ImmU32Reg(31, R2)
                }
                OpCode::ArithLeftShiftU32RegU32Reg => {
                    Instruction::ArithLeftShiftU32RegU32Reg(R2, R3)
                }
                OpCode::RightShiftU32ImmU32Reg => Instruction::RightShiftU32ImmU32Reg(31, R2),
                OpCode::RightShiftU32RegU32Reg => Instruction::RightShiftU32RegU32Reg(R2, R3),
                OpCode::ArithRightShiftU32ImmU32Reg => {
                    Instruction::ArithRightShiftU32ImmU32Reg(31, R2)
                }
                OpCode::ArithRightShiftU32RegU32Reg => {
                    Instruction::ArithRightShiftU32RegU32Reg(R2, R3)
                }
                OpCode::SwapU32RegU32Reg => Instruction::SwapU32RegU32Reg(R2, R3),
                OpCode::MovU32ImmU32Reg => Instruction::MovU32ImmU32Reg(0x123, R2),
                OpCode::MovU32RegU32Reg => Instruction::MovU32RegU32Reg(R2, R3),
                OpCode::MovU32ImmMemRelSimple => Instruction::MovU32ImmMemRelSimple(0x123, 0x321),
                OpCode::MovU32RegMemRelSimple => Instruction::MovU32RegMemRelSimple(R2, 0x123),
                OpCode::MovMemU32RegRelSimple => Instruction::MovMemU32RegRelSimple(0x123, R2),
                OpCode::MovU32RegPtrU32RegRelSimple => {
                    Instruction::MovU32RegPtrU32RegRelSimple(R2, R3)
                }
                OpCode::MovU32ImmMemExprRel => Instruction::MovU32ImmMemExprRel(0x123, 0x321),
                OpCode::MovMemExprU32RegRel => Instruction::MovMemExprU32RegRel(0x123, R2),
                OpCode::MovU32RegMemExprRel => Instruction::MovU32RegMemExprRel(R2, 0x123),
                OpCode::ByteSwapU32 => Instruction::ByteSwapU32(R2),
                OpCode::ZeroHighBitsByIndexU32Reg => {
                    Instruction::ZeroHighBitsByIndexU32Reg(R2, R3, R4)
                }
                OpCode::ZeroHighBitsByIndexU32RegU32Imm => {
                    Instruction::ZeroHighBitsByIndexU32RegU32Imm(0x123, R2, R3)
                }
                OpCode::BitTestU32Reg => Instruction::BitTestU32Reg(0x40, R2),
                OpCode::BitTestU32Mem => Instruction::BitTestU32Mem(0x40, 0x123),
                OpCode::BitTestResetU32Reg => Instruction::BitTestResetU32Reg(0x40, R2),
                OpCode::BitTestResetU32Mem => Instruction::BitTestResetU32Mem(0x40, 0x123),
                OpCode::BitTestSetU32Reg => Instruction::BitTestSetU32Reg(0x40, R2),
                OpCode::BitTestSetU32Mem => Instruction::BitTestSetU32Mem(0x40, 0x123),
                OpCode::BitScanReverseU32RegU32Reg => {
                    Instruction::BitScanReverseU32RegU32Reg(R2, R3)
                }
                OpCode::BitScanReverseU32MemU32Reg => {
                    Instruction::BitScanReverseU32MemU32Reg(0x123, R2)
                }
                OpCode::BitScanReverseU32RegMemU32 => {
                    Instruction::BitScanReverseU32RegMemU32(R2, 0x123)
                }
                OpCode::BitScanReverseU32MemU32Mem => {
                    Instruction::BitScanReverseU32MemU32Mem(0x123, 0x321)
                }
                OpCode::BitScanForwardU32RegU32Reg => {
                    Instruction::BitScanForwardU32RegU32Reg(R2, R3)
                }
                OpCode::BitScanForwardU32MemU32Reg => {
                    Instruction::BitScanForwardU32MemU32Reg(0x123, R2)
                }
                OpCode::BitScanForwardU32RegMemU32 => {
                    Instruction::BitScanForwardU32RegMemU32(R2, 0x123)
                }
                OpCode::BitScanForwardU32MemU32Mem => {
                    Instruction::BitScanForwardU32MemU32Mem(0x123, 0x321)
                }
                OpCode::Ret => Instruction::Ret,
                OpCode::Mret => Instruction::Mret,
                OpCode::Hlt => Instruction::Hlt,
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
        for (i, (original, decompiled)) in decompiled_instructions
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
