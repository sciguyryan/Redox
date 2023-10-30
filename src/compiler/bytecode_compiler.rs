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
            | Instruction::BitScanForwardU32MemU32Reg(imm, reg) => {
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
            | Instruction::BitScanForwardU32RegU32Reg(reg1, reg2) => {
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
            | Instruction::BitScanForwardU32RegMemU32(reg, imm) => {
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
