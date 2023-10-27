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
        self.write_opcode(&OpCode::from(*instruction));

        // Next, we need to push the argument bytes.
        match *instruction {
            Instruction::Nop => {}

            /******** [Arithmetic Instructions] ********/
            Instruction::AddU32ImmU32Reg(imm, reg) => {
                self.write_u32(imm);
                self.write_register_id(&reg);
            }
            Instruction::AddU32RegU32Reg(in_reg, out_reg) => {
                self.write_register_id(&in_reg);
                self.write_register_id(&out_reg);
            }

            /******** [Bit Operation Instructions] ********/
            Instruction::LeftShiftU32ImmU32Reg(imm, reg) => {
                self.write_u32(imm);
                self.write_register_id(&reg);
            }
            Instruction::LeftShiftU32RegU32Reg(shift_reg, reg) => {
                self.write_register_id(&shift_reg);
                self.write_register_id(&reg);
            }
            Instruction::ArithLeftShiftU32ImmU32Reg(imm, reg) => {
                self.write_u32(imm);
                self.write_register_id(&reg);
            }
            Instruction::ArithLeftShiftU32RegU32Reg(shift_reg, reg) => {
                self.write_register_id(&shift_reg);
                self.write_register_id(&reg);
            }
            Instruction::RightShiftU32ImmU32Reg(imm, reg) => {
                self.write_u32(imm);
                self.write_register_id(&reg);
            }
            Instruction::RightShiftU32RegU32Reg(shift_reg, reg) => {
                self.write_register_id(&shift_reg);
                self.write_register_id(&reg);
            }

            /******** [Move Instructions - NO EXPRESSIONS] ********/
            Instruction::SwapU32RegU32Reg(reg1, reg2) => {
                self.write_register_id(&reg1);
                self.write_register_id(&reg2);
            }
            Instruction::MovU32ImmU32Reg(imm, reg) => {
                self.write_u32(imm);
                self.write_register_id(&reg);
            }
            Instruction::MovU32RegU32Reg(in_reg, out_reg) => {
                self.write_register_id(&in_reg);
                self.write_register_id(&out_reg);
            }
            Instruction::MovU32ImmMemRelSimple(imm, addr) => {
                self.write_u32(imm);
                self.write_u32(addr);
            }
            Instruction::MovU32RegMemRelSimple(reg, addr) => {
                self.write_register_id(&reg);
                self.write_u32(addr);
            }
            Instruction::MovMemU32RegRelSimple(addr, reg) => {
                self.write_u32(addr);
                self.write_register_id(&reg);
            }
            Instruction::MovU32RegPtrU32RegRelSimple(in_reg, out_reg) => {
                self.write_register_id(&in_reg);
                self.write_register_id(&out_reg);
            }

            /******** [Move Instructions - WITH EXPRESSIONS] ********/
            Instruction::MovU32ImmMemExprRel(imm, expr) => {
                self.write_u32(imm);
                self.write_u32(expr);
            }
            Instruction::MovMemExprU32RegRel(expr, reg) => {
                self.write_u32(expr);
                self.write_register_id(&reg);
            }
            Instruction::MovU32RegMemExprRel(reg, expr) => {
                self.write_register_id(&reg);
                self.write_u32(expr);
            }

            /******** [Special Instructions] ********/
            Instruction::Ret => todo!(),
            Instruction::Mret => {}
            Instruction::Hlt => {}
        }
    }

    /// Write an opcode into the byte sequence.
    ///
    /// # Arguments
    ///
    /// * `opcode` - The [`OpCode`] to be written.
    fn write_opcode(&mut self, opcode: &OpCode) {
        self.write_u32(*opcode as u32);
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
}
