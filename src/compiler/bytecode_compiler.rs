use crate::ins::{instruction::Instruction, op_codes::OpCode};

pub struct Compiler {}

impl Compiler {
    /// Compile a sequence of instructions into bytecode.
    ///
    /// # Arguments
    ///
    /// * `instructions` - A slice of [`Instruction`] objects to be compiled.
    ///
    /// # Returns
    ///
    /// A vector of u8 bytes containing the compiled data.
    pub fn compile(instructions: &[Instruction]) -> Vec<u8> {
        let mut compiled_bytes = Vec::new();

        for ins in instructions {
            let bytecode = Compiler::compile_instruction(ins);
            compiled_bytes.extend_from_slice(&bytecode);
        }

        compiled_bytes
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
    fn compile_instruction(instruction: &Instruction) -> Vec<u8> {
        let mut bytecode = Vec::new();

        // First, we push the bytes for the opcode.
        let opcode_value = OpCode::from(*instruction) as u32;
        let opcode_bytes = opcode_value.to_le_bytes();
        bytecode.extend_from_slice(&opcode_bytes);

        // Next, we need to push the argument bytes.
        match *instruction {
            Instruction::Nop => {}

            /******** [Arithmetic Instructions] ********/
            Instruction::AddU32ImmU32Reg(imm, reg) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.push(reg.into());
            }
            Instruction::AddU32RegU32Reg(in_reg, out_reg) => {
                bytecode.push(in_reg.into());
                bytecode.push(out_reg.into());
            }

            /******** [Bit Operation Instructions] ********/
            Instruction::LeftShiftU32ImmU32Reg(imm, reg) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.push(reg.into());
            }
            Instruction::LeftShiftU32RegU32Reg(shift_reg, reg) => {
                bytecode.push(shift_reg.into());
                bytecode.push(reg.into());
            }
            Instruction::ArithLeftShiftU32ImmU32Reg(imm, reg) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.push(reg.into());
            }
            Instruction::ArithLeftShiftU32RegU32Reg(shift_reg, reg) => {
                bytecode.push(shift_reg.into());
                bytecode.push(reg.into());
            }
            Instruction::RightShiftU32ImmU32Reg(imm, reg) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.push(reg.into());
            }
            Instruction::RightShiftU32RegU32Reg(shift_reg, reg) => {
                bytecode.push(shift_reg.into());
                bytecode.push(reg.into());
            }

            /******** [Move Instructions - NO EXPRESSIONS] ********/
            Instruction::SwapU32RegU32Reg(reg1, reg2) => {
                bytecode.push(reg1.into());
                bytecode.push(reg2.into());
            }
            Instruction::MovU32ImmU32Reg(imm, reg) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.push(reg.into());
            }
            Instruction::MovU32RegU32Reg(in_reg, out_reg) => {
                bytecode.push(in_reg.into());
                bytecode.push(out_reg.into());
            }
            Instruction::MovU32ImmMemRelSimple(imm, addr) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.extend_from_slice(&addr.to_le_bytes());
            }
            Instruction::MovU32RegMemRelSimple(reg, addr) => {
                bytecode.push(reg.into());
                bytecode.extend_from_slice(&addr.to_le_bytes());
            }
            Instruction::MovMemU32RegRelSimple(addr, reg) => {
                bytecode.extend_from_slice(&addr.to_le_bytes());
                bytecode.push(reg.into());
            }
            Instruction::MovU32RegPtrU32RegRelSimple(in_reg, out_reg) => {
                bytecode.push(in_reg.into());
                bytecode.push(out_reg.into());
            }

            /******** [Move Instructions - WITH EXPRESSIONS] ********/
            Instruction::MovU32ImmMemExprRel(imm, expr) => {
                bytecode.extend_from_slice(&imm.to_le_bytes());
                bytecode.extend_from_slice(&expr.to_le_bytes());
            }
            Instruction::MovMemExprU32RegRel(expr, reg) => {
                bytecode.extend_from_slice(&expr.to_le_bytes());
                bytecode.push(reg.into());
            }
            Instruction::MovU32RegMemExprRel(reg, expr) => {
                bytecode.push(reg.into());
                bytecode.extend_from_slice(&expr.to_le_bytes());
            }

            /******** [Special Instructions] ********/
            Instruction::Ret => todo!(),
            Instruction::Mret => {}
            Instruction::Hlt => {}
        }

        bytecode
    }
}
