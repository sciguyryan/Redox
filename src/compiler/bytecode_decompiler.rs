use num_traits::FromPrimitive;

use crate::{
    ins::{instruction::Instruction, op_codes::OpCode},
    reg::registers::RegisterId,
};

pub struct Decompiler<'a> {
    bytes: &'a [u8],
    cursor: usize,
}

impl<'a> Decompiler<'a> {
    pub fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, cursor: 0 }
    }

    fn len(&self) -> usize {
        self.bytes.len()
    }

    pub fn read(&mut self, len: usize) -> Option<&'a [u8]> {
        let start = self.cursor;
        let end = start + len;
        if end >= self.len() {
            return None;
        }

        self.cursor += len;

        Some(&self.bytes[start..end])
    }

    pub fn read_next_instruction(&mut self) -> Option<Instruction> {
        // Ensure we have enough data to read four bytes.
        let start = self.cursor;
        let end = start + 4;
        if end > self.len() {
            return None;
        }

        let opcode_bytes: [u8; 4] = self.bytes[start..end].try_into().expect("");
        self.cursor += 4;

        // Are we handling an extended instruction or a basic one?
        let opcode_id = u32::from_le_bytes(opcode_bytes);

        // Validate the opcode is one of the ones we know about.
        let opcode_validate: Option<OpCode> = FromPrimitive::from_u32(opcode_id);
        let opcode = match opcode_validate {
            Some(op) => op,
            None => panic!("unexpected opcode: {opcode_id}"),
        };

        let ins = match opcode {
            OpCode::Nop => Instruction::Nop,

            /******** [Arithmetic Instructions] ********/
            OpCode::AddU32ImmU32Reg => {
                let imm = self.read_u32();
                let reg = self.read_register_id();

                if imm.is_none() {
                    eprintln!("AddU32ImmU32Reg - no immediate value for first argument.");
                    return None;
                }

                if reg.is_none() {
                    eprintln!("AddU32ImmU32Reg - no valid register ID for second argument.");
                    return None;
                }

                Instruction::AddU32ImmU32Reg(imm.unwrap(), reg.unwrap())
            }
            OpCode::AddU32RegU32Reg => {
                let reg_1 = self.read_register_id();
                let reg_2 = self.read_register_id();

                if reg_1.is_none() {
                    eprintln!("AddU32RegU32Reg - no valid register ID for first argument.");
                    return None;
                }

                if reg_2.is_none() {
                    eprintln!("AddU32RegU32Reg - no valid register ID for second argument.");
                    return None;
                }

                Instruction::AddU32RegU32Reg(reg_1.unwrap(), reg_2.unwrap())
            }

            /******** [Simple Move Instructions - NO EXPRESSIONS] ********/
            OpCode::SwapU32RegU32Reg => {
                let reg1 = self.read_register_id();
                let reg2 = self.read_register_id();

                if reg1.is_none() {
                    eprintln!("SwapU32RegU32Reg - no valid register ID for first argument.");
                    return None;
                }

                if reg2.is_none() {
                    eprintln!("SwapU32RegU32Reg - no valid register ID for second argument.");
                    return None;
                }

                Instruction::SwapU32RegU32Reg(reg1.unwrap(), reg2.unwrap())
            }
            OpCode::MovU32ImmU32Reg => {
                let imm = self.read_u32();
                let reg = self.read_register_id();

                if imm.is_none() {
                    eprintln!("MovU32ImmU32Reg - no immediate value for first argument.");
                    return None;
                }

                if reg.is_none() {
                    eprintln!("MovU32ImmU32Reg - no valid register ID for second argument.");
                    return None;
                }

                Instruction::MovU32ImmU32Reg(imm.unwrap(), reg.unwrap())
            }
            OpCode::MovU32RegU32Reg => {
                let in_reg = self.read_register_id();
                let out_reg = self.read_register_id();

                if in_reg.is_none() {
                    eprintln!("MovU32RegU32Reg - no valid register ID for first argument.");
                    return None;
                }

                if out_reg.is_none() {
                    eprintln!("MovU32RegU32Reg - no valid register ID for second argument.");
                    return None;
                }

                Instruction::MovU32RegU32Reg(in_reg.unwrap(), out_reg.unwrap())
            }
            OpCode::MovU32ImmMemRelSimple => {
                let imm = self.read_u32();
                let addr = self.read_u32();

                if imm.is_none() {
                    eprintln!("MovU32ImmMemRelSimple - no valid immediate for first argument.");
                    return None;
                }

                if addr.is_none() {
                    eprintln!("MovU32ImmMemRelSimple - no valid address for second argument.");
                    return None;
                }

                Instruction::MovU32ImmMemRelSimple(imm.unwrap(), addr.unwrap())
            }
            OpCode::MovU32RegMemRelSimple => {
                let reg = self.read_register_id();
                let addr = self.read_u32();

                if reg.is_none() {
                    eprintln!("MovU32RegMemRelSimple - no valid register ID for first argument.");
                    return None;
                }

                if addr.is_none() {
                    eprintln!("MovU32RegMemRelSimple - no valid address for second argument.");
                    return None;
                }

                Instruction::MovU32RegMemRelSimple(reg.unwrap(), addr.unwrap())
            }
            OpCode::MovMemU32RegRelSimple => {
                let addr = self.read_u32();
                let reg = self.read_register_id();

                if addr.is_none() {
                    eprintln!("MovMemU32RegRelSimple - no valid address for first argument.");
                    return None;
                }

                if reg.is_none() {
                    eprintln!("MovMemU32RegRelSimple - no valid register ID for second argument.");
                    return None;
                }

                Instruction::MovMemU32RegRelSimple(addr.unwrap(), reg.unwrap())
            }
            OpCode::MovU32RegPtrU32RegRelSimple => {
                let in_reg = self.read_register_id();
                let out_reg = self.read_register_id();

                if in_reg.is_none() {
                    eprintln!(
                        "MovU32RegPtrU32RegRelSimple - no valid register ID for first argument."
                    );
                    return None;
                }

                if out_reg.is_none() {
                    eprintln!(
                        "MovU32RegPtrU32RegRelSimple - no valid register ID for second argument."
                    );
                    return None;
                }

                Instruction::MovU32RegPtrU32RegRelSimple(in_reg.unwrap(), out_reg.unwrap())
            }

            /******** [Complex Move Instructions - WITH EXPRESSIONS] ********/
            OpCode::MovU32ImmMemRelExpr => {
                let imm = self.read_u32();
                let expr = self.read_u32();

                if imm.is_none() {
                    eprintln!("MovU32ImmMemRelExpr - no valid immediate for first argument.");
                    return None;
                }

                if expr.is_none() {
                    eprintln!("MovU32ImmMemRelExpr - no valid expression for second argument.");
                    return None;
                }

                Instruction::MovU32ImmMemRelExpr(imm.unwrap(), expr.unwrap())
            }

            /******** [Special Instructions] ********/
            OpCode::Ret => Instruction::Ret,
            OpCode::Mret => Instruction::Mret,
            OpCode::Hlt => Instruction::Hlt,
        };

        //println!("opcode = {:#?}", opcode);

        Some(ins)
    }

    fn read_register_id(&mut self) -> Option<RegisterId> {
        let id = self.read_u8()?;

        FromPrimitive::from_u8(*id)
    }

    fn read_u8(&mut self) -> Option<&u8> {
        let pos = self.cursor;
        if pos >= self.len() {
            return None;
        }

        self.cursor += 1;
        Some(&self.bytes[pos])
    }

    fn read_u16(&mut self) -> Option<u16> {
        let start = self.cursor;
        let end = start + 2;
        if end >= self.len() {
            return None;
        }

        let mut bytes: [u8; 2] = [0; 2];
        (0..2).for_each(|i| {
            bytes[i] = self.bytes[self.cursor];
            self.cursor += 1;
        });

        Some(u16::from_le_bytes(bytes))
    }

    fn read_u32(&mut self) -> Option<u32> {
        let start = self.cursor;
        let end = start + 4;
        if end >= self.len() {
            return None;
        }

        let mut bytes: [u8; 4] = [0; 4];
        (0..4).for_each(|i| {
            bytes[i] = self.bytes[self.cursor];
            self.cursor += 1;
        });

        Some(u32::from_le_bytes(bytes))
    }

    pub fn decompile(&mut self) -> Vec<Instruction> {
        let mut instructions = Vec::new();

        println!("compiled: {:?}", self.bytes);

        while let Some(ins) = self.read_next_instruction() {
            instructions.push(ins);
        }

        println!("instructions: {instructions:?}");

        instructions
    }
}
