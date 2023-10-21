use crate::ins::instruction::Instruction;

pub struct Compiler {}

impl Compiler {
    pub fn compile(instructions: &[Instruction]) -> Vec<u8> {
        let mut compiled_bytes: Vec<u8> = Vec::new();

        for ins in instructions {
            let bytecode = ins.get_bytecode();
            compiled_bytes.extend_from_slice(&bytecode);
        }

        println!("{:?}", compiled_bytes);
        compiled_bytes
    }
}
