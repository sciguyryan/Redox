use crate::{ins::instruction::Instruction, mem::memory::Memory};

#[allow(dead_code)]
pub struct Decompiler {}

#[allow(dead_code)]
impl Decompiler {
    pub fn decompile(bytes: &[u8]) -> Vec<Instruction> {
        let memory = Memory::from(bytes);
        memory.decompile_instructions(0, memory.len())
    }
}

#[cfg(test)]
mod tests_compiler {
    use crate::{compiler::bytecode_decompiler::Decompiler, ins::op_codes::OpCode};

    /// Test decompiling with invalid opcode.
    #[test]
    #[should_panic]
    fn test_decompile_invalid_opcode() {
        let bytes = u32::MAX.to_le_bytes();

        let _ = Decompiler::decompile(&bytes);
    }

    /// Test decompiling with insufficient instruction arguments.
    #[test]
    #[should_panic]
    fn test_decompile_insufficient_args() {
        // This instruction should have a u32 argument and a register ID argument.
        let bytes = (OpCode::AddU32ImmU32Reg as u32).to_le_bytes();

        let _ = Decompiler::decompile(&bytes);
    }

    /// Test decompiling with an invalid register ID argument.
    #[test]
    #[should_panic]
    fn test_decompile_invalid_register_id() {
        // This will have the correct number of arguments, but the register ID is invalid.
        let mut bytes = (OpCode::AddU32ImmU32Reg as u32).to_le_bytes().to_vec();

        // Add the u32 value.
        bytes.extend_from_slice(&0xDEADBEEFu32.to_le_bytes());

        // Add an invalid register ID.
        bytes.push(u8::MAX);

        let _ = Decompiler::decompile(&bytes);
    }
}
