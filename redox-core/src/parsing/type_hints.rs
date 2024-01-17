use crate::ins::op_codes::OpCode;

#[derive(Debug, Clone, Eq, PartialEq, Hash)]
pub enum ArgTypeHint {
    /// A f32 argument.
    F32,
    /// A u32 argument.
    U32,
    /// A u32 pointer argument (memory address).
    U32Pointer,
    /// A u8 argument.
    U8,
    /// A u8 pointer argument (memory address).
    U8Pointer,
    /// A register argument.
    Register,
    /// A register pointer argument (memory address).
    RegisterPointer,
    /// An expression argument.
    Expression,
    /// An expression argument (memory address).
    ExpressionPointer,
}

#[derive(Debug, Clone)]
pub struct InstructionLookup {
    name: String,
    args: Vec<ArgTypeHint>,
    opcode: OpCode,
}

impl InstructionLookup {
    pub fn new(name: String, args: Vec<ArgTypeHint>, opcode: OpCode) -> Self {
        Self { name, opcode, args }
    }
}

macro_rules! gen_hint {
    ($name:expr, $args:expr, $op:expr) => {{
        InstructionLookup::new($name.to_string(), $args.to_vec(), $op)
    }};
}

#[derive(Debug, Clone)]
pub struct InstructionHints {
    pub hints: Vec<InstructionLookup>,
}

impl InstructionHints {
    pub fn new() -> Self {
        use ArgTypeHint::*;

        Self {
            hints: vec![
                gen_hint!("nop", [], OpCode::Nop),
                /******** [Arithmetic Instructions] ********/
                gen_hint!("add", [U32, Register], OpCode::AddU32ImmU32Reg),
                /******** [Bit Operation Instructions] ********/


                /******** [Branching Instructions] ********/
                gen_hint!("call", [U32Pointer], OpCode::CallU32Imm),
                gen_hint!("call", [RegisterPointer], OpCode::CallU32Reg),
                /******** [Data Instructions] ********/
                gen_hint!(
                    "mov",
                    [ExpressionPointer, RegisterPointer],
                    OpCode::MovMemExprU32Reg
                ),
                gen_hint!("push", [F32], OpCode::PushF32Imm),
                gen_hint!("push", [U32], OpCode::PushU32Imm),
                gen_hint!("push", [Register], OpCode::PushU32Reg),
                gen_hint!("pop", [Register], OpCode::PopF32ToF32Reg),
                gen_hint!("pop", [Register], OpCode::PopU32ToU32Reg),
                /******** [IO Instructions] ********/


                /******** [Logic Instructions] ********/
                gen_hint!(
                    "bsr",
                    [U32Pointer, U32Pointer],
                    OpCode::BitScanReverseU32MemU32Mem
                ),
                /******** [Special Instructions] ********/


                /******** [Reserved Instructions] ********/


                /******** [Pseudo Instructions] ********/
            ],
        }
    }

    pub fn find_by(&self, name: &str, argument_hints: &[ArgTypeHint]) -> Option<OpCode> {
        self.hints
            .iter()
            .find(|h| h.name == name && h.args == argument_hints)
            .map(|h| h.opcode)
    }
}

impl Default for InstructionHints {
    fn default() -> Self {
        Self::new()
    }
}
