use crate::ins::op_codes::OpCode;

#[derive(Debug, Clone)]
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
    /// An expression argument (memory address.
    Expression,
}

#[derive(Debug, Clone)]
pub struct InstructionHint {
    name: String,
    args: Vec<ArgTypeHint>,
    opcode: OpCode,
}

impl InstructionHint {
    pub fn new(name: String, args: Vec<ArgTypeHint>, opcode: OpCode) -> Self {
        Self { name, opcode, args }
    }
}

macro_rules! hint {
    ($name:expr, $args:expr, $op:expr) => {{
        InstructionHint::new($name.to_string(), $args.to_vec(), $op)
    }};
}

#[derive(Debug, Clone)]
pub struct InstructionHints {
    pub hints: Vec<InstructionHint>,
}

impl InstructionHints {
    pub fn new() -> Self {
        Self {
            hints: vec![hint!(
                "add",
                [ArgTypeHint::U32, ArgTypeHint::Register],
                OpCode::AddU32ImmU32Reg
            )],
        }
    }
}

impl Default for InstructionHints {
    fn default() -> Self {
        Self::new()
    }
}
