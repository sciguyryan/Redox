use crate::ins::op_codes::OpCode;

#[derive(Debug, Clone)]
pub enum ArgTypeHint {
    F32,
    U32,
    U8,
    Register,
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
