use redox_core::ins::op_codes::OpCode;

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
    /// A f32 register argument.
    RegisterF32,
    /// A u32 register argument.
    RegisterU32,
    /// A register pointer argument (memory address).
    RegisterU32Pointer,
    /// An expression argument.
    Expression,
    /// An expression argument (memory address).
    ExpressionPointer,
}

#[derive(Debug, Clone)]
pub struct InstructionLookup<'a> {
    names: Vec<&'a str>,
    args: Vec<ArgTypeHint>,
    opcode: OpCode,
}

impl<'a> InstructionLookup<'a> {
    pub fn new(names: Vec<&'a str>, args: Vec<ArgTypeHint>, opcode: OpCode) -> Self {
        Self {
            names,
            opcode,
            args,
        }
    }
}

macro_rules! gen_hint {
    ($names:expr, $args:expr, $op:expr) => {{
        InstructionLookup::new($names, $args.to_vec(), $op)
    }};
}

#[derive(Debug, Clone)]
pub struct InstructionHints<'a> {
    pub hints: Vec<InstructionLookup<'a>>,
}

impl<'a> InstructionHints<'a> {
    pub fn new() -> Self {
        use ArgTypeHint::*;

        Self {
            hints: vec![
                gen_hint!(vec!["nop"], [], OpCode::Nop),
                /******** [Arithmetic Instructions] ********/
                gen_hint!(vec!["add"], [U32, RegisterU32], OpCode::AddU32ImmU32Reg),
                gen_hint!(
                    vec!["add"],
                    [RegisterU32, RegisterU32],
                    OpCode::AddU32RegU32Reg
                ),
                gen_hint!(vec!["sub"], [U32, RegisterU32], OpCode::SubU32ImmU32Reg),
                gen_hint!(vec!["sub"], [RegisterU32, U32], OpCode::SubU32RegU32Imm),
                gen_hint!(
                    vec!["sub"],
                    [RegisterU32, RegisterU32],
                    OpCode::SubU32RegU32Reg
                ),
                gen_hint!(vec!["mul"], [U32, RegisterU32], OpCode::MulU32ImmU32Reg),
                gen_hint!(
                    vec!["mul"],
                    [RegisterU32, RegisterU32],
                    OpCode::MulU32RegU32Reg
                ),
                gen_hint!(vec!["div"], [U32, RegisterU32], OpCode::DivU32ImmU32Reg),
                gen_hint!(vec!["div"], [RegisterU32, U32], OpCode::DivU32RegU32Imm),
                gen_hint!(
                    vec!["div"],
                    [RegisterU32, RegisterU32],
                    OpCode::DivU32RegU32Reg
                ),
                gen_hint!(vec!["mod"], [U32, RegisterU32], OpCode::ModU32ImmU32Reg),
                gen_hint!(vec!["mod"], [RegisterU32, U32], OpCode::ModU32RegU32Imm),
                gen_hint!(
                    vec!["mod"],
                    [RegisterU32, RegisterU32],
                    OpCode::ModU32RegU32Reg
                ),
                gen_hint!(vec!["inc"], [RegisterU32], OpCode::IncU32Reg),
                gen_hint!(vec!["dec"], [RegisterU32], OpCode::DecU32Reg),
                gen_hint!(vec!["and"], [U32, RegisterU32], OpCode::AndU32ImmU32Reg),
                /******** [Bit Operation Instructions] ********/
                gen_hint!(vec!["shl"], [U8, RegisterU32], OpCode::LeftShiftU8ImmU32Reg),
                gen_hint!(
                    vec!["shl"],
                    [RegisterU32, RegisterU32],
                    OpCode::LeftShiftU32RegU32Reg
                ),
                gen_hint!(
                    vec!["sal"],
                    [U8, RegisterU32],
                    OpCode::ArithLeftShiftU8ImmU32Reg
                ),
                gen_hint!(
                    vec!["sal"],
                    [RegisterU32, RegisterU32],
                    OpCode::ArithLeftShiftU32RegU32Reg
                ),
                gen_hint!(
                    vec!["shr"],
                    [U8, RegisterU32],
                    OpCode::RightShiftU8ImmU32Reg
                ),
                gen_hint!(
                    vec!["shr"],
                    [RegisterU32, RegisterU32],
                    OpCode::RightShiftU32RegU32Reg
                ),
                gen_hint!(
                    vec!["sar"],
                    [U8, RegisterU32],
                    OpCode::ArithRightShiftU8ImmU32Reg
                ),
                gen_hint!(
                    vec!["sar"],
                    [RegisterU32, RegisterU32],
                    OpCode::ArithRightShiftU32RegU32Reg
                ),
                /******** [Branching Instructions] ********/
                gen_hint!(vec!["call"], [U32Pointer], OpCode::CallU32Imm),
                gen_hint!(vec!["call"], [RegisterU32Pointer], OpCode::CallU32Reg),
                gen_hint!(vec!["iret"], [], OpCode::RetArgsU32),
                gen_hint!(vec!["int"], [U8], OpCode::Int),
                gen_hint!(vec!["intret"], [], OpCode::IntRet),
                gen_hint!(vec!["jmp"], [U32Pointer], OpCode::JumpAbsU32Imm),
                gen_hint!(vec!["jmp"], [RegisterU32Pointer], OpCode::JumpAbsU32Reg),
                /******** [Data Instructions] ********/
                gen_hint!(
                    vec!["swap"],
                    [RegisterU32, RegisterU32],
                    OpCode::SwapU32RegU32Reg
                ),
                gen_hint!(vec!["mov"], [U32, RegisterU32], OpCode::MovU32ImmU32Reg),
                gen_hint!(
                    vec!["mov"],
                    [RegisterU32, RegisterU32],
                    OpCode::MovU32RegU32Reg
                ),
                gen_hint!(vec!["mov"], [U32, U32Pointer], OpCode::MovU32ImmMemSimple),
                gen_hint!(
                    vec!["mov"],
                    [RegisterU32, U32Pointer],
                    OpCode::MovU32RegMemSimple
                ),
                gen_hint!(
                    vec!["mov"],
                    [U32Pointer, RegisterU32],
                    OpCode::MovMemU32RegSimple
                ),
                gen_hint!(
                    vec!["mov"],
                    [RegisterU32Pointer, RegisterU32],
                    OpCode::MovU32RegPtrU32RegSimple
                ),
                gen_hint!(
                    vec!["mov"],
                    [U32, ExpressionPointer],
                    OpCode::MovU32ImmMemExpr
                ),
                gen_hint!(
                    vec!["mov"],
                    [ExpressionPointer, RegisterU32],
                    OpCode::MovMemExprU32Reg
                ),
                gen_hint!(
                    vec!["mov"],
                    [RegisterU32, ExpressionPointer],
                    OpCode::MovU32RegMemExpr
                ),
                gen_hint!(vec!["bswap"], [RegisterU32], OpCode::ByteSwapU32),
                gen_hint!(
                    vec!["zhbi"],
                    [RegisterU32, RegisterU32, RegisterU32],
                    OpCode::ZeroHighBitsByIndexU32Reg
                ),
                gen_hint!(
                    vec!["zhbi"],
                    [U32, RegisterU32, RegisterU32],
                    OpCode::ZeroHighBitsByIndexU32RegU32Imm
                ),
                gen_hint!(vec!["push"], [F32], OpCode::PushF32Imm),
                gen_hint!(vec!["push"], [U32], OpCode::PushU32Imm),
                gen_hint!(vec!["push"], [RegisterU32], OpCode::PushU32Reg),
                gen_hint!(vec!["pop"], [RegisterF32], OpCode::PopF32ToF32Reg),
                gen_hint!(vec!["pop"], [RegisterU32], OpCode::PopU32ToU32Reg),
                /******** [IO Instructions] ********/
                gen_hint!(vec!["out", "out.f32"], [F32, U8], OpCode::OutF32Imm),
                gen_hint!(vec!["out", "out.i32"], [U32, U8], OpCode::OutU32Imm),
                gen_hint!(
                    vec!["out", "out.ir32"],
                    [RegisterU32, U8],
                    OpCode::OutU32Reg
                ),
                gen_hint!(vec!["out", "out.i8"], [U8, U8], OpCode::OutU8Imm),
                gen_hint!(vec!["in", "in.ir8"], [U8, RegisterU32], OpCode::InU8Reg),
                gen_hint!(vec!["in", "in.ir32"], [U8, RegisterU32], OpCode::InU32Reg),
                gen_hint!(vec!["in", "in.fr32"], [U8, RegisterF32], OpCode::InF32Reg),
                gen_hint!(vec!["in", "in.m8"], [U8, U32Pointer], OpCode::InU8Mem),
                gen_hint!(vec!["in", "in.m32"], [U8, U32Pointer], OpCode::InU32Mem),
                gen_hint!(vec!["in", "in.fm32"], [U8, U32Pointer], OpCode::InF32Mem),
                /******** [Logic Instructions] ********/
                gen_hint!(vec!["bt"], [U8, RegisterU32], OpCode::BitTestU32Reg),
                gen_hint!(vec!["bt"], [U8, U32Pointer], OpCode::BitTestU32Mem),
                gen_hint!(vec!["btr"], [U8, RegisterU32], OpCode::BitTestResetU32Reg),
                gen_hint!(vec!["btr"], [U8, U32Pointer], OpCode::BitTestResetU32Mem),
                gen_hint!(vec!["bts"], [U8, RegisterU32], OpCode::BitTestSetU32Reg),
                gen_hint!(vec!["bts"], [U8, U32Pointer], OpCode::BitTestSetU32Mem),
                gen_hint!(
                    vec!["bsr"],
                    [RegisterU32, RegisterU32],
                    OpCode::BitScanReverseU32RegU32Reg
                ),
                gen_hint!(
                    vec!["bsr"],
                    [U32Pointer, RegisterU32],
                    OpCode::BitScanReverseU32MemU32Reg
                ),
                gen_hint!(
                    vec!["bsr"],
                    [RegisterU32, U32Pointer],
                    OpCode::BitScanReverseU32RegMemU32
                ),
                gen_hint!(
                    vec!["bsr"],
                    [U32Pointer, U32Pointer],
                    OpCode::BitScanReverseU32MemU32Mem
                ),
                gen_hint!(
                    vec!["bsf"],
                    [RegisterU32, RegisterU32],
                    OpCode::BitScanForwardU32RegU32Reg
                ),
                gen_hint!(
                    vec!["bsf"],
                    [U32Pointer, RegisterU32],
                    OpCode::BitScanForwardU32MemU32Reg
                ),
                gen_hint!(
                    vec!["bsf"],
                    [RegisterU32, U32Pointer],
                    OpCode::BitScanForwardU32RegMemU32
                ),
                gen_hint!(
                    vec!["bsf"],
                    [U32Pointer, U32Pointer],
                    OpCode::BitScanForwardU32MemU32Mem
                ),
                /******** [Special Instructions] ********/
                gen_hint!(vec!["mint"], [U8], OpCode::MaskInterrupt),
                gen_hint!(vec!["umint"], [U8], OpCode::UnmaskInterrupt),
                gen_hint!(vec!["livt"], [U32Pointer], OpCode::LoadIVTAddrU32Imm),
                gen_hint!(vec!["mret"], [], OpCode::MachineReturn),
                gen_hint!(vec!["hlt"], [], OpCode::Halt),
            ],
        }
    }

    pub fn find_by(&self, name: &str, argument_hints: &[ArgTypeHint]) -> Option<OpCode> {
        self.hints
            .iter()
            .find(|h| h.names.contains(&name) && h.args == argument_hints)
            .map(|h| h.opcode)
    }
}

impl<'a> Default for InstructionHints<'a> {
    fn default() -> Self {
        Self::new()
    }
}
