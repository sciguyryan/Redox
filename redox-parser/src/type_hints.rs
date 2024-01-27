use redox_core::ins::op_codes::OpCode;

#[allow(unused)]
#[derive(Debug, Clone, Eq, PartialEq, Hash)]
pub enum ArgTypeHint {
    /// A f32 argument.
    F32,
    /// A f64 argument.
    F64,
    /// A u32 argument.
    U32,
    /// A u64 argument.
    U64,
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

/// The size of a f32 value, in bytes.
const SIZE_OF_F32: usize = 4;
/// The size of a f64 value, in bytes.
const SIZE_OF_F64: usize = 8;
/// The size of a u32 value, in bytes.
const SIZE_OF_U32: usize = 4;
/// The size of a u64 value, in bytes.
const SIZE_OF_U64: usize = 8;
/// The size of a u8 value, in bytes.
const SIZE_OF_U8: usize = 1;
/// The size of a register identifier, in bytes.
const SIZE_OF_REGISTER_ID: usize = 1;

#[derive(Debug, Clone)]
pub struct InstructionLookup<'a> {
    pub names: Vec<&'a str>,
    pub args: Vec<ArgTypeHint>,
    pub opcode: OpCode,
}

impl<'a> InstructionLookup<'a> {
    pub fn new(names: Vec<&'a str>, args: Vec<ArgTypeHint>, opcode: OpCode) -> Self {
        Self {
            names,
            opcode,
            args,
        }
    }

    pub fn total_argument_size(&self) -> usize {
        use ArgTypeHint as AH;

        let mut total = 0;
        for arg in &self.args {
            total += match arg {
                AH::F32 => SIZE_OF_F32,
                AH::F64 => SIZE_OF_F64,
                AH::U32 => SIZE_OF_U32,
                AH::U32Pointer => SIZE_OF_U32,
                AH::U64 => SIZE_OF_U64,
                AH::U8 => SIZE_OF_U8,
                AH::U8Pointer => SIZE_OF_U8,
                AH::RegisterF32 => SIZE_OF_REGISTER_ID,
                AH::RegisterU32 => SIZE_OF_REGISTER_ID,
                AH::RegisterU32Pointer => SIZE_OF_REGISTER_ID,
                AH::Expression => SIZE_OF_U32,
                AH::ExpressionPointer => SIZE_OF_U32,
            };
        }
        total
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
        use OpCode as O;

        Self {
            hints: vec![
                gen_hint!(vec!["nop"], [], O::Nop),
                /******** [Arithmetic Instructions] ********/
                gen_hint!(vec!["add"], [U32, RegisterU32], O::AddU32ImmU32Reg),
                gen_hint!(vec!["add"], [RegisterU32, RegisterU32], O::AddU32RegU32Reg),
                gen_hint!(vec!["sub"], [U32, RegisterU32], O::SubU32ImmU32Reg),
                gen_hint!(vec!["sub"], [RegisterU32, RegisterU32], O::SubU32RegU32Reg),
                gen_hint!(vec!["mul"], [U32], O::MulU32Imm),
                gen_hint!(vec!["mul"], [RegisterU32], O::MulU32Reg),
                gen_hint!(vec!["div"], [U32], O::DivU32Imm),
                gen_hint!(vec!["div"], [RegisterU32], O::DivU32Reg),
                gen_hint!(vec!["inc"], [RegisterU32], O::IncU32Reg),
                gen_hint!(vec!["dec"], [RegisterU32], O::DecU32Reg),
                gen_hint!(vec!["and"], [U32, RegisterU32], O::AndU32ImmU32Reg),
                /******** [Bit Operation Instructions] ********/
                gen_hint!(vec!["shl"], [U8, RegisterU32], O::LeftShiftU8ImmU32Reg),
                gen_hint!(
                    vec!["shl"],
                    [RegisterU32, RegisterU32],
                    O::LeftShiftU32RegU32Reg
                ),
                gen_hint!(vec!["sal"], [U8, RegisterU32], O::ArithLeftShiftU8ImmU32Reg),
                gen_hint!(
                    vec!["sal"],
                    [RegisterU32, RegisterU32],
                    O::ArithLeftShiftU32RegU32Reg
                ),
                gen_hint!(vec!["shr"], [U8, RegisterU32], O::RightShiftU8ImmU32Reg),
                gen_hint!(
                    vec!["shr"],
                    [RegisterU32, RegisterU32],
                    O::RightShiftU32RegU32Reg
                ),
                gen_hint!(
                    vec!["sar"],
                    [U8, RegisterU32],
                    O::ArithRightShiftU8ImmU32Reg
                ),
                gen_hint!(
                    vec!["sar"],
                    [RegisterU32, RegisterU32],
                    O::ArithRightShiftU32RegU32Reg
                ),
                /******** [Branching Instructions] ********/
                gen_hint!(vec!["call"], [U32Pointer], O::CallU32Imm),
                gen_hint!(vec!["call"], [RegisterU32Pointer], O::CallU32Reg),
                gen_hint!(vec!["iret"], [], O::RetArgsU32),
                gen_hint!(vec!["int"], [U8], O::Int),
                gen_hint!(vec!["intret"], [], O::IntRet),
                gen_hint!(vec!["jmp"], [U32Pointer], O::JumpAbsU32Imm),
                gen_hint!(vec!["jmp"], [RegisterU32Pointer], O::JumpAbsU32Reg),
                /******** [Data Instructions] ********/
                gen_hint!(
                    vec!["swap"],
                    [RegisterU32, RegisterU32],
                    O::SwapU32RegU32Reg
                ),
                gen_hint!(vec!["mov"], [U32, RegisterU32], O::MovU32ImmU32Reg),
                gen_hint!(vec!["mov"], [RegisterU32, RegisterU32], O::MovU32RegU32Reg),
                gen_hint!(vec!["mov"], [U32, U32Pointer], O::MovU32ImmMemSimple),
                gen_hint!(
                    vec!["mov"],
                    [RegisterU32, U32Pointer],
                    O::MovU32RegMemSimple
                ),
                gen_hint!(
                    vec!["mov"],
                    [U32Pointer, RegisterU32],
                    O::MovMemU32RegSimple
                ),
                gen_hint!(
                    vec!["mov"],
                    [RegisterU32Pointer, RegisterU32],
                    O::MovU32RegPtrU32RegSimple
                ),
                gen_hint!(vec!["mov"], [U32, ExpressionPointer], O::MovU32ImmMemExpr),
                gen_hint!(
                    vec!["mov"],
                    [ExpressionPointer, RegisterU32],
                    O::MovMemExprU32Reg
                ),
                gen_hint!(
                    vec!["mov"],
                    [RegisterU32, ExpressionPointer],
                    O::MovU32RegMemExpr
                ),
                gen_hint!(vec!["bswap"], [RegisterU32], O::ByteSwapU32),
                gen_hint!(
                    vec!["zhbi"],
                    [RegisterU32, RegisterU32, RegisterU32],
                    O::ZeroHighBitsByIndexU32Reg
                ),
                gen_hint!(
                    vec!["zhbi"],
                    [U32, RegisterU32, RegisterU32],
                    O::ZeroHighBitsByIndexU32RegU32Imm
                ),
                gen_hint!(vec!["push"], [F32], O::PushF32Imm),
                gen_hint!(vec!["push"], [U32], O::PushU32Imm),
                gen_hint!(vec!["push"], [RegisterU32], O::PushU32Reg),
                gen_hint!(vec!["pop"], [RegisterF32], O::PopF32ToF32Reg),
                gen_hint!(vec!["pop"], [RegisterU32], O::PopU32ToU32Reg),
                /******** [IO Instructions] ********/
                gen_hint!(vec!["outf"], [F32, U8], O::OutF32Imm),
                gen_hint!(vec!["outwd"], [U32, U8], O::OutU32Imm),
                gen_hint!(vec!["outrw"], [RegisterU32, U8], O::OutU32Reg),
                gen_hint!(vec!["outb"], [U8, U8], O::OutU8Imm),
                gen_hint!(vec!["inrb"], [U8, RegisterU32], O::InU8Reg),
                gen_hint!(vec!["inrw"], [U8, RegisterU32], O::InU32Reg),
                gen_hint!(vec!["inrf"], [U8, RegisterF32], O::InF32Reg),
                gen_hint!(vec!["inmb"], [U8, U32Pointer], O::InU8Mem),
                gen_hint!(vec!["inmw"], [U8, U32Pointer], O::InU32Mem),
                gen_hint!(vec!["inmf"], [U8, U32Pointer], O::InF32Mem),
                /******** [Logic Instructions] ********/
                gen_hint!(vec!["bt"], [U8, RegisterU32], O::BitTestU32Reg),
                gen_hint!(vec!["bt"], [U8, U32Pointer], O::BitTestU32Mem),
                gen_hint!(vec!["btr"], [U8, RegisterU32], O::BitTestResetU32Reg),
                gen_hint!(vec!["btr"], [U8, U32Pointer], O::BitTestResetU32Mem),
                gen_hint!(vec!["bts"], [U8, RegisterU32], O::BitTestSetU32Reg),
                gen_hint!(vec!["bts"], [U8, U32Pointer], O::BitTestSetU32Mem),
                gen_hint!(
                    vec!["bsr"],
                    [RegisterU32, RegisterU32],
                    O::BitScanReverseU32RegU32Reg
                ),
                gen_hint!(
                    vec!["bsr"],
                    [U32Pointer, RegisterU32],
                    O::BitScanReverseU32MemU32Reg
                ),
                gen_hint!(
                    vec!["bsr"],
                    [RegisterU32, U32Pointer],
                    O::BitScanReverseU32RegMemU32
                ),
                gen_hint!(
                    vec!["bsr"],
                    [U32Pointer, U32Pointer],
                    O::BitScanReverseU32MemU32Mem
                ),
                gen_hint!(
                    vec!["bsf"],
                    [RegisterU32, RegisterU32],
                    O::BitScanForwardU32RegU32Reg
                ),
                gen_hint!(
                    vec!["bsf"],
                    [U32Pointer, RegisterU32],
                    O::BitScanForwardU32MemU32Reg
                ),
                gen_hint!(
                    vec!["bsf"],
                    [RegisterU32, U32Pointer],
                    O::BitScanForwardU32RegMemU32
                ),
                gen_hint!(
                    vec!["bsf"],
                    [U32Pointer, U32Pointer],
                    O::BitScanForwardU32MemU32Mem
                ),
                /******** [Special Instructions] ********/
                gen_hint!(vec!["mint"], [U8], O::MaskInterrupt),
                gen_hint!(vec!["umint"], [U8], O::UnmaskInterrupt),
                gen_hint!(vec!["livt"], [U32Pointer], O::LoadIVTAddrU32Imm),
                gen_hint!(vec!["mret"], [], O::MachineReturn),
                gen_hint!(vec!["hlt"], [], O::Halt),
            ],
        }
    }
}

impl<'a> Default for InstructionHints<'a> {
    fn default() -> Self {
        Self::new()
    }
}
