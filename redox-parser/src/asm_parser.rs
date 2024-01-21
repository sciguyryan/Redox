use std::{num::ParseIntError, str::FromStr};

use itertools::Itertools;
use redox_core::{
    ins::{
        expressions::{
            Expression,
            {ExpressionArgs::*, ExpressionOperator::*},
        },
        instruction::Instruction,
        op_codes::OpCode,
    },
    reg::registers::RegisterId,
};

use crate::type_hints::{ArgTypeHint, InstructionLookup};

use super::type_hints::InstructionHints;

const F32_REGISTERS: [RegisterId; 2] = [RegisterId::FR1, RegisterId::FR2];

const U32_REGISTERS: [RegisterId; 18] = [
    RegisterId::ER1,
    RegisterId::ER2,
    RegisterId::ER3,
    RegisterId::ER4,
    RegisterId::ER5,
    RegisterId::ER6,
    RegisterId::ER7,
    RegisterId::ER8,
    RegisterId::EAC,
    RegisterId::EIP,
    RegisterId::EFP,
    RegisterId::ESP,
    RegisterId::EFL,
    RegisterId::EIM,
    RegisterId::IDTR,
    RegisterId::ESS,
    RegisterId::ECS,
    RegisterId::EDS,
];

#[derive(Debug, Clone)]
pub enum Argument {
    /// A f64 argument.
    Float(f64),
    /// A u128 argument.
    UnsignedInt(u128),
    /// A f32 register argument.
    RegisterF32(RegisterId),
    /// A u32 register argument.
    RegisterU32(RegisterId),
    /// An expression argument.
    Expression(Expression),
}

/// Cheekily get the inner value of an enum.
macro_rules! get_inner_arg {
    ($target:expr, $enum:path) => {{
        if let $enum(a) = $target {
            a
        } else {
            unreachable!();
        }
    }};
}

/// Cheekily get the inner value of an enum, with casting.
macro_rules! get_inner_arg_and_cast {
    ($target:expr, $enum:path, $cast:ident) => {{
        if let $enum(a) = $target {
            a as $cast
        } else {
            unreachable!();
        }
    }};
}

/// Cheekily get and pack an inner enum expression.
macro_rules! get_inner_expr_arg {
    ($target:expr) => {{
        if let Argument::Expression(expr) = &$target {
            expr.pack()
        } else {
            unreachable!()
        }
    }};
}

pub struct AsmParser<'a> {
    hints: InstructionHints<'a>,
}

impl<'a> AsmParser<'a> {
    pub fn new() -> Self {
        Self {
            hints: InstructionHints::new(),
        }
    }

    pub fn parse_code(&self, string: &str) -> Vec<Instruction> {
        let mut instructions: Vec<Instruction> = vec![];

        // Split the string into lines.
        for line in string.lines() {
            // Skip comment lines.
            if line.starts_with(';') {
                continue;
            }

            let raw_args = AsmParser::parse_instruction_line(line.trim_matches(' '));

            // The name should always be the first argument we've extracted, the arguments should follow.
            let name = raw_args[0].to_lowercase();

            // Used to hold the parsed arguments and the hints for the argument types.
            let mut arguments = vec![];
            let mut argument_hints = vec![];

            let shortlist: Vec<InstructionLookup> = self
                .hints
                .hints
                .iter()
                .filter(|h| h.names.contains(&name.as_str()))
                .cloned()
                .collect();

            // Do we have any arguments to process.
            for raw_arg in &raw_args[1..] {
                let mut has_value_pushed = false;
                let mut hints = vec![];

                let first_char = raw_arg.chars().nth(0).unwrap();

                // This will track whether the argument is a pointer.
                let is_pointer = first_char == '&';

                // Skip past the address prefix identifier, if it exists.
                let substring = if is_pointer { &raw_arg[1..] } else { raw_arg };

                /*
                 * IMPORTANT -
                 * it's important to check ALL numeric values in -reverse- size order since,
                 * for example, all u8 values could be a u32, but the reverse isn't true!
                 * This means that the -smallest- numeric type that can hold the value
                 * will be used, by default.
                 */

                if let Some((arg, hint)) = AsmParser::try_parse_expression(substring, is_pointer) {
                    // The argument could be an expression?
                    if !has_value_pushed {
                        arguments.push(arg);
                        has_value_pushed = true;
                    }

                    hints.push(hint);
                }

                if let Some((arg, hint)) = AsmParser::try_parse_register_id(substring, is_pointer) {
                    // The argument could be a register identifier?
                    if !has_value_pushed {
                        arguments.push(arg);
                        has_value_pushed = true;
                    }
                    hints.push(hint);
                }

                if let Ok(val) = AsmParser::try_parse_u8_immediate(substring) {
                    // The argument could be a u8 immediate.
                    if !has_value_pushed {
                        arguments.push(Argument::UnsignedInt(val as u128));
                        has_value_pushed = true;
                    }

                    if is_pointer {
                        // A u8 argument could also represent a u32 argument.
                        hints.push(ArgTypeHint::U8Pointer);
                    } else {
                        // A u8 argument could also represent a u32 argument.
                        hints.push(ArgTypeHint::U8);
                    }
                }

                if let Ok(val) = AsmParser::try_parse_u32_immediate(substring) {
                    // The argument could be a u32 immediate.
                    if !has_value_pushed {
                        arguments.push(Argument::UnsignedInt(val as u128));
                        has_value_pushed = true;
                    }

                    if is_pointer {
                        hints.push(ArgTypeHint::U32Pointer);
                    } else {
                        hints.push(ArgTypeHint::U32);
                    }
                }

                if substring.contains('.') {
                    // Was an address prefix used? This is invalid syntax since f32 values can't
                    // be used as pointers.
                    if is_pointer {
                        panic!(
                            "invalid syntax - unable to use a 32-bit floating value as a pointer!"
                        );
                    }

                    if let Ok(val) = f32::from_str(substring) {
                        // The argument could be a f32 immediate?
                        if !has_value_pushed {
                            arguments.push(Argument::Float(val as f64));
                            has_value_pushed = true;
                        }

                        hints.push(ArgTypeHint::F32);
                    }

                    if let Ok(val) = f64::from_str(substring) {
                        // The argument could be a f64 immediate?
                        if !has_value_pushed {
                            arguments.push(Argument::Float(val));

                            // TODO - Uncomment if further types are added below.
                            //has_value_pushed = true;
                        }

                        hints.push(ArgTypeHint::F64);
                    }

                    // The argument was expected to be a valid floating-point value,
                    // but we failed to parse it as such. We can't go any further.
                    if !hints
                        .iter()
                        .any(|h| *h == ArgTypeHint::F32 || *h == ArgTypeHint::F64)
                    {
                        panic!("Failed to parse floating-point value.");
                    }
                }

                argument_hints.push(hints);
            }

            // Calculate the multi-Cartesian product of the argument types.
            let arg_permutations = argument_hints
                .into_iter()
                .multi_cartesian_product()
                .collect_vec();

            let mut final_options = vec![];
            if !arguments.is_empty() {
                for permutation in arg_permutations {
                    for entry in &shortlist {
                        if entry.args == permutation {
                            final_options.push(entry);
                        }
                    }
                }
            } else {
                final_options = shortlist.iter().collect();
            }

            // Did we fail to find a match?
            // This can happen because a shortname isn't valid, or because the number or type
            // of arguments don't match.
            assert!(
                !final_options.is_empty(),
                "unable to find an instruction that matches the name and provided arguments."
            );

            // We will want to select the match with the lowest total argument size.
            final_options.sort_by_key(|a| a.total_argument_size());

            // Finally, the final match will be whatever entry has been sorted at the top
            // of our vector. The unwrap is safe since we know there is at least one.
            let final_option = final_options.first().unwrap();

            // Build our instruction and push it to the list.
            let ins = AsmParser::try_build_instruction(final_option.opcode, &arguments);
            instructions.push(ins);
        }

        instructions
    }

    /// Try to parse an expression.
    ///
    /// # Arguments
    ///
    /// * `string` - The input string.
    /// * `is_pointer` - Is this argument a pointer?
    #[inline]
    fn try_parse_expression(string: &str, is_pointer: bool) -> Option<(Argument, ArgTypeHint)> {
        // Expressions must start with an open square bracket and end with a close square bracket.
        let first_char = string.chars().nth(0).unwrap();
        let last_char = string.chars().last().unwrap();
        if first_char != '[' || last_char != ']' {
            return None;
        }

        // Skip over the brackets.
        let expr_substring = &string[1..string.len() - 1];

        // An expression should be composed of two or three components separated by an operator.
        // Each component may be either a register ID or a u8 value.
        let mut expr_arguments = vec![];

        let mut segment_end = false;
        let mut start_pos = 0;
        let mut end_pos = 0;

        let mut chars = expr_substring.chars().peekable();
        while let Some(c) = chars.next() {
            match c {
                ' ' | '+' | '-' | '*' => {
                    segment_end = true;
                }
                _ => {}
            }

            // We always want to be sure to catch the last character.
            if chars.peek().is_none() {
                segment_end = true;
                end_pos += 1;
            }

            if segment_end {
                let argument = &expr_substring[start_pos..end_pos];

                // If we have a non-empty string then we can add it to our processing list.
                if !argument.is_empty() {
                    if let Some((arg, _)) = AsmParser::try_parse_register_id(argument, false) {
                        // Do we have a register identifier.
                        let value = get_inner_arg!(arg, Argument::RegisterU32);
                        expr_arguments.push(Register(value));
                    } else if let Some((arg, _)) = AsmParser::try_parse_u8(argument, false) {
                        // Do we have a u8 immediate?
                        let value = get_inner_arg_and_cast!(arg, Argument::UnsignedInt, u8);
                        expr_arguments.push(Immediate(value));
                    } else {
                        // Something other than our permitted components.
                        break;
                    }
                }

                // Do we need to add an operator?
                match c {
                    '+' => {
                        expr_arguments.push(Operator(Add));
                    }
                    '-' => {
                        expr_arguments.push(Operator(Subtract));
                    }
                    '*' => {
                        expr_arguments.push(Operator(Multiply));
                    }
                    _ => {}
                }

                // Skip over the current character to the next one.
                start_pos = end_pos + 1;

                segment_end = false;
            }

            end_pos += 1;
        }

        // Is the expression argument list valid?
        if let Ok(expr) = Expression::try_from(&expr_arguments[..]) {
            if is_pointer {
                Some((Argument::Expression(expr), ArgTypeHint::ExpressionPointer))
            } else {
                Some((Argument::Expression(expr), ArgTypeHint::Expression))
            }
        } else {
            panic!("Invalid expression syntax - {expr_substring}");
        }
    }

    /// Try to parse an argument string as a u8.
    ///
    /// # Arguments
    ///
    /// * `string` - The input string.
    /// * `is_pointer` - Is this argument a pointer?
    #[inline]
    fn try_parse_u8(string: &str, is_pointer: bool) -> Option<(Argument, ArgTypeHint)> {
        match AsmParser::try_parse_u8_immediate(string) {
            Ok(val) => {
                if is_pointer {
                    Some((Argument::UnsignedInt(val as u128), ArgTypeHint::U8Pointer))
                } else {
                    Some((Argument::UnsignedInt(val as u128), ArgTypeHint::U8))
                }
            }
            Err(_) => None,
        }
    }

    /// Try to parse a a register ID.
    ///
    /// # Arguments
    ///
    /// * `string` - The input string.
    /// * `is_pointer` - Is this argument a pointer?
    #[inline]
    fn try_parse_register_id(string: &str, is_pointer: bool) -> Option<(Argument, ArgTypeHint)> {
        match RegisterId::from_str(string) {
            Ok(id) => {
                if U32_REGISTERS.contains(&id) {
                    if is_pointer {
                        Some((Argument::RegisterU32(id), ArgTypeHint::RegisterU32Pointer))
                    } else {
                        Some((Argument::RegisterU32(id), ArgTypeHint::RegisterU32))
                    }
                } else if F32_REGISTERS.contains(&id) {
                    if is_pointer {
                        panic!("It's not possible to use a f32 register as a pointer!")
                    } else {
                        Some((Argument::RegisterF32(id), ArgTypeHint::RegisterF32))
                    }
                } else {
                    panic!("Unclassified register identifier = {id}");
                }
            }
            Err(_) => None,
        }
    }

    /// Try to build an [`Instruction`] from an [`OpCode`] and a set of arguments.
    ///
    /// # Arguments
    ///
    /// * `opcode` - The [`OpCode`] for the instruction.
    /// * `args` - The arguments to be used to build the instruction.
    fn try_build_instruction(opcode: OpCode, args: &[Argument]) -> Instruction {
        use {Argument::*, OpCode::*};

        // This will only ever be called internally and since we have confirmed that the arguments
        // match those that would be needed for the instruction associated with the opcode, it's safe
        // to make some assumptions regarding the sanity of the data.
        match opcode {
            Nop => Instruction::Nop,

            /******** [Arithmetic Instructions] ********/
            AddU32ImmU32Reg => Instruction::AddU32ImmU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            AddU32RegU32Reg => Instruction::AddU32RegU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            SubU32ImmU32Reg => Instruction::SubU32ImmU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            SubU32RegU32Imm => Instruction::SubU32RegU32Imm(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),
            SubU32RegU32Reg => Instruction::SubU32RegU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            MulU32ImmU32Reg => Instruction::MulU32ImmU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            MulU32RegU32Reg => Instruction::MulU32RegU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            DivU32ImmU32Reg => Instruction::DivU32ImmU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            DivU32RegU32Imm => Instruction::DivU32RegU32Imm(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),
            DivU32RegU32Reg => Instruction::DivU32RegU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            ModU32ImmU32Reg => Instruction::ModU32ImmU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            ModU32RegU32Imm => Instruction::ModU32RegU32Imm(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),
            ModU32RegU32Reg => Instruction::ModU32RegU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            IncU32Reg => Instruction::IncU32Reg(get_inner_arg!(args[0], RegisterU32)),
            DecU32Reg => Instruction::DecU32Reg(get_inner_arg!(args[0], RegisterU32)),
            AndU32ImmU32Reg => Instruction::AndU32ImmU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg!(args[1], RegisterU32),
            ),

            /******** [Bit Operation Instructions] ********/
            LeftShiftU8ImmU32Reg => Instruction::LeftShiftU8ImmU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg!(args[1], RegisterU32),
            ),
            LeftShiftU32RegU32Reg => Instruction::LeftShiftU32RegU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            ArithLeftShiftU8ImmU32Reg => Instruction::ArithLeftShiftU8ImmU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg!(args[1], RegisterU32),
            ),
            ArithLeftShiftU32RegU32Reg => Instruction::ArithLeftShiftU32RegU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            RightShiftU8ImmU32Reg => Instruction::RightShiftU8ImmU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg!(args[1], RegisterU32),
            ),
            RightShiftU32RegU32Reg => Instruction::RightShiftU32RegU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            ArithRightShiftU8ImmU32Reg => Instruction::ArithRightShiftU8ImmU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg!(args[1], RegisterU32),
            ),
            ArithRightShiftU32RegU32Reg => Instruction::ArithRightShiftU32RegU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
            ),

            /******** [Branching Instructions] ********/
            CallU32Imm => {
                Instruction::CallU32Imm(get_inner_arg_and_cast!(args[0], UnsignedInt, u32))
            }
            CallU32Reg => Instruction::CallU32Reg(get_inner_arg!(args[0], RegisterU32)),
            RetArgsU32 => Instruction::RetArgsU32,
            Int => Instruction::Int(get_inner_arg_and_cast!(args[0], UnsignedInt, u8)),
            IntRet => Instruction::IntRet,
            JumpAbsU32Imm => {
                Instruction::JumpAbsU32Imm(get_inner_arg_and_cast!(args[0], UnsignedInt, u32))
            }
            JumpAbsU32Reg => Instruction::JumpAbsU32Reg(get_inner_arg!(args[0], RegisterU32)),

            /******** [Data Instructions] ********/
            SwapU32RegU32Reg => Instruction::SwapU32RegU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            MovU32ImmU32Reg => Instruction::MovU32ImmU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            MovU32RegU32Reg => Instruction::MovU32RegU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            MovU32ImmMemSimple => Instruction::MovU32ImmMemSimple(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),
            MovU32RegMemSimple => Instruction::MovU32RegMemSimple(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),
            MovMemU32RegSimple => Instruction::MovMemU32RegSimple(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            MovU32RegPtrU32RegSimple => Instruction::MovU32RegPtrU32RegSimple(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            MovU32ImmMemExpr => Instruction::MovU32ImmMemExpr(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_expr_arg!(args[1]),
            ),
            MovMemExprU32Reg => Instruction::MovMemExprU32Reg(
                get_inner_expr_arg!(args[0]),
                get_inner_arg!(args[1], RegisterU32),
            ),
            MovU32RegMemExpr => Instruction::MovU32RegMemExpr(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_expr_arg!(args[1]),
            ),
            ByteSwapU32 => Instruction::ByteSwapU32(get_inner_arg!(args[0], RegisterU32)),
            ZeroHighBitsByIndexU32Reg => Instruction::ZeroHighBitsByIndexU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
                get_inner_arg!(args[2], RegisterU32),
            ),
            ZeroHighBitsByIndexU32RegU32Imm => Instruction::ZeroHighBitsByIndexU32RegU32Imm(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg!(args[1], RegisterU32),
                get_inner_arg!(args[2], RegisterU32),
            ),
            PushF32Imm => Instruction::PushF32Imm(get_inner_arg_and_cast!(args[0], Float, f32)),
            PushU32Imm => {
                Instruction::PushU32Imm(get_inner_arg_and_cast!(args[0], UnsignedInt, u32))
            }
            PushU32Reg => Instruction::PushU32Reg(get_inner_arg!(args[0], RegisterU32)),
            PopF32ToF32Reg => Instruction::PopF32ToF32Reg(get_inner_arg!(args[0], RegisterF32)),
            PopU32ToU32Reg => Instruction::PopU32ToU32Reg(get_inner_arg!(args[0], RegisterU32)),

            /******** [IO Instructions] ********/
            OutF32Imm => Instruction::OutF32Imm(
                get_inner_arg_and_cast!(args[0], Float, f32),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u8),
            ),
            OutU32Imm => Instruction::OutU32Imm(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u8),
            ),
            OutU32Reg => Instruction::OutU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u8),
            ),
            OutU8Imm => Instruction::OutU8Imm(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u8),
            ),
            InU8Reg => Instruction::InU8Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg!(args[1], RegisterU32),
            ),
            InU8Mem => Instruction::InU8Mem(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),
            InU32Reg => Instruction::InU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg!(args[1], RegisterU32),
            ),
            InU32Mem => Instruction::InU32Mem(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),
            InF32Reg => Instruction::InF32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg!(args[1], RegisterF32),
            ),
            InF32Mem => Instruction::InF32Mem(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),

            /******** [Logic Instructions] ********/
            BitTestU32Reg => Instruction::BitTestU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg!(args[1], RegisterU32),
            ),
            BitTestU32Mem => Instruction::BitTestU32Mem(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),
            BitTestResetU32Reg => Instruction::BitTestResetU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg!(args[1], RegisterU32),
            ),
            BitTestResetU32Mem => Instruction::BitTestResetU32Mem(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),
            BitTestSetU32Reg => Instruction::BitTestSetU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg!(args[1], RegisterU32),
            ),
            BitTestSetU32Mem => Instruction::BitTestSetU32Mem(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u8),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),
            BitScanReverseU32RegU32Reg => Instruction::BitScanReverseU32RegU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            BitScanReverseU32MemU32Reg => Instruction::BitScanReverseU32MemU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            BitScanReverseU32RegMemU32 => Instruction::BitScanReverseU32RegMemU32(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),
            BitScanReverseU32MemU32Mem => Instruction::BitScanReverseU32MemU32Mem(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),
            BitScanForwardU32RegU32Reg => Instruction::BitScanForwardU32RegU32Reg(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            BitScanForwardU32MemU32Reg => Instruction::BitScanForwardU32MemU32Reg(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg!(args[1], RegisterU32),
            ),
            BitScanForwardU32RegMemU32 => Instruction::BitScanForwardU32RegMemU32(
                get_inner_arg!(args[0], RegisterU32),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),
            BitScanForwardU32MemU32Mem => Instruction::BitScanForwardU32MemU32Mem(
                get_inner_arg_and_cast!(args[0], UnsignedInt, u32),
                get_inner_arg_and_cast!(args[1], UnsignedInt, u32),
            ),

            /******** [Special Instructions] ********/
            MaskInterrupt => {
                Instruction::MaskInterrupt(get_inner_arg_and_cast!(args[0], UnsignedInt, u8))
            }
            UnmaskInterrupt => {
                Instruction::UnmaskInterrupt(get_inner_arg_and_cast!(args[0], UnsignedInt, u8))
            }
            LoadIVTAddrU32Imm => {
                Instruction::LoadIVTAddrU32Imm(get_inner_arg_and_cast!(args[0], UnsignedInt, u32))
            }
            MachineReturn => Instruction::MachineReturn,
            Halt => Instruction::Halt,

            /******** [Reserved Instructions] ********/
            Reserved1 => unreachable!(),
            Reserved2 => unreachable!(),
            Reserved3 => unreachable!(),
            Reserved4 => unreachable!(),
            Reserved5 => unreachable!(),
            Reserved6 => unreachable!(),
            Reserved7 => unreachable!(),
            Reserved8 => unreachable!(),
            Reserved9 => unreachable!(),

            /******** [Pseudo Instructions] ********/
            Label => unreachable!(),
            Unknown => unreachable!(),
        }
    }

    /// Try to parse a u8 immediate value.
    ///
    /// # Arguments
    ///
    /// * `string` - A string that may contain the value.
    fn try_parse_u8_immediate(string: &str) -> Result<u8, ParseIntError> {
        // Check for immediate values in specific bases.
        // I can't imagine these would be used much... but why not?
        if string.starts_with("0b") {
            // Binary.
            let stripped = string.strip_prefix("0b").expect("");
            u8::from_str_radix(stripped, 2)
        } else if string.starts_with("0o") {
            // Octal.
            let stripped = string.strip_prefix("0o").expect("");
            u8::from_str_radix(stripped, 8)
        } else if string.starts_with("0x") {
            // Hex.
            let stripped = string.strip_prefix("0x").expect("");
            u8::from_str_radix(stripped, 16)
        } else {
            string.parse::<u8>()
        }
    }

    /// Try to parse a u32 immediate value.
    ///
    /// # Arguments
    ///
    /// * `string` - A string that may contain the value.
    fn try_parse_u32_immediate(string: &str) -> Result<u32, ParseIntError> {
        // Check for immediate values in specific bases.
        // I can't imagine these would be used much... but why not?
        if string.starts_with("0b") {
            // Binary.
            let stripped = string.strip_prefix("0b").expect("");
            u32::from_str_radix(stripped, 2)
        } else if string.starts_with("0o") {
            // Octal.
            let stripped = string.strip_prefix("0o").expect("");
            u32::from_str_radix(stripped, 8)
        } else if string.starts_with("0x") {
            // Hex.
            let stripped = string.strip_prefix("0x").expect("");
            u32::from_str_radix(stripped, 16)
        } else {
            string.parse::<u32>()
        }
    }

    /// Try to parse an instruction line.
    ///
    /// # Arguments
    ///
    /// * `line` - A string giving the line to be parsed.
    pub fn parse_instruction_line(line: &str) -> Vec<String> {
        let mut segments = Vec::with_capacity(5);

        let mut skip_to_end = false;
        let mut segment_end = false;
        let mut start_pos = 0;
        let mut end_pos = 0;
        let mut chars = line.chars().peekable();

        while let Some(c) = chars.next() {
            // What type of character are we dealing with?
            match c {
                ' ' | ',' => {
                    segment_end = true;
                }
                ';' => {
                    segment_end = true;
                    skip_to_end = true;
                }
                _ => {}
            }

            // We always want to be sure to catch the last character.
            if chars.peek().is_none() {
                segment_end = true;
                end_pos += 1;
            }

            if segment_end {
                let string = &line[start_pos..end_pos];

                // If we have a non-empty string then we can add it to our processing list.
                if !string.is_empty() {
                    segments.push(string.to_string());
                }

                // Skip over the current character to the next one.
                start_pos = end_pos + 1;

                segment_end = false;
            }

            if skip_to_end {
                break;
            }

            end_pos += 1;
        }

        segments
    }
}

impl<'a> Default for AsmParser<'a> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests_asm_parsing {
    use std::panic;

    use strum::IntoEnumIterator;

    use redox_core::{
        ins::{instruction::Instruction, op_codes::OpCode},
        reg::registers::RegisterId,
    };

    use crate::asm_parser::AsmParser;

    #[derive(Clone)]
    struct ParserTest {
        /// The input string to be tested.
        pub input: String,
        /// A vector of [`Instruction`]s that should result from the parsing.
        pub expected_instructions: Vec<Instruction>,
        /// A boolean indicating whether the test should panic or not.
        pub should_panic: bool,
        /// A string slice that provides the message to be displayed if the test fails.
        pub fail_message: String,
    }

    impl ParserTest {
        /// Create a new [`ParserTest`] instance.
        ///
        /// # Arguments
        ///
        /// * `input` - The input string to be tested.
        /// * `parsed_instructions` - A slice of [`Instruction`]s that should result from the parsing.
        /// * `should_panic` - A boolean indicating whether the test should panic or not.
        /// * `fail_message` - A string slice that provides the message to be displayed if the test fails.
        ///
        /// # Note
        ///
        /// If the results need to check the user segment memory contents then the VM will automatically be
        /// created with a memory segment of the correct size. It doesn't need to be specified manually.
        fn new(
            input: &str,
            expected_instructions: &[Instruction],
            should_panic: bool,
            fail_message: &str,
        ) -> Self {
            Self {
                input: input.to_string(),
                expected_instructions: expected_instructions.to_vec(),
                should_panic,
                fail_message: fail_message.to_string(),
            }
        }

        /// Run this specific test entry.
        ///
        /// # Arguments
        ///
        /// * `id` - The ID of this test.
        pub fn run_test(&self, id: usize) {
            let parser = AsmParser::new();

            // Attempt to execute the code.
            let result = panic::catch_unwind(|| parser.parse_code(&self.input));

            // Confirm whether the test panicked, and whether that panic was expected or not.
            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                self.should_panic,
                "{}",
                self.fail_message(id, did_panic)
            );

            // We don't have a viable list to interrogate here.
            if !did_panic {
                assert_eq!(result.unwrap(), self.expected_instructions);
            }
        }

        /// Generate a fail message for this test instance.
        ///
        /// # Arguments
        ///
        /// * `id` - The ID of this test.
        /// * `did_panic` - Did the test panic?
        pub fn fail_message(&self, id: usize, did_panic: bool) -> String {
            format!(
                "Test {id} Failed - Should Panic? {}, Panicked? {did_panic}. Message = {}",
                self.should_panic, self.fail_message
            )
        }
    }

    struct ParserTests {
        tests: Vec<ParserTest>,
    }

    impl ParserTests {
        pub fn new(tests: &[ParserTest]) -> Self {
            Self {
                tests: tests.to_vec(),
            }
        }

        /// Run each unit test in the specified sequence.
        pub fn run_all(&self) {
            for (id, test) in self.tests.iter().enumerate() {
                test.run_test(id);
            }
        }
    }

    #[test]
    fn code_parser_comments() {
        let tests = [
            ParserTest::new(
                "call &0xdeadbeef ; this comment should be ignored.",
                &[Instruction::CallU32Imm(0xdeadbeef)],
                false,
                "failed to correctly parse instruction and comment.",
            ),
            ParserTest::new(
                "nop\r\n;this should be ignored\r\nnop",
                &[Instruction::Nop, Instruction::Nop],
                false,
                "failed to correctly parse instruction and comment.",
            ),
            ParserTest::new(
                "nop\r\n;nop\r\nnop",
                &[Instruction::Nop, Instruction::Nop],
                false,
                "failed to correctly parse instruction and comment.",
            ),
        ];

        ParserTests::new(&tests).run_all();
    }

    /// Instruction parsing tests - all invalid.
    #[test]
    fn code_parser_tests_invalid() {
        let tests = [
            // This is invalid because the argument to call must be an integer register or integer immediate pointer.
            ParserTest::new(
                "call 0xdeadbeef",
                &[],
                true,
                "succeeded in parsing instruction with invalid arguments.",
            ),
            // This is invalid because the argument to call must be an integer register or integer immediate pointer.
            ParserTest::new(
                "call ER1",
                &[],
                true,
                "succeeded in parsing instruction with invalid arguments.",
            ),
            // This is invalid because the argument to call must be an integer register or integer immediate pointer.
            ParserTest::new(
                "call 0.1234",
                &[],
                true,
                "succeeded in parsing instruction with invalid arguments.",
            ),
            // This is invalid because the argument to call must be an integer register or integer immediate pointer.
            ParserTest::new(
                "call &FP1",
                &[],
                true,
                "succeeded in parsing instruction with invalid arguments.",
            ),
            // This is invalid because the register ID doesn't exist.
            ParserTest::new(
                "call &AAA",
                &[],
                true,
                "succeeded in parsing instruction with invalid arguments.",
            ),
            // This is invalid because we have an open square bracket (indicating an expression) but no
            // closing one. This is invalid syntax.
            ParserTest::new(
                "mov &[ER1*2, ER1",
                &[],
                true,
                "succeeded in parsing instruction with invalid arguments.",
            ),
            // This is invalid because we have a closing square bracket (indicating an expression) but no
            // opening one. This is invalid syntax.
            ParserTest::new(
                "mov &ER1*2], ER1",
                &[],
                true,
                "succeeded in parsing instruction with invalid arguments.",
            ),
            // This is invalid because the expression containing an invalid register.
            ParserTest::new(
                "mov &[ERQ*2], ER1",
                &[],
                true,
                "succeeded in parsing instruction with invalid arguments.",
            ),
            // This is invalid because the value is larger than supported by a u8 value.
            // In an expression we may only use u8 values.
            ParserTest::new(
                "mov &[ER1*999], ER1",
                &[],
                true,
                "succeeded in parsing instruction with invalid arguments.",
            ),
            // This is invalid because we can't use floats in expressions.
            ParserTest::new(
                "mov &[ER1*1.0], ER1",
                &[],
                true,
                "succeeded in parsing instruction with invalid arguments.",
            ),
            // This is invalid because we have an operator with nothing following it.
            ParserTest::new(
                "mov &[ER1*], ER1",
                &[],
                true,
                "succeeded in parsing instruction with invalid arguments.",
            ),
            // This is invalid because we have an operator with nothing preceding it.
            ParserTest::new(
                "mov &[*ER1], ER1",
                &[],
                true,
                "succeeded in parsing instruction with invalid arguments.",
            ),
            // This is invalid because we have an operator with nothing following it.
            ParserTest::new(
                "mov &[ER1*ER2*], ER1",
                &[],
                true,
                "succeeded in parsing instruction with invalid arguments.",
            ),
            // This is invalid because we have too many arguments within the expression.
            // We may, at most, have three values.
            ParserTest::new(
                "mov &[ER1*ER2*ER3*ER4], ER1",
                &[],
                true,
                "succeeded in parsing instruction with invalid arguments.",
            ),
        ];

        ParserTests::new(&tests).run_all();
    }

    /// Single instruction parsing tests - with numbers in different bases.
    #[test]
    fn code_parser_tests_numeric_bases() {
        let tests = [
            ParserTest::new(
                "push 0b1111111111",
                &[Instruction::PushU32Imm(0b1111111111)],
                false,
                "failed to parse instruction with u32 argument - binary edition.",
            ),
            ParserTest::new(
                "push 0o12345",
                &[Instruction::PushU32Imm(0o12345)],
                false,
                "failed to parse instruction with u32 argument - octal edition.",
            ),
            ParserTest::new(
                "push 1234",
                &[Instruction::PushU32Imm(1234)],
                false,
                "failed to parse instruction with u32 argument - decimal edition.",
            ),
            ParserTest::new(
                "push 0x1234",
                &[Instruction::PushU32Imm(0x1234)],
                false,
                "failed to parse instruction with u32 argument - hex edition.",
            ),
        ];

        ParserTests::new(&tests).run_all();
    }

    #[test]
    fn parse_instruction_round_trip() {
        use redox_core::{
            ins::{
                expressions::{Expression, ExpressionArgs::*, ExpressionOperator::*},
                op_codes::OpCode::*,
            },
            reg::registers::RegisterId::*,
        };

        let asm_parser = AsmParser::new();

        let expr = Expression::try_from(&[Immediate(0x8), Operator(Add), Immediate(0x8)][..])
            .expect("")
            .pack();

        let mut instructions = Vec::new();

        // This might seem a little long-winded, but it's done this way to ensure
        // that each time a new instruction is added that a corresponding entry
        // is added here.
        for opcode in OpCode::iter() {
            let ins = match opcode {
                Nop => Instruction::Nop,
                AddU32ImmU32Reg => Instruction::AddU32ImmU32Reg(0x123, ER2),
                AddU32RegU32Reg => Instruction::AddU32RegU32Reg(ER2, ER3),
                SubU32ImmU32Reg => Instruction::SubU32ImmU32Reg(0x123, ER2),
                SubU32RegU32Imm => Instruction::SubU32RegU32Imm(ER2, 0x123),
                SubU32RegU32Reg => Instruction::SubU32RegU32Reg(ER2, ER3),
                MulU32ImmU32Reg => Instruction::MulU32ImmU32Reg(0x123, ER2),
                MulU32RegU32Reg => Instruction::MulU32RegU32Reg(ER2, ER3),
                DivU32ImmU32Reg => Instruction::DivU32ImmU32Reg(0x123, ER2),
                DivU32RegU32Imm => Instruction::DivU32RegU32Imm(ER2, 0x123),
                DivU32RegU32Reg => Instruction::DivU32RegU32Reg(ER2, ER3),
                ModU32ImmU32Reg => Instruction::ModU32ImmU32Reg(0x123, ER2),
                ModU32RegU32Imm => Instruction::ModU32RegU32Imm(ER2, 0x123),
                ModU32RegU32Reg => Instruction::ModU32RegU32Reg(ER2, ER3),
                IncU32Reg => Instruction::IncU32Reg(ER2),
                DecU32Reg => Instruction::DecU32Reg(ER2),
                AndU32ImmU32Reg => Instruction::AndU32ImmU32Reg(0x123, ER2),
                LeftShiftU8ImmU32Reg => Instruction::LeftShiftU8ImmU32Reg(31, ER2),
                LeftShiftU32RegU32Reg => Instruction::LeftShiftU32RegU32Reg(ER2, ER3),
                ArithLeftShiftU8ImmU32Reg => Instruction::ArithLeftShiftU8ImmU32Reg(31, ER2),
                ArithLeftShiftU32RegU32Reg => Instruction::ArithLeftShiftU32RegU32Reg(ER2, ER3),
                RightShiftU8ImmU32Reg => Instruction::RightShiftU8ImmU32Reg(31, ER2),
                RightShiftU32RegU32Reg => Instruction::RightShiftU32RegU32Reg(ER2, ER3),
                ArithRightShiftU8ImmU32Reg => Instruction::ArithRightShiftU8ImmU32Reg(31, ER2),
                ArithRightShiftU32RegU32Reg => Instruction::ArithRightShiftU32RegU32Reg(ER2, ER3),
                CallU32Imm => Instruction::CallU32Imm(0xdeafbeef),
                CallU32Reg => Instruction::CallU32Reg(RegisterId::ER2),
                RetArgsU32 => Instruction::RetArgsU32,
                Int => Instruction::Int(0xff),
                IntRet => Instruction::IntRet,
                JumpAbsU32Imm => Instruction::JumpAbsU32Imm(0xdeadbeef),
                JumpAbsU32Reg => Instruction::JumpAbsU32Reg(ER1),
                SwapU32RegU32Reg => Instruction::SwapU32RegU32Reg(ER2, ER3),
                MovU32ImmU32Reg => Instruction::MovU32ImmU32Reg(0x123, ER2),
                MovU32RegU32Reg => Instruction::MovU32RegU32Reg(ER2, ER3),
                MovU32ImmMemSimple => Instruction::MovU32ImmMemSimple(0x123, 0x321),
                MovU32RegMemSimple => Instruction::MovU32RegMemSimple(ER2, 0x123),
                MovMemU32RegSimple => Instruction::MovMemU32RegSimple(0x123, ER2),
                MovU32RegPtrU32RegSimple => Instruction::MovU32RegPtrU32RegSimple(ER2, ER3),
                MovU32ImmMemExpr => Instruction::MovU32ImmMemExpr(0x321, expr),
                MovMemExprU32Reg => Instruction::MovMemExprU32Reg(expr, ER2),
                MovU32RegMemExpr => Instruction::MovU32RegMemExpr(ER2, expr),
                ByteSwapU32 => Instruction::ByteSwapU32(ER2),
                ZeroHighBitsByIndexU32Reg => Instruction::ZeroHighBitsByIndexU32Reg(ER2, ER3, ER4),
                ZeroHighBitsByIndexU32RegU32Imm => {
                    Instruction::ZeroHighBitsByIndexU32RegU32Imm(0x123, ER2, ER3)
                }
                PushU32Imm => Instruction::PushU32Imm(0x123),
                PushF32Imm => Instruction::PushF32Imm(0.1),
                PushU32Reg => Instruction::PushU32Reg(ER2),
                PopF32ToF32Reg => Instruction::PopF32ToF32Reg(FR2),
                PopU32ToU32Reg => Instruction::PopU32ToU32Reg(ER2),
                OutF32Imm => Instruction::OutF32Imm(1.0, 0xab),
                OutU32Imm => Instruction::OutU32Imm(0xdeadbeef, 0xab),
                OutU32Reg => Instruction::OutU32Reg(ER2, 0xab),
                OutU8Imm => Instruction::OutU8Imm(0xba, 0xab),
                InU8Reg => Instruction::InU8Reg(0xab, ER2),
                InU8Mem => Instruction::InU8Mem(0xab, 0xdeadbeef),
                InU32Reg => Instruction::InU32Reg(0xab, ER2),
                InU32Mem => Instruction::InU32Mem(0xab, 0xdeadbeef),
                InF32Reg => Instruction::InF32Reg(0xab, FR2),
                InF32Mem => Instruction::InF32Mem(0xab, 0xdeadbeef),
                BitTestU32Reg => Instruction::BitTestU32Reg(0x40, ER2),
                BitTestU32Mem => Instruction::BitTestU32Mem(0x40, 0x123),
                BitTestResetU32Reg => Instruction::BitTestResetU32Reg(0x40, ER2),
                BitTestResetU32Mem => Instruction::BitTestResetU32Mem(0x40, 0x123),
                BitTestSetU32Reg => Instruction::BitTestSetU32Reg(0x40, ER2),
                BitTestSetU32Mem => Instruction::BitTestSetU32Mem(0x40, 0x123),
                BitScanReverseU32RegU32Reg => Instruction::BitScanReverseU32RegU32Reg(ER2, ER3),
                BitScanReverseU32MemU32Reg => Instruction::BitScanReverseU32MemU32Reg(0x123, ER2),
                BitScanReverseU32RegMemU32 => Instruction::BitScanReverseU32RegMemU32(ER2, 0x123),
                BitScanReverseU32MemU32Mem => Instruction::BitScanReverseU32MemU32Mem(0x123, 0x321),
                BitScanForwardU32RegU32Reg => Instruction::BitScanForwardU32RegU32Reg(ER2, ER3),
                BitScanForwardU32MemU32Reg => Instruction::BitScanForwardU32MemU32Reg(0x123, ER2),
                BitScanForwardU32RegMemU32 => Instruction::BitScanForwardU32RegMemU32(ER2, 0x123),
                BitScanForwardU32MemU32Mem => Instruction::BitScanForwardU32MemU32Mem(0x123, 0x321),
                MaskInterrupt => Instruction::MaskInterrupt(0xff),
                UnmaskInterrupt => Instruction::UnmaskInterrupt(0xff),
                LoadIVTAddrU32Imm => Instruction::LoadIVTAddrU32Imm(0xdeadbeef),
                MachineReturn => Instruction::MachineReturn,
                Halt => Instruction::Halt,

                // We don't want to test constructing these instructions.
                Reserved1 | Reserved2 | Reserved3 | Reserved4 | Reserved5 | Reserved6
                | Reserved7 | Reserved8 | Reserved9 | Label | Unknown => continue,
            };

            instructions.push(ins);
        }

        let mut failed = false;
        for (i, ins) in instructions.iter().enumerate() {
            let ins_str = ins.to_string();
            let parsed = panic::catch_unwind(|| asm_parser.parse_code(&ins_str));
            if let Ok(p) = parsed {
                let result = p.first().expect("");
                if result != ins {
                    println!("Test {i} failed! Original = {ins:?}, actual = {result:?}.");
                    failed = true;
                }
            } else {
                println!("Test {i} failed! No matching instruction was found. Input = {ins_str}");
                failed = true;
            }
        }

        assert!(!failed, "one or more tests failed to correctly round-trip");
    }
}
