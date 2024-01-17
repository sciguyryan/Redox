use std::{num::ParseIntError, str::FromStr};

use crate::{
    ins::{
        expressions::{Expression, ExpressionArgs, ExpressionOperator},
        instruction::Instruction,
        op_codes::OpCode,
    },
    parsing::type_hints::ArgTypeHint,
    reg::registers::RegisterId,
};

use super::type_hints::InstructionHints;

#[derive(Debug, Clone)]
pub enum Argument {
    /// A f32 argument.
    F32(f32),
    /// A u32 argument.
    U32(u32),
    /// A u8 argument.
    U8(u8),
    /// A register argument.
    Register(RegisterId),
    /// An expression argument.
    Expression(Expression),
}

/// Cheekily get the inner value of an enum.
macro_rules! get_inner {
    ($target:expr, $enum:path) => {{
        if let $enum(a) = $target {
            a
        } else {
            unreachable!();
        }
    }};
}

/// Cheekily get and pack an inner enum expression.
macro_rules! get_inner_expr {
    ($target:expr) => {{
        if let Argument::Expression(expr) = &$target {
            expr.pack()
        } else {
            unreachable!()
        }
    }};
}

pub struct AsmParser {
    hints: InstructionHints,
}

impl AsmParser {
    pub fn new() -> Self {
        Self {
            hints: InstructionHints::new(),
        }
    }

    pub fn parse_code(&self, string: &str) -> Vec<Instruction> {
        let mut instructions: Vec<Instruction> = vec![];

        // Split the string into lines.
        for line in string.lines() {
            let segments = AsmParser::parse_instruction_line(line.trim_matches(' '));
            //println!("{segments:?}");

            // The name should always be the first argument we've extracted, the arguments should follow.
            let name = segments[0].to_lowercase();
            //println!("------------------[Instruction {name}]------------------");

            // Used to hold the parsed arguments and the hints for the argument types.
            let mut arguments = vec![];
            let mut argument_hints = vec![];

            // Do we have any arguments?
            for segment in &segments[1..] {
                //println!("------------------[Argument {i}]------------------");

                let first_char = segment.chars().nth(0).unwrap();

                // This will track whether the argument is a pointer.
                let is_pointer = first_char == '&';

                // Skip past the address prefix identifier, if it exists.
                let substring = if is_pointer { &segment[1..] } else { segment };

                // Is the argument an expression?
                if let Some((arg, hint)) = AsmParser::try_parse_expression(substring, is_pointer) {
                    arguments.push(arg);
                    argument_hints.push(hint);
                    continue;
                }

                // Is the argument a register identifier?
                if let Some((arg, hint)) = AsmParser::try_parse_register_id(substring, is_pointer) {
                    arguments.push(arg);
                    argument_hints.push(hint);
                    continue;
                }

                // IMPORTANT
                // it's important to check ALL numeric values in reverse size order since,
                // for example, all u8 values could be parsed as a u32, but the reverse isn't true!

                // Is the argument a f32 immediate?
                if let Some((arg, hint)) = AsmParser::try_parse_f32(substring, is_pointer) {
                    arguments.push(arg);
                    argument_hints.push(hint);
                    continue;
                }

                // Is the argument a u8 immediate?
                if let Some((arg, hint)) = AsmParser::try_parse_u8(substring, is_pointer) {
                    arguments.push(arg);
                    argument_hints.push(hint);
                    continue;
                }

                // Is the argument a u32 immediate?
                if let Some((arg, hint)) = AsmParser::try_parse_u32(substring, is_pointer) {
                    arguments.push(arg);
                    argument_hints.push(hint);
                    continue;
                }
            }

            //println!("------------------------------------");
            //println!("Instruction name = '{name}', argument types = {argument_hints:?}");

            match self.hints.find_by(name.as_str(), &argument_hints) {
                Some(op) => {
                    let ins = AsmParser::try_build_instruction(op, &arguments);
                    //println!("Matching instruction found! {ins}");
                    instructions.push(ins);
                }
                None => {
                    panic!("unable to find an instruction that matches the name and provided arguments.");
                }
            }
        }

        instructions
    }

    /// Try to parse an expression.
    ///
    /// # Arguments
    ///
    /// * `is_pointer` - Is this argument a pointer?
    #[inline]
    fn try_parse_expression(string: &str, is_pointer: bool) -> Option<(Argument, ArgTypeHint)> {
        //print!("Checking for an expression... ");
        // Expressions must start with an open square bracket and end with a close square bracket.
        let first_char = string.chars().nth(0).unwrap();
        let last_char = string.chars().last().unwrap();
        if first_char != '[' || last_char != ']' {
            //println!("no match.");
            return None;
        } else {
            //println!("found! Parsing and validating...");
            //println!("-----");
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
                        let value = get_inner!(arg, Argument::Register);
                        expr_arguments.push(ExpressionArgs::Register(value));
                    } else if let Some((arg, _)) = AsmParser::try_parse_u8(argument, false) {
                        // Do we have a u8 immediate?
                        let value = get_inner!(arg, Argument::U8);
                        expr_arguments.push(ExpressionArgs::Immediate(value));
                    } else {
                        // Something other than our permitted components.
                        break;
                    }
                }

                // Do we need to add an operator?
                match c {
                    '+' => {
                        expr_arguments.push(ExpressionArgs::Operator(ExpressionOperator::Add));
                    }
                    '-' => {
                        expr_arguments.push(ExpressionArgs::Operator(ExpressionOperator::Subtract));
                    }
                    '*' => {
                        expr_arguments.push(ExpressionArgs::Operator(ExpressionOperator::Multiply));
                    }
                    _ => {}
                }

                // Skip over the current character to the next one.
                start_pos = end_pos + 1;

                segment_end = false;
            }

            end_pos += 1;
        }

        //println!("-----");

        // Is the expression argument list valid?
        if let Ok(expr) = Expression::try_from(&expr_arguments[..]) {
            //println!("found! Expression = {expr}.");
            if is_pointer {
                Some((Argument::Expression(expr), ArgTypeHint::ExpressionPointer))
            } else {
                Some((Argument::Expression(expr), ArgTypeHint::Expression))
            }
        } else {
            panic!("Invalid expression syntax - {expr_substring}");
        }
    }

    /// Try to parse an argument string as a u32.
    ///
    /// # Arguments
    ///
    /// * `is_pointer` - Is this argument a pointer?
    #[inline]
    fn try_parse_u32(string: &str, is_pointer: bool) -> Option<(Argument, ArgTypeHint)> {
        //print!("Checking for a u32 immediate... ");
        match AsmParser::try_parse_u32_immediate(string) {
            Ok(val) => {
                if is_pointer {
                    //println!("found! Value = {val} (pointer).");
                    Some((Argument::U32(val), ArgTypeHint::U32Pointer))
                } else {
                    //println!("found! Value = {val}.");
                    Some((Argument::U32(val), ArgTypeHint::U32))
                }
            }
            Err(_) => {
                //println!("no match.");
                None
            }
        }
    }

    /// Try to parse an argument string as a u8.
    ///
    /// # Arguments
    ///
    /// * `is_pointer` - Is this argument a pointer?
    #[inline]
    fn try_parse_u8(string: &str, is_pointer: bool) -> Option<(Argument, ArgTypeHint)> {
        //print!("Checking for a u8 immediate... ");
        match AsmParser::try_parse_u8_immediate(string) {
            Ok(val) => {
                if is_pointer {
                    //println!("found! Value = {val} (pointer).");
                    Some((Argument::U8(val), ArgTypeHint::U8Pointer))
                } else {
                    //println!("found! Value = {val}.");
                    Some((Argument::U8(val), ArgTypeHint::U8))
                }
            }
            Err(_) => {
                //println!("no match.");
                None
            }
        }
    }

    /// Try to parse an argument string as a u8.
    ///
    /// # Arguments
    ///
    /// * `is_pointer` - Is this argument a pointer?
    #[inline]
    fn try_parse_f32(string: &str, is_pointer: bool) -> Option<(Argument, ArgTypeHint)> {
        //print!("Checking for a f32 immediate... ");
        if !string.contains('.') {
            return None;
        }

        // Was an address prefix used? If so that is invalid syntax.
        if is_pointer {
            panic!("invalid syntax - unable to use a 32-bit floating value as a pointer!");
        }

        if let Ok(val) = f32::from_str(string) {
            //println!("found! Value = {val}.");
            Some((Argument::F32(val), ArgTypeHint::F32))
        } else {
            None
        }
    }

    #[inline]
    fn try_parse_register_id(string: &str, is_pointer: bool) -> Option<(Argument, ArgTypeHint)> {
        //print!("Checking for a register identifier... ");
        match RegisterId::from_str(string) {
            Ok(id) => {
                if is_pointer {
                    //println!("found! ID = {id} (pointer).");
                    Some((Argument::Register(id), ArgTypeHint::RegisterPointer))
                } else {
                    //println!("found! ID = {id}.");
                    Some((Argument::Register(id), ArgTypeHint::Register))
                }
            }
            Err(_) => {
                //println!("no match.");
                None
            }
        }
    }

    fn try_build_instruction(opcode: OpCode, args: &[Argument]) -> Instruction {
        // This will only ever be called internally and since we have confirmed that the arguments
        // match those that would be needed for the instruction associated with the opcode, it's safe
        // to make some assumptions regarding the sanity of the data.
        match opcode {
            OpCode::Nop => Instruction::Nop,

            /******** [Arithmetic Instructions] ********/
            OpCode::AddU32ImmU32Reg => Instruction::AddU32ImmU32Reg(
                get_inner!(args[0], Argument::U32),
                get_inner!(args[1], Argument::Register),
            ),
            OpCode::AddU32RegU32Reg => todo!(),
            OpCode::SubU32ImmU32Reg => todo!(),
            OpCode::SubU32RegU32Imm => todo!(),
            OpCode::SubU32RegU32Reg => todo!(),
            OpCode::MulU32ImmU32Reg => todo!(),
            OpCode::MulU32RegU32Reg => todo!(),
            OpCode::DivU32ImmU32Reg => todo!(),
            OpCode::DivU32RegU32Imm => todo!(),
            OpCode::DivU32RegU32Reg => todo!(),
            OpCode::ModU32ImmU32Reg => todo!(),
            OpCode::ModU32RegU32Imm => todo!(),
            OpCode::ModU32RegU32Reg => todo!(),
            OpCode::IncU32Reg => todo!(),
            OpCode::DecU32Reg => todo!(),
            OpCode::AndU32ImmU32Reg => todo!(),

            /******** [Bit Operation Instructions] ********/
            OpCode::LeftShiftU32ImmU32Reg => todo!(),
            OpCode::LeftShiftU32RegU32Reg => todo!(),
            OpCode::ArithLeftShiftU32ImmU32Reg => todo!(),
            OpCode::ArithLeftShiftU32RegU32Reg => todo!(),
            OpCode::RightShiftU32ImmU32Reg => todo!(),
            OpCode::RightShiftU32RegU32Reg => todo!(),
            OpCode::ArithRightShiftU32ImmU32Reg => todo!(),
            OpCode::ArithRightShiftU32RegU32Reg => todo!(),

            /******** [Branching Instructions] ********/
            OpCode::CallU32Imm => Instruction::CallU32Imm(get_inner!(args[0], Argument::U32)),
            OpCode::CallU32Reg => Instruction::CallU32Reg(get_inner!(args[0], Argument::Register)),
            OpCode::RetArgsU32 => todo!(),
            OpCode::Int => todo!(),
            OpCode::IntRet => todo!(),
            OpCode::JumpAbsU32Imm => todo!(),
            OpCode::JumpAbsU32Reg => todo!(),

            /******** [Data Instructions] ********/
            OpCode::SwapU32RegU32Reg => todo!(),
            OpCode::MovU32ImmU32Reg => todo!(),
            OpCode::MovU32RegU32Reg => todo!(),
            OpCode::MovU32ImmMemSimple => todo!(),
            OpCode::MovU32RegMemSimple => todo!(),
            OpCode::MovMemU32RegSimple => todo!(),
            OpCode::MovU32RegPtrU32RegSimple => todo!(),
            OpCode::MovU32ImmMemExpr => todo!(),
            OpCode::MovMemExprU32Reg => Instruction::MovMemExprU32Reg(
                get_inner_expr!(args[0]),
                get_inner!(args[1], Argument::Register),
            ),
            OpCode::MovU32RegMemExpr => todo!(),
            OpCode::ByteSwapU32 => todo!(),
            OpCode::ZeroHighBitsByIndexU32Reg => todo!(),
            OpCode::ZeroHighBitsByIndexU32RegU32Imm => todo!(),
            OpCode::PushU32Imm => Instruction::PushU32Imm(get_inner!(args[0], Argument::U32)),
            OpCode::PushF32Imm => Instruction::PushF32Imm(get_inner!(args[0], Argument::F32)),
            OpCode::PushU32Reg => Instruction::PushU32Reg(get_inner!(args[0], Argument::Register)),
            OpCode::PopF32ToF32Reg => {
                Instruction::PopF32ToF32Reg(get_inner!(args[0], Argument::Register))
            }
            OpCode::PopU32ToU32Reg => {
                Instruction::PopU32ToU32Reg(get_inner!(args[0], Argument::Register))
            }

            /******** [IO Instructions] ********/
            OpCode::OutF32Imm => todo!(),
            OpCode::OutU32Imm => todo!(),
            OpCode::OutU32Reg => todo!(),
            OpCode::OutU8Imm => todo!(),
            OpCode::InU8Reg => todo!(),
            OpCode::InU8Mem => todo!(),
            OpCode::InU32Reg => todo!(),
            OpCode::InU32Mem => todo!(),
            OpCode::InF32Reg => todo!(),
            OpCode::InF32Mem => todo!(),

            /******** [Logic Instructions] ********/
            OpCode::BitTestU32Reg => todo!(),
            OpCode::BitTestU32Mem => todo!(),
            OpCode::BitTestResetU32Reg => todo!(),
            OpCode::BitTestResetU32Mem => todo!(),
            OpCode::BitTestSetU32Reg => todo!(),
            OpCode::BitTestSetU32Mem => todo!(),
            OpCode::BitScanReverseU32RegU32Reg => todo!(),
            OpCode::BitScanReverseU32MemU32Reg => todo!(),
            OpCode::BitScanReverseU32RegMemU32 => todo!(),
            OpCode::BitScanReverseU32MemU32Mem => Instruction::BitScanReverseU32MemU32Mem(
                get_inner!(args[0], Argument::U32),
                get_inner!(args[1], Argument::U32),
            ),
            OpCode::BitScanForwardU32RegU32Reg => todo!(),
            OpCode::BitScanForwardU32MemU32Reg => todo!(),
            OpCode::BitScanForwardU32RegMemU32 => todo!(),
            OpCode::BitScanForwardU32MemU32Mem => todo!(),

            /******** [Special Instructions] ********/
            OpCode::MaskInterrupt => todo!(),
            OpCode::UnmaskInterrupt => todo!(),
            OpCode::LoadIVTAddrU32Imm => todo!(),
            OpCode::MachineReturn => todo!(),
            OpCode::Halt => todo!(),

            /******** [Reserved Instructions] ********/
            OpCode::Reserved1 => unreachable!(),
            OpCode::Reserved2 => unreachable!(),
            OpCode::Reserved3 => unreachable!(),
            OpCode::Reserved4 => unreachable!(),
            OpCode::Reserved5 => unreachable!(),
            OpCode::Reserved6 => unreachable!(),
            OpCode::Reserved7 => unreachable!(),
            OpCode::Reserved8 => unreachable!(),
            OpCode::Reserved9 => unreachable!(),

            /******** [Pseudo Instructions] ********/
            OpCode::Label => unreachable!(),
            OpCode::Unknown => unreachable!(),
        }
    }

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

    pub fn parse_instruction_line(line: &str) -> Vec<String> {
        let mut segments = Vec::with_capacity(5);

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

            end_pos += 1;
        }

        segments
    }
}

impl Default for AsmParser {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests_asm_parsing {
    use std::panic;

    use crate::{
        ins::instruction::Instruction, parsing::asm_parser::AsmParser, reg::registers::RegisterId,
    };

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

    /// Single instruction parsing tests - all valid.
    #[test]
    fn code_parser_tests_single_valid() {
        let tests = [
            ParserTest::new(
                "nop",
                &[Instruction::Nop],
                false,
                "failed to parse single instruction with no arguments.",
            ),
            ParserTest::new(
                "push 1234",
                &[Instruction::PushU32Imm(1234)],
                false,
                "failed to parse single instruction with one u32 argument.",
            ),
            ParserTest::new(
                "push 0.1",
                &[Instruction::PushF32Imm(0.1)],
                false,
                "failed to parse single instruction with one f32 argument.",
            ),
            ParserTest::new(
                "push ER1",
                &[Instruction::PushU32Reg(RegisterId::ER1)],
                false,
                "failed to parse single instruction with one u32 register argument.",
            ),
            ParserTest::new(
                "call &0xdeadbeef",
                &[Instruction::CallU32Imm(0xdeadbeef)],
                false,
                "failed to parse single instruction with one u32 immediate pointer argument.",
            ),
            ParserTest::new(
                "call &ER1",
                &[Instruction::CallU32Reg(RegisterId::ER1)],
                false,
                "failed to parse single instruction with one u32 register pointer argument.",
            ),
        ];

        ParserTests::new(&tests).run_all();
    }

    /// Single instruction parsing tests - all invalid.
    #[test]
    fn code_parser_tests_single_invalid() {
        let tests = [
            ParserTest::new(
                "call 0xdeadbeef",
                &[],
                true,
                "succeeded in finding an instruction with invalid arguments.",
            ),
            ParserTest::new(
                "call ER1",
                &[],
                true,
                "succeeded in finding an instruction with invalid arguments.",
            ),
            ParserTest::new(
                "call 0.1234",
                &[],
                true,
                "succeeded in finding an instruction with invalid arguments.",
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
}
