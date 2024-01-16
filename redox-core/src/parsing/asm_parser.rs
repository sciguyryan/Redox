use std::{num::ParseIntError, str::FromStr};

use crate::{
    ins::{instruction::Instruction, op_codes::OpCode},
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
    /// An expression argument (memory address.
    Expression,
}

/// Cheekily get the inner value of an enum.
macro_rules! get_inner {
    ($target:expr, $enum:path) => {{
        if let $enum(a) = $target {
            a
        } else {
            panic!();
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

    pub fn parse(&self, string: &str) -> Vec<Instruction> {
        let mut instructions: Vec<Instruction> = vec![];

        // Split the string into lines.
        for line in string.lines() {
            let segments = AsmParser::parse_instruction_line(line.trim_matches(' '));
            println!("{segments:?}");

            // Next, we need to try and parse the arguments to see if we can figure out
            // the type of instruction we're dealing with.
            let name = segments[0].to_lowercase();

            let mut arguments = vec![];
            let mut argument_hints = vec![];

            // Do we have any arguments?
            let mut i = 0;
            for segment in &segments[1..] {
                println!("------------------[Argument {i}]------------------");
                i += 1;

                let mut is_pointer = false;

                let first_char = segment.chars().nth(0).unwrap();
                if first_char == '&' {
                    is_pointer = true;
                } else if first_char == '[' {
                    todo!();
                }

                // Skip past the address prefix identifier, if it exists.
                let sub_string = if is_pointer { &segment[1..] } else { segment };

                // First, check if this could be a register identifier.
                print!("Checking for a register identifier... ");
                match RegisterId::from_str(sub_string) {
                    Ok(id) => {
                        if is_pointer {
                            arguments.push(Argument::Register(id));
                            argument_hints.push(ArgTypeHint::RegisterPointer);

                            println!("found! ID = {id} (pointer).");
                        } else {
                            arguments.push(Argument::Register(id));
                            argument_hints.push(ArgTypeHint::Register);

                            println!("found! ID = {id}.");
                        }
                        // We're done with this argument!
                        continue;
                    }
                    Err(_) => {
                        println!("not found.");
                    }
                }

                // Next, check if this value could be a f32 immediate.
                print!("Checking for a f32 immediate... ");
                if sub_string.contains('.') {
                    // Was an address prefix used?
                    if is_pointer {
                        panic!(
                            "invalid syntax - unable to use a 32-bit floating value as a pointer!"
                        );
                    }

                    match f32::from_str(sub_string) {
                        Ok(val) => {
                            arguments.push(Argument::F32(val));
                            argument_hints.push(ArgTypeHint::F32);

                            println!("found! value = {val}.");

                            // We're done with this argument!
                            continue;
                        }
                        Err(_) => {
                            println!("not found.")
                        }
                    }
                } else {
                    println!("not found.");
                }

                // Next, check if this value could be a u8 immediate.
                // IMPORTANT - it's important to check all integers in reverse size
                // order since all u8 values could be parsed as a u32, but the reverse isn't true.
                print!("Checking for a u8 immediate... ");
                match AsmParser::try_parse_u8_immediate(sub_string) {
                    Ok(val) => {
                        if is_pointer {
                            arguments.push(Argument::U8(val));
                            argument_hints.push(ArgTypeHint::U8Pointer);

                            println!("found! Value = {val} (pointer).");
                        } else {
                            arguments.push(Argument::U8(val));
                            argument_hints.push(ArgTypeHint::U8);

                            println!("found! Value = {val}.");
                        }
                        // We're done with this argument!
                        break;
                    }
                    Err(_) => {
                        println!("not found.");
                    }
                }

                // Next, check if this value could be a u32 immediate.
                print!("Checking for a u32 immediate... ");
                match AsmParser::try_parse_u32_immediate(sub_string) {
                    Ok(val) => {
                        if is_pointer {
                            arguments.push(Argument::U32(val));
                            argument_hints.push(ArgTypeHint::U32Pointer);

                            println!("found! Value = {val} (pointer).");
                        } else {
                            arguments.push(Argument::U32(val));
                            argument_hints.push(ArgTypeHint::U32);

                            println!("found! Value = {val}.");
                        }
                        // We're done with this argument!
                        continue;
                    }
                    Err(_) => {
                        println!("not found.");
                    }
                }
            }

            println!("------------------------------------");
            println!("Instruction name = '{name}', argument types = {argument_hints:?}");

            match self.hints.find_by(name.as_str(), &argument_hints) {
                Some(op) => {
                    println!("Matching instruction found! {op:?}");
                    let ins = AsmParser::try_build_instruction(op, &arguments);
                    println!("ins = {ins}");
                    instructions.push(ins);
                }
                None => {
                    panic!("unable to find an instruction that matches the name and provided arguments.");
                }
            }
        }

        instructions
    }

    fn try_build_instruction(opcode: OpCode, args: &[Argument]) -> Instruction {
        // This will only ever be called internally and since we have confirmed that the arguments
        // match those that would be needed for the instruction associated with the opcode, it's safe
        // to make some assumptions regarding the sanity of the data.
        match opcode {
            OpCode::Nop => Instruction::Nop,
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
            OpCode::LeftShiftU32ImmU32Reg => todo!(),
            OpCode::LeftShiftU32RegU32Reg => todo!(),
            OpCode::ArithLeftShiftU32ImmU32Reg => todo!(),
            OpCode::ArithLeftShiftU32RegU32Reg => todo!(),
            OpCode::RightShiftU32ImmU32Reg => todo!(),
            OpCode::RightShiftU32RegU32Reg => todo!(),
            OpCode::ArithRightShiftU32ImmU32Reg => todo!(),
            OpCode::ArithRightShiftU32RegU32Reg => todo!(),
            OpCode::CallU32Imm => todo!(),
            OpCode::CallU32Reg => todo!(),
            OpCode::RetArgsU32 => todo!(),
            OpCode::Int => todo!(),
            OpCode::IntRet => todo!(),
            OpCode::JumpAbsU32Imm => todo!(),
            OpCode::JumpAbsU32Reg => todo!(),
            OpCode::SwapU32RegU32Reg => todo!(),
            OpCode::MovU32ImmU32Reg => todo!(),
            OpCode::MovU32RegU32Reg => todo!(),
            OpCode::MovU32ImmMemSimple => todo!(),
            OpCode::MovU32RegMemSimple => todo!(),
            OpCode::MovMemU32RegSimple => todo!(),
            OpCode::MovU32RegPtrU32RegSimple => todo!(),
            OpCode::MovU32ImmMemExpr => todo!(),
            OpCode::MovMemExprU32Reg => todo!(),
            OpCode::MovU32RegMemExpr => todo!(),
            OpCode::ByteSwapU32 => todo!(),
            OpCode::ZeroHighBitsByIndexU32Reg => todo!(),
            OpCode::ZeroHighBitsByIndexU32RegU32Imm => todo!(),
            OpCode::PushU32Imm => todo!(),
            OpCode::PushU32Reg => todo!(),
            OpCode::PopU32ImmU32Reg => todo!(),
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
            OpCode::MaskInterrupt => todo!(),
            OpCode::UnmaskInterrupt => todo!(),
            OpCode::LoadIVTAddrU32Imm => todo!(),
            OpCode::MachineReturn => todo!(),
            OpCode::Halt => todo!(),
            OpCode::Reserved1 => todo!(),
            OpCode::Reserved2 => todo!(),
            OpCode::Reserved3 => todo!(),
            OpCode::Reserved4 => todo!(),
            OpCode::Reserved5 => todo!(),
            OpCode::Reserved6 => todo!(),
            OpCode::Reserved7 => todo!(),
            OpCode::Reserved8 => todo!(),
            OpCode::Reserved9 => todo!(),
            OpCode::Label => todo!(),
            OpCode::Unknown => todo!(),
        }
    }

    fn try_parse_u8_immediate(string: &str) -> Result<u8, ParseIntError> {
        // Check for immediate values in specific bases.
        // I can't imagine these would be used much... but why not?
        if string.starts_with("0b") {
            // Binary.
            let stripped = string.strip_prefix("0b").expect("");
            u8::from_str_radix(stripped, 2)
        } else if string.starts_with('q') {
            // Octal.
            let stripped = string.strip_prefix('q').expect("");
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
        } else if string.starts_with('q') {
            // Octal.
            let stripped = string.strip_prefix('q').expect("");
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
