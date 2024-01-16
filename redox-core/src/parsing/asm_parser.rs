use std::{num::ParseIntError, str::FromStr};

use crate::{
    ins::instruction::Instruction, parsing::type_hints::ArgTypeHint, reg::registers::RegisterId,
};

use super::type_hints::InstructionHints;

#[derive(Debug, Clone)]
pub enum Argument {
    /// A f32 argument.
    F32(f32),
    /// A u32 argument.
    U32(u32),
    /// A u32 pointer argument (memory address).
    U32Pointer(u32),
    /// A u8 argument.
    U8(u8),
    /// A u8 pointer argument (memory address).
    U8Pointer(u8),
    /// A register argument.
    Register(RegisterId),
    /// A register pointer argument (memory address).
    RegisterPointer(RegisterId),
    /// An expression argument (memory address.
    Expression,
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

    pub fn test(&self) {
        for hint in &self.hints.hints {
            println!("{hint:?}");
        }
    }

    pub fn parse(&self, string: &str) -> Vec<String> {
        let instructions: Vec<Instruction> = vec![];

        // Split the string into lines.
        for line in string.lines() {
            //println!("line = {line}");
            let bits = AsmParser::parse_instruction_line(line.trim_matches(' '));
            println!("{bits:?}");

            // Next, we need to try and parse the arguments to see if we can figure out
            // the type of instruction we're dealing with.
            let name = bits[0].to_lowercase();

            let mut arguments = vec![];
            let mut argument_hints = vec![];

            // Do we have any arguments?
            let mut i = 0;
            for bit in &bits[1..] {
                println!("------------------[Argument {i}]------------------");
                i += 1;

                let mut is_pointer = false;

                let first_char = bit.chars().nth(0).unwrap();
                if first_char == '&' {
                    is_pointer = true;
                } else if first_char == '[' {
                    todo!();
                }

                // Skip past the address prefix identifier, if it exists.
                let sub_string = if is_pointer { &bit[1..] } else { bit };

                // First, check if this could be a register identifier.
                print!("Checking for a register identifier... ");
                match RegisterId::from_str(sub_string) {
                    Ok(id) => {
                        if is_pointer {
                            arguments.push(Argument::RegisterPointer(id));
                            argument_hints.push(ArgTypeHint::RegisterPointer);

                            println!("found! ID = {id} (pointer).");
                        } else {
                            arguments.push(Argument::Register(id));
                            argument_hints.push(ArgTypeHint::Register);

                            println!("found! ID = {id}.");
                        }
                        // We're done with this argument!
                        break;
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
                            break;
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
                            arguments.push(Argument::U8Pointer(val));
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
                            arguments.push(Argument::U32Pointer(val));
                            argument_hints.push(ArgTypeHint::U32Pointer);

                            println!("found! Value = {val} (pointer).");
                        } else {
                            arguments.push(Argument::U32(val));
                            argument_hints.push(ArgTypeHint::U32);

                            println!("found! Value = {val}.");
                        }
                        // We're done with this argument!
                        break;
                    }
                    Err(_) => {
                        println!("not found.");
                    }
                }
            }
        }

        vec![]
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
