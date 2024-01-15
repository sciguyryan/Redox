use super::type_hints::InstructionHints;

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

    pub fn parse(&self, string: &str) {
        // Split the string into lines.
        for line in string.lines() {
            println!("line = {line}");
            let bits = AsmParser::parse_line(line.trim_matches(' '));
            println!("{bits:?}");

            // Next, we need to try and parse the arguments to see if we can figure out
            // the type of instruction we're dealing with.
            // That's a problem for tomorrow!
        }
    }

    pub fn parse_line(line: &str) -> Vec<String> {
        let mut segments = vec![];
        let mut identifier_end = false;
        let mut skip_char: bool;

        let mut current = Vec::with_capacity(100);
        let count = line.chars().count();
        for (i, c) in line.chars().enumerate() {
            match c {
                ' ' | ',' => {
                    identifier_end = true;
                    skip_char = true;
                }
                _ => {
                    skip_char = false;
                }
            }

            if !skip_char {
                current.push(c);
            }

            if identifier_end || i == count - 1 {
                let string: String = current.iter().collect();

                // If we have a non-empty string then we can add it to our processing list.
                if !string.is_empty() {
                    segments.push(string);
                }

                identifier_end = false;
                current.clear();
            }
        }

        // We have one more change to make.
        // The first entry should be the instruction identifier and we want that to be
        // in lower case, just to make our lives easier.
        segments[0] = segments[0].to_lowercase();

        segments
    }
}

impl Default for AsmParser {
    fn default() -> Self {
        Self::new()
    }
}
