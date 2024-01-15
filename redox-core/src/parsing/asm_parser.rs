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

    pub fn parse(&self, string: &str) -> Vec<String> {
        let mut output = vec![];

        // Split the string into lines.
        for line in string.lines() {
            //println!("line = {line}");
            let bits = AsmParser::parse_instruction_line(line.trim_matches(' '));
            //println!("{bits:?}");
            output = bits;

            // Next, we need to try and parse the arguments to see if we can figure out
            // the type of instruction we're dealing with.
            // That's a problem for tomorrow!
        }

        output
    }

    pub fn parse_instruction_line(line: &str) -> Vec<String> {
        let mut segments = Vec::with_capacity(5);

        let mut segment_end = false;
        let mut start_pos = 0;
        let mut end_pos = 0;

        let mut chars = line.chars().peekable();
        loop {
            let c = chars.next();
            if c.is_none() {
                break;
            }

            let c = c.unwrap();

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
