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
}

impl Default for AsmParser {
    fn default() -> Self {
        Self::new()
    }
}
