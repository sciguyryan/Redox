#[derive(Debug, Eq, PartialEq)]
pub enum SecurityContext {
    /// Direct execution of a command, usually via the CPU.
    System,
    /// Indirect execution of a command, usually via an
    /// executed byte code instruction.
    User,
}
