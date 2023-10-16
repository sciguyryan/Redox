#[derive(Debug, Eq, PartialEq)]
pub enum SecurityContext {
    /// Execution of a command with machine-level privilege.
    Machine,
    /// Execution of a command with user-level privilege.
    User,
}
