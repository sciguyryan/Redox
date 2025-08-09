#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PrivilegeLevel {
    /// Execution of a command with machine-level privilege.
    Machine,
    /// Execution of a command with user-level privilege.
    User,
}
