#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DataAccessType {
    /// Execute.
    Execute,
    /// Read.
    Read,
    /// Write.
    Write,
}
