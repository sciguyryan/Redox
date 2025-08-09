#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DataAccessType {
    /// Read.
    Read,
    /// Write.
    Write,
}
