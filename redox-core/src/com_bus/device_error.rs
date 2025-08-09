use thiserror::Error;

/// Result with internal [`Error`] type.
pub type DeviceResult<T> = core::result::Result<T, DeviceError>;

/// Error type.
#[derive(Error, Debug)]
pub enum DeviceError {
    /// The device has not been properly configured.
    #[error("the specified device has not been properly configured")]
    Misconfigured,
    /// The specified device was not found.
    #[error("the specified device was not found")]
    NotFound,
    /// The specified operation was not supported.
    #[error("the specified operation was not supported")]
    OperationNotSupported,
}
