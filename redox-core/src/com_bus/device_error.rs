use core::fmt;

/// Result with internal [`Error`] type.
pub type DeviceResult<T> = core::result::Result<T, DeviceError>;

/// Error type.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DeviceError {
    /// The device has not been properly configured.
    Misconfigured,
    /// The specified device was not found.
    NotFound,
    /// The specified operation was not supported.
    OperationNotSupported,
}

impl fmt::Display for DeviceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            DeviceError::Misconfigured => "The device has not been properly configured.",
            DeviceError::NotFound => "The specified device was not found.",
            DeviceError::OperationNotSupported => "The specified operation was not supported.",
        })
    }
}

impl std::error::Error for DeviceError {}
