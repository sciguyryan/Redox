use core::fmt;

/// Result with internal [`Error`] type.
pub type DeviceResult<T> = core::result::Result<T, DeviceError>;

/// Error type.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DeviceError {
    /// The device has not been properly configured.
    Misconfigured,
}

impl fmt::Display for DeviceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            DeviceError::Misconfigured => "The device has not been properly configured.",
        })
    }
}

impl std::error::Error for DeviceError {}
