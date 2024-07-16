#[derive(Debug, Clone)]
pub struct DataDeclaration {
    /// The [`DataDeclarationType`].
    pub declaration_type: DataDeclarationType,
    /// The label associated with the statement.
    pub label: String,
    /// The stored bytes.
    pub bytes: Vec<u8>,
}

impl DataDeclaration {
    pub fn new(declaration_type: DataDeclarationType, label: String, bytes: Vec<u8>) -> Self {
        Self {
            declaration_type,
            label,
            bytes,
        }
    }
}

/// The supported data declaration statement types.
#[derive(Debug, Clone)]
pub enum DataDeclarationType {
    /// Declare a byte storage block.
    DB,
}

impl TryFrom<&str> for DataDeclarationType {
    type Error = ();

    fn try_from(string: &str) -> Result<Self, Self::Error> {
        match string.to_lowercase().as_str() {
            "db" => Ok(DataDeclarationType::DB),
            _ => {
                panic!("invalid syntax - an unrecognized data declaration {string}");
            }
        }
    }
}
