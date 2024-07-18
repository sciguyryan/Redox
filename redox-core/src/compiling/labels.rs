#[derive(Debug, Eq, PartialEq)]
pub enum LabelType {
    Code,
    Data,
    Global,
}

#[derive(Debug, Eq, PartialEq)]
pub struct LabelEntry {
    /// The [`LabelType`] of the label.
    pub label_type: LabelType,
    /// The name of the label.
    pub label_name: String,
    /// The position of the destination.
    /// Note that this may be relative to the block or absolute, depending on the label type.
    pub position: usize,
}

impl LabelEntry {
    pub fn new(label_type: LabelType, label_name: String, position: usize) -> Self {
        Self {
            label_type,
            label_name,
            position,
        }
    }
}
