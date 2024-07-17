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
    /// The position of the destination. This may be relative to the block or absolute, depending on the label type.
    pub position: usize,
}
