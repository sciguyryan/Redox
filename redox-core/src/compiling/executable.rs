// The structure of the executable file is as follows:
// [CODE]
// [DATA]
// [READ-ONLY DATA]
// The data will also be loaded into memory in the same configuration,
// though different segments will have slightly different permissions.

pub struct Executable {}

impl Executable {
    pub fn new() -> Self {
        Self {}
    }
}
