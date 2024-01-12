use crate::mem::memory_handler::MemoryHandler;

pub struct CommunicationBus {
    /// The [`MemoryHandler`] instance to be connected to this bus.
    pub mem: MemoryHandler,
    // TODO - add the components here.
}

impl CommunicationBus {
    /// Build a new [`CommunicationBus`] instance.
    ///
    /// # Arguments
    ///
    /// * `mem` - The [`MemoryHandler`] instance to be connected to this bus.
    pub fn new(mem: MemoryHandler) -> Self {
        Self { mem }
    }
}
