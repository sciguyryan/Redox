use crate::mem::memory_handler::MemoryHandler;

pub struct CommunicationBus {
    /// The [`MemoryHandler`] instance to be connected to this bus.
    pub mem: MemoryHandler,
}

impl CommunicationBus {
    /// Build a new [`CommunicationBus`] instance.
    ///
    /// # Arguments
    ///
    /// * `mem` - The [`MemoryHandler`] instance to be connected to this bus.
    pub fn new(
        mem: MemoryHandler,
    ) -> Self {
        Self {
            mem,
        }
    }

    pub fn read_u32() {
        println!("here");
    }
}
