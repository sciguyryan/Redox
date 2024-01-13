use crate::mem::memory_handler::MemoryHandler;

use super::random_device::RandomDevice;

pub struct CommunicationBus {
    /// The [`MemoryHandler`] instance connected to this bus.
    pub mem: MemoryHandler,
    /// The [`RandomGenerator`] instance connected to this bus.
    pub random: RandomDevice,
}

impl CommunicationBus {
    /// Build a new [`CommunicationBus`] instance.
    ///
    /// # Arguments
    ///
    /// * `mem` - The [`MemoryHandler`] instance to be connected to this bus.
    pub fn new(mem: MemoryHandler) -> Self {
        Self {
            mem,
            random: RandomDevice::default(),
        }
    }
}
