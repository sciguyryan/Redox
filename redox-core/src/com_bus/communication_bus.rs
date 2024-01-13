use crate::mem::memory_handler::MemoryHandler;

use super::{
    com_bus_io::ComBusIO,
    device_error::{DeviceError, DeviceResult},
    random_device::RandomDevice,
};

/// The ID of the random number generating device.
pub const RANDOM_DEVICE_ID: u8 = 0x0;

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

    /// Write a u32 value to a specific device.
    ///
    /// # Arguments
    ///
    /// * `value` - The u32 value to be written to the device.
    /// * `device_id` - A u8 giving the ID of the device.
    pub fn write_u32(&mut self, value: u32, device_id: u8) -> DeviceResult<()> {
        match device_id {
            RANDOM_DEVICE_ID => self.random.write_u32(value),
            _ => Err(DeviceError::NotFound),
        }
    }
}
