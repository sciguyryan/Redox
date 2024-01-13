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

    /// Write a f32 value to a specific port.
    ///
    /// # Arguments
    ///
    /// * `value` - The f32 value to be written to the port.
    /// * `port` - A u8 giving the ID of the port.
    pub fn write_f32(&mut self, value: f32, port: u8) -> DeviceResult<()> {
        match port {
            RANDOM_DEVICE_ID => self.random.write_f32(value),
            _ => Err(DeviceError::NotFound),
        }
    }

    /// Write a u32 value to a specific port.
    ///
    /// # Arguments
    ///
    /// * `value` - The u32 value to be written to the port.
    /// * `port` - A u8 giving the ID of the port.
    pub fn write_u32(&mut self, value: u32, port: u8) -> DeviceResult<()> {
        match port {
            RANDOM_DEVICE_ID => self.random.write_u32(value),
            _ => Err(DeviceError::NotFound),
        }
    }

    /// Write a u8 value to a specific port.
    ///
    /// # Arguments
    ///
    /// * `value` - The u32 value to be written to the port.
    /// * `port` - A u8 giving the ID of the port.
    pub fn write_u8(&mut self, value: u8, port: u8) -> DeviceResult<()> {
        match port {
            RANDOM_DEVICE_ID => self.random.write_u8(value),
            _ => Err(DeviceError::NotFound),
        }
    }
}
