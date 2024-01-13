use crate::mem::memory_handler::MemoryHandler;

use super::{
    com_bus_io::ComBusIO,
    device_error::{DeviceError, DeviceResult},
    random_device::RandomDevice,
};

#[cfg(test)]
use super::test_debug_device::TestDebugDevice;

/// The ID of the random number generating device.
pub const RANDOM_DEVICE_ID: u8 = 0x0;

#[cfg(test)]
/// The ID of the random number generating device.
pub const TEST_DEBUG_DEVICE_ID: u8 = 0xff;

pub struct CommunicationBus {
    /// The [`MemoryHandler`] instance connected to this bus.
    pub mem: MemoryHandler,
    /// The [`RandomGenerator`] instance connected to this bus.
    pub random: RandomDevice,

    /// The test debugger device.
    #[cfg(test)]
    pub test_debug: TestDebugDevice,
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

            #[cfg(test)]
            test_debug: TestDebugDevice::default(),
        }
    }

    /// Read a f32 value from a specific port.
    ///
    /// # Arguments
    ///
    /// * `port` - A u8 giving the ID of the port.
    pub fn read_f32(&mut self, port: u8) -> DeviceResult<f32> {
        match port {
            #[cfg(test)]
            TEST_DEBUG_DEVICE_ID => self.test_debug.read_f32(),

            RANDOM_DEVICE_ID => self.random.read_f32(),
            _ => Err(DeviceError::NotFound),
        }
    }

    /// Read a u32 value from a specific port.
    ///
    /// # Arguments
    ///
    /// * `port` - A u8 giving the ID of the port.
    pub fn read_u32(&mut self, port: u8) -> DeviceResult<u32> {
        match port {
            #[cfg(test)]
            TEST_DEBUG_DEVICE_ID => self.test_debug.read_u32(),

            RANDOM_DEVICE_ID => self.random.read_u32(),
            _ => Err(DeviceError::NotFound),
        }
    }

    /// Read a u8 value from a specific port.
    ///
    /// # Arguments
    ///
    /// * `port` - A u8 giving the ID of the port.
    pub fn read_u8(&mut self, port: u8) -> DeviceResult<u8> {
        match port {
            #[cfg(test)]
            TEST_DEBUG_DEVICE_ID => self.test_debug.read_u8(),

            RANDOM_DEVICE_ID => self.random.read_u8(),
            _ => Err(DeviceError::NotFound),
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
            #[cfg(test)]
            TEST_DEBUG_DEVICE_ID => self.test_debug.write_f32(value),

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
            #[cfg(test)]
            TEST_DEBUG_DEVICE_ID => self.test_debug.write_u32(value),

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
            #[cfg(test)]
            TEST_DEBUG_DEVICE_ID => self.test_debug.write_u8(value),

            RANDOM_DEVICE_ID => self.random.write_u8(value),
            _ => Err(DeviceError::NotFound),
        }
    }
}
