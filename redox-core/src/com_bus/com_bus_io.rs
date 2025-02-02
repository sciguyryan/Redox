use super::device_error::DeviceResult;

pub trait ComBusIO {
    /// Read a u8 value from the device.
    fn read_u8(&mut self) -> DeviceResult<u8>;
    /// Read a u32 value from the device.
    fn read_u32(&mut self) -> DeviceResult<u32>;
    /// Read a uf32 value from the device.
    fn read_f32(&mut self) -> DeviceResult<f32>;

    /// Write a u8 value to the device.
    fn write_u8(&mut self, value: u8) -> DeviceResult<()>;
    /// Write a u32 value to the device.
    fn write_u32(&mut self, value: u32) -> DeviceResult<()>;
    /// Write a f32 value to the device.
    fn write_f32(&mut self, value: f32) -> DeviceResult<()>;
}
