use super::{
    com_bus_io::ComBusIO,
    device_error::{DeviceError, DeviceResult},
};

/// A debug device that is enabled just for tests.
pub struct TestDebugDevice {
    should_fail: [bool; 6],
}

impl TestDebugDevice {
    pub fn new() -> Self {
        Self {
            should_fail: [false, false, false, false, false, true],
        }
    }

    pub fn set_failures(&mut self, fails: [bool; 6]) {
        self.should_fail = fails;
    }
}

impl Default for TestDebugDevice {
    fn default() -> Self {
        Self::new()
    }
}

impl ComBusIO for TestDebugDevice {
    fn read_u8(&mut self) -> DeviceResult<u8> {
        if self.should_fail[0] {
            return DeviceResult::Err(DeviceError::OperationNotSupported);
        }

        Ok(u8::MAX)
    }

    fn read_u32(&mut self) -> DeviceResult<u32> {
        if self.should_fail[1] {
            return DeviceResult::Err(DeviceError::OperationNotSupported);
        }

        Ok(u32::MAX)
    }

    fn read_f32(&mut self) -> DeviceResult<f32> {
        if self.should_fail[2] {
            return DeviceResult::Err(DeviceError::OperationNotSupported);
        }

        Ok(f32::MAX)
    }

    fn write_u8(&mut self, _value: u8) -> DeviceResult<()> {
        if self.should_fail[3] {
            return DeviceResult::Err(DeviceError::OperationNotSupported);
        }

        Ok(())
    }

    fn write_u32(&mut self, _value: u32) -> DeviceResult<()> {
        if self.should_fail[4] {
            return DeviceResult::Err(DeviceError::OperationNotSupported);
        }

        Ok(())
    }

    fn write_f32(&mut self, _value: f32) -> DeviceResult<()> {
        if self.should_fail[5] {
            return DeviceResult::Err(DeviceError::OperationNotSupported);
        }

        Ok(())
    }
}
