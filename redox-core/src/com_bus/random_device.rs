use rand::{rngs::OsRng, Rng, SeedableRng};
use rand_xoshiro::Xoshiro256Plus;
use std::cell::RefCell;

use super::{
    com_bus_io::ComBusIO,
    device_error::{DeviceError, DeviceResult},
};

thread_local! {
    static PRNG: RefCell<Xoshiro256Plus> = RefCell::new(Xoshiro256Plus::from_entropy());
    static CRNG: RefCell<OsRng> = const { RefCell::new(OsRng) };
}

#[derive(PartialEq, Eq)]
enum RandomMode {
    Crypto,
    Seeded,
    Standard,
}

/// The seedable random generator chip!
pub struct RandomDevice {
    /// Has the random number generator been initialized?
    is_initialized: bool,
    /// The random generator mode currently being used.
    mode: RandomMode,
}

impl RandomDevice {
    pub fn new() -> Self {
        Self {
            // By default we initialize the random number generator using entropy.
            is_initialized: true,
            mode: RandomMode::Standard,
        }
    }
}

impl Default for RandomDevice {
    fn default() -> Self {
        Self::new()
    }
}

impl ComBusIO for RandomDevice {
    fn read_u8(&self) -> DeviceResult<u8> {
        if !self.is_initialized {
            return DeviceResult::Err(DeviceError::Misconfigured);
        }

        Ok(match self.mode {
            RandomMode::Crypto => CRNG.with(|r| r.borrow_mut().gen::<u8>()),
            RandomMode::Seeded => PRNG.with(|r| r.borrow_mut().gen::<u8>()),
            RandomMode::Standard => rand::thread_rng().gen::<u8>(),
        })
    }

    fn read_u32(&self) -> DeviceResult<u32> {
        if !self.is_initialized {
            return DeviceResult::Err(DeviceError::Misconfigured);
        }

        Ok(match self.mode {
            RandomMode::Crypto => CRNG.with(|r| r.borrow_mut().gen::<u32>()),
            RandomMode::Seeded => PRNG.with(|r| r.borrow_mut().gen::<u32>()),
            RandomMode::Standard => rand::thread_rng().gen::<u32>(),
        })
    }

    fn read_f32(&self) -> DeviceResult<f32> {
        if !self.is_initialized {
            return DeviceResult::Err(DeviceError::Misconfigured);
        }

        Ok(match self.mode {
            RandomMode::Crypto => CRNG.with(|r| r.borrow_mut().gen::<f32>()),
            RandomMode::Seeded => PRNG.with(|r| r.borrow_mut().gen::<f32>()),
            RandomMode::Standard => rand::thread_rng().gen::<f32>(),
        })
    }

    fn write_u8(&mut self, value: u8) -> DeviceResult<()> {
        let mut result = DeviceResult::Ok(());

        // The instruction indicating the type of random number generator to be used.
        match value {
            0 => {
                // Standard mode - a PRNG derived from entropy.
                PRNG.with(|r| *r.borrow_mut() = Xoshiro256Plus::from_entropy());
                self.mode = RandomMode::Standard;
                self.is_initialized = true;
            }
            1 => {
                // Seeded random mode - the seed must be set via sending a u32 command after this one.
                self.mode = RandomMode::Seeded;
                self.is_initialized = false;
            }
            2 => {
                // Cryptographic random mode - a cryptographically secure random number generator.
                self.mode = RandomMode::Crypto;
                self.is_initialized = true;
            }
            _ => {
                result = DeviceResult::Err(DeviceError::OperationNotSupported);
            }
        }

        result
    }

    fn write_u32(&mut self, value: u32) -> DeviceResult<()> {
        let mut result = DeviceResult::Ok(());

        if self.mode == RandomMode::Seeded {
            PRNG.with(|r| *r.borrow_mut() = Xoshiro256Plus::seed_from_u64(value as u64));

            // Indicate that the PRNG has been seeded and initialized.
            self.is_initialized = true;
        } else {
            result = DeviceResult::Err(DeviceError::OperationNotSupported);
        }

        result
    }

    fn write_f32(&mut self, _value: f32) -> DeviceResult<()> {
        DeviceResult::Err(DeviceError::OperationNotSupported)
    }
}

#[cfg(test)]
mod tests_random_device {
    use crate::com_bus::com_bus_io::ComBusIO;

    use super::RandomDevice;

    /// Test the PRNG to ensure it is non-repeating.
    #[test]
    fn test_prng() {
        // By default, the generator should use an entropy-seeded PRNG.
        let device = RandomDevice::default();
        let mut sequence_1 = vec![];
        for _ in 0..1000 {
            sequence_1.push(device.read_u32().expect(""));
        }

        let device = RandomDevice::default();
        let mut sequence_2 = vec![];
        for _ in 0..1000 {
            sequence_2.push(device.read_u32().expect(""));
        }

        assert_ne!(sequence_1, sequence_2);
    }

    /// Test the seeded PRNG to ensure it is repeating.
    #[test]
    fn test_seeded_prng() {
        let mut device = RandomDevice::default();
        _ = device.write_u8(0x1);
        _ = device.write_u32(0xdeadbeef);

        let mut sequence_1 = vec![];
        for _ in 0..1000 {
            sequence_1.push(device.read_u32().expect(""));
        }

        let mut device = RandomDevice::default();
        _ = device.write_u8(0x1);
        _ = device.write_u32(0xdeadbeef);

        let mut sequence_2 = vec![];
        for _ in 0..1000 {
            sequence_2.push(device.read_u32().expect(""));
        }

        assert_eq!(sequence_1, sequence_2);
    }
}
