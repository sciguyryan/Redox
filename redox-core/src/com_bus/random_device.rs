use getrandom::SysRng;
use rand::{RngExt, SeedableRng, TryRng};
use rand_xoshiro::Xoshiro256Plus;

use super::{
    com_bus_io::ComBusIO,
    device_error::{DeviceError, DeviceResult},
};

#[repr(u8)]
#[derive(PartialEq, Eq)]
enum RandomMode {
    Crypto,
    Pseudorandom,
}

/// The random number generator device!
pub struct RandomDevice {
    /// Has the random number generator been initialized?
    is_initialized: bool,
    /// The random generator mode currently being used.
    mode: RandomMode,
    /// The pseudorandom number generator.
    prng: Xoshiro256Plus,
    /// The cryptograph random number generator.
    crng: SysRng,
}

impl RandomDevice {
    pub fn new() -> Self {
        Self {
            // By default we initialize the random number generator using entropy.
            is_initialized: true,
            mode: RandomMode::Crypto,
            prng: rand::make_rng(),
            crng: getrandom::SysRng,
        }
    }
}

impl Default for RandomDevice {
    fn default() -> Self {
        Self::new()
    }
}

impl ComBusIO for RandomDevice {
    fn read_u8(&mut self) -> DeviceResult<u8> {
        if !self.is_initialized {
            return DeviceResult::Err(DeviceError::Misconfigured);
        }

        Ok(match self.mode {
            RandomMode::Crypto => self.crng.try_next_u32().expect("failed to get u8 value") as u8,
            RandomMode::Pseudorandom => self.prng.random::<u8>(),
        })
    }

    fn read_u32(&mut self) -> DeviceResult<u32> {
        if !self.is_initialized {
            return DeviceResult::Err(DeviceError::Misconfigured);
        }

        Ok(match self.mode {
            RandomMode::Crypto => self.crng.try_next_u32().expect("failed to get u32 value"),
            RandomMode::Pseudorandom => self.prng.random::<u32>(),
        })
    }

    fn read_f32(&mut self) -> DeviceResult<f32> {
        if !self.is_initialized {
            return DeviceResult::Err(DeviceError::Misconfigured);
        }

        Ok(match self.mode {
            RandomMode::Crypto => {
                // Construct a f32 value from scratch using a u32 base value
                // to avoid generating NANs and infinities.
                let random_val = self.crng.try_next_u32().expect("failed to get u32 value");
                let sign = random_val & 0x8000_0000;
                // Top 8 bits, capped to 0–254.
                let exponent = random_val & 0x7F80_0000;
                let mantissa = random_val & 0x007F_FFFF;

                f32::from_bits(sign | exponent | mantissa)
            }
            RandomMode::Pseudorandom => self.prng.random::<f32>(),
        })
    }

    fn write_u8(&mut self, value: u8) -> DeviceResult<()> {
        let mut result = DeviceResult::Ok(());

        // The instruction indicating the type of random number generator to be used.
        match value {
            0 => {
                // CRNG - a cryptographically secure random number generator.
                // This is the default.
                // Writing a seed value to this state will result in an error.
                self.mode = RandomMode::Crypto;
                self.is_initialized = true;
            }
            1 => {
                // PRNG mode - a PRNG seeded by entropic randomness unless
                // a seed is written in a following call.
                self.prng = rand::make_rng();
                self.mode = RandomMode::Pseudorandom;
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

        if self.mode == RandomMode::Pseudorandom {
            self.prng = Xoshiro256Plus::seed_from_u64(value as u64);

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
    use crate::com_bus::{com_bus_io::ComBusIO, random_device::RandomMode};

    use super::RandomDevice;

    /// Test the cryptographic RNG to ensure it is non-repeating.
    #[test]
    fn test_crng() {
        // By default, the generator should use an entropy-seeded RNG.
        let mut device = RandomDevice::default();
        let mut sequence_1 = vec![];
        for _ in 0..1000 {
            sequence_1.push(device.read_u32().expect(""));
        }

        let mut device = RandomDevice::default();
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
        _ = device.write_u8(RandomMode::Pseudorandom as u8);
        _ = device.write_u32(0xdeadbeef);

        let mut sequence_1 = vec![];
        for _ in 0..1000 {
            sequence_1.push(device.read_u32().expect(""));
        }

        let mut device = RandomDevice::default();
        _ = device.write_u8(RandomMode::Pseudorandom as u8);
        _ = device.write_u32(0xdeadbeef);

        let mut sequence_2 = vec![];
        for _ in 0..1000 {
            sequence_2.push(device.read_u32().expect(""));
        }

        assert_eq!(sequence_1, sequence_2);
    }
}
