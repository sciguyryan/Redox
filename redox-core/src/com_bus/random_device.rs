use std::cell::RefCell;

use rand::{
    rngs::{OsRng, StdRng},
    Rng, SeedableRng,
};

use super::{
    com_bus_io::ComBusIO,
    device_error::{DeviceError, DeviceResult},
};

thread_local! {
    static PRNG: RefCell<StdRng> = RefCell::new(StdRng::from_entropy());
    static OSRNG: RefCell<OsRng> = RefCell::new(OsRng);
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

        let value: u8 = match self.mode {
            RandomMode::Crypto => OSRNG.with(|r| r.borrow_mut().gen::<u8>()),
            RandomMode::Seeded | RandomMode::Standard => PRNG.with(|r| r.borrow_mut().gen::<u8>()),
        };
        Ok(value)
    }

    fn read_u32(&self) -> DeviceResult<u32> {
        if !self.is_initialized {
            return DeviceResult::Err(DeviceError::Misconfigured);
        }

        let value: u32 = match self.mode {
            RandomMode::Crypto => OSRNG.with(|r| r.borrow_mut().gen::<u32>()),
            RandomMode::Seeded | RandomMode::Standard => PRNG.with(|r| r.borrow_mut().gen::<u32>()),
        };
        Ok(value)
    }

    fn read_f32(&self) -> DeviceResult<f32> {
        if !self.is_initialized {
            return DeviceResult::Err(DeviceError::Misconfigured);
        }

        let value: f32 = match self.mode {
            RandomMode::Crypto => OSRNG.with(|r| r.borrow_mut().gen::<f32>()),
            RandomMode::Seeded | RandomMode::Standard => PRNG.with(|r| r.borrow_mut().gen::<f32>()),
        };
        Ok(value)
    }

    fn write_u8(&mut self, value: u8) -> DeviceResult<()> {
        let mut result = DeviceResult::Ok(());

        // The instruction indicating the type of random number generator to be used.
        match value {
            0 => {
                // Standard mode - a PRNG derived from entropy.
                PRNG.with(|r| *r.borrow_mut() = StdRng::from_entropy());
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
            PRNG.with(|r| *r.borrow_mut() = StdRng::seed_from_u64(value as u64));
        } else {
            result = DeviceResult::Err(DeviceError::OperationNotSupported);
        }

        result
    }

    fn write_f32(&mut self, _value: f32) -> DeviceResult<()> {
        DeviceResult::Err(DeviceError::OperationNotSupported)
    }
}
