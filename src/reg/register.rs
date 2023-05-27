use bitflags::bitflags;
use std::fmt::{self, Display};

use crate::{data_access_type::DataAccessType, security_context::SecurityContext};

use super::registers::RegisterId;

bitflags! {
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct RegisterPermission: u8 {
        /// None.
        const N = 1 << 0;
        /// Public read.
        const R = 1 << 1;
        /// Public write.
        const W = 1 << 2;
        /// Private read.
        const PR = 1 << 3;
        /// Private write.
        const PW = 1 << 4;
    }
}

impl Display for RegisterPermission {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

pub struct RegisterU32 {
    /// The ID of this register.
    pub id: RegisterId,
    /// The permissions of this register.
    pub permissions: RegisterPermission,
    /// The value of this register.
    value: u32,
}

impl RegisterU32 {
    pub fn new(id: RegisterId, permissions: RegisterPermission, value: u32) -> Self {
        Self {
            id,
            permissions,
            value,
        }
    }

    pub fn read(&self, context: &SecurityContext) -> &u32 {
        // Check whether the register has read permissions.
        self.validate_access(&DataAccessType::Read, context);

        &self.value
    }

    pub fn write(&mut self, value: u32, context: &SecurityContext) {
        // Check whether the register has write permissions.
        self.validate_access(&DataAccessType::Write, context);

        self.value = value;
    }

    #[inline]
    fn validate_access(&self, access_type: &DataAccessType, context: &SecurityContext) {
        // System-level contexts are permitted to do anything, without limitation.
        // NOTE: This might end up being replaced with a ring-permission type system.
        if *context == SecurityContext::System {
            return;
        }

        let permissions = &self.permissions;

        // We need to check the read-write permissions on the specific register.
        let has_required_perms = match access_type {
            DataAccessType::Read => {
                permissions.intersects(RegisterPermission::R)
                    || (permissions.intersects(RegisterPermission::PR)
                        && *context == SecurityContext::System)
            }
            DataAccessType::Write => {
                permissions.intersects(RegisterPermission::W)
                    || (permissions.intersects(RegisterPermission::PW)
                        && *context == SecurityContext::System)
            }
            DataAccessType::Execute => panic!("Invalid access type specified."),
        };

        if !has_required_perms {
            panic!("attempted to access memory without the correct security context or access flags. Register = {:?}, Access Type = {access_type:?}, permissions = {permissions}.", self.id);
        }
    }
}

pub struct RegisterF32 {
    /// The ID of this register.
    pub id: RegisterId,
    /// The permissions of this register.
    pub permissions: RegisterPermission,
    /// The value of this register.
    value: f32,
}

impl RegisterF32 {
    pub fn new(id: RegisterId, permissions: RegisterPermission, value: f32) -> Self {
        Self {
            id,
            permissions,
            value,
        }
    }

    pub fn read(&self, context: &SecurityContext) -> &f32 {
        // Check whether the register has read permissions.
        self.validate_access(&DataAccessType::Read, context);

        &self.value
    }

    pub fn write(&mut self, value: f32, context: &SecurityContext) {
        // Check whether the register has write permissions.
        self.validate_access(&DataAccessType::Write, context);

        self.value = value;
    }

    #[inline]
    fn validate_access(&self, access_type: &DataAccessType, context: &SecurityContext) {
        // System-level contexts are permitted to do anything, without limitation.
        // NOTE: This might end up being replaced with a ring-permission type system.
        if *context == SecurityContext::System {
            return;
        }

        let permissions = &self.permissions;

        // We need to check the read-write permissions on the specific register.
        let has_required_perms = match access_type {
            DataAccessType::Read => {
                permissions.intersects(RegisterPermission::R)
                    || (permissions.intersects(RegisterPermission::PR)
                        && *context == SecurityContext::System)
            }
            DataAccessType::Write => {
                permissions.intersects(RegisterPermission::W)
                    || (permissions.intersects(RegisterPermission::PW)
                        && *context == SecurityContext::System)
            }
            DataAccessType::Execute => panic!("Invalid access type specified."),
        };

        if !has_required_perms {
            panic!("attempted to access memory without the correct security context or access flags. Register = {:?}, Access Type = {access_type:?}, permissions = {permissions}.", self.id);
        }
    }
}

#[cfg(test)]
mod tests_registers {
    use std::panic;

    use crate::{reg::registers::RegisterId, security_context::SecurityContext};

    use super::{RegisterF32, RegisterPermission, RegisterU32};

    /// Test the reading of a (u32) register.
    #[test]
    fn test_register_read_u32() {
        let rw = RegisterPermission::R | RegisterPermission::W;
        let prpw = RegisterPermission::PR | RegisterPermission::PW;

        let tests = [
            (10, rw, SecurityContext::User, false),
            (10, rw, SecurityContext::System, false),
            (10, prpw, SecurityContext::User, true),
            (10, prpw, SecurityContext::System, false),
        ];

        for test in tests {
            let result = panic::catch_unwind(|| {
                let register = RegisterU32::new(RegisterId::R1, test.1, 10);
                *register.read(&test.2)
            });

            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                test.3,
                "Read Failed - Expected Value: {}, Got Value: {}, Context: {:?}, Should Panic? {}, Panicked? {did_panic}.",
                test.0, result.unwrap(), test.2, test.3
            );
        }
    }

    /// Test the writing of a (u32) register.
    #[test]
    fn test_register_write_u32() {
        let rw = RegisterPermission::R | RegisterPermission::W;
        let prpw = RegisterPermission::PR | RegisterPermission::PW;

        let tests = [
            (10, rw, SecurityContext::User, false),
            (10, rw, SecurityContext::System, false),
            (10, prpw, SecurityContext::User, true),
            (10, prpw, SecurityContext::System, false),
        ];

        for test in tests {
            let result = panic::catch_unwind(|| {
                let mut register = RegisterU32::new(RegisterId::R1, test.1, 0);
                register.write(test.0, &test.2);
                *register.read(&test.2)
            });

            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                test.3,
                "Write Failed - Expected Value: {}, Got Value: {}, Context: {:?}, Should Panic? {}, Panicked? {did_panic}.",
                test.0, result.unwrap(), test.2, test.3
            );
        }
    }

    /// Test the reading of a (f32) register.
    #[test]
    fn test_register_read_f32() {
        let rw = RegisterPermission::R | RegisterPermission::W;
        let prpw = RegisterPermission::PR | RegisterPermission::PW;

        let tests = [
            (10f32, rw, SecurityContext::User, false),
            (10f32, rw, SecurityContext::System, false),
            (10f32, prpw, SecurityContext::User, true),
            (10f32, prpw, SecurityContext::System, false),
        ];

        for test in tests {
            let result = panic::catch_unwind(|| {
                let register = RegisterF32::new(RegisterId::R1, test.1, 10f32);
                *register.read(&test.2)
            });

            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                test.3,
                "Read Failed - Expected Value: {}, Got Value: {}, Context: {:?}, Should Panic? {}, Panicked? {did_panic}.",
                test.0, result.unwrap(), test.2, test.3
            );
        }
    }

    /// Test the writing of a (f32) register.
    #[test]
    fn test_register_write_f32() {
        let rw = RegisterPermission::R | RegisterPermission::W;
        let prpw = RegisterPermission::PR | RegisterPermission::PW;

        let tests = [
            (10f32, rw, SecurityContext::User, false),
            (10f32, rw, SecurityContext::System, false),
            (10f32, prpw, SecurityContext::User, true),
            (10f32, prpw, SecurityContext::System, false),
        ];

        for test in tests {
            let result = panic::catch_unwind(|| {
                let mut register = RegisterF32::new(RegisterId::R1, test.1, 0f32);
                register.write(test.0, &test.2);
                *register.read(&test.2)
            });

            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                test.3,
                "Write Failed - Expected Value: {}, Got Value: {}, Context: {:?}, Should Panic? {}, Panicked? {did_panic}.",
                test.0, result.unwrap(), test.2, test.3
            );
        }
    }
}
