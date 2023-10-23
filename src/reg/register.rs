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

#[derive(Debug)]
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

    #[inline(always)]
    pub fn add(&mut self, val: u32, context: &SecurityContext) {
        // Check whether the register has read/write permissions.
        self.validate_access(&DataAccessType::Write, context);

        self.value += val;
    }

    #[inline(always)]
    pub fn add_unchecked(&mut self, value: u32) {
        self.value += value;
    }

    #[inline(always)]
    pub fn increment(&mut self, context: &SecurityContext) {
        self.add(1, context);
    }

    #[inline(always)]
    pub fn increment_unchecked(&mut self) {
        self.add_unchecked(1);
    }

    pub fn read(&self, context: &SecurityContext) -> &u32 {
        // Check whether the register has read permissions.
        self.validate_access(&DataAccessType::Read, context);

        &self.value
    }

    pub fn read_unchecked(&self) -> &u32 {
        &self.value
    }

    #[inline(always)]
    pub fn subtract(&mut self, val: u32, context: &SecurityContext) {
        // Check whether the register has read/write permissions.
        self.validate_access(&DataAccessType::Write, context);

        self.value -= val;
    }

    pub fn write(&mut self, value: u32, context: &SecurityContext) {
        // Check whether the register has write permissions.
        self.validate_access(&DataAccessType::Write, context);

        self.value = value;
    }

    pub fn write_unchecked(&mut self, value: u32) {
        self.value = value;
    }

    #[inline(always)]
    fn validate_access(&self, access_type: &DataAccessType, context: &SecurityContext) {
        // System-level contexts are permitted to do anything, without limitation.
        // NOTE: This might end up being replaced with a ring-permission type system.
        if *context == SecurityContext::Machine {
            return;
        }

        let permissions = &self.permissions;

        // We need to check the read-write permissions on the specific register.
        let has_required_perms = match access_type {
            DataAccessType::Read => {
                permissions.intersects(RegisterPermission::R)
                    || (permissions.intersects(RegisterPermission::PR)
                        && *context == SecurityContext::Machine)
            }
            DataAccessType::Write => {
                permissions.intersects(RegisterPermission::W)
                    || (permissions.intersects(RegisterPermission::PW)
                        && *context == SecurityContext::Machine)
            }
        };

        if !has_required_perms {
            panic!("attempted to access memory without the correct security context or access flags. Register = {:?}, Access Type = {access_type:?}, permissions = {permissions}.", self.id);
        }
    }
}

#[derive(Debug)]
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

    pub fn read_unchecked(&self) -> &f32 {
        &self.value
    }

    pub fn write(&mut self, value: f32, context: &SecurityContext) {
        // Check whether the register has write permissions.
        self.validate_access(&DataAccessType::Write, context);

        self.value = value;
    }

    pub fn write_unchecked(&mut self, value: f32) {
        self.value = value;
    }

    #[inline]
    fn validate_access(&self, access_type: &DataAccessType, context: &SecurityContext) {
        // System-level contexts are permitted to do anything, without limitation.
        // NOTE: This might end up being replaced with a ring-permission type system.
        if *context == SecurityContext::Machine {
            return;
        }

        let permissions = &self.permissions;

        // We need to check the read-write permissions on the specific register.
        let has_required_perms = match access_type {
            DataAccessType::Read => {
                permissions.intersects(RegisterPermission::R)
                    || (permissions.intersects(RegisterPermission::PR)
                        && *context == SecurityContext::Machine)
            }
            DataAccessType::Write => {
                permissions.intersects(RegisterPermission::W)
                    || (permissions.intersects(RegisterPermission::PW)
                        && *context == SecurityContext::Machine)
            }
        };

        if !has_required_perms {
            panic!("attempted to access memory without the correct security context or access flags. Register = {:?}, Access Type = {access_type:?}, permissions = {permissions}.", self.id);
        }
    }
}

pub trait RegisterTypes {}
impl RegisterTypes for RegisterF32 {}
impl RegisterTypes for RegisterU32 {}

#[cfg(test)]
mod tests_registers {
    use std::{
        fmt::{Debug, Display},
        panic,
    };

    use crate::{reg::registers::RegisterId, security_context::SecurityContext};

    use super::{RegisterF32, RegisterPermission, RegisterU32};

    trait RegisterValueTest {}
    impl RegisterValueTest for f32 {}
    impl RegisterValueTest for u32 {}

    enum TestType {
        Read,
        Write,
    }

    struct TestEntry<T: RegisterValueTest + Display + Debug> {
        pub test_type: TestType,
        pub initial_value: T,
        pub write_value: Option<T>,
        pub expected_value: T,
        pub permissions: RegisterPermission,
        pub context: SecurityContext,
        pub should_panic: bool,
        pub fail_message: String,
    }

    impl<T: RegisterValueTest + Display + Debug> TestEntry<T> {
        #[allow(clippy::too_many_arguments)]
        pub fn new(
            test_type: TestType,
            initial_value: T,
            write_value: Option<T>,
            expected_value: T,
            permissions: &RegisterPermission,
            context: SecurityContext,
            should_panic: bool,
            fail_message: &str,
        ) -> Self {
            Self {
                test_type,
                initial_value,
                write_value,
                expected_value,
                permissions: *permissions,
                context,
                should_panic,
                fail_message: fail_message.to_string(),
            }
        }

        pub fn fail_message(&self, id: usize, got_value: Option<T>, panicked: bool) -> String {
            let test_type = match self.test_type {
                TestType::Read => "Read",
                TestType::Write => "Write",
            };

            format!(
                "{test_type} Test {id} Failed - Expected: {:?}, Got: {got_value:?}, Should Panic? {}, Panicked? {panicked}. Message = {}",
                self.expected_value, self.should_panic, self.fail_message
            )
        }
    }

    /// Test the reading and writing of a (u32) register.
    #[test]
    fn test_register_u32() {
        let rw = RegisterPermission::R | RegisterPermission::W;
        let prpw = RegisterPermission::PR | RegisterPermission::PW;

        let tests = [
            TestEntry::new(
                TestType::Read,
                10,
                None,
                10,
                &rw,
                SecurityContext::User,
                false,
                "failed to read a value from a u32 R|W register, with user context",
            ),
            TestEntry::new(
                TestType::Read,
                10,
                None,
                10,
                &rw,
                SecurityContext::Machine,
                false,
                "failed to read a value from a u32 R|W register, with system context",
            ),
            TestEntry::new(
                TestType::Read,
                10,
                None,
                10,
                &prpw,
                SecurityContext::User,
                true,
                "succeeded in reading a value from a u32 PR|PW register, with user context",
            ),
            TestEntry::new(
                TestType::Read,
                10,
                None,
                10,
                &prpw,
                SecurityContext::Machine,
                false,
                "failed to read a value from a u32 PR|PW register, with system context",
            ),
            TestEntry::new(
                TestType::Write,
                0,
                Some(10),
                10,
                &rw,
                SecurityContext::User,
                false,
                "failed to write a value to a u32 R|W register, with user context",
            ),
            TestEntry::new(
                TestType::Write,
                0,
                Some(10),
                10,
                &rw,
                SecurityContext::Machine,
                false,
                "failed to write a value to a u32 R|W register, with system context",
            ),
            TestEntry::new(
                TestType::Write,
                0,
                Some(10),
                10,
                &prpw,
                SecurityContext::User,
                true,
                "succeeded in writing a value to a u32 PR|PW register, with user context",
            ),
            TestEntry::new(
                TestType::Write,
                0,
                Some(10),
                10,
                &prpw,
                SecurityContext::Machine,
                false,
                "failed to write a value to a u32 R|W register, with system context",
            ),
        ];

        for (i, test) in tests.iter().enumerate() {
            let result = panic::catch_unwind(|| {
                let mut register =
                    RegisterU32::new(RegisterId::R1, test.permissions, test.initial_value);

                if matches!(test.test_type, TestType::Write) {
                    register.write(test.write_value.unwrap(), &test.context);
                }

                let value = *register.read(&test.context);

                // Check that the read value is correct.
                assert_eq!(
                    value,
                    test.expected_value,
                    "{}",
                    test.fail_message(i, Some(value), false)
                );
            });

            // Check whether we panicked, and if we should have.
            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                test.should_panic,
                "{}",
                test.fail_message(i, None, did_panic)
            );
        }
    }

    /// Test the reading and writing of a (f32) register.
    #[test]
    fn test_register_f32() {
        let rw = RegisterPermission::R | RegisterPermission::W;
        let prpw = RegisterPermission::PR | RegisterPermission::PW;

        let tests = [
            TestEntry::new(
                TestType::Read,
                10f32,
                None,
                10f32,
                &rw,
                SecurityContext::User,
                false,
                "failed to read a value from a u32 R|W register, with user context",
            ),
            TestEntry::new(
                TestType::Read,
                10f32,
                None,
                10f32,
                &rw,
                SecurityContext::Machine,
                false,
                "failed to read a value from a u32 R|W register, with system context",
            ),
            TestEntry::new(
                TestType::Read,
                10f32,
                None,
                10f32,
                &prpw,
                SecurityContext::User,
                true,
                "succeeded in reading a value from a u32 PR|PW register, with user context",
            ),
            TestEntry::new(
                TestType::Read,
                10f32,
                None,
                10f32,
                &prpw,
                SecurityContext::Machine,
                false,
                "failed to read a value from a u32 PR|PW register, with system context",
            ),
            TestEntry::new(
                TestType::Write,
                0f32,
                Some(10f32),
                10f32,
                &rw,
                SecurityContext::User,
                false,
                "failed to write a value to a u32 R|W register, with user context",
            ),
            TestEntry::new(
                TestType::Write,
                0f32,
                Some(10f32),
                10f32,
                &rw,
                SecurityContext::Machine,
                false,
                "failed to write a value to a u32 R|W register, with system context",
            ),
            TestEntry::new(
                TestType::Write,
                0f32,
                Some(10f32),
                10f32,
                &prpw,
                SecurityContext::User,
                true,
                "succeeded in writing a value to a u32 PR|PW register, with user context",
            ),
            TestEntry::new(
                TestType::Write,
                0f32,
                Some(10f32),
                10f32,
                &prpw,
                SecurityContext::Machine,
                false,
                "failed to write a value to a u32 R|W register, with system context",
            ),
        ];

        for (i, test) in tests.iter().enumerate() {
            let result = panic::catch_unwind(|| {
                let mut register =
                    RegisterF32::new(RegisterId::R1, test.permissions, test.initial_value);

                if matches!(test.test_type, TestType::Write) {
                    register.write(test.write_value.unwrap(), &test.context);
                }

                let value = *register.read(&test.context);

                // Check that the read value is correct.
                assert_eq!(
                    value,
                    test.expected_value,
                    "{}",
                    test.fail_message(i, Some(value), false)
                );
            });

            // Check whether we panicked, and if we should have.
            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                test.should_panic,
                "{}",
                test.fail_message(i, None, did_panic)
            );
        }
    }
}
