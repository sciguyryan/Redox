use bitflags::bitflags;
use std::{
    cmp::Ordering,
    fmt::{self, Display},
};

use crate::{
    cpu::CpuFlag, data_access_type::DataAccessType, privilege_level::PrivilegeLevel, utils,
};

use super::registers::RegisterId;

bitflags! {
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct RegisterPermission: u8 {
        /// None.
        const N = 0b00000001;
        /// Public read.
        const R = 0b00000010;
        /// Public write.
        const W = 0b00000100;
        /// Private read.
        const PR = 0b00001000;
        /// Private write.
        const PW = 0b00010000;
    }
}

impl Display for RegisterPermission {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[derive(Debug, Clone, Eq, PartialEq)]
pub struct RegisterU32 {
    /// The [`RegisterId`] of this register.
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

    /// Add a specific value to this register.
    ///
    /// # Arguments
    ///
    /// * `val` - The value to be added.
    /// * `privilege` - A reference to the [`PrivilegeLevel`] to be used when completing this action.
    #[inline(always)]
    pub fn add(&mut self, val: u32, privilege: &PrivilegeLevel) {
        // Check whether the register has read/write permissions.
        self.validate_access(&DataAccessType::Write, privilege);

        self.value += val;
    }

    /// Add a specific value to this register without performing a permission check.
    ///
    /// # Arguments
    ///
    /// * `val` - The value to be added.
    #[inline(always)]
    pub fn add_unchecked(&mut self, value: u32) {
        self.value += value;
    }

    /// Subtract a specific value from this register without performing a permission check.
    ///
    /// # Arguments
    ///
    /// * `val` - The value to be subtracted.
    #[inline(always)]
    pub fn subtract_unchecked(&mut self, value: u32) {
        self.value -= value;
    }

    /// Get the value of the CPU flags register as a human-readable string.
    pub fn get_flags_register_string(&self) -> String {
        let mut flags = String::new();

        if self.id == RegisterId::EFL {
            let states: Vec<String> = CpuFlag::iterator()
                .map(|flag_type| {
                    let is_set = if utils::is_bit_set(self.value, (*flag_type) as u8) {
                        '1'
                    } else {
                        '0'
                    };
                    format!("{flag_type} = {is_set}")
                })
                .collect();
            flags = states.join(", ");
        }

        flags
    }

    /// Increment the value of this register.
    ///
    /// # Arguments
    ///
    /// * `privilege` - A reference to the [`PrivilegeLevel`] to be used when completing this action.
    #[inline(always)]
    pub fn increment(&mut self, privilege: &PrivilegeLevel) {
        self.add(1, privilege);
    }

    /// Increment the value of this register without performing a permission check.
    #[inline(always)]
    pub fn increment_unchecked(&mut self) {
        self.add_unchecked(1);
    }

    /// Read the value of this register.
    ///
    /// # Arguments
    ///
    /// * `privilege` - A reference to the [`PrivilegeLevel`] to be used when completing this action.
    ///
    /// # Returns
    ///
    /// A reference to the u32 value of this register.
    #[inline(always)]
    pub fn read(&self, privilege: &PrivilegeLevel) -> u32 {
        // Check whether the register has read permissions.
        self.validate_access(&DataAccessType::Read, privilege);

        self.value
    }

    /// Read the value of this register without performing a permission check.
    ///
    /// # Returns
    ///
    /// A reference to the u32 value of this register.
    #[inline(always)]
    pub fn read_unchecked(&self) -> u32 {
        self.value
    }

    /// Subtract a specific value from this register.
    ///
    /// # Arguments
    ///
    /// * `val` - The value to be subtracted.
    /// * `privilege` - The [`PrivilegeLevel`] to be used when completing this action.
    #[inline(always)]
    pub fn subtract(&mut self, val: u32, privilege: &PrivilegeLevel) {
        // Check whether the register has read/write permissions.
        self.validate_access(&DataAccessType::Write, privilege);

        self.value -= val;
    }

    /// Write a value to this register.
    ///
    /// # Arguments
    ///
    /// * `value` - The u32 value to be written to this register.
    /// * `privilege` - The [`PrivilegeLevel`] to be used when completing this action.
    #[inline(always)]
    pub fn write(&mut self, value: u32, privilege: &PrivilegeLevel) {
        // Check whether the register has write permissions.
        self.validate_access(&DataAccessType::Write, privilege);

        self.value = value;
    }

    /// Write a value to this register without performing a permission check.
    ///
    /// # Arguments
    ///
    /// * `value` - The u32 value to be written to this register.
    #[inline(always)]
    pub fn write_unchecked(&mut self, value: u32) {
        self.value = value;
    }

    /// Validate whether a specific action can be performed based on the privilege level.
    ///
    /// # Arguments
    ///
    /// * `access_type` - A reference to the [`DataAccessType`] indicating the action to be performed.
    /// * `privilege` - A reference to the [`PrivilegeLevel`] to be used when completing this action.
    #[inline(always)]
    fn validate_access(&self, access_type: &DataAccessType, privilege: &PrivilegeLevel) {
        // System-level contexts are permitted to do anything, without limitation.
        if *privilege == PrivilegeLevel::Machine {
            return;
        }

        let permissions = &self.permissions;

        // We need to check the read-write permissions on the specific register.
        let has_required_perms = match access_type {
            DataAccessType::Read => permissions.intersects(RegisterPermission::R),
            DataAccessType::Write => permissions.intersects(RegisterPermission::W),
        };

        if !has_required_perms {
            panic!("attempted to access register without the correct security context and access flags. Register = {:?}, Access Type = {access_type:?}, permissions = {permissions}.", self.id);
        }
    }
}

impl Ord for RegisterU32 {
    fn cmp(&self, other: &Self) -> Ordering {
        let id_ordering = self.id.cmp(&other.id);

        // If ids are equal, use lexicographical ordering by value.
        if id_ordering == Ordering::Equal {
            self.value.cmp(&other.value)
        } else {
            id_ordering
        }
    }
}

impl PartialOrd for RegisterU32 {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Debug, Clone)]
pub struct RegisterF32 {
    /// The [`RegisterId`] of this register.
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

    /// Read the value of this register.
    ///
    /// # Arguments
    ///
    /// * `privilege` - A reference to the [`PrivilegeLevel`] to be used when completing this action.
    ///
    /// # Returns
    ///
    /// A reference to the f32 value of this register.
    pub fn read(&self, privilege: &PrivilegeLevel) -> f32 {
        // Check whether the register has read permissions.
        self.validate_access(&DataAccessType::Read, privilege);

        self.value
    }

    /// Read the value of this register without performing a permission check.
    ///
    /// # Returns
    ///
    /// A reference to the f32 value of this register.
    pub fn read_unchecked(&self) -> f32 {
        self.value
    }

    /// Write a value to this register without performing a permission check.
    ///
    /// # Arguments
    ///
    /// * `value` - The f32 value to be written to this register.
    pub fn write(&mut self, value: f32, privilege: &PrivilegeLevel) {
        // Check whether the register has write permissions.
        self.validate_access(&DataAccessType::Write, privilege);

        self.value = value;
    }

    /// Write a value to this register without performing a permission check.
    ///
    /// # Arguments
    ///
    /// * `value` - The f32 value to be written to this register.
    pub fn write_unchecked(&mut self, value: f32) {
        self.value = value;
    }

    /// Validate whether a specific action can be performed based on the privilege level.
    ///
    /// # Arguments
    ///
    /// * `access_type` - A reference to the [`DataAccessType`] indicating the action to be performed.
    /// * `privilege` - A reference to the [`PrivilegeLevel`] to be used when completing this action.
    #[inline]
    fn validate_access(&self, access_type: &DataAccessType, privilege: &PrivilegeLevel) {
        // System-level contexts are permitted to do anything, without limitation.
        if *privilege == PrivilegeLevel::Machine {
            return;
        }

        let permissions = &self.permissions;

        // We need to check the read-write permissions on the specific register.
        let has_required_perms = match access_type {
            DataAccessType::Read => permissions.intersects(RegisterPermission::R),
            DataAccessType::Write => permissions.intersects(RegisterPermission::W),
        };

        if !has_required_perms {
            panic!("attempted to access register without the correct security context or access flags. Register = {:?}, Access Type = {access_type:?}, permissions = {permissions}.", self.id);
        }
    }
}

impl Ord for RegisterF32 {
    fn cmp(&self, other: &Self) -> Ordering {
        let id_ordering = self.id.cmp(&other.id);

        // If ids are equal, use lexicographical ordering by value.
        if id_ordering == Ordering::Equal {
            if self.value < other.value {
                Ordering::Less
            } else if self.value == other.value {
                Ordering::Equal
            } else {
                Ordering::Greater
            }
        } else {
            id_ordering
        }
    }
}

impl PartialOrd for RegisterF32 {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for RegisterF32 {}

impl PartialEq for RegisterF32 {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id && self.permissions == other.permissions && self.value == other.value
    }
}

#[cfg(test)]
mod tests_registers {
    use std::{
        fmt::{Debug, Display},
        panic,
    };

    use crate::{privilege_level::PrivilegeLevel, reg::registers::RegisterId};

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
        pub privilege: PrivilegeLevel,
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
            context: PrivilegeLevel,
            should_panic: bool,
            fail_message: &str,
        ) -> Self {
            Self {
                test_type,
                initial_value,
                write_value,
                expected_value,
                permissions: *permissions,
                privilege: context,
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
                PrivilegeLevel::User,
                false,
                "failed to read a value from a u32 R|W register, with user privilege",
            ),
            TestEntry::new(
                TestType::Read,
                10,
                None,
                10,
                &rw,
                PrivilegeLevel::Machine,
                false,
                "failed to read a value from a u32 R|W register, with machine privilege",
            ),
            TestEntry::new(
                TestType::Read,
                10,
                None,
                10,
                &prpw,
                PrivilegeLevel::User,
                true,
                "succeeded in reading a value from a u32 PR|PW register, with user privilege",
            ),
            TestEntry::new(
                TestType::Read,
                10,
                None,
                10,
                &prpw,
                PrivilegeLevel::Machine,
                false,
                "failed to read a value from a u32 PR|PW register, with machine privilege",
            ),
            TestEntry::new(
                TestType::Write,
                0,
                Some(10),
                10,
                &rw,
                PrivilegeLevel::User,
                false,
                "failed to write a value to a u32 R|W register, with user privilege",
            ),
            TestEntry::new(
                TestType::Write,
                0,
                Some(10),
                10,
                &rw,
                PrivilegeLevel::Machine,
                false,
                "failed to write a value to a u32 R|W register, with machine privilege",
            ),
            TestEntry::new(
                TestType::Write,
                0,
                Some(10),
                10,
                &prpw,
                PrivilegeLevel::User,
                true,
                "succeeded in writing a value to a u32 PR|PW register, with user privilege",
            ),
            TestEntry::new(
                TestType::Write,
                0,
                Some(10),
                10,
                &prpw,
                PrivilegeLevel::Machine,
                false,
                "failed to write a value to a u32 R|W register, with machine privilege",
            ),
        ];

        for (i, test) in tests.iter().enumerate() {
            let result = panic::catch_unwind(|| {
                let mut register =
                    RegisterU32::new(RegisterId::EAX, test.permissions, test.initial_value);

                if matches!(test.test_type, TestType::Write) {
                    register.write(test.write_value.unwrap(), &test.privilege);
                }

                let value = register.read(&test.privilege);

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
                PrivilegeLevel::User,
                false,
                "failed to read a value from a u32 R|W register, with user privilege",
            ),
            TestEntry::new(
                TestType::Read,
                10f32,
                None,
                10f32,
                &rw,
                PrivilegeLevel::Machine,
                false,
                "failed to read a value from a u32 R|W register, with machine privilege",
            ),
            TestEntry::new(
                TestType::Read,
                10f32,
                None,
                10f32,
                &prpw,
                PrivilegeLevel::User,
                true,
                "succeeded in reading a value from a u32 PR|PW register, with user privilege",
            ),
            TestEntry::new(
                TestType::Read,
                10f32,
                None,
                10f32,
                &prpw,
                PrivilegeLevel::Machine,
                false,
                "failed to read a value from a u32 PR|PW register, with machine privilege",
            ),
            TestEntry::new(
                TestType::Write,
                0f32,
                Some(10f32),
                10f32,
                &rw,
                PrivilegeLevel::User,
                false,
                "failed to write a value to a u32 R|W register, with user privilege",
            ),
            TestEntry::new(
                TestType::Write,
                0f32,
                Some(10f32),
                10f32,
                &rw,
                PrivilegeLevel::Machine,
                false,
                "failed to write a value to a u32 R|W register, with machine privilege",
            ),
            TestEntry::new(
                TestType::Write,
                0f32,
                Some(10f32),
                10f32,
                &prpw,
                PrivilegeLevel::User,
                true,
                "succeeded in writing a value to a u32 PR|PW register, with user privilege",
            ),
            TestEntry::new(
                TestType::Write,
                0f32,
                Some(10f32),
                10f32,
                &prpw,
                PrivilegeLevel::Machine,
                false,
                "failed to write a value to a u32 R|W register, with machine privilege",
            ),
        ];

        for (i, test) in tests.iter().enumerate() {
            let result = panic::catch_unwind(|| {
                let mut register =
                    RegisterF32::new(RegisterId::EAX, test.permissions, test.initial_value);

                if matches!(test.test_type, TestType::Write) {
                    register.write(test.write_value.unwrap(), &test.privilege);
                }

                let value = register.read(&test.privilege);

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
