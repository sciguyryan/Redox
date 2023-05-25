use bitflags::bitflags;
use std::fmt::{self, Display};

use crate::{data_access_type::DataAccessType, security_context::SecurityContext};

use super::registers::RegisterId;

bitflags! {
    #[derive(Clone, Debug, Eq, PartialEq)]
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

pub enum RegisterValue {
    Float(f32),
    U32(u32),
}

pub struct Register {
    /// The ID of this register.
    pub id: RegisterId,
    /// The permissions of this register.
    pub permissions: RegisterPermission,
    /// The value of this register.
    value: RegisterValue,
}

impl Register {
    pub fn new(id: RegisterId, permissions: RegisterPermission, value: RegisterValue) -> Self {
        Self {
            id,
            permissions,
            value,
        }
    }

    pub fn read(&self, context: &SecurityContext) -> &RegisterValue {
        self.validate_access(&DataAccessType::Read, context);

        &self.value
    }

    pub fn write(&mut self, value: RegisterValue, context: &SecurityContext) {
        self.validate_access(&DataAccessType::Write, context);

        let is_type_matching = match value {
            RegisterValue::Float(_) => matches!(self.value, RegisterValue::Float(_)),
            RegisterValue::U32(_) => matches!(self.value, RegisterValue::U32(_)),
        };

        if !is_type_matching {
            panic!("attempted to change register value type");
        }

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
