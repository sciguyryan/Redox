use bitflags::bitflags;
use prettytable::{row, Table};
use std::fmt::{self, Display};

use crate::{data_access_type::DataAccessType, security_context::SecurityContext};

bitflags! {
    #[derive(Clone, Debug, Eq, PartialEq)]
    pub struct MemoryPermission: u8 {
        /// None.
        const N = 1 << 0;
        /// Public read.
        const R = 1 << 1;
        /// Public write.
        const W = 1 << 2;
        /// Private read. Currently unused.
        const PR = 1 << 3;
        /// Private write. Currently unused.
        const PW = 1 << 4;
        /// Execute.
        const EX = 1 << 5;
    }
}

impl Display for MemoryPermission {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MemoryRegion {
    /// The start position of this memory region.
    pub start: usize,

    /// The end position of this memory region.
    pub end: usize,

    /// The access flags for this memory region.
    pub permissions: MemoryPermission,

    /// The unique memory sequence identifier for this memory region.
    pub seq_id: usize,

    /// The name of this memory region.
    pub name: String,
}

impl MemoryRegion {
    pub fn new(
        start: usize,
        end: usize,
        access: MemoryPermission,
        seq_id: usize,
        name: String,
    ) -> Self {
        Self {
            start,
            end,
            permissions: access,
            seq_id,
            name,
        }
    }
}

pub struct Ram {
    /// The raw byte storage of this memory module.
    storage: Vec<u8>,
    /// A list of memory regions and their associated permissions.
    memory_regions: Vec<MemoryRegion>,
    /// An internal counter for the memory sequence IDs.
    seq_id: usize,
}

impl Ram {
    // https://stackoverflow.com/questions/15045375/what-enforces-memory-protection-in-an-os

    pub fn new(size: usize) -> Self {
        let mut ram = Self {
            storage: vec![0x0; size],
            memory_regions: Vec::new(),
            seq_id: 0,
        };

        // Set the root read and write permissions for the first (root) memory block.
        ram.add_memory_region(
            0,
            ram.len() - 1,
            MemoryPermission::R | MemoryPermission::W,
            "Root",
        );

        ram
    }

    /// Add memory region with specific permissions to the memory region list.
    ///
    /// # Arguments
    ///
    /// * `start` - The starting index of the memory region.
    /// * `end` - The ending index of the memory region.
    /// * `access` - The [`MemoryAccessPerms`] for the specific memory region.
    /// * `name` - A string giving the name of this memory region.
    fn add_memory_region(
        &mut self,
        start: usize,
        end: usize,
        access: MemoryPermission,
        name: &str,
    ) -> usize {
        let old_seq_id = self.seq_id;
        self.memory_regions.push(MemoryRegion::new(
            start,
            end,
            access,
            old_seq_id,
            name.to_string(),
        ));

        self.seq_id += 1;

        old_seq_id
    }

    /// Assert that a specific memory position is within the valid memory region.
    ///
    /// # Arguments
    ///
    /// * `pos` - The point in memory to be checked.
    #[inline]
    fn assert_point_in_bounds(&self, pos: usize) {
        assert!(
            self.is_in_bounds(pos),
            "The memory location is outside of the valid memory bounds. Point = {pos}"
        );
    }

    /// Checks whether the allocated memory region has a size greater than 0.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Checks whether a specific memory position is within the valid memory region bounds.
    ///
    /// # Arguments
    ///
    /// * `pos` - The point in memory to be checked.
    ///
    /// # Returns
    ///
    /// True if the memory region is within the valid memory region bounds, false otherwise.
    #[inline]
    fn is_in_bounds(&self, pos: usize) -> bool {
        pos <= self.len()
    }

    /// Get a specific memory region, by its id.
    ///
    /// # Arguments
    ///
    /// * `seq_id` - The unique id for the region.
    ///
    /// # Returns
    ///
    /// An [`Option`] containing a reference to the [`MemoryRegion`] object if found, [`None`] otherwise.
    pub fn get_region(&self, seq_id: usize) -> Option<&MemoryRegion> {
        self.memory_regions.iter().find(|r| r.seq_id == seq_id)
    }

    /// Get a vector containing a clone of the current [`MemoryRegion`] objects for this instance.
    ///
    /// # Returns
    ///
    /// A vector containing copies of each of the [`MemoryRegion`] objects present in this object instance.
    pub fn get_regions(&self) -> Vec<MemoryRegion> {
        self.memory_regions.to_vec()
    }

    /// Note regions are in reversed order (newest first).
    fn get_matching_regions(&self, start: usize, end: usize) -> Vec<&MemoryRegion> {
        let mut regions = Vec::new();

        for r in self.memory_regions.iter().rev() {
            // The first case is a match where the range is -completely- within a region.
            // No cross-region permission issues can arise here.
            // The second case is a cross-region match, additional checks will need to be
            // made to ensure the correct permissions are applied.
            if (start >= r.start && end <= r.end) || (start <= r.end && r.start <= end) {
                regions.push(r);
            }
        }

        regions
    }

    /// Try to find the top-most (newest) memory region that is applicable to a given point in memory.
    ///
    /// # Arguments
    ///
    /// * `pos` - The point in memory to be checked.
    ///
    /// # Returns
    ///
    /// An Option containing a reference to the [`MemoryRegion`] instance that is applicable to the point in memory, or None if one wasn't found.
    fn get_topmost_matching_region(&self, pos: usize) -> Option<&MemoryRegion> {
        self.memory_regions
            .iter()
            .rev()
            .find(|r| pos >= r.start && pos <= r.end)
    }

    pub fn get_byte_ptr(&self, index: usize, context: &SecurityContext) -> &u8 {
        self.assert_point_in_bounds(index);

        // Check whether the memory region has read permissions.
        self.validate_access(index, &DataAccessType::Read, context, false);

        &self.storage[index]
    }

    pub fn get_byte_clone(&self, index: usize, context: &SecurityContext) -> u8 {
        assert!(self.is_in_bounds(index));

        // Check whether the memory region has read permissions.
        self.validate_access(index, &DataAccessType::Read, context, false);

        self.storage[index]
    }

    pub fn get_range_ptr(&self, start: usize, len: usize, context: &SecurityContext) -> &[u8] {
        let end = start + len;
        self.assert_point_in_bounds(start);
        self.assert_point_in_bounds(end);

        // Check whether the memory region has read permissions.
        self.validate_access_range(start, end, &DataAccessType::Read, context, false);

        &self.storage[start..end]
    }

    pub fn get_range_clone(&self, start: usize, len: usize, context: &SecurityContext) -> Vec<u8> {
        let end = start + len;
        self.assert_point_in_bounds(start);
        self.assert_point_in_bounds(end);

        // Check whether the memory region has read permissions.
        self.validate_access_range(start, end, &DataAccessType::Read, context, false);

        self.storage[start..end].to_vec()
    }

    pub fn len(&self) -> usize {
        self.storage.len()
    }

    pub fn set(&mut self, _pos: usize, _value: u8, _context: SecurityContext) {}

    pub fn set_range(&mut self, _pos: usize, _value: u8, _context: SecurityContext) {}

    pub fn print(&self) {
        println!("{:?}", self.storage);
    }

    pub fn print_range(&self, start: usize, len: usize) {
        let end = start + len;
        self.assert_point_in_bounds(start);
        self.assert_point_in_bounds(end);

        println!("{:?}", self.storage[start..end].to_vec());
    }

    pub fn print_memory_regions(&self) {
        let mut table = Table::new();

        table.add_row(row!["Start", "End", "Permissions", "ID", "Name"]);

        for region in &self.memory_regions {
            table.add_row(row![
                region.start,
                region.end,
                region.permissions,
                region.seq_id,
                region.name
            ]);
        }

        table.printstd();
    }

    #[inline]
    fn validate_access(
        &self,
        point: usize,
        access_type: &DataAccessType,
        context: &SecurityContext,
        is_exec: bool,
    ) {
        // There will always have a memory region here since the root memory
        // region will always be present, with its default permissions.
        // We want to look at the topmost layer when considering the permissions
        // applicable to this memory region.
        let region = self
            .get_topmost_matching_region(point)
            .expect("no matching region was found");
        let permissions = &region.permissions;

        // We also need to check the read/write/execute permissions on
        // the region, depending on what has been requested.
        // If the request has a user context of system then it will be granted
        // the permissions automatically.
        let has_required_perms = match access_type {
            DataAccessType::Execute => {
                (permissions.intersects(MemoryPermission::EX) && is_exec)
                    || *context == SecurityContext::System
            }
            DataAccessType::Read => {
                permissions.intersects(MemoryPermission::R)
                    || (permissions.intersects(MemoryPermission::PR)
                        && *context == SecurityContext::System)
            }
            DataAccessType::Write => {
                permissions.intersects(MemoryPermission::W)
                    || (permissions.intersects(MemoryPermission::PW)
                        && *context == SecurityContext::System)
            }
        };

        if !has_required_perms {
            panic!("attempted to access memory without the correct security context or access flags. Access Type = {access_type:?}, Executable = {is_exec}, permissions = {permissions}.");
        }
    }

    #[inline]
    fn validate_access_range(
        &self,
        start: usize,
        len: usize,
        access_type: &DataAccessType,
        context: &SecurityContext,
        is_exec: bool,
    ) {
        for i in start..(start + len) {
            self.validate_access(i, access_type, context, is_exec);
        }
    }
}

#[cfg(test)]
mod tests_ram {
    use std::panic;

    use crate::{
        ram::{MemoryPermission, MemoryRegion},
        security_context::SecurityContext,
    };

    use super::Ram;

    struct TestEntry {
        pub start: usize,
        pub end: usize,
        pub context: SecurityContext,
        pub should_panic: bool,
        pub fail_message: String,
    }

    impl TestEntry {
        pub fn new(
            start: usize,
            end: usize,
            context: SecurityContext,
            should_panic: bool,
            fail_message: &str,
        ) -> Self {
            Self {
                start,
                end,
                context,
                should_panic,
                fail_message: fail_message.to_string(),
            }
        }

        pub fn fail_message(&self, did_panic: bool) -> String {
            format!(
                "Start: {}, End: {}, Context: {:?}, Should Panic? {}. Panicked? {did_panic}. Message = {}",
                self.start, self.end, self.context, self.should_panic, self.fail_message
            )
        }
    }

    fn get_test_ram_instance() -> Ram {
        let mut ram = Ram::new(100);

        ram.add_memory_region(
            50,
            100,
            MemoryPermission::PR | MemoryPermission::PW,
            "Private",
        );

        ram
    }

    /// Test the basic aspects of creating a RAM module.
    #[test]
    fn test_ram_creation() {
        let size = 100;
        let ram = Ram::new(size);

        assert_eq!(
            ram.len(),
            size,
            "failed to create a RAM module of the specified size"
        );

        assert_eq!(
            ram.get_range_ptr(0, size - 1, &SecurityContext::System),
            vec![0x0; size - 1],
            "failed to correctly initialize RAM"
        );

        let regions = ram.get_regions();
        let root_region = MemoryRegion::new(
            0,
            size - 1,
            MemoryPermission::R | MemoryPermission::W,
            0,
            "Root".to_string(),
        );
        assert_eq!(regions, vec![root_region]);
    }

    /// Test region-based permissions.
    #[test]
    fn test_region_permissions() {
        let ram = get_test_ram_instance();

        let tests = [
            TestEntry::new(
                0,
                50,
                SecurityContext::User,
                false,
                "failed to read R|W memory with user context",
            ),
            TestEntry::new(
                0,
                51,
                SecurityContext::User,
                true,
                "succeeded in reading PR|PW memory with user context",
            ),
            TestEntry::new(
                0,
                50,
                SecurityContext::System,
                false,
                "failed to read R|W memory with system context",
            ),
            TestEntry::new(
                0,
                51,
                SecurityContext::System,
                false,
                "failed to read PR|PW memory with system context",
            ),
        ];

        for test in tests {
            let result = panic::catch_unwind(|| {
                _ = ram.get_range_clone(test.start, test.end, &test.context);
            });

            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                test.should_panic,
                "{}",
                test.fail_message(did_panic)
            );
        }
    }
}
