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

    /// Checks whether the allocated memory has a size greater than 0.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Check if a specific memory location is within the valid memory bounds.
    fn is_in_bounds(&self, index: usize) -> bool {
        index <= self.len()
    }

    pub fn get_region(&self, seq_id: usize) -> Option<&MemoryRegion> {
        self.memory_regions.iter().find(|r| r.seq_id == seq_id)
    }

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

    fn get_matching_region_with_highest_permissions(
        &self,
        start: usize,
        end: usize,
    ) -> &MemoryRegion {
        let mut regions = self.get_matching_regions(start, end);

        // The unwrap here should be safe since there should always be at least one
        // matching memory region.
        let region = regions.pop().unwrap();

        // The first case is a match where the range is -completely- within a region.
        // No cross-region permission issues can arise here.
        if start >= region.start && end <= region.end {
            return region;
        }

        // The second case is a cross-region match. This is where the range
        // is not contained completely within a single region. In theory this should
        // never occur, but it's best to be safe.
        assert!(!regions.is_empty(), "a coordinate was specified that fell outside the bounds of one region, but not into another");
        let region_2 = regions.pop().unwrap();

        // Here the memory region with the highest permissions out of the two topmost
        // regions will be returned.
        if region.permissions.0 > region_2.permissions.0 {
            region
        } else {
            region_2
        }
    }

    pub fn get_byte_ptr(&self, pos: usize, context: SecurityContext) -> &u8 {
        assert!(self.is_in_bounds(pos));

        // Check whether the memory region has read permissions.
        self.validate_access(pos, pos, DataAccessType::Read, context, false);

        &self.storage[pos]
    }

    pub fn get_byte_clone(&self, pos: usize, context: SecurityContext) -> u8 {
        assert!(self.is_in_bounds(pos));

        // Check whether the memory region has read permissions.
        self.validate_access(pos, pos, DataAccessType::Read, context, false);

        self.storage[pos]
    }

    pub fn get_range_ptr(&self, start: usize, len: usize, context: SecurityContext) -> &[u8] {
        let end = start + len;
        assert!(self.is_in_bounds(start));
        assert!(self.is_in_bounds(end));

        // Check whether the memory region has read permissions.
        self.validate_access(start, end, DataAccessType::Read, context, false);

        &self.storage[start..end]
    }

    pub fn get_range_clone(&self, start: usize, len: usize, context: SecurityContext) -> Vec<u8> {
        let end = start + len;
        assert!(self.is_in_bounds(start));
        assert!(self.is_in_bounds(end));

        // Check whether the memory region has read permissions.
        self.validate_access(start, end, DataAccessType::Read, context, false);

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
        assert!(self.is_in_bounds(start));
        assert!(self.is_in_bounds(end));

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

    fn validate_access(
        &self,
        start: usize,
        end: usize,
        access_type: DataAccessType,
        context: SecurityContext,
        is_exec: bool,
    ) {
        if !self.is_in_bounds(start) || !self.is_in_bounds(end) {
            panic!(
                "one or more memory locations are outside of the memory bounds. Start = {start}, End = {end}"
            );
        }

        // We always assume that we have value valid access permissions.
        let mut has_required_perms = true;

        // There will always have a memory region here since the root memory
        // region will always be present, with its default permissions.
        // We want to loop at the topmost layer when considering the permissions
        // applicable to this memory region.
        let region = self.get_matching_region_with_highest_permissions(start, end);
        let permissions = &region.permissions;

        // We also need to check the read/write/execute permissions on
        // the region, depending on what has been requested.
        // If the request has a user context of system then it will be granted
        // the permissions automatically.
        has_required_perms &= match access_type {
            DataAccessType::Execute => {
                (permissions.intersects(MemoryPermission::EX) && is_exec)
                    || context == SecurityContext::System
            }
            DataAccessType::Read => {
                permissions.intersects(MemoryPermission::R)
                    || (permissions.intersects(MemoryPermission::PR)
                        && context == SecurityContext::System)
            }
            DataAccessType::Write => {
                permissions.intersects(MemoryPermission::W)
                    || (permissions.intersects(MemoryPermission::PW)
                        && context == SecurityContext::System)
            }
        };

        if !has_required_perms {
            panic!("attempted to access memory without the correct security context or access flags. Access Type = {access_type:?}, Executable = {is_exec}, permissions = {permissions}.");
        }
    }
}

#[cfg(test)]
mod tests_ram {
    use crate::{
        ram::{MemoryPermission, MemoryRegion},
        security_context::SecurityContext,
    };

    use super::Ram;

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
            ram.get_range_ptr(0, size - 1, SecurityContext::System),
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
}
