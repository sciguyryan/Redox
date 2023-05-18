use bitflags::bitflags;
use prettytable::{row, Table};
use std::fmt::{self, Display};

use crate::{data_access_type::DataAccessType, security_context::SecurityContext};

bitflags! {
    #[derive(Clone, Debug)]
    pub struct MemoryPermission: u8 {
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
        /// Execute.
        const EX = 1 << 5;
    }
}

impl Display for MemoryPermission {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[derive(Clone, Debug)]
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

    pub fn get_memory_region(&self, seq_id: usize) -> Option<&MemoryRegion> {
        self.memory_regions.iter().find(|r| r.seq_id == seq_id)
    }

    pub fn get_memory_regions(&self) -> Vec<MemoryRegion> {
        self.memory_regions.to_vec()
    }

    pub fn get_byte_ptr(&mut self, _pos: usize) -> &u8 {
        &0
    }

    pub fn get_byte_clone(&mut self, _pos: usize) -> u8 {
        0
    }

    pub fn get_range_ptr(&mut self, _start: usize, _len: usize) -> &[u8] {
        &[0]
    }

    pub fn get_range_clone(&mut self, _start: usize, _len: usize) -> Vec<u8> {
        Vec::new()
    }

    pub fn len(&self) -> usize {
        self.storage.len()
    }

    pub fn set(&mut self, _pos: usize, _value: u8) {}

    pub fn set_range(&mut self, _pos: usize, _value: u8) {}

    pub fn print(&self) {
        println!("{:?}", self.storage);
    }

    pub fn print_range(&self, start: usize, len: usize) {
        let end = start + len;
        if start >= self.len() || end >= self.len() {
            // This should fire a segfault.
            panic!();
        }

        println!("{:?}", self.storage[start..len].to_vec());
    }

    pub fn print_memory_regions(&self) {
        let mut table = Table::new();

        // |             0,64400 |            R, W |          0 |            Root|
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

    fn get_memory_permissions(&self, start: usize, end: usize) -> Vec<&MemoryRegion> {
        let mut regions = Vec::new();

        for r in self.memory_regions.iter().rev() {
            // The first case is a match where the range is -completely- within a region.
            // No cross-region permission issues can arise here.
            // The second case is a cross-region match, additional checks will need to be
            // made to ensure the correct permissions are applied.
            if (start >= r.start && end <= r.end) || (start <= r.end && r.start <= end) {
                regions.push(r);
                continue;
            }
        }

        regions
    }

    fn validate_access(
        &self,
        start: usize,
        end: usize,
        access_type: DataAccessType,
        context: SecurityContext,
        is_exec: bool,
    ) {
        // System security contexts are permitted to do anything.
        if context == SecurityContext::System {
            return;
        }

        if start >= self.len() || end >= self.len() {
            panic!(
                "the memory location is outside of the memory bounds. Start = {start}, End = {end}"
            );
        }

        // If we have an address range that intersects one or more memory
        // regions then we need to choose the access flags from the region
        // that has the highest permissions of those that were returned.
        // The logic being that the highest permissions will need to be met
        // for access to be granted to any point within the range.
        let mut has_required_perms = true;

        // We can safely assume that we will always have a memory region here
        // since the root memory region will always be present.
        let regions = self.get_memory_permissions(start, end);
        let mut permission_region = regions.first().expect("");

        for r in &regions {
            // Does the region have greater permissions than the current one?
            if r.permissions.0 > permission_region.permissions.0 {
                permission_region = r;
            }
        }

        let permissions = &permission_region.permissions;

        // We also need to check the read/write/execute permissions on
        // the region, depending on what has been requested.
        // If the request has a user context of system then it will be granted
        // the permissions automatically.
        has_required_perms &= match access_type {
            DataAccessType::Execute => permissions.intersects(MemoryPermission::EX) && is_exec,
            DataAccessType::Read => {
                permissions.intersects(MemoryPermission::R)
                    || permissions.intersects(MemoryPermission::PR)
            }
            DataAccessType::Write => {
                permissions.intersects(MemoryPermission::W)
                    || permissions.intersects(MemoryPermission::PW)
            }
        };

        if !has_required_perms {
            panic!("attempted to access memory without the correct security context or access flags. Access Type = {access_type:?}, Executable = {is_exec}, permissions = {permissions}.");
        }
    }
}
