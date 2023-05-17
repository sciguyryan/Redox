use bitflags::bitflags;
use prettytable::{row, Table};
use std::fmt::{self, Display};

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

pub struct RAM {
    /// The raw byte storage of this memory module.
    storage: Vec<u8>,
    /// A list of memory regions and their associated permissions.
    memory_regions: Vec<MemoryRegion>,
    /// An internal counter for the memory sequence IDs.
    seq_id: usize,
}

impl RAM {
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

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn get_memory_region(&self, seq_id: usize) -> Option<&MemoryRegion> {
        self.memory_regions.iter().find(|r| r.seq_id == seq_id)
    }

    pub fn get_memory_regions(&self) -> Vec<MemoryRegion> {
        self.memory_regions.to_vec()
    }

    pub fn get_ptr(&mut self, _pos: usize) -> &u8 {
        &0
    }

    pub fn get_clone(&mut self, _pos: usize) -> u8 {
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
}
