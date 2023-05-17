use bitflags::bitflags;

bitflags! {
    pub struct MemoryAccess: u8 {
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

pub struct MemoryRegion {
    /// The start position of this memory region.
    pub start: usize,

    /// The end position of this memory region.
    pub end: usize,

    /// The access flags for this memory region.
    pub access: MemoryAccess,

    /// The unique memory sequence identifier for this memory region.
    pub seq_id: usize,

    /// The name of this memory region.
    pub name: String,
}

impl MemoryRegion {
    pub fn set_end(&mut self, end: usize) {
        self.end = end;
    }
}

impl MemoryRegion {
    pub fn new(
        start: usize,
        end: usize,
        access: MemoryAccess,
        seq_id: usize,
        name: String,
    ) -> Self {
        Self {
            start,
            end,
            access,
            seq_id,
            name,
        }
    }
}

pub struct RAM {
    /// The raw byte storage of the memory module.
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

        // Read and write permissions are set
        // for the entire root memory block.
        ram.add_memory_region(0, ram.len() - 1, MemoryAccess::R | MemoryAccess::W, "Root");

        ram
    }

    fn add_memory_region(
        &mut self,
        start: usize,
        end: usize,
        access: MemoryAccess,
        name: &str,
    ) -> usize {
        self.memory_regions.push(MemoryRegion::new(
            start,
            end,
            access,
            self.seq_id,
            name.to_string(),
        ));

        self.seq_id += 1;

        self.resize_root_memory_region();

        self.memory_regions.last().unwrap().seq_id
    }

    pub fn get(&mut self, pos: usize) -> &u8 {
        &0
    }

    pub fn get_clone(&mut self, pos: usize) -> u8 {
        0
    }

    pub fn get_range(&mut self, start: usize, len: usize) -> &[u8] {
        &[0]
    }

    pub fn get_range_clone(&mut self, start: usize, len: usize) -> Vec<u8> {
        Vec::new()
    }

    /// Resize the root memory region to equal the maximum memory bound.
    fn resize_root_memory_region(&mut self) {
        let mut max_end = 0;
        for r in &self.memory_regions {
            if r.end > max_end {
                max_end = r.end;
            }
        }

        let mut root = self.memory_regions.get_mut(0).expect("");
        root.end = max_end;
    }

    pub fn set(&mut self, pos: usize, value: u8) {}

    pub fn set_range(&mut self, pos: usize, value: u8) {}

    pub fn len(&self) -> usize {
        self.storage.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

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
}
