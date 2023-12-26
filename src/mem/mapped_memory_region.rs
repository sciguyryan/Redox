#[derive(Debug)]
pub struct MappedMemoryRegion {
    /// The name of this memory region.
    pub name: String,
    // The starting index of this mapped memory region.
    pub start: usize,
    /// The end index of this mapped memory region.
    pub end: usize,
    /// The physical contents of this memory region.
    pub memory: Vec<u8>,
}

impl MappedMemoryRegion {
    pub fn new(start: usize, end: usize, name: String) -> Self {
        Self {
            name,
            start,
            end,
            memory: vec![0x0; end],
        }
    }

    /// Clear the contents of this memory region.
    pub fn clear(&mut self) {
        self.memory = vec![0; self.len()]
    }

    /// Check whether a range completely exists within this memory region.
    ///
    /// # Arguments
    ///
    /// * `start` - The starting memory location.
    /// * `end` - The ending memory location.
    ///
    /// # Returns
    ///
    /// A boolean indicating whether this memory region contains the specified address range.
    pub fn contains_range(&self, start: usize, end: usize) -> bool {
        start >= self.start && end <= self.end
    }

    /// Get the length of this memory region.
    pub fn len(&self) -> usize {
        self.memory.len()
    }

    /// Print the entire contents of memory.
    pub fn print(&self) {
        println!("{:?}", self.memory);
    }

    /// Print a specific range of bytes from memory.
    ///
    /// # Arguments
    ///
    /// * `start` - The starting memory location.
    /// * `len` - The number of bytes to print.
    pub fn print_range(&self, start: usize, len: usize) {
        let end = start + len;
        assert!(self.contains_range(start, end));

        println!("{:?}", self.memory[start..end].to_vec());
    }
}
