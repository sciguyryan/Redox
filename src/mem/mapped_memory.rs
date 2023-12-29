#[derive(Debug)]
pub struct MappedMemory {
    /// The name of this memory segment.
    pub name: String,
    // The starting index of this mapped memory segment.
    pub start: usize,
    /// The end index of this mapped memory segment.
    pub end: usize,
    /// Whether reading from this mapped memory segment is permitted.
    pub can_read: bool,
    /// Whether writing to this mapped memory segment is permitted.
    pub can_write: bool,
    /// The physical contents of this memory segment.
    pub data: Vec<u8>,
}

impl MappedMemory {
    pub fn new(start: usize, length: usize, can_read: bool, can_write: bool, name: &str) -> Self {
        Self {
            name: name.to_string(),
            start,
            end: start + length,
            can_read,
            can_write,
            data: vec![0x0; length],
        }
    }

    /// Clear the contents of this memory segment.
    pub fn clear(&mut self) {
        for b in &mut self.data {
            *b = 0;
        }
    }

    /// Check whether a point exists within this memory region.
    ///
    /// # Arguments
    ///
    /// * `pos` - The memory location.
    ///
    /// # Returns
    ///
    /// A boolean indicating whether this memory segment contains the specified address.
    #[inline(always)]
    pub fn contains_point(&self, pos: usize) -> bool {
        self.contains_range(pos, pos)
    }

    /// Check whether a range completely exists within this memory segment.
    ///
    /// # Arguments
    ///
    /// * `start` - The starting memory location.
    /// * `end` - The ending memory location.
    ///
    /// # Returns
    ///
    /// A boolean indicating whether this memory segment contains the specified address range.
    #[inline(always)]
    pub fn contains_range(&self, start: usize, end: usize) -> bool {
        start >= self.start && end <= self.end
    }

    /// Get the length of this memory segment.
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Print the entire contents of this memory segment.
    pub fn print(&self) {
        println!("{:?}", self.data);
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

        println!("{:?}", self.data[start..end].to_vec());
    }

    /// Set the contents of this memory segment.
    ///
    /// # Arguments
    ///
    /// * `data` - The raw bytes to be loaded into this memory segment.
    ///
    /// # Notes
    ///
    /// This method enforced the fact that the new data will fit within the currently allocated segment.
    pub fn set_contents(&mut self, bytes: &[u8]) {
        assert!(bytes.len() < self.len());

        self.data[..bytes.len()].copy_from_slice(bytes);
    }
}
