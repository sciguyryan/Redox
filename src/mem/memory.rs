use bitflags::bitflags;
use prettytable::{row, Table};
use std::fmt::{self, Display};

use crate::{
    data_access_type::DataAccessType, ins::instruction::Instruction,
    security_context::SecurityContext,
};

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

pub struct Memory {
    /// The raw byte storage of this memory module.
    storage: Vec<u8>,
    /// A list of memory regions and their associated permissions.
    memory_regions: Vec<MemoryRegion>,
    /// An internal counter for the memory sequence IDs.
    seq_id: usize,
}

impl Memory {
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

    pub fn get_byte_ptr(&self, pos: usize, context: &SecurityContext) -> &u8 {
        self.assert_point_in_bounds(pos);

        // Check whether the memory region has read permissions.
        self.validate_access(pos, &DataAccessType::Read, context, false);

        &self.storage[pos]
    }

    pub fn get_byte_clone(&self, pos: usize, context: &SecurityContext) -> u8 {
        assert!(self.is_in_bounds(pos));

        // Check whether the memory region has read permissions.
        self.validate_access(pos, &DataAccessType::Read, context, false);

        self.storage[pos]
    }

    pub fn get_instruction(&self, pos: usize, context: &SecurityContext) -> Instruction {
        let range = self.get_range_ptr(pos, 2, true, context);

        Instruction::Nop
    }

    pub fn get_u32(&self, pos: usize, context: &SecurityContext) -> u32 {
        let bytes: [u8; 4] = self
            .get_range_ptr(pos, pos + 4, false, context)
            .try_into()
            .expect("failed to create a u32 from memory bytes");

        u32::from_le_bytes(bytes)
    }

    pub fn get_range_ptr(
        &self,
        start: usize,
        len: usize,
        needs_exec: bool,
        context: &SecurityContext,
    ) -> &[u8] {
        let end = start + len;
        self.assert_point_in_bounds(start);
        self.assert_point_in_bounds(end);

        // Check whether the memory region has read permissions.
        self.validate_access_range(start, end, &DataAccessType::Read, context, false);

        if needs_exec {
            self.validate_access_range(start, end, &DataAccessType::Execute, context, true);
        }

        &self.storage[start..end]
    }

    pub fn get_range_clone(
        &self,
        start: usize,
        len: usize,
        needs_exec: bool,
        context: &SecurityContext,
    ) -> Vec<u8> {
        self.get_range_ptr(start, len, needs_exec, context).to_vec()
    }

    pub fn get_storage(&self) -> &[u8] {
        &self.storage
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

    pub fn len(&self) -> usize {
        self.storage.len()
    }

    pub fn clear(&mut self) {
        self.storage = vec![0; self.storage.len()];
    }

    pub fn set(&mut self, pos: usize, value: u8, context: &SecurityContext) {
        self.assert_point_in_bounds(pos);

        // Check whether the memory region has write permissions.
        self.validate_access(pos, &DataAccessType::Write, context, false);

        self.storage[pos] = value;
    }

    pub fn set_range(&mut self, start: usize, values: &[u8], context: &SecurityContext) {
        let end = start + values.len();
        self.assert_point_in_bounds(start);
        self.assert_point_in_bounds(end);

        // Check whether the memory region has write permissions.
        self.validate_access_range(start, end, &DataAccessType::Write, context, false);

        for (i, b) in values.iter().enumerate() {
            self.storage[start + i] = *b;
        }
    }

    pub fn set_u32(&mut self, pos: usize, value: u32, context: &SecurityContext) {
        let bytes = u32::to_le_bytes(value);

        self.set_range(pos, &bytes, context);
    }

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
        exec: bool,
    ) {
        // System-level contexts are permitted to do anything, without limitation.
        // NOTE: This might end up being replaced with a ring-permission type system.
        if *context == SecurityContext::Machine {
            return;
        }

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
            DataAccessType::Execute => permissions.intersects(MemoryPermission::EX) && exec,
            DataAccessType::Read => {
                permissions.intersects(MemoryPermission::R)
                    || (permissions.intersects(MemoryPermission::PR)
                        && *context == SecurityContext::Machine)
            }
            DataAccessType::Write => {
                permissions.intersects(MemoryPermission::W)
                    || (permissions.intersects(MemoryPermission::PW)
                        && *context == SecurityContext::Machine)
            }
        };

        if !has_required_perms {
            panic!("attempted to access memory without the correct security context or access flags. Access Type = {access_type:?}, Executable = {exec}, permissions = {permissions}.");
        }
    }

    #[inline]
    fn validate_access_range(
        &self,
        start: usize,
        len: usize,
        access_type: &DataAccessType,
        context: &SecurityContext,
        exec: bool,
    ) {
        for i in start..(start + len) {
            self.validate_access(i, access_type, context, exec);
        }
    }
}

#[cfg(test)]
mod tests_memory {
    use std::panic;

    use crate::{mem::memory::MemoryRegion, security_context::SecurityContext};

    use super::{Memory, MemoryPermission};

    enum TestType {
        Read,
        Write,
        Execute,
    }

    struct TestEntry {
        pub test_type: TestType,
        pub start: usize,
        pub end: usize,
        pub write_values: Option<Vec<u8>>,
        pub expected_values: Vec<u8>,
        pub context: SecurityContext,
        pub should_panic: bool,
        pub fail_message: String,
    }

    impl TestEntry {
        #[allow(clippy::too_many_arguments)]
        pub fn new(
            test_type: TestType,
            start: usize,
            end: usize,
            write_values: Option<Vec<u8>>,
            expected_values: Vec<u8>,
            context: SecurityContext,
            should_panic: bool,
            fail_message: &str,
        ) -> Self {
            Self {
                test_type,
                start,
                end,
                write_values,
                expected_values,
                context,
                should_panic,
                fail_message: fail_message.to_string(),
            }
        }

        pub fn fail_message(&self, id: usize, did_panic: bool, values: &[u8]) -> String {
            let test_type = match self.test_type {
                TestType::Read => "Read",
                TestType::Write => "Write",
                TestType::Execute => "Execute",
            };

            format!(
                "{test_type} Test {id} Failed - Expected: {:?}, Got: {:?}, Should Panic? {}, Panicked? {did_panic}. Message = {}",
                self.expected_values, values, self.should_panic, self.fail_message
            )
        }
    }

    fn fill_memory_sequential(mem: &mut Memory) {
        for (i, byte) in &mut mem.storage.iter_mut().enumerate() {
            *byte = (i as u8) % 255;
        }
    }

    fn get_test_ram_instance() -> Memory {
        let mut ram = Memory::new(100);

        ram.add_memory_region(
            0,
            25,
            MemoryPermission::R | MemoryPermission::W | MemoryPermission::EX,
            "Public Execution",
        );

        ram.add_memory_region(
            50,
            100,
            MemoryPermission::PR | MemoryPermission::PW,
            "Private",
        );

        // Fill the memory with debug data.
        fill_memory_sequential(&mut ram);

        ram
    }

    fn get_test_nested_ram_instance() -> Memory {
        let mut ram = Memory::new(100);

        // Layer 1 - a private read/write region.
        ram.add_memory_region(
            50,
            100,
            MemoryPermission::PR | MemoryPermission::PW,
            "Private",
        );

        // Layer 2 - a public read/write region. This one should take precedence.
        // This should make the entire memory region public read/write.
        ram.add_memory_region(50, 100, MemoryPermission::R | MemoryPermission::W, "Public");

        // Fill the memory with debug data.
        fill_memory_sequential(&mut ram);

        ram
    }

    /// Test the basic aspects of creating a RAM module.
    #[test]
    fn test_ram_creation() {
        let size = 100;
        let ram = Memory::new(size);

        assert_eq!(
            ram.len(),
            size,
            "failed to create a RAM module of the specified size"
        );

        assert_eq!(
            ram.get_range_ptr(0, size - 1, false, &SecurityContext::Machine),
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

    /// Test basic reading and writing.
    #[test]
    fn test_read_write() {
        let mut ram = get_test_ram_instance();

        // Read/write a single byte.
        let byte = 0xff;
        ram.set(0, byte, &SecurityContext::Machine);
        assert_eq!(
            byte,
            *ram.get_byte_ptr(0, &SecurityContext::Machine),
            "failed to read correct value from memory"
        );

        // Read/write a range of bytes.
        let input: Vec<u8> = (0..50).collect();
        ram.set_range(0, &input, &SecurityContext::Machine);
        assert_eq!(
            &input,
            ram.get_range_ptr(0, 50, false, &SecurityContext::Machine),
            "failed to read correct values from memory"
        );
    }

    /// Test execution with region-based permissions.
    #[test]
    fn test_region_execute_permissions() {
        let ram = get_test_ram_instance();

        let tests = [
            TestEntry::new(
                TestType::Execute,
                0,
                0,
                None,
                vec![0],
                SecurityContext::User,
                false,
                "failed to get an executable byte from memory, with user context",
            ),
            TestEntry::new(
                TestType::Execute,
                0,
                0,
                None,
                vec![0],
                SecurityContext::User,
                false,
                "failed to get an executable byte from memory, with user context",
            ),
            TestEntry::new(
                TestType::Execute,
                51,
                51,
                None,
                vec![51],
                SecurityContext::User,
                true,
                "succeeded in getting an executable byte from memory, with user context",
            ),
            TestEntry::new(
                TestType::Execute,
                0,
                0,
                None,
                vec![0],
                SecurityContext::Machine,
                false,
                "failed to get an executable byte from EX memory, with machine context",
            ),
            TestEntry::new(
                TestType::Execute,
                51,
                51,
                None,
                vec![51],
                SecurityContext::Machine,
                false,
                "failed to get an executable byte from non-EX memory, with machine context",
            ),
        ];

        for (i, test) in tests.iter().enumerate() {
            let result = panic::catch_unwind(|| {
                let value = ram.get_range_clone(test.start, 1, true, &test.context);

                // Check that the read value is correct.
                assert_eq!(
                    value,
                    test.expected_values,
                    "{}",
                    test.fail_message(i, false, &value)
                );
            });

            // Check whether we panicked, and if we should have.
            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                test.should_panic,
                "{}",
                test.fail_message(i, did_panic, &[])
            );
        }
    }

    /// Test reading with region-based permissions.
    #[test]
    fn test_region_read_permissions() {
        let ram = get_test_ram_instance();

        let tests = [
            TestEntry::new(
                TestType::Read,
                0,
                50,
                None,
                (0..50).collect(),
                SecurityContext::User,
                false,
                "failed to read from R|W memory with user context",
            ),
            TestEntry::new(
                TestType::Read,
                0,
                51,
                None,
                (0..51).collect(),
                SecurityContext::User,
                true,
                "succeeded in reading from PR|PW memory with user context",
            ),
            TestEntry::new(
                TestType::Read,
                0,
                50,
                None,
                (0..50).collect(),
                SecurityContext::Machine,
                false,
                "failed to read from R|W memory with machine context",
            ),
            TestEntry::new(
                TestType::Read,
                0,
                51,
                None,
                (0..51).collect(),
                SecurityContext::Machine,
                false,
                "failed to read from PR|PW memory with machine context",
            ),
        ];

        for (i, test) in tests.iter().enumerate() {
            let result = panic::catch_unwind(|| {
                let values = ram.get_range_clone(test.start, test.end, false, &test.context);

                // Check that the read value is correct.
                assert_eq!(
                    values,
                    test.expected_values,
                    "{}",
                    test.fail_message(i, false, &values)
                );
            });

            // Check whether we panicked, and if we should have.
            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                test.should_panic,
                "{}",
                test.fail_message(i, did_panic, &[])
            );
        }
    }

    /// Test writing with region-based permissions.
    #[test]
    fn test_region_write_permissions() {
        let tests = [
            TestEntry::new(
                TestType::Write,
                0,
                50,
                Some(vec![0; 50]),
                vec![0; 50],
                SecurityContext::User,
                false,
                "failed to write to R|W memory with user context",
            ),
            TestEntry::new(
                TestType::Write,
                0,
                51,
                Some(vec![0; 51]),
                vec![0; 51],
                SecurityContext::User,
                true,
                "succeeded in writing to PR|PW memory with user context",
            ),
            TestEntry::new(
                TestType::Write,
                0,
                50,
                Some(vec![0; 50]),
                vec![0; 50],
                SecurityContext::Machine,
                false,
                "failed to write to R|W memory with machine context",
            ),
            TestEntry::new(
                TestType::Write,
                0,
                51,
                Some(vec![0; 51]),
                vec![0; 51],
                SecurityContext::Machine,
                false,
                "failed to write to PR|PW memory with machine context",
            ),
        ];

        for (i, test) in tests.iter().enumerate() {
            let result = panic::catch_unwind(|| {
                let mut ram = get_test_ram_instance();
                ram.set_range(
                    test.start,
                    &test.write_values.clone().unwrap(),
                    &test.context,
                );

                let values = ram.get_range_clone(test.start, test.end, false, &test.context);

                // Check that the read value is correct.
                assert_eq!(
                    values,
                    test.expected_values,
                    "{}",
                    test.fail_message(i, false, &values)
                );
            });

            // Check whether we panicked, and if we should have.
            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                test.should_panic,
                "{}",
                test.fail_message(i, did_panic, &[])
            );
        }
    }

    /// Test reading with region-based permissions with a more complex memory region layout
    #[test]
    fn test_region_read_correct_permissions() {
        let ram = get_test_nested_ram_instance();

        let tests = [
            TestEntry::new(
                TestType::Read,
                0,
                50,
                None,
                (0..50).collect(),
                SecurityContext::User,
                false,
                "failed to read from R|W memory with user context",
            ),
            TestEntry::new(
                TestType::Read,
                0,
                51,
                None,
                (0..51).collect(),
                SecurityContext::User,
                false,
                "succeeded in reading from R|W memory with user context",
            ),
            TestEntry::new(
                TestType::Read,
                0,
                50,
                None,
                (0..50).collect(),
                SecurityContext::Machine,
                false,
                "failed to read from R|W memory with machine context",
            ),
            TestEntry::new(
                TestType::Read,
                0,
                51,
                None,
                (0..51).collect(),
                SecurityContext::Machine,
                false,
                "failed to read from R|W memory with machine context",
            ),
        ];

        for (i, test) in tests.iter().enumerate() {
            let result = panic::catch_unwind(|| {
                let values = ram.get_range_clone(test.start, test.end, false, &test.context);

                // Check that the read value is correct.
                assert_eq!(
                    values,
                    test.expected_values,
                    "{}",
                    test.fail_message(i, false, &values)
                );
            });

            // Check whether we panicked, and if we should have.
            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                test.should_panic,
                "{}",
                test.fail_message(i, did_panic, &[])
            );
        }
    }

    /// Test writing with region-based permissions with a more complex memory region layout
    #[test]
    fn test_region_write_correct_permissions() {
        let tests = [
            TestEntry::new(
                TestType::Write,
                0,
                50,
                Some(vec![0; 50]),
                vec![0; 50],
                SecurityContext::User,
                false,
                "failed to write to R|W memory with user context",
            ),
            TestEntry::new(
                TestType::Write,
                0,
                51,
                Some(vec![0; 51]),
                vec![0; 51],
                SecurityContext::User,
                false,
                "succeeded in writing to R|W memory with user context",
            ),
            TestEntry::new(
                TestType::Write,
                0,
                50,
                Some(vec![0; 50]),
                vec![0; 50],
                SecurityContext::Machine,
                false,
                "failed to write to R|W memory with machine context",
            ),
            TestEntry::new(
                TestType::Write,
                0,
                51,
                Some(vec![0; 51]),
                vec![0; 51],
                SecurityContext::Machine,
                false,
                "failed to write to R|W memory with machine context",
            ),
        ];

        for (i, test) in tests.iter().enumerate() {
            let result = panic::catch_unwind(|| {
                let mut ram = get_test_nested_ram_instance();
                ram.set_range(
                    test.start,
                    &test.write_values.clone().unwrap(),
                    &test.context,
                );

                let values = ram.get_range_clone(test.start, test.end, false, &test.context);

                // Check that the read value is correct.
                assert_eq!(
                    values,
                    test.expected_values,
                    "{}",
                    test.fail_message(i, false, &values)
                );
            });

            let did_panic = result.is_err();
            assert_eq!(
                did_panic,
                test.should_panic,
                "{}",
                test.fail_message(i, did_panic, &[])
            );
        }
    }

    /// Test a u32 round-trip by writing to and reading from memory.
    #[test]
    fn test_read_write_u32() {
        let mut ram = get_test_ram_instance();

        let input = 0xDEADBEEF;
        ram.set_u32(0, input, &SecurityContext::User);

        assert_eq!(input, ram.get_u32(0, &SecurityContext::User));
    }
}
