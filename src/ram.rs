pub struct RAM {
    storage: Vec<u8>,
}

impl RAM {
    pub fn new(size: usize) -> Self {
        Self {
            storage: vec![0x0; size],
        }
    }

    pub fn get(&mut self, pos: usize) -> &u8 {
        if pos >= self.len() {
            // This should fire an assertion.
            panic!();
        }

        &self.storage[pos]
    }

    pub fn get_clone(&mut self, pos: usize) -> u8 {
        if pos >= self.len() {
            // This should fire an assertion.
            panic!();
        }

        self.storage[pos]
    }

    pub fn get_range(&mut self, start: usize, len: usize) -> &[u8] {
        let end = start + len;
        if start >= self.len() || end >= self.len() {
            // This should fire an assertion.
            panic!();
        }

        &self.storage[start..end]
    }

    pub fn get_range_clone(&mut self, start: usize, len: usize) -> Vec<u8> {
        let end = start + len;
        if start >= self.len() || end >= self.len() {
            // This should fire an assertion.
            panic!();
        }

        self.storage[start..end].to_vec()
    }

    pub fn set(&mut self, pos: usize, value: u8) {
        if pos >= self.len() {
            // This should fire an assertion.
            panic!();
        }

        self.storage[pos] = value;
    }

    pub fn set_range(&mut self, pos: usize, value: u8) {
        if pos >= self.len() {
            // This should fire an assertion.
            panic!();
        }

        self.storage[pos] = value;
    }

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
            // This should fire an assertion.
            panic!();
        }

        println!("{:?}", self.storage[start..len].to_vec());
    }
}
