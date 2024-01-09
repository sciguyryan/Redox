use core::fmt;
use hashbrown::HashMap;
use num_derive::FromPrimitive;
use prettytable::{row, Table};
use std::fmt::Display;

use crate::boot_rom::BOOT_MEMORY_START;

use super::register::{RegisterF32, RegisterPermission, RegisterU32};

#[derive(Clone, Copy, Debug, Default, Eq, FromPrimitive, Hash, Ord, PartialEq, PartialOrd)]
#[repr(u8)]
pub enum RegisterId {
    // Layout
    // bits  : 8     16     24     32
    // bytes : 1     2      3      4
    //               [      R1X     ]
    //               [  R1L ][  R1H ]

    // [ User Registers ] //
    /// Data register 1.
    #[default]
    ER1,
    /// Data register 1, two highest order bytes.
    //R1X,
    /// Data register 1, highest order byte.
    //R1H,
    /// Data register 1, lowest order byte.
    //R1L,

    /// Data register 2.
    ER2,
    /// Data register 3.
    ER3,
    /// Data register 4.
    ER4,
    /// Data register 5.
    ER5,
    /// Data register 6.
    ER6,
    /// Data register 7.
    ER7,
    /// Data register 8.
    ER8,

    /// Float data register 1.
    FR1,
    /// Float data register 2.
    FR2,

    // [ System Registers ] //
    /// Accumulator register.
    EAC,
    /// Instruction pointer register.
    EIP,
    /// Stack frame pointer.
    EFP,
    /// Stack pointer register.
    ESP,
    /// CPU flags register.
    EFL,
    /// Program counter register.
    EPC,
    /// Interrupt mask register.
    EIM,

    // [ Segment Registers ] //
    /// Stack segment register.
    ESS,
    /// Code segment register.
    ECS,
    /// Data segment register.
    EDS,
}

impl Display for RegisterId {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let printable = match *self {
            RegisterId::ER1 => "ER1",
            RegisterId::ER2 => "ER2",
            RegisterId::ER3 => "ER3",
            RegisterId::ER4 => "ER4",
            RegisterId::ER5 => "ER5",
            RegisterId::ER6 => "ER6",
            RegisterId::ER7 => "ER7",
            RegisterId::ER8 => "ER8",
            RegisterId::FR1 => "FR1",
            RegisterId::FR2 => "FR2",
            RegisterId::EAC => "EAC",

            RegisterId::EIP => "EIP",
            RegisterId::EFP => "EFP",
            RegisterId::ESP => "ESP",

            RegisterId::EFL => "EFL",
            RegisterId::EIM => "EIM",
            RegisterId::EPC => "EPC",

            RegisterId::ESS => "ESS",
            RegisterId::ECS => "ECS",
            RegisterId::EDS => "EDS",
        };
        write!(f, "{printable}")
    }
}

impl From<RegisterId> for u8 {
    fn from(m: RegisterId) -> u8 {
        m as u8
    }
}

macro_rules! register_u32 {
    ($b:expr, $c:expr, $d:expr) => {{
        use super::register::RegisterU32;
        ($b, RegisterU32::new($b, $c.clone(), $d))
    }};
}

macro_rules! register_f32 {
    ($b:expr, $c:expr, $d:expr) => {{
        use super::register::RegisterF32;
        ($b, RegisterF32::new($b, $c.clone(), $d))
    }};
}

#[derive(Debug, Clone)]
pub struct Registers {
    /// A hashmap of the u32 registers.
    pub registers_u32: HashMap<RegisterId, RegisterU32>,
    /// A hashmap of the f32 registers.
    pub registers_f32: HashMap<RegisterId, RegisterF32>,
}

impl PartialEq for Registers {
    fn eq(&self, other: &Self) -> bool {
        for (self_reg, other_reg) in self
            .registers_u32
            .values()
            .zip(other.registers_u32.values())
        {
            if self_reg.read_unchecked() != other_reg.read_unchecked() {
                return false;
            }
        }
        for (self_reg, other_reg) in self
            .registers_f32
            .values()
            .zip(other.registers_f32.values())
        {
            if self_reg.read_unchecked() != other_reg.read_unchecked() {
                return false;
            }
        }
        true
    }
}

impl Eq for Registers {}

impl Registers {
    pub fn new() -> Self {
        let rw = RegisterPermission::R | RegisterPermission::W;
        let rpw = RegisterPermission::R | RegisterPermission::PW;
        let prpw = RegisterPermission::PR | RegisterPermission::PW;

        Self {
            registers_u32: HashMap::from([
                // [ User Registers ] //
                register_u32!(RegisterId::ER1, &rw, 0),
                register_u32!(RegisterId::ER2, &rw, 0),
                register_u32!(RegisterId::ER3, &rw, 0),
                register_u32!(RegisterId::ER4, &rw, 0),
                register_u32!(RegisterId::ER5, &rw, 0),
                register_u32!(RegisterId::ER6, &rw, 0),
                register_u32!(RegisterId::ER7, &rw, 0),
                register_u32!(RegisterId::ER8, &rw, 0),
                // [ System Registers ] //
                register_u32!(RegisterId::EAC, &rw, 0),
                register_u32!(RegisterId::EIP, &rw, BOOT_MEMORY_START as u32),
                register_u32!(RegisterId::EFP, &prpw, 0),
                register_u32!(RegisterId::ESP, &prpw, 0),
                register_u32!(RegisterId::EFL, &rpw, 0),
                register_u32!(RegisterId::EPC, &rpw, 0),
                register_u32!(RegisterId::EIM, &rpw, 0),
                // [ Segment Registers ] //
                register_u32!(RegisterId::ESS, &rpw, 0),
                register_u32!(RegisterId::ECS, &rpw, 0),
                register_u32!(RegisterId::EDS, &rpw, 0),
            ]),
            registers_f32: HashMap::from([
                // [ User Registers ] //
                register_f32!(RegisterId::FR1, &rw, 0f32),
                register_f32!(RegisterId::FR2, &rw, 0f32),
            ]),
        }
    }

    /// Get a reference to a specific f32 register.
    ///
    /// # Arguments
    ///
    /// * `id` - The [`RegisterId`] for the register in question.
    ///
    /// # Returns
    ///
    /// A reference to the specific [`RegisterF32`] instance.
    #[inline(always)]
    pub fn get_register_f32(&self, id: RegisterId) -> &RegisterF32 {
        self.registers_f32.get(&id).expect("failed to get register")
    }

    /// Get a reference to a specific u32 register.
    ///
    /// # Arguments
    ///
    /// * `id` - The [`RegisterId`] for the register in question.
    ///
    /// # Returns
    ///
    /// A reference to the specific [`RegisterU32`] instance.
    #[inline(always)]
    pub fn get_register_u32(&self, id: RegisterId) -> &RegisterU32 {
        self.registers_u32.get(&id).expect("failed to get register")
    }

    /// Get a mutable reference to a specific f32 register.
    ///
    /// # Arguments
    ///
    /// * `id` - The [`RegisterId`] for the register in question.
    ///
    /// # Returns
    ///
    /// A reference to the specific [`RegisterF32`] instance.
    #[inline(always)]
    pub fn get_register_f32_mut(&mut self, id: RegisterId) -> &mut RegisterF32 {
        self.registers_f32
            .get_mut(&id)
            .expect("failed to get register")
    }

    /// Get a mutable reference to a specific u32 register.
    ///
    /// # Arguments
    ///
    /// * `id` - The [`RegisterId`] for the register in question.
    ///
    /// # Returns
    ///
    /// A reference to the specific [`RegisterU32`] instance.
    #[inline(always)]
    pub fn get_register_u32_mut(&mut self, id: RegisterId) -> &mut RegisterU32 {
        self.registers_u32
            .get_mut(&id)
            .expect("failed to get register")
    }

    /// Print the differences between two [`Registers`] instances.
    ///
    /// # Arguments
    ///
    /// * `other` - A reference to the other [`Registers`] instance against which this instance should be compared.
    /// * `names` - The names to be displayed in the output.
    #[cfg(test)]
    pub fn print_differences(&self, other: &Registers, names: &[&str; 2]) {
        let mut u32_different = vec![];
        let mut f32_different = vec![];

        for (self_reg, other_reg) in self
            .registers_u32
            .values()
            .zip(other.registers_u32.values())
        {
            if self_reg.read_unchecked() != other_reg.read_unchecked() {
                let mut self_val = self_reg.read_unchecked().to_string();
                let mut other_val = other_reg.read_unchecked().to_string();

                // We want to show the flags (rather than the value) for simplicity.
                if self_reg.id == RegisterId::EFL {
                    self_val = format!("[{}]", self_reg.get_flags_register_string());
                    other_val = format!("[{}]", other_reg.get_flags_register_string());
                }

                u32_different.push(format!(
                    "{} - {} = {self_val}, {} = {other_val}",
                    self_reg.id, names[0], names[1],
                ));
            }
        }

        for (self_reg, other_reg) in self
            .registers_f32
            .values()
            .zip(other.registers_f32.values())
        {
            if self_reg.read_unchecked() != other_reg.read_unchecked() {
                f32_different.push(format!(
                    "{} - {} = {}, {} = {}",
                    self_reg.id,
                    names[0],
                    self_reg.read_unchecked(),
                    names[1],
                    other_reg.read_unchecked()
                ));
            }
        }

        if !u32_different.is_empty() {
            println!();
            println!("---- [u32 Register Differences] ----");
            for s in u32_different {
                println!("{s}");
            }
            println!();
        }

        if !f32_different.is_empty() {
            println!();
            println!("---- [f32 Register Differences] ----");
            for s in f32_different {
                println!("{s}");
            }
            println!();
        }
    }

    /// Print the value of each register in the collection.
    pub fn print_registers(&self) {
        let mut table = Table::new();

        table.add_row(row!["Register", "Value", "Type", "Notes"]);

        let mut u32_reg_values: Vec<RegisterU32> = self.registers_u32.values().cloned().collect();
        u32_reg_values.sort();

        for reg in &u32_reg_values {
            let reg_value = *reg.read_unchecked();
            let formatted_value = format!("{reg_value:0>8X}");
            let mut notes = String::new();

            if reg.id == RegisterId::EFL {
                notes = reg.get_flags_register_string();
            }

            table.add_row(row![reg.id, formatted_value, "u32", notes]);
        }

        let mut f32_reg_values: Vec<RegisterF32> = self.registers_f32.values().cloned().collect();
        f32_reg_values.sort();

        for reg in &f32_reg_values {
            table.add_row(row![reg.id, format!("{}", reg.read_unchecked()), "f32", ""]);
        }

        table.printstd();
    }
}

impl Default for Registers {
    fn default() -> Self {
        Self::new()
    }
}
