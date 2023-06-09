use core::fmt;
use prettytable::{row, Table};
use std::{collections::BTreeMap, fmt::Display};

use crate::{cpu::CpuFlag, utils};

use super::register::{RegisterF32, RegisterPermission, RegisterU32};

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
#[repr(u8)]
pub enum RegisterId {
    // Layout
    // bits  : 8 16 24 32
    // bytes : 1 2  3  4

    // Bits  : 8 16
    // Bytes : 1  2
    // Data register 1: two highest order bytes.
    //AX,
    // Bits  : 16
    // Byte  : 2
    // Data register 1: second highest order byte.
    //AH,
    // Bits  : 8
    // Byte  : 1
    // Data register 1 - highest order byte.
    //AL,

    // [ User Registers ] //
    /// Data register 1.
    R1,
    /// Data register 1, two highest order bytes.
    //R1X,
    /// Data register 1, highest order byte.
    //R1H,
    /// Data register 1, lowest order byte.
    //R1L,

    /// Data register 2.
    R2,
    /// Data register 3.
    R3,
    /// Data register 4.
    R4,
    /// Data register 5.
    R5,
    /// Data register 6.
    R6,
    /// Data register 7.
    R7,
    /// Data register 8.
    R8,

    /// Float data register 1.
    F1,

    // [System Registers] //
    /// Accumulator register.
    AC,
    /// Instruction pointer register.
    IP,
    /// Stack pointer register.
    SP,
    /// Frame pointer register.
    FP,
    /// CPU flags register.
    FL,
    /// Program counter register.
    PC,
    /// A debug testing register.
    DG,
}

impl Display for RegisterId {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let printable = match *self {
            RegisterId::R1 => "R1",
            RegisterId::R2 => "R2",
            RegisterId::R3 => "R3",
            RegisterId::R4 => "R4",
            RegisterId::R5 => "R5",
            RegisterId::R6 => "R6",
            RegisterId::R7 => "R7",
            RegisterId::R8 => "R8",
            RegisterId::F1 => "F1",
            RegisterId::AC => "AC",
            RegisterId::FL => "FL",
            RegisterId::IP => "IP",
            RegisterId::SP => "SP",
            RegisterId::FP => "FP",
            RegisterId::PC => "PC",
            RegisterId::DG => "DG",
        };
        write!(f, "{}", printable)
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

#[derive(Debug)]
pub struct Registers {
    /// A hashmap of the u32 registers.
    pub registers_u32: BTreeMap<RegisterId, RegisterU32>,
    /// A hashmap of the f32 registers.
    pub registers_f32: BTreeMap<RegisterId, RegisterF32>,
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
            registers_u32: BTreeMap::from([
                // [ User Registers ] //
                register_u32!(RegisterId::R1, &rw, 0),
                register_u32!(RegisterId::R2, &rw, 0),
                register_u32!(RegisterId::R3, &rw, 0),
                register_u32!(RegisterId::R4, &rw, 0),
                register_u32!(RegisterId::R5, &rw, 0),
                register_u32!(RegisterId::R6, &rw, 0),
                register_u32!(RegisterId::R7, &rw, 0),
                register_u32!(RegisterId::R8, &rw, 0),
                // [System Registers] //
                register_u32!(RegisterId::AC, &rw, 0),
                register_u32!(RegisterId::IP, &rw, 0),
                register_u32!(RegisterId::SP, &prpw, 0),
                register_u32!(RegisterId::FP, &prpw, 0),
                register_u32!(RegisterId::FL, &prpw, 0),
                //register!(RegisterId::T0, &prpw, 0),
                register_u32!(RegisterId::PC, &rpw, 0),
            ]),
            registers_f32: BTreeMap::from([
                // [ User Registers ] //
                register_f32!(RegisterId::F1, &rw, 0f32),
            ]),
        }
    }

    pub fn reset(&mut self) {
        for reg in &mut self.registers_u32 {
            reg.1.write_unchecked(0);
        }

        for reg in &mut self.registers_f32 {
            reg.1.write_unchecked(0f32);
        }
    }

    #[inline(always)]
    pub fn get_register_u32(&self, id: RegisterId) -> &RegisterU32 {
        self.registers_u32.get(&id).expect("failed to get register")
    }

    #[inline(always)]
    pub fn get_register_u32_mut(&mut self, id: RegisterId) -> &mut RegisterU32 {
        self.registers_u32
            .get_mut(&id)
            .expect("failed to get register")
    }

    pub fn print_differences(&self, other: &Registers) {
        println!("--- [u32 Register Differences] ---");
        for (self_reg, other_reg) in self
            .registers_u32
            .values()
            .zip(other.registers_u32.values())
        {
            println!(
                "{} - Self = {}, Other = {}",
                self_reg.id,
                self_reg.read_unchecked(),
                other_reg.read_unchecked()
            );
        }

        println!("--- [f32 Register Differences] ---");
        for (self_reg, other_reg) in self
            .registers_f32
            .values()
            .zip(other.registers_f32.values())
        {
            println!(
                "{} - Self = {}, Other = {}",
                self_reg.id,
                self_reg.read_unchecked(),
                other_reg.read_unchecked()
            );
        }
    }

    pub fn print_registers(&self) {
        let mut table = Table::new();

        table.add_row(row!["Register", "Value", "Type", "Notes"]);

        for (id, reg) in &self.registers_u32 {
            let reg_value = *reg.read_unchecked();
            let formatted_value = format!("{reg_value:0>8X}");
            let mut notes = String::new();

            if *id == RegisterId::FL {
                let states: Vec<String> = CpuFlag::iterator()
                    .map(|flag_type| {
                        let is_set = utils::is_bit_set(reg_value, (*flag_type) as u8);
                        format!("{flag_type} = {is_set}")
                    })
                    .collect();
                notes = states.join(", ");
            }

            table.add_row(row![id, formatted_value, "u32", notes]);
        }

        for (id, reg) in &self.registers_f32 {
            table.add_row(row![id, format!("{}", reg.read_unchecked()), "f32", ""]);
        }

        table.printstd();
    }
}

impl Default for Registers {
    fn default() -> Self {
        Self::new()
    }
}
