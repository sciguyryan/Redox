use core::fmt;
use prettytable::{row, Table};
use std::fmt::Display;
use strum::IntoEnumIterator;
use strum_macros::EnumIter;

use crate::{boot_rom::BOOT_MEMORY_START, privilege_level::PrivilegeLevel};

use super::register::{RegisterF32, RegisterPermission, RegisterU32};

#[derive(Clone, Copy, Debug, Default, Eq, Hash, Ord, PartialEq, PartialOrd, EnumIter)]
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
    ER1 = 0,
    /// Data register 1, two highest order bytes.
    //R1X = 1,
    /// Data register 1, highest order byte.
    //R1H = 2,
    /// Data register 1, lowest order byte.
    //R1L = 3,

    /// Data register 2.
    ER2 = 4,
    /// Data register 3.
    ER3 = 8,
    /// Data register 4.
    ER4 = 12,
    /// Data register 5.
    ER5 = 16,
    /// Data register 6.
    ER6 = 20,
    /// Data register 7.
    ER7 = 24,
    /// Data register 8.
    ER8 = 28,

    /// Float data register 1.
    FR1 = 29,
    /// Float data register 2.
    FR2 = 30,

    // [ System Registers ] //
    /// Accumulator register.
    EAC = 100,
    /// Instruction pointer register.
    EIP = 101,
    /// Stack frame pointer.
    EFP = 102,
    /// Stack pointer register.
    ESP = 103,
    /// CPU flags register.
    EFL = 104,
    /// Interrupt mask register. Only the lowest order byte is used.
    EIM = 105,
    /// Interrupt descriptor table register.
    IDTR = 106,

    // [ Segment Registers ] //
    /// Stack segment register.
    ESS = 200,
    /// Code segment register.
    ECS = 201,
    /// Data segment register.
    EDS = 202,
}

impl From<u8> for RegisterId {
    fn from(value: u8) -> Self {
        match value {
            0 => RegisterId::ER1,
            4 => RegisterId::ER2,
            8 => RegisterId::ER3,
            12 => RegisterId::ER4,
            16 => RegisterId::ER5,
            20 => RegisterId::ER6,
            24 => RegisterId::ER7,
            28 => RegisterId::ER8,
            29 => RegisterId::FR1,
            30 => RegisterId::FR2,
            100 => RegisterId::EAC,
            101 => RegisterId::EIP,
            102 => RegisterId::EFP,
            103 => RegisterId::ESP,
            104 => RegisterId::EFL,
            105 => RegisterId::EIM,
            106 => RegisterId::IDTR,
            200 => RegisterId::ESS,
            201 => RegisterId::ECS,
            202 => RegisterId::EDS,
            id => panic!("invalid register id {id}"),
        }
    }
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
            RegisterId::IDTR => "IDTR",

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
#[derive(Debug, Clone)]
pub struct Registers {
    /****** [u32 Registers] ******/
    // [ User Registers ] //
    er1: RegisterU32,
    er2: RegisterU32,
    er3: RegisterU32,
    er4: RegisterU32,
    er5: RegisterU32,
    er6: RegisterU32,
    er7: RegisterU32,
    er8: RegisterU32,

    // [ System Registers ] //
    eac: RegisterU32,
    eip: RegisterU32,
    efp: RegisterU32,
    esp: RegisterU32,
    efl: RegisterU32,
    eim: RegisterU32,
    idtr: RegisterU32,

    // [ Segment Registers ] //
    ess: RegisterU32,
    ecs: RegisterU32,
    eds: RegisterU32,

    /****** [f32 Registers] ******/
    // [ User Registers ] //
    fr1: RegisterF32,
    fr2: RegisterF32,
}

impl Registers {
    pub fn new() -> Self {
        let rw = RegisterPermission::R | RegisterPermission::W;
        let rpw = RegisterPermission::R | RegisterPermission::PW;

        Self {
            /****** [u32 Registers] ******/
            // [ User Registers ] //
            er1: RegisterU32::new(RegisterId::ER1, rw, 0),
            er2: RegisterU32::new(RegisterId::ER2, rw, 0),
            er3: RegisterU32::new(RegisterId::ER3, rw, 0),
            er4: RegisterU32::new(RegisterId::ER4, rw, 0),
            er5: RegisterU32::new(RegisterId::ER5, rw, 0),
            er6: RegisterU32::new(RegisterId::ER6, rw, 0),
            er7: RegisterU32::new(RegisterId::ER7, rw, 0),
            er8: RegisterU32::new(RegisterId::ER8, rw, 0),

            // [ System Registers ] //
            eac: RegisterU32::new(RegisterId::EAC, rw, 0),
            eip: RegisterU32::new(RegisterId::EIP, rw, BOOT_MEMORY_START as u32),
            efp: RegisterU32::new(RegisterId::EFP, rpw, 0),
            esp: RegisterU32::new(RegisterId::ESP, rpw, 0),
            efl: RegisterU32::new(RegisterId::EFL, rpw, 0),
            eim: RegisterU32::new(RegisterId::EIM, rw, 0),
            idtr: RegisterU32::new(RegisterId::IDTR, rpw, 0),

            // [ Segment Registers ] //
            ess: RegisterU32::new(RegisterId::ESS, rpw, 0),
            ecs: RegisterU32::new(RegisterId::ECS, rpw, 0),
            eds: RegisterU32::new(RegisterId::EDS, rpw, 0),

            /****** [f32 Registers] ******/
            // [ User Registers ] //
            fr1: RegisterF32::new(RegisterId::FR1, rpw, 0.0),
            fr2: RegisterF32::new(RegisterId::FR2, rpw, 0.0),
        }
    }

    /// Get a reference to every register in this instance.
    ///
    /// # Returns
    ///
    /// A tuple of vector of u32 register references (first) and a vector of f32 register references (second).
    pub fn get_registers_as_vectors(&self) -> (Vec<&RegisterU32>, Vec<&RegisterF32>) {
        let mut u32_vec = vec![];
        let mut f32_vec = vec![];

        for id in RegisterId::iter() {
            match id {
                RegisterId::ER1 => u32_vec.push(&self.er1),
                RegisterId::ER2 => u32_vec.push(&self.er2),
                RegisterId::ER3 => u32_vec.push(&self.er3),
                RegisterId::ER4 => u32_vec.push(&self.er4),
                RegisterId::ER5 => u32_vec.push(&self.er5),
                RegisterId::ER6 => u32_vec.push(&self.er6),
                RegisterId::ER7 => u32_vec.push(&self.er7),
                RegisterId::ER8 => u32_vec.push(&self.er8),
                RegisterId::EAC => u32_vec.push(&self.eac),
                RegisterId::EIP => u32_vec.push(&self.eip),
                RegisterId::EFP => u32_vec.push(&self.efp),
                RegisterId::ESP => u32_vec.push(&self.esp),
                RegisterId::EFL => u32_vec.push(&self.efl),
                RegisterId::EIM => u32_vec.push(&self.eim),
                RegisterId::IDTR => u32_vec.push(&self.idtr),
                RegisterId::ESS => u32_vec.push(&self.ess),
                RegisterId::ECS => u32_vec.push(&self.ecs),
                RegisterId::EDS => u32_vec.push(&self.eds),
                RegisterId::FR1 => f32_vec.push(&self.fr1),
                RegisterId::FR2 => f32_vec.push(&self.fr2),
            }
        }

        (u32_vec, f32_vec)
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
        match id {
            RegisterId::FR1 => &self.fr1,
            RegisterId::FR2 => &self.fr2,
            RegisterId::ER1
            | RegisterId::ER2
            | RegisterId::ER3
            | RegisterId::ER4
            | RegisterId::ER5
            | RegisterId::ER6
            | RegisterId::ER7
            | RegisterId::ER8
            | RegisterId::EAC
            | RegisterId::EIP
            | RegisterId::EFP
            | RegisterId::ESP
            | RegisterId::EFL
            | RegisterId::EIM
            | RegisterId::IDTR
            | RegisterId::ESS
            | RegisterId::ECS
            | RegisterId::EDS => panic!("invalid f32 register ID"),
        }
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
        match id {
            RegisterId::ER1 => &self.er1,
            RegisterId::ER2 => &self.er2,
            RegisterId::ER3 => &self.er3,
            RegisterId::ER4 => &self.er4,
            RegisterId::ER5 => &self.er5,
            RegisterId::ER6 => &self.er6,
            RegisterId::ER7 => &self.er7,
            RegisterId::ER8 => &self.er8,
            RegisterId::EAC => &self.eac,
            RegisterId::EIP => &self.eip,
            RegisterId::EFP => &self.efp,
            RegisterId::ESP => &self.esp,
            RegisterId::EFL => &self.efl,
            RegisterId::EIM => &self.eim,
            RegisterId::IDTR => &self.idtr,
            RegisterId::ESS => &self.ess,
            RegisterId::ECS => &self.ecs,
            RegisterId::EDS => &self.eds,
            RegisterId::FR1 | RegisterId::FR2 => panic!("invalid u32 register ID"),
        }
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
        match id {
            RegisterId::FR1 => &mut self.fr1,
            RegisterId::FR2 => &mut self.fr2,
            RegisterId::ER1
            | RegisterId::ER2
            | RegisterId::ER3
            | RegisterId::ER4
            | RegisterId::ER5
            | RegisterId::ER6
            | RegisterId::ER7
            | RegisterId::ER8
            | RegisterId::EAC
            | RegisterId::EIP
            | RegisterId::EFP
            | RegisterId::ESP
            | RegisterId::EFL
            | RegisterId::EIM
            | RegisterId::IDTR
            | RegisterId::ESS
            | RegisterId::ECS
            | RegisterId::EDS => panic!("invalid f32 register ID"),
        }
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
        match id {
            RegisterId::ER1 => &mut self.er1,
            RegisterId::ER2 => &mut self.er2,
            RegisterId::ER3 => &mut self.er3,
            RegisterId::ER4 => &mut self.er4,
            RegisterId::ER5 => &mut self.er5,
            RegisterId::ER6 => &mut self.er6,
            RegisterId::ER7 => &mut self.er7,
            RegisterId::ER8 => &mut self.er8,
            RegisterId::EAC => &mut self.eac,
            RegisterId::EIP => &mut self.eip,
            RegisterId::EFP => &mut self.efp,
            RegisterId::ESP => &mut self.esp,
            RegisterId::EFL => &mut self.efl,
            RegisterId::EIM => &mut self.eim,
            RegisterId::IDTR => &mut self.idtr,
            RegisterId::ESS => &mut self.ess,
            RegisterId::ECS => &mut self.ecs,
            RegisterId::EDS => &mut self.eds,
            RegisterId::FR1 | RegisterId::FR2 => panic!("invalid u32 register ID"),
        }
    }

    /// Print the value of each register in the collection.
    pub fn print_registers(&self) {
        let mut table = Table::new();
        table.add_row(row!["Register", "Value", "Type", "Notes"]);

        let (mut u32_registers, mut f32_registers) = self.get_registers_as_vectors();
        u32_registers.sort();
        f32_registers.sort();

        for reg in &u32_registers {
            let reg_value = *reg.read_unchecked();
            let formatted_value = format!("{reg_value:0>8X}");
            let mut notes = String::new();

            if reg.id == RegisterId::EFL {
                notes = reg.get_flags_register_string();
            }

            table.add_row(row![reg.id, formatted_value, "u32", notes]);
        }

        for reg in &f32_registers {
            table.add_row(row![reg.id, format!("{}", reg.read_unchecked()), "f32", ""]);
        }

        table.printstd();
    }

    /// Get the value of a specific f32 register.
    ///
    /// # Arguments
    ///
    /// * `reg` - Reference to the [`RegisterId`] for the register in question.
    /// * `privilege` - The [`PrivilegeLevel`] with which the read request should be processed.
    ///
    /// # Returns
    ///
    /// The f32 value of the register.
    #[inline(always)]
    #[allow(dead_code)]
    pub fn read_reg_f32(&self, reg: &RegisterId, privilege: &PrivilegeLevel) -> f32 {
        *self.get_register_f32(*reg).read(privilege)
    }

    /// Get the value of a specific f32 register, without privilege checking.
    ///
    /// # Arguments
    ///
    /// * `reg` - Reference to the [`RegisterId`] for the register in question.
    ///
    /// # Returns
    ///
    /// The f32 value of the register.
    #[inline(always)]
    pub fn read_reg_f32_unchecked(&self, reg: &RegisterId) -> f32 {
        *self.get_register_f32(*reg).read_unchecked()
    }

    /// Get the value of a specific u32 register.
    ///
    /// # Arguments
    ///
    /// * `reg` - Reference to the [`RegisterId`] for the register in question.
    /// * `privilege` - The [`PrivilegeLevel`] with which the read request should be processed.
    ///
    /// # Returns
    ///
    /// The u32 value of the register.
    #[inline(always)]
    pub fn read_reg_u32(&self, reg: &RegisterId, privilege: &PrivilegeLevel) -> u32 {
        *self.get_register_u32(*reg).read(privilege)
    }

    /// Get the value of a specific u32 register, without privilege checking.
    ///
    /// # Arguments
    ///
    /// * `reg` - Reference to the [`RegisterId`] for the register in question.
    ///
    /// # Returns
    ///
    /// The u32 value of the register.
    #[inline(always)]
    pub fn read_reg_u32_unchecked(&self, reg: &RegisterId) -> u32 {
        *self.get_register_u32(*reg).read_unchecked()
    }
}

impl Default for Registers {
    fn default() -> Self {
        Self::new()
    }
}
