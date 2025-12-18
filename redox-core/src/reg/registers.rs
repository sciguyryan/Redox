use core::fmt;
use prettytable::{Table, row};
use std::{fmt::Display, str::FromStr};
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
    /// Accumulator register.
    #[default]
    EAX = 0,
    /// Data register 1, two highest order bytes.
    //AX = 1,
    /// Data register 1, highest order byte.
    //AH = 2,
    /// Data register 1, lowest order byte.
    //AL = 3,

    /// Base register.
    EBX = 4,
    /// Counter register.
    ECX = 8,
    /// Data register.
    EDX = 12,
    /// Data register 5.
    E5X = 16,
    /// Data register 6.
    E6X = 20,
    /// Data register 7.
    E7X = 24,
    /// Data register 8.
    E8X = 28,

    /// Float data register 1.
    FR1 = 29,
    /// Float data register 2.
    FR2 = 30,

    // [ System Registers ] //
    /// Instruction pointer register.
    EIP = 100,
    /// Stack base pointer - the pointer to the base of the current stack frame.
    EBP = 101,
    /// Stack pointer register.
    ESP = 102,
    /// CPU flags register.
    EFL = 103,
    /// Interrupt mask register. Only the lowest order byte is used.
    EIM = 104,
    /// Interrupt descriptor table register.
    IDTR = 105,

    // [ Segment Registers ] //
    /// Stack segment register.
    ESS = 200,
    /// Code segment register.
    ECS = 201,
    /// Data segment register.
    EDS = 202,
}

impl FromStr for RegisterId {
    type Err = ();

    fn from_str(input: &str) -> Result<RegisterId, Self::Err> {
        match input.to_uppercase().as_str() {
            "EAX" => Ok(RegisterId::EAX),
            "EBX" => Ok(RegisterId::EBX),
            "ECX" => Ok(RegisterId::ECX),
            "EDX" => Ok(RegisterId::EDX),
            "E5X" => Ok(RegisterId::E5X),
            "E6X" => Ok(RegisterId::E6X),
            "E7X" => Ok(RegisterId::E7X),
            "E8X" => Ok(RegisterId::E8X),
            "FR1" => Ok(RegisterId::FR1),
            "FR2" => Ok(RegisterId::FR2),
            "EIP" => Ok(RegisterId::EIP),
            "EBP" => Ok(RegisterId::EBP),
            "ESP" => Ok(RegisterId::ESP),
            "EFL" => Ok(RegisterId::EFL),
            "EIM" => Ok(RegisterId::EIM),
            "IDTR" => Ok(RegisterId::IDTR),
            "ESS" => Ok(RegisterId::ESS),
            "ECS" => Ok(RegisterId::ECS),
            "EDS" => Ok(RegisterId::EDS),

            _ => Err(()),
        }
    }
}

impl From<u8> for RegisterId {
    fn from(value: u8) -> Self {
        match value {
            0 => RegisterId::EAX,
            4 => RegisterId::EBX,
            8 => RegisterId::ECX,
            12 => RegisterId::EDX,
            16 => RegisterId::E5X,
            20 => RegisterId::E6X,
            24 => RegisterId::E7X,
            28 => RegisterId::E8X,
            29 => RegisterId::FR1,
            30 => RegisterId::FR2,
            100 => RegisterId::EIP,
            101 => RegisterId::EBP,
            102 => RegisterId::ESP,
            103 => RegisterId::EFL,
            104 => RegisterId::EIM,
            105 => RegisterId::IDTR,
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
            RegisterId::EAX => "EAX",
            RegisterId::EBX => "EBX",
            RegisterId::ECX => "ECX",
            RegisterId::EDX => "EDX",
            RegisterId::E5X => "E5X",
            RegisterId::E6X => "E6X",
            RegisterId::E7X => "E7X",
            RegisterId::E8X => "E8X",
            RegisterId::FR1 => "FR1",
            RegisterId::FR2 => "FR2",
            RegisterId::EIP => "EIP",
            RegisterId::EBP => "EFP",
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
    eip: RegisterU32,
    ebp: RegisterU32,
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
            er1: RegisterU32::new(RegisterId::EAX, rw, 0),
            er2: RegisterU32::new(RegisterId::EBX, rw, 0),
            er3: RegisterU32::new(RegisterId::ECX, rw, 0),
            er4: RegisterU32::new(RegisterId::EDX, rw, 0),
            er5: RegisterU32::new(RegisterId::E5X, rw, 0),
            er6: RegisterU32::new(RegisterId::E6X, rw, 0),
            er7: RegisterU32::new(RegisterId::E7X, rw, 0),
            er8: RegisterU32::new(RegisterId::E8X, rw, 0),

            // [ System Registers ] //
            eip: RegisterU32::new(RegisterId::EIP, rw, BOOT_MEMORY_START as u32),
            ebp: RegisterU32::new(RegisterId::EBP, rpw, 0),
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
                RegisterId::EAX => u32_vec.push(&self.er1),
                RegisterId::EBX => u32_vec.push(&self.er2),
                RegisterId::ECX => u32_vec.push(&self.er3),
                RegisterId::EDX => u32_vec.push(&self.er4),
                RegisterId::E5X => u32_vec.push(&self.er5),
                RegisterId::E6X => u32_vec.push(&self.er6),
                RegisterId::E7X => u32_vec.push(&self.er7),
                RegisterId::E8X => u32_vec.push(&self.er8),
                RegisterId::EIP => u32_vec.push(&self.eip),
                RegisterId::EBP => u32_vec.push(&self.ebp),
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
            RegisterId::EAX
            | RegisterId::EBX
            | RegisterId::ECX
            | RegisterId::EDX
            | RegisterId::E5X
            | RegisterId::E6X
            | RegisterId::E7X
            | RegisterId::E8X
            | RegisterId::EIP
            | RegisterId::EBP
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
            RegisterId::EAX => &self.er1,
            RegisterId::EBX => &self.er2,
            RegisterId::ECX => &self.er3,
            RegisterId::EDX => &self.er4,
            RegisterId::E5X => &self.er5,
            RegisterId::E6X => &self.er6,
            RegisterId::E7X => &self.er7,
            RegisterId::E8X => &self.er8,
            RegisterId::EIP => &self.eip,
            RegisterId::EBP => &self.ebp,
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
            RegisterId::EAX
            | RegisterId::EBX
            | RegisterId::ECX
            | RegisterId::EDX
            | RegisterId::E5X
            | RegisterId::E6X
            | RegisterId::E7X
            | RegisterId::E8X
            | RegisterId::EIP
            | RegisterId::EBP
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
            RegisterId::EAX => &mut self.er1,
            RegisterId::EBX => &mut self.er2,
            RegisterId::ECX => &mut self.er3,
            RegisterId::EDX => &mut self.er4,
            RegisterId::E5X => &mut self.er5,
            RegisterId::E6X => &mut self.er6,
            RegisterId::E7X => &mut self.er7,
            RegisterId::E8X => &mut self.er8,
            RegisterId::EIP => &mut self.eip,
            RegisterId::EBP => &mut self.ebp,
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
            let reg_value = reg.read_unchecked();
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
        self.get_register_f32(*reg).read(privilege)
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
        self.get_register_f32(*reg).read_unchecked()
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
        self.get_register_u32(*reg).read(privilege)
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
        self.get_register_u32(*reg).read_unchecked()
    }
}

impl Default for Registers {
    fn default() -> Self {
        Self::new()
    }
}
