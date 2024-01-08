use crate::{
    compiler::bytecode_compiler::Compiler,
    cpu::CpuFlag,
    ins::instruction::Instruction,
    mem::memory_handler::{MemoryHandler, MEGABYTE},
    reg::registers::RegisterId,
};

/// The start index of the boot mapped memory region.
pub const BOOT_MEMORY_START: usize = 0xdeadbeef; // Starting at the 300 megabyte region.
/// The end index of the boot mapped memory region.
pub const BOOT_MEMORY_LENGTH: usize = MEGABYTE; // Extending for 1 megabyte.
/// The name of the region.
pub const BOOT_REGION_NAME: &str = "boot";

pub struct BootRom {}

impl BootRom {
    pub fn compile(mem: &MemoryHandler) -> Vec<u8> {
        // A set of instructions to setup the virtual machine's CPU.
        let instructions = vec![
            // Set the interrupt mask.
            Instruction::MovU32ImmU32Reg(0xffffffff, RegisterId::EIM),
            // Setup the stack.
            Instruction::MovU32ImmU32Reg(mem.stack_segment_end as u32, RegisterId::EFP),
            Instruction::MovU32ImmU32Reg(mem.stack_segment_end as u32, RegisterId::ESP),
            // Setup the segment registers.
            Instruction::MovU32ImmU32Reg(mem.stack_segment_start as u32, RegisterId::ESS),
            Instruction::MovU32ImmU32Reg(mem.code_segment_start as u32, RegisterId::ECS),
            Instruction::MovU32ImmU32Reg(mem.data_segment_start as u32, RegisterId::EDS),
            // Enable CPU interrupts.
            Instruction::MovU32ImmU32Reg(CpuFlag::compute_from(&[CpuFlag::IF]), RegisterId::EFL),
            // Jump to the start of the user executable code.
            Instruction::JumpAbsU32Reg(RegisterId::ECS),
        ];

        let mut compiler = Compiler::new();
        compiler.compile(&instructions).to_vec()
    }
}
