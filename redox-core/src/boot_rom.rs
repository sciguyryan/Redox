use hashbrown::HashMap;

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
    /// Compile the boot ROM for the virtual machine.
    ///
    /// # Arguments
    ///
    /// * `mem` - A reference to the virtual machine's [`MemoryHandler`].
    pub fn compile(mem: &MemoryHandler) -> (Vec<u8>, HashMap<u8, u32>) {
        // A set of instructions to setup the virtual machine's CPU.
        let boot_instructions = vec![
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

        // Now that we have the base instructions, we build the default IVT
        // handlers for interrupts we want to handle by default.
        let mut default_ivt_handlers = HashMap::new();

        // Calculate the position of the start of the interrupt vector.
        let instruction_start_pos = BootRom::total_size_of_instructions(&boot_instructions);

        // The first interrupt we want to handle is division by zero, which has the code 0x0.
        let interrupt_0_instructions = vec![Instruction::Hlt];
        default_ivt_handlers.insert(0, instruction_start_pos);

        let mut final_instructions = vec![];
        final_instructions.extend_from_slice(&boot_instructions);
        final_instructions.extend_from_slice(&interrupt_0_instructions);

        let mut compiler = Compiler::new();
        (
            compiler.compile(&boot_instructions).to_vec(),
            default_ivt_handlers,
        )
    }

    fn total_size_of_instructions(instructions: &[Instruction]) -> u32 {
        let mut size = 0;
        for ins in instructions {
            size += ins.get_total_instruction_size();
        }
        size as u32
    }
}
