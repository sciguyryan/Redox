use hashbrown::HashMap;

use crate::{
    compiler::bytecode_compiler::Compiler,
    cpu::Interrupt,
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
    pub fn compile(mem: &MemoryHandler) -> (Vec<u8>, HashMap<u8, usize>) {
        // A set of instructions to setup the virtual machine's CPU.
        let boot_instructions = vec![
            // Disable maskable interrupts.
            Instruction::CLI,
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
            Instruction::SLI,
            // Jump to the start of the user executable code.
            Instruction::JumpAbsU32Reg(RegisterId::ECS),
        ];

        // Now that we have the base instructions, we build the default IVT
        // handlers for interrupts we want to handle by default.
        let mut default_ivt_handlers = HashMap::new();

        // Calculate the position of the start of the interrupt vector.
        let mut next_handler_pos =
            BOOT_MEMORY_START + BootRom::total_size_of_instructions(&boot_instructions);

        // NOTE - in the instances where we are building a custom interrupt handler
        // we would need to use the intret instruction to mark the end of the handler subroutine.
        // This isn't technically needed in the ones we are using here since these all
        // result in the CPU halting, but they've been included just in case I end up
        // implementing the stepping interrupt.

        // The division by zero interrupt handler.
        let interrupt_div_zero_handler = vec![Instruction::Hlt, Instruction::IntRet];
        default_ivt_handlers.insert(Interrupt::DivideByZero.into(), next_handler_pos);

        next_handler_pos += BootRom::total_size_of_instructions(&interrupt_div_zero_handler);

        // The non-maskable interrupt (NMI) handler.
        let interrupt_nmi_handler = vec![Instruction::Hlt, Instruction::IntRet];
        default_ivt_handlers.insert(Interrupt::Nmi.into(), next_handler_pos);

        // Build and compile the final bootable ROM.
        let mut final_instructions = vec![];
        final_instructions.extend_from_slice(&boot_instructions);
        final_instructions.extend_from_slice(&interrupt_div_zero_handler);
        final_instructions.extend_from_slice(&interrupt_nmi_handler);

        let mut compiler = Compiler::new();
        (
            compiler.compile(&final_instructions).to_vec(),
            default_ivt_handlers,
        )
    }

    /// Compute the total size of an [`Instruction`] slice.
    fn total_size_of_instructions(instructions: &[Instruction]) -> usize {
        let mut size = 0;
        for ins in instructions {
            size += ins.get_total_instruction_size();
        }
        size
    }
}
