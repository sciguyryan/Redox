use crate::{
    compiler::bytecode_compiler::Compiler,
    cpu::{DIVIDE_BY_ZERO_INT, NON_MASKABLE_INT},
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

/// The default interrupt vector table address.
const DEFAULT_IVT_ADDRESS: u32 = 0x0;

pub struct BootRom {}

impl BootRom {
    /// Compile the boot ROM for the virtual machine.
    ///
    /// # Arguments
    ///
    /// * `mem` - A reference to the virtual machine's [`MemoryHandler`].
    pub fn compile(mem: &MemoryHandler) -> Vec<u8> {
        // A set of instructions to setup the virtual machine's CPU.
        let boot_instructions = vec![
            // Completely disable maskable interrupts.
            Instruction::ClearInterruptFlag,
            Instruction::MovU32ImmU32Reg(0, RegisterId::EIM),
            // Setup the stack.
            Instruction::MovU32ImmU32Reg(mem.stack_segment_end as u32, RegisterId::EFP),
            Instruction::MovU32ImmU32Reg(mem.stack_segment_end as u32, RegisterId::ESP),
            // Setup the segment registers.
            Instruction::MovU32ImmU32Reg(mem.stack_segment_start as u32, RegisterId::ESS),
            Instruction::MovU32ImmU32Reg(mem.code_segment_start as u32, RegisterId::ECS),
            Instruction::MovU32ImmU32Reg(mem.data_segment_start as u32, RegisterId::EDS),
            // Call the setup code for the interrupt vector table.
            // This is a placeholder and it will be replaced further in this method.
            Instruction::PushU32Imm(0),
            Instruction::CallU32Imm(0xffffffff),
            // Load the interrupt descriptor table register with the IVT location in memory.
            Instruction::LoadIVTAddrU32Imm(DEFAULT_IVT_ADDRESS),
            // Enable CPU interrupts.
            Instruction::SetInterruptFlag,
            Instruction::MovU32ImmU32Reg(0xffffffff, RegisterId::EIM),
            // Jump to the start of the user executable code.
            Instruction::JumpAbsU32Reg(RegisterId::ECS),
            // We shouldn't be able to hit this, but just to be safe.
            Instruction::Halt,
        ];

        // Calculate the position of the start of the interrupt vector.
        let mut next_handler_pos =
            BOOT_MEMORY_START + BootRom::total_size_of_instructions(&boot_instructions);

        // NOTE - in the instances where we are building a custom interrupt handler
        // we would need to use the intret instruction to mark the end of the handler subroutine.
        // This isn't technically needed in the ones we are using here since these all
        // result in the CPU halting, but they've been included just in case I end up
        // implementing the stepping interrupt.
        let default_handlers: Vec<(u8, Vec<Instruction>)> = vec![
            // The division by zero interrupt handler.
            (
                DIVIDE_BY_ZERO_INT,
                vec![Instruction::Halt, Instruction::IntRet],
            ),
            // The non-maskable interrupt (NMI) handler.
            (
                NON_MASKABLE_INT,
                vec![Instruction::Halt, Instruction::IntRet],
            ),
        ];

        // This will hold the final boot ROM instruction list.
        let mut final_instructions = boot_instructions.clone();

        // This will hold the IVT initialization subroutine.
        let mut init_ivt_subroutine = vec![];

        // Add the interrupt handlers to the final boot ROM.
        for (int_code, instructions) in default_handlers {
            // Add the initialization entry for this interrupt code.
            init_ivt_subroutine.push(Instruction::MovU32ImmMemSimple(
                next_handler_pos as u32,
                (int_code * 4) as u32,
            ));

            // Add this handler into the boot ROM instruction list.
            final_instructions.extend_from_slice(&instructions);

            // Advance the next handler location.
            next_handler_pos += BootRom::total_size_of_instructions(&instructions);
        }

        // Add the return instruction from our IVT initialization subroutine and add
        // the subroutine to the boot ROM.
        init_ivt_subroutine.push(Instruction::RetArgsU32);
        final_instructions.extend_from_slice(&init_ivt_subroutine);

        // Now that we know the location of the IVT initialization routine we can
        // update that in our earlier call code. It will directly follow the last IVT handler.
        let index = final_instructions
            .iter()
            .position(|e| *e == Instruction::CallU32Imm(0xffffffff))
            .expect("failed to find correct instruction");

        // Replace the dummy call instruction with one that has the correct
        // subroutine address.
        final_instructions[index] = Instruction::CallU32Imm(next_handler_pos as u32);

        // Return the compiled bytecode.
        Compiler::new().compile(&final_instructions).to_vec()
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
