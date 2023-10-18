#![crate_name = "redox"]

mod cpu;
mod data_access_type;
mod ins;
mod mem;
mod reg;
mod security_context;
mod utils;
pub mod vm;

use vm::VirtualMachine;

use crate::{ins::instruction::Instruction, reg::registers::RegisterId};

// https://onlinedocs.microchip.com/pr/GUID-0E320577-28E6-4365-9BB8-9E1416A0A6E4-en-US-6/index.html?GUID-4983CB0C-7FEB-40F1-99D3-0608805404F3
// https://www.youtube.com/watch?v=KkenLT8S9Hs&list=WL&index=17

fn main() {
    let mut vm = VirtualMachine::new(64_000);

    //vm.ram.set(0, 0x12, &SecurityContext::System);
    //vm.ram.print_range(0, 10);

    vm.run_instructions(&[
        Instruction::AddU32ImmU32Reg(u32::MAX, RegisterId::R1),
        Instruction::AddU32ImmU32Reg(2, RegisterId::R1),
        Instruction::Mret,
        Instruction::Hlt,
    ]);

    println!("----------[CPU]----------");
    println!("Machine Mode? {}", vm.cpu.is_machine_mode);
    println!();

    println!("----------[Registers]----------");
    vm.cpu.registers.print_registers();
    println!();

    println!("----------[Memory Regions]----------");
    vm.ram.print_memory_regions();
    println!();

    //println!("{}", vm.ram.len());
}
