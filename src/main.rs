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

fn main() {
    let mut vm = VirtualMachine::new(64_000);

    //vm.ram.set(0, 0x12, &SecurityContext::System);
    //vm.ram.print_range(0, 10);

    vm.run_instructions(&[
        Instruction::AddU32LitReg(u32::MAX, RegisterId::R1),
        Instruction::AddU32LitReg(2, RegisterId::R1),
        Instruction::Hlt,
    ]);

    println!("----------[Registers]----------");
    vm.cpu.registers.print_registers();
    println!();

    println!("----------[Memory Regions]----------");
    vm.ram.print_memory_regions();
    println!();

    //println!("{}", vm.ram.len());
}
