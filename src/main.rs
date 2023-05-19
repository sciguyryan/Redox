mod cpu;
mod data_access_type;
mod ram;
mod security_context;
pub mod vm;

use vm::VirtualMachine;

use crate::security_context::SecurityContext;

fn main() {
    let mut vm = VirtualMachine::new(64_000);

    vm.ram.set(0, 0x12, SecurityContext::System);

    vm.ram.print_range(0, 10);

    println!("----------[Memory Regions]----------");
    println!("{:?}", vm.ram.print_memory_regions());

    //println!("{}", vm.ram.len());
}
