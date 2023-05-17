pub mod cpu;
pub mod ram;
mod security_context;
pub mod vm;

use vm::VirtualMachine;

fn main() {
    let mut vm = VirtualMachine::new(64_000);

    vm.ram.set(0, 0x12);

    vm.ram.print_range(0, 10);

    println!("----------[Memory Regions]----------");
    println!("{:?}", vm.ram.print_memory_regions());

    //println!("{}", vm.ram.len());
}
