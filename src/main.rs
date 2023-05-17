pub mod cpu;
pub mod ram;
pub mod vm;

use vm::VirtualMachine;

fn main() {
    let mut vm = VirtualMachine::new(64_000);

    vm.ram.set(0, 0x12);

    vm.ram.print_range(0, 10);

    //println!("{}", vm.ram.len());
}
