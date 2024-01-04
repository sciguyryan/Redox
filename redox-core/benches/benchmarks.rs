use redox_core::{
    compiler::bytecode_compiler::Compiler,
    ins::instruction::Instruction,
    mem,
    vm::{self, VirtualMachine},
};

fn main() {
    // Run registered benchmarks.
    divan::main();
}

#[divan::bench]
fn virtual_machine_creation<'a>() -> VirtualMachine<'a> {
    VirtualMachine::new(
        vm::MIN_USER_SEGMENT_SIZE,
        &[],
        &[],
        vm::U32_STACK_CAPACITY * mem::memory_handler::BYTES_IN_U32,
    )
}

#[divan::bench]
fn virtual_machine_execution(bencher: divan::Bencher) {
    let instructions = &[
        Instruction::PushU32Imm(1234),
        Instruction::PushU32Imm(4321),
        Instruction::Hlt,
    ];

    let mut compiler = Compiler::new();
    let data = compiler.compile(instructions);

    bencher.bench(|| {
        let mut vm = VirtualMachine::new(
            vm::MIN_USER_SEGMENT_SIZE,
            data,
            &[],
            vm::U32_STACK_CAPACITY * mem::memory_handler::BYTES_IN_U32,
        );

        vm.run();
    });
}
