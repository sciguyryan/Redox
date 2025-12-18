use criterion::{Criterion, criterion_group, criterion_main};

use redox_core::{
    compiling::compiler::Compiler, ins::instruction::Instruction, reg::registers::RegisterId,
    vm::VirtualMachine, *,
};

pub fn criterion_vm_creation(c: &mut Criterion) {
    c.bench_function("vm creation", |b| {
        b.iter(|| {
            VirtualMachine::new(
                vm::MIN_USER_SEGMENT_SIZE,
                &[],
                &[],
                vm::U32_STACK_CAPACITY * mem::memory_handler::SIZE_OF_U32,
            )
        })
    });
}

pub fn criterion_vm_simple_program(c: &mut Criterion) {
    let instructions = &[Instruction::MovU32ImmU32Reg(0x1, RegisterId::EAX)];

    let mut compiler = Compiler::new();
    let data = compiler.compile(instructions);

    c.bench_function("vm run - simple program", |b| {
        b.iter(|| {
            let mut vm = VirtualMachine::new(
                vm::MIN_USER_SEGMENT_SIZE,
                data,
                &[],
                vm::U32_STACK_CAPACITY * mem::memory_handler::SIZE_OF_U32,
            );

            vm.run();
        })
    });
}

pub fn criterion_vm_complex_program(c: &mut Criterion) {
    let base_offset = vm::MIN_USER_SEGMENT_SIZE as u32;

    let instructions = &[
        Instruction::PushU32Imm(3), // Starts at [base]. Length = 8. This should remain in place.
        Instruction::PushU32Imm(2), // Starts at [base + 8]. Length = 8. Subroutine argument 1.
        Instruction::PushU32Imm(1), // Starts at [base + 16]. Length = 8. The number of arguments.
        Instruction::CallAbsU32Imm(base_offset + 36, String::default()), // Starts at [base + 24]. Length = 8.
        Instruction::Halt, // Starts at [base + 32]. Length = 4.
        /***** FUNC_AAAA - Subroutine 1 starts here. *****/
        Instruction::PushU32Imm(0), // Starts at [base + 36]. Length = 8. The number of arguments.
        Instruction::CallAbsU32Imm(base_offset + 65, String::default()), // Starts at [base + 44]. Length = 8.
        Instruction::AddU32ImmU32Reg(5, RegisterId::EAX), // Starts at [base + 52]. Length = 9.
        Instruction::RetArgsU32,                          // Starts at [base + 61]. Length = 4.
        /***** FUNC_BBBB - Subroutine 2 starts here. *****/
        Instruction::PushU32Imm(100), // Starts at [base + 65]. Length = 8. This should NOT be preserved.
        Instruction::PopU32ToU32Reg(RegisterId::EAX),
        Instruction::RetArgsU32,
    ];

    let mut compiler = Compiler::new();
    let data = compiler.compile(instructions);

    c.bench_function("vm run - complex program", |b| {
        b.iter(|| {
            let mut vm = VirtualMachine::new(
                vm::MIN_USER_SEGMENT_SIZE,
                data,
                &[],
                vm::U32_STACK_CAPACITY * mem::memory_handler::SIZE_OF_U32,
            );

            vm.run();
        })
    });
}

criterion_group!(
    benches,
    criterion_vm_creation,
    criterion_vm_simple_program,
    criterion_vm_complex_program
);
criterion_main!(benches);
