#![crate_name = "redox"]

mod compiler;
mod cpu;
mod data_access_type;
mod ins;
mod mem;
mod parsing;
mod privilege_level;
mod reg;
mod utils;
pub mod vm;

use vm::VirtualMachine;

use crate::{compiler::bytecode_compiler::Compiler, ins::instruction::Instruction};

// https://onlinedocs.microchip.com/pr/GUID-0E320577-28E6-4365-9BB8-9E1416A0A6E4-en-US-6/index.html?GUID-4983CB0C-7FEB-40F1-99D3-0608805404F3
// https://www.youtube.com/watch?v=KkenLT8S9Hs&list=WL&index=17
// https://stackoverflow.com/questions/71621573/how-does-the-processor-know-where-to-go-when-an-exception-is-thrown
// Stack stuff - https://www.youtube.com/watch?v=CRTR5ljBjPM

/*fn run_test_2() {
    use std::time::Instant;

    let iterations = 1;

    let mut out_val_1 = 0;
    let mut out_val_2 = 0;

    let val: u32 = 0b1111_1111_1111_1111_1111_1111_1111_1111;
    let mut cpu1 = cpu::Cpu::new();
    let mut cpu2 = cpu::Cpu::new();

    let now1 = Instant::now();
    {
        for _ in 0..iterations {
            out_val_1 = perform_checked_lsh_u32(&mut cpu1, val, 2);
        }
    }
    let elapsed1 = now1.elapsed().as_nanos() as f32;

    let now2 = Instant::now();
    {
        for _ in 0..iterations {
            out_val_2 = perform_checked_lsh_u32_2(&mut cpu2, val, 2);
        }
    }
    let elapsed2 = now2.elapsed().as_nanos() as f32;

    println!(
        "Elapsed (1): {:.2?} microseconds per iteration",
        elapsed1 / iterations as f32
    );
    println!(
        "Elapsed (2): {:.2?} microseconds per iteration",
        elapsed2 / iterations as f32
    );

    assert_eq!(out_val_1, out_val_2);
    assert_eq!(
        cpu1.registers
            .get_register_u32(RegisterId::FL)
            .read_unchecked(),
        cpu2.registers
            .get_register_u32(RegisterId::FL)
            .read_unchecked()
    );
}*/

fn main() {
    if cfg!(target_endian = "big") {
        panic!("currently unsupported");
    }

    /*let expr_args_1 = [
        ExpressionArgs::Register(RegisterId::R7),
        ExpressionArgs::Operator(ExpressionOperator::Add),
        ExpressionArgs::Register(RegisterId::R8),
    ];

    let expr = MoveExpressionHandler::from(&expr_args_1[..]).pack();*/

    let instructions = &[
        Instruction::PushU32Imm(1234),
        Instruction::PushU32Imm(4321),
        Instruction::Hlt,
    ];

    let mut compiler = Compiler::new();
    let data = compiler.compile(instructions);

    let mut vm = VirtualMachine::new(
        vm::MIN_USER_SEGMENT_SIZE,
        data,
        &[],
        vm::U32_STACK_CAPACITY * vm::BYTES_IN_U32,
    );

    println!("----------[Instructions]----------");
    for ins in instructions {
        println!("{ins}");
    }
    println!();

    vm.run();

    //vm.run_instructions(&instructions[..]);

    println!("----------[CPU]----------");
    println!("Machine Mode? {}", vm.cpu.is_machine_mode);
    println!("Stack Frame Size: {}", vm.cpu.get_stack_frame_size());
    println!();

    println!("----------[Registers]----------");
    vm.cpu.registers.print_registers();
    println!();

    println!("----------[Code Memory Segment]----------");
    println!("{:?}", &vm.mem.get_code_segment_storage());
    println!();

    println!("----------[Stack]----------");
    vm.mem.print_stack();
    println!();

    println!("----------[Mapped Memory Regions]----------");
    vm.mem.print_mapped_memory_regions();
    println!();
}
