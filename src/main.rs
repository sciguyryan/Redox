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

use crate::{
    compiler::bytecode_compiler::Compiler,
    ins::{
        instruction::Instruction,
        move_expressions::{ExpressionArgs, ExpressionOperator, MoveExpressionHandler},
    },
    reg::registers::RegisterId,
};

// https://onlinedocs.microchip.com/pr/GUID-0E320577-28E6-4365-9BB8-9E1416A0A6E4-en-US-6/index.html?GUID-4983CB0C-7FEB-40F1-99D3-0608805404F3
// https://www.youtube.com/watch?v=KkenLT8S9Hs&list=WL&index=17
// https://stackoverflow.com/questions/71621573/how-does-the-processor-know-where-to-go-when-an-exception-is-thrown

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

    let mut vm = VirtualMachine::new(64_000);

    //vm.ram.set(0, 0x12, &SecurityContext::Machine);
    //vm.ram.print_range(0, 10);

    let expr_args_1 = [
        ExpressionArgs::Register(RegisterId::R7),
        ExpressionArgs::Operator(ExpressionOperator::Add),
        ExpressionArgs::Register(RegisterId::R8),
    ];
    let expr_args_2 = [
        ExpressionArgs::Constant(123),
        ExpressionArgs::Operator(ExpressionOperator::Subtract),
        ExpressionArgs::Constant(65),
    ];
    let mut expression_encoder = MoveExpressionHandler::new();
    let expr = expression_encoder.encode(&expr_args_1);

    let instructions = &[
        /*Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
        Instruction::MovU32ImmU32Reg(0x2, RegisterId::R2),
        Instruction::SwapU32RegU32Reg(RegisterId::R1, RegisterId::R2),
        Instruction::Mret,
        Instruction::MovU32ImmU32Reg(100, RegisterId::R7),
        Instruction::MovU32ImmU32Reg(50, RegisterId::R8),
        Instruction::MovU32ImmMemExprRel(0x123, expr),*/

        /*Instruction::MovU32ImmMemRelSimple(0x123, 150),
        Instruction::MovMemExprU32RegRel(
            expression_encoder.encode(&[
                ExpressionArgs::Constant(25),
                ExpressionArgs::Operator(ExpressionOperator::Add),
                ExpressionArgs::Constant(25),
            ]),
            RegisterId::R1,
        ),*/

        /*Instruction::MovU32ImmMemRelSimple(0x123, 150),
        Instruction::MovU32RegMemExprRel(
            RegisterId::R1,
            expression_encoder.encode(&[
                ExpressionArgs::Constant(25),
                ExpressionArgs::Operator(ExpressionOperator::Add),
                ExpressionArgs::Constant(25),
            ]),
        ),*/
        Instruction::Hlt,
    ];

    let mut compiler = Compiler::new();
    let data = compiler.compile(instructions);
    //println!("compiled data = {data:?}");
    //println!("compiled data len = {}", data.len());

    println!("----------[Instructions]----------");
    for ins in instructions {
        println!("{ins}");
    }
    println!();

    vm.load_code_block(0, data);
    vm.run();

    //vm.run_instructions(&instructions[..]);

    println!("----------[CPU]----------");
    println!("Machine Mode? {}", vm.cpu.is_machine_mode);
    println!();

    println!("----------[Registers]----------");
    vm.cpu.registers.print_registers();
    println!();

    println!("----------[RAM]----------");
    let segment = &vm.ram.get_storage()[..150];
    println!("{segment:?}");
}
