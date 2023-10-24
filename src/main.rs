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

fn run_test() {
    use std::time::Instant;
    let now = Instant::now();

    let expr_args = [
        ExpressionArgs::Register(RegisterId::R7),
        ExpressionArgs::Operator(ExpressionOperator::Add),
        ExpressionArgs::Register(RegisterId::R8),
    ];

    let mut expression_encoder = MoveExpressionHandler::new();
    let expr = expression_encoder.encode(&expr_args);

    let instructions = &[
        Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
        Instruction::MovU32ImmU32Reg(0x2, RegisterId::R2),
        Instruction::SwapU32RegU32Reg(RegisterId::R1, RegisterId::R2),
        Instruction::Mret,
        Instruction::MovU32ImmU32Reg(50, RegisterId::R7),
        Instruction::MovU32ImmU32Reg(50, RegisterId::R8),
        Instruction::MovU32ImmMemExprRel(0x123, expr),
        Instruction::Hlt,
    ];

    let data = Compiler::compile(instructions);

    let iterations = 100_000;
    // Code block to measure.
    {
        for _ in 0..iterations {
            let mut vm = VirtualMachine::new(500);
            vm.load_code_block(0, &data);
            vm.run();
        }
    }

    let elapsed = now.elapsed().as_micros() as f32;
    println!(
        "Elapsed: {:.2?} microseconds per iteration",
        elapsed / iterations as f32
    );
}

fn main() {
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
        Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
        Instruction::MovU32ImmU32Reg(0x2, RegisterId::R2),
        Instruction::SwapU32RegU32Reg(RegisterId::R1, RegisterId::R2),
        Instruction::Mret,
        Instruction::MovU32ImmU32Reg(50, RegisterId::R7),
        Instruction::MovU32ImmU32Reg(50, RegisterId::R8),
        Instruction::MovU32ImmMemExprRel(0x123, expr),
        /*Instruction::MovU32ImmMemRelSimple(0x123, 50),
        Instruction::MovMemExprU32RegRel(
            expression_encoder.encode(&[
                ExpressionArgs::Constant(25),
                ExpressionArgs::Operator(ExpressionOperator::Add),
                ExpressionArgs::Constant(25),
            ]),
            RegisterId::R1,
        ),*/
        /*Instruction::MovU32ImmMemRelSimple(0x123, 50),
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

    let data = Compiler::compile(instructions);
    //println!("compiled data = {data:?}");
    //println!("compiled data len = {}", data.len());

    println!("----------[Instructions]----------");
    for ins in instructions {
        println!("{ins}");
    }
    println!();

    vm.load_code_block(0, &data);
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
