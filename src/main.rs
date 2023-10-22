#![crate_name = "redox"]

mod compiler;
mod cpu;
mod data_access_type;
mod ins;
mod mem;
mod parsing;
mod reg;
mod security_context;
mod utils;
pub mod vm;

use vm::VirtualMachine;

use crate::{
    compiler::{bytecode_compiler::Compiler, bytecode_decompiler::Decompiler},
    ins::{
        instruction::Instruction,
        move_expressions::{ExpressionArgs, ExpressionOperator, MoveExpressionHandler},
    },
    reg::registers::RegisterId,
};

// https://onlinedocs.microchip.com/pr/GUID-0E320577-28E6-4365-9BB8-9E1416A0A6E4-en-US-6/index.html?GUID-4983CB0C-7FEB-40F1-99D3-0608805404F3
// https://www.youtube.com/watch?v=KkenLT8S9Hs&list=WL&index=17

fn main() {
    let mut vm = VirtualMachine::new(64_000);

    //vm.ram.set(0, 0x12, &SecurityContext::Machine);
    //vm.ram.print_range(0, 10);

    let expr_args = vec![
        ExpressionArgs::Register(RegisterId::R7),
        ExpressionArgs::Operator(ExpressionOperator::Multiply),
        ExpressionArgs::Register(RegisterId::R8),
    ];
    let mut expression_encoder = MoveExpressionHandler::new();
    let expr = expression_encoder.encode(&expr_args);
    //println!("{:#018b}", expr);
    //return;

    let instructions = &[
        Instruction::MovU32ImmU32Reg(0x1, RegisterId::R1),
        Instruction::MovU32ImmU32Reg(0x2, RegisterId::R2),
        Instruction::SwapU32RegU32Reg(RegisterId::R1, RegisterId::R2),
        Instruction::Mret,
        Instruction::MovU32ImmMemRelExpr(0x123, expr),
        Instruction::Hlt,
    ];

    let data = Compiler::compile(instructions);

    let mut decompiler = Decompiler::new(&data);
    let insssss = decompiler.decompile();

    println!("----------[Instructions]----------");
    for ins in instructions {
        println!("{ins}");
    }

    return;

    for ins in instructions {
        ins.get_instruction_size();
    }

    vm.run_instructions(&instructions[..]);

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
