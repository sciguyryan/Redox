#![crate_name = "redox_terminal"]

use redox_core::{
    compiler::bytecode_compiler::Compiler,
    ins::instruction::Instruction,
    mem,
    reg::registers::RegisterId,
    vm::{self, VirtualMachine},
};

use std::time::Instant;

// https://onlinedocs.microchip.com/pr/GUID-0E320577-28E6-4365-9BB8-9E1416A0A6E4-en-US-6/index.html?GUID-4983CB0C-7FEB-40F1-99D3-0608805404F3
// https://www.youtube.com/watch?v=KkenLT8S9Hs&list=WL&index=17
// https://stackoverflow.com/questions/71621573/how-does-the-processor-know-where-to-go-when-an-exception-is-thrown
// Stack stuff - https://www.youtube.com/watch?v=CRTR5ljBjPM

/*fn run_test_2() {
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
        "Elapsed (1): {:.2?} microseconds per iteration. Total = {elapsed1}",
        elapsed1 / iterations as f32
    );
    println!(
        "Elapsed (2): {:.2?} microseconds per iteration. Total = {elapsed2}",
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

fn testing_calls() {
    let instructions = &[
        // The number of arguments for the subroutine.
        Instruction::PushU32Imm(3), // Starts at 100. Length = 8. This should remain in place.
        Instruction::PushU32Imm(2), // Starts at 108. Length = 8. Subroutine argument 1.
        Instruction::PushU32Imm(1), // Starts at 116. Length = 8. The number of arguments.
        Instruction::CallU32Imm(136, 0), // Starts at 124. Length = 8.
        Instruction::Hlt,           // Starts at 132. Length = 4.
        // FUNC_AAAA - Subroutine starts here.
        Instruction::Nop,                              // Starts at 136. Length = 4.
        Instruction::PushU32Imm(10),                   // Starts at 140. Length = 8.
        Instruction::PopU32ImmU32Reg(RegisterId::ER1), // Starts at 148. Length = 5.
        Instruction::RetArgsU32,                       // Starts at 156. Length = 8.
    ];

    let mut compiler = Compiler::new();
    let data = compiler.compile(instructions);

    let mut vm = VirtualMachine::new(100, data, &[], 150 * mem::memory_handler::BYTES_IN_U32);

    vm.run();

    vm.mem.print_stack();
    println!();
    vm.cpu.registers.print_registers();
}

fn main() {
    if cfg!(target_endian = "big") {
        panic!("currently unsupported");
    }

    testing_calls();
    return;

    let instructions = &[
        Instruction::PushU32Imm(1234),
        Instruction::PushU32Imm(4321),
        Instruction::Hlt,
    ];

    /*let instructions = &[
        // We expect that the first move instruction will be skipped entirely.
        Instruction::AddU32ImmU32Reg(23, RegisterId::ECS), // 9
        Instruction::JumpAbsU32Reg(RegisterId::EAC), // 5
        // This instruction should be skipped, so ER1 should remain at the default value of 0.
        Instruction::MovU32ImmU32Reg(0xf, RegisterId::ER1), // 9
        // The jump should start execution here.
        Instruction::MovU32ImmU32Reg(0xa, RegisterId::ER2),
        Instruction::Hlt,
    ];*/

    let mut compiler = Compiler::new();
    let data = compiler.compile(instructions);

    let mut vm = VirtualMachine::new(
        vm::MIN_USER_SEGMENT_SIZE,
        data,
        &[],
        vm::U32_STACK_CAPACITY * mem::memory_handler::BYTES_IN_U32,
    );

    println!("----------[Instructions]----------");
    for ins in instructions {
        println!("{ins}");
    }
    println!();

    let now = Instant::now();
    vm.run();
    let elapsed = now.elapsed().as_nanos();

    println!("----------[CPU]----------");
    println!("Code successfully executed in {elapsed} ns!");
    println!();
    println!("Machine Mode? {}", vm.cpu.is_machine_mode);
    println!("Stack Frame Size: {}", vm.mem.stack_frame_size);
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

    println!("----------[Mapped Memory Segments]----------");
    vm.mem.print_mapped_memory_segments();
    println!();
}
