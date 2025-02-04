#![crate_name = "redox_terminal"]

use redox_core::{
    compiling::{compiler::Compiler, decompiler::Decompiler},
    ins::instruction::Instruction,
    mem,
    reg::registers::RegisterId,
    vm::{self, VirtualMachine},
};

use std::{panic, time::Instant};

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
        "Elapsed (1): {:.2?} nanoseconds per iteration. Total = {elapsed1}",
        elapsed1 / iterations as f32
    );
    println!(
        "Elapsed (2): {:.2?} nanoseconds per iteration. Total = {elapsed2}",
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

    let code = "section .data
    test db 'bananas'
    section .text
    push 0
    call :LABEL_1
    hlt
    :LABEL_1
    mov 0xdeadbeef, EAX
    ret.i";
    let mut compiler = Compiler::new();
    let data = compiler.compile_assembly(code, true);

    /*let instructions = &[
        // Indicate that we want to make a seeded random number generator.
        Instruction::OutU8Imm(0x1, 0x0),
        // Specify our seed.
        Instruction::OutU32Imm(0xdeadbeef, 0x0),
        // Read a PRNG from the device.
        Instruction::InU32Reg(0x0, RegisterId::EAX),
        Instruction::PushU32Imm(0xdeadbeef),
        Instruction::Halt,
    ];

    let mut compiler = Compiler::new();
    let data = compiler.compile(instructions);*/

    println!("{data:?}");

    let mut vm = VirtualMachine::new(
        vm::MIN_USER_SEGMENT_SIZE,
        data,
        &[],
        vm::U32_STACK_CAPACITY * mem::memory_handler::SIZE_OF_U32,
    );

    println!("----------[Instructions]----------");
    for ins in Decompiler::decompile(data) {
        println!("{ins}");
    }
    println!();

    let now = Instant::now();
    vm.run();
    let elapsed = now.elapsed().as_nanos();

    println!("----------[CPU]----------");
    println!("Code successfully executed in {elapsed} ns!");
    println!();
    println!("Machine Mode? {}", vm.cpu.is_in_machine_mode());
    println!(
        "Stack Frame Size: {}",
        vm.com_bus.mem.get_stack_frame_size()
    );
    println!();

    println!("----------[Registers]----------");
    vm.cpu.registers.print_registers();
    println!();

    println!("----------[Code Memory Segment]----------");
    println!("{:?}", &vm.com_bus.mem.get_code_segment_storage());
    println!();

    println!("----------[Stack]----------");
    vm.com_bus.mem.print_stack();
    println!();

    println!("----------[Mapped Memory Segments]----------");
    vm.com_bus.mem.print_mapped_memory_segments();
    println!();
}
