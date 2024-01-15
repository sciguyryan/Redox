#![crate_name = "redox_terminal"]

use redox_core::{
    compiler::bytecode_compiler::Compiler,
    ins::instruction::Instruction,
    mem,
    reg::registers::RegisterId,
    vm::{self, VirtualMachine},
};

use std::{arch::asm, time::Instant};

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

const BIT_MASKS: [u32; 32] = [
    0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000,
    0x8000, 0x10000, 0x20000, 0x40000, 0x80000, 0x100000, 0x200000, 0x400000, 0x800000, 0x1000000,
    0x2000000, 0x4000000, 0x8000000, 0x10000000, 0x20000000, 0x40000000, 0x80000000,
];

#[inline(never)]
pub fn is_bit_set_2(value: u32, bit: u8) -> bool {
    unsafe { (value & BIT_MASKS.get_unchecked(bit as usize)) != 0 }
}

#[inline(never)]
pub fn is_bit_set_3(value: u32, bit: u32) -> bool {
    let result: u32;
    unsafe {
        asm!(
            "bt {0:e}, {1:e}",
            inout(reg) value => _,
            in(reg) bit
        );
        asm!(
            "adc {0:e}, 0",
            out(reg) result
        );
    }
    result != 0
}

fn main() {
    if cfg!(target_endian = "big") {
        panic!("currently unsupported");
    }

    let instructions = &[
        // Indicate that we want to make a seeded random number generator.
        Instruction::OutU8Imm(0x1, 0x0),
        // Specify our seed.
        Instruction::OutU32Imm(0xdeadbeef, 0x0),
        // Read a PRNG from the device.
        Instruction::InU32Reg(0x0, RegisterId::ER1),
        /*// Write the handler addresses into the IVT.
        // Handler for 0x00. This is a non-maskable interrupt.
        // We are essentially replacing the default handler for the division by zero interrupt.
        Instruction::MovU32ImmMemSimple(base_offset + 56, 0), // Starts at [base]. Length = 12.
        // Handler for 0xff.
        Instruction::MovU32ImmMemSimple(base_offset + 69, 255 * 4), // Starts at [base + 12]. Length = 12.
        // Disable maskable CPU interrupts.
        Instruction::MovU32ImmU32Reg(0x0, RegisterId::EIM), // Starts at [base + 24]. Length = 9.
        // This interrupt handler should not be executed.
        Instruction::Int(0xff), // Starts at [base + 33]. Length = 5.
        // This interrupt handler should be executed since it can't be disabled or masked.
        Instruction::Int(0x0), // Starts at [base + 38]. Length = 5.
        Instruction::AddU32ImmU32Reg(0x5, RegisterId::ER1), // Starts at [base + 43]. Length = 9.
        Instruction::Halt,     // Starts at [base + 52]. Length = 4.
        /***** INT_00 - Interrupt handler for interrupt 0x00 starts here. *****/
        Instruction::MovU32ImmU32Reg(0x64, RegisterId::ER1), // Starts at [base + 56]. Length = 9.
        Instruction::IntRet,                                 // Starts at [base + 65]. Length = 4.
        /***** INT_FF - Interrupt handler for interrupt 0xff starts here. *****/
        Instruction::MovU32ImmU32Reg(0x5, RegisterId::ER1), // Starts at [base + 69]. Length = 9.
        Instruction::IntRet,*/
        Instruction::Halt,
    ];

    let mut compiler = Compiler::new();
    let data = compiler.compile(instructions);

    let mut vm = VirtualMachine::new(
        vm::MIN_USER_SEGMENT_SIZE,
        data,
        &[],
        vm::U32_STACK_CAPACITY * mem::memory_handler::SIZE_OF_U32,
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
    println!("Stack Frame Size: {}", vm.com_bus.mem.stack_frame_size);
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
