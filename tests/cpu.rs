use rustic::bus::Bus;
use rustic::cpu::CPU;

use expectest::prelude::*;

#[test]
fn clock_cycle() {
    let mut bus = Bus::new();
    bus.write(0x0000, 0x00); // BRK instruction, IMM addressing mode
    bus.write(0x0001, 0x10); // IMM value provided is 16

    let mut cpu = CPU::new(&mut bus);
    cpu.clock();

    expect!(cpu.a).to(be_eq(0x0001)); // The next byte of input after the opcode is the value of the instruction's input
}

#[test]
fn clock_cycle_next() {
    let mut bus = Bus::new();
    bus.write(0x0000, 0x00); // BRK instruction, IMM addressing mode - 6 cycles needed
    bus.write(0x0001, 0x10); // IMM value provided is 16

    let mut cpu = CPU::new(&mut bus);
    cpu.clock();
    cpu.clock();

    expect!(cpu.a).to(be_eq(0x0001)); // Value in A has not changed
}

#[test]
fn interrupt_ignored() {
    let mut bus = Bus::new();
    bus.write(0x0000, 0x78); // Run SEI first to set the Interrupt flag

    // To verify that we execute the interrupt, we just set two values in the bus
    // and make sure that the program counter is NOT set to the 2-byte address
    // specified by memory addresses 0xFFFE..0xFFFF.
    bus.write(0xFFFE, 0x55);
    bus.write(0xFFFF, 0xCC);

    let mut cpu = CPU::new(&mut bus);
    cpu.clock();
    cpu.interrupt();

    expect!(cpu.p_ctr).to_not(be_eq(0xCC55)); // Make sure we did nothing
}

#[test]
fn interrupt_followed() {
    let mut bus = Bus::new();
    bus.write(0x0000, 0x58); // Run CLI first to clear the Interrupt flag.

    // To verify that we execute the interrupt, we just set two values in the bus
    // and make sure that the program counter is set to the 2-byte address specified
    // by memory addresses 0xFFFE..0xFFFF.
    bus.write(0xFFFE, 0x55);
    bus.write(0xFFFF, 0xCC);

    let mut cpu = CPU::new(&mut bus);
    cpu.clock();
    cpu.interrupt();

    expect!(cpu.p_ctr).to(be_eq(0xCC55));
}

#[test]
fn interrupt_no_mask_cannot_ignore() {
    let mut bus = Bus::new();
    bus.write(0x0000, 0x78); // Run SEI first to set the Interrupt flag

    // To verify that we execute the interrupt, we just set two values in the bus
    // and make sure that the program counter is set to the 2-byte address specified
    // by memory addresses 0xFFFA..0xFFFB.
    bus.write(0xFFFA, 0x55);
    bus.write(0xFFFB, 0xCC);

    let mut cpu = CPU::new(&mut bus);
    cpu.clock();
    cpu.interrupt_no_mask();

    expect!(cpu.p_ctr).to(be_eq(0xCC55));
}

#[test]
fn reset() {
    let mut bus = Bus::new();
    bus.write(0xFFFC, 0x10);
    bus.write(0xFFFD, 0x30);

    let mut cpu = CPU::new(&mut bus);
    cpu.reset();

    expect!(cpu.a).to(be_eq(0x00));
    expect!(cpu.x).to(be_eq(0x00));
    expect!(cpu.y).to(be_eq(0x00));
    expect!(cpu.stk_ptr).to(be_eq(0xFD));

    // Program counter should be set to the value that was stored at the two
    // successive bytes 0xFFFC and 0xFFFD (the latter being its higher byte).
    expect!(cpu.p_ctr).to(be_eq(0x3010));
}