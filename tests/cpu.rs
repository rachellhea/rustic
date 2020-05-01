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
    // To verify that we did not execute the interrupt, we just set two values in the bus
    // and make sure that we get the one from 0x0000 (the starting address).
    bus.write(0xFFFE, 0x55);
    bus.write(0x0000, 0x33);

    let mut cpu = CPU::new(&mut bus);
    cpu.set_interrupt_flag(true);
    cpu.interrupt();

    expect!(cpu.fetch()).to_not(be_eq(0x55)); // Make sure we did nothing
}

#[test]
fn interrupt_followed() {
    let mut bus = Bus::new();

    // To verify that we execute the interrupt, we just set two values in the bus
    // and make sure that we get the one from 0xFFFE (the interrupt address).
    bus.write(0xFFFE, 0x55);
    bus.write(0x0000, 0x33);

    let mut cpu = CPU::new(&mut bus);
    cpu.set_interrupt_flag(false);
    cpu.interrupt();

    expect!(cpu.fetch()).to(be_eq(0x55));
}

#[test]
fn interrupt_no_mask_cannot_ignore() {
    let mut bus = Bus::new();
    // To verify that we execute the interrupt, we just set two values in the bus
    // and make sure that we get the one from 0xFFFA (the no-mask interrupt address).
    bus.write(0xFFFA, 0x55);
    bus.write(0x0000, 0x33);

    let mut cpu = CPU::new(&mut bus);
    cpu.set_interrupt_flag(true);
    cpu.interrupt_no_mask();

    expect!(cpu.fetch()).to(be_eq(0x55));
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
    expect!(cpu.stat).to(be_eq(0b00100000)); // Only U is set (Unused)

    // Program counter should be set to the value that was stored at the two
    // successive bytes 0xFFFC and 0xFFFD (the latter being its higher byte).
    expect!(cpu.p_ctr).to(be_eq(0x3010));
}