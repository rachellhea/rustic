use crate::cpu::CPU;

/// A basic structure of data defining an instruction supported by the CPU. Each instruction is
/// submitted to the CPU via a single byte which determines the functionality of the instruction
/// in how it loads its data from memory (if any) and how it operates on that data.
/// 
/// There are 56 different operations and 12 different addressing modes. Not all operations support
/// all addressing modes. The input format for the instruction opcode (a byte) allows for 256 total
/// combinations, many of which are unused.
/// 
/// For reference: http://www.obelisk.me.uk/6502/reference.html
pub struct Instruction<'a> {
	// The name of the instruction (3 alphabetic characters, all upper-case)
	pub name: &'a str,
	// The number of cycles required for computing this instruction
	pub cycles: u8,
	// A pointer to the function which implements the Operation logic for this instruction 
	// operate_function: Option<u8>,
	// A pointer to the function which implements the Addressing Mode logic for this instruction
	pub address_mode: Box<dyn FnOnce(&mut CPU<'a>) -> u8>,
}

/// The set of flags which are used to compose the status code bitmask on the CPU.
/// The status code bitmask is updated after every instruction, and thus the bitmask can be used
/// to inform future instructions about prior results, or to control various aspects of functionality,
/// e.g., if the I flag is set (disable interrupts), then IRQ signals can be ignored (but NMI cannot).
pub enum StatusFlag {
    C = 1 << 0, // Carry Bit
    Z = 1 << 1, // Zero
    I = 1 << 2, // Disable Interrupts
    D = 1 << 3, // Decimal Mode (unused)
    B = 1 << 4, // Break
    U = 1 << 5, // Unused
    V = 1 << 6, // Overflow
    N = 1 << 7, // Negative
}
