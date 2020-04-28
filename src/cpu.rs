use crate::bus::Bus;
use crate::model::{StatusFlag, INSTRUCTION_LOOKUP};

/// Core data structure for emulating a 6502 CPU.
pub struct CPU<'a> {
	/// A hook to the bus, which serves as our interface through which we can read from and write to
	/// memory. This includes retrieving opcodes and data for instruction processing.
	bus: Option<&'a mut Bus>,
	/// The status code. This is a bitmask composed of a number of flags, each of which serves to
	/// surface some meaning about the state of the CPU and its last executed instruction.
	/// See: model::StatusFlag
	stat: u8,
	// The accumulator register.
	a: u8,
	// The X register.
	x: u8,
	// The Y register.
	y: u8,
	// The stack pointer.
	stk_ptr: u8,
	// The program counter.
	p_ctr: u16,
	// A reference bank for data fetched from memory.
	m: u8,
	// The address (absolute) from which we'll fetch data from memory.
	addr: u16,
	/// The address (relative) from which we'll fetch data from memory.
	/// Not all operations have access to the full memory bank; some can only make jumps from the
	/// current memory address. This register stores that input.
	addr_rel: u8,
	// The current opcode input.
	opcode: u8,
	// The number of cycles left to execute for the current command.
	cycles: u8
}

impl<'a> CPU<'a> {
	/// Construct a new CPU instance. All values are initialized to 0 or None.
	pub fn new(bus: &'a mut Bus) -> CPU<'a> {
		CPU {
			bus: 		Some(bus),
			stat: 		0x00,
			a: 			0x00,
			x: 			0x00,
			y: 			0x00,
			stk_ptr: 	0x00,
			p_ctr: 		0x0000,
			m: 			0x00,
			addr: 		0x0000,
			addr_rel: 	0x00,
			opcode: 	0x00,
			cycles: 	0,
		}
	}

	/// Iterate forward one clock cycle. Accept the input associated with that clock
	/// cycle and act on it.
	pub fn clock(&mut self) {
		match self.cycles {
			0 => {
				self.opcode = self.read(self.p_ctr);
				self.p_ctr += 1;

				let instruction = &INSTRUCTION_LOOKUP[self.opcode as usize];
				self.cycles = instruction.cycles;
			},
			_ => (),
		};

		self.cycles -= 1;
	}

	/// Read data from the connected Bus.
	fn read(&self, addr: u16) -> u8 {
		self.bus
			.as_ref()
			.map(|bus| bus.read(addr))
			.unwrap_or_default()
	}

	/// Write data to the connected Bus.
	fn write(&mut self, addr: u16, data: u8) {
		self.bus
			.as_mut()
			.map(|bus| bus.write(addr, data));
	}

	/// Retrieve the boolean value of a particular flag in the status code.
	fn get_status_flag(&self, status_flag: StatusFlag) -> bool {
		(self.stat & (status_flag as u8)) != 0
	}
	
	/// Set the boolean value of a particular flag in the status code.
	fn set_status_flag(&mut self, status_flag: StatusFlag, v: bool) {
		match v {
			true => self.stat |= status_flag as u8,
			false => self.stat &= !(status_flag as u8),
		};
	}
}
