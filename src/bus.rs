const RAM_SIZE: usize = 64 * 1024;

pub struct Bus {
	ram: [u8; RAM_SIZE],
}

impl Bus {
	pub fn new() -> Bus {
		Bus {
			ram: [0x0000; RAM_SIZE],
		}
	}

	pub fn read(&self, addr: u16) -> u8 {
		match addr {
			0x0000..=0xFFFF => self.ram[addr as usize],
		}
	}

	pub fn write(&mut self, addr: u16, data: u8) {
		match addr {
			0x0000..=0xFFFF => self.ram[addr as usize] = data,
		};
	}
}
