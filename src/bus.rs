/// Defines the total amount of RAM to be allocated.
const RAM_SIZE: usize = 64 * 1024;

/// Core structure of the Bus, which controls access to RAM across connected devices.
pub struct Bus {
    /// The allocated RAM for the Bus.
    ram: [u8; RAM_SIZE],
}

impl Bus {
    #[allow(dead_code)]
    /// Construct a new Bus, instantiating its RAM block to all 0s.
    pub fn new() -> Bus {
        Bus {
            ram: [0x0000; RAM_SIZE],
        }
    }

    /// Read a byte stored in RAM at the given address.
    pub fn read(&self, addr: u16) -> u8 {
        match addr {
            // Note that this matcher will change as we connect devices to the Bus.
            0x0000..=0xFFFF => self.ram[addr as usize],
        }
    }

    /// Write a byte value to RAM at the given address.
    pub fn write(&mut self, addr: u16, data: u8) {
        match addr {
            // Note that this matcher will change as we connect devices to the Bus.
            0x0000..=0xFFFF => self.ram[addr as usize] = data,
        };
    }
}
