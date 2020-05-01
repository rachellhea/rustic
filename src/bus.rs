use std::fmt;

/// Defines the total amount of RAM to be allocated.
const RAM_SIZE: usize = 64 * 1024;

/// Core structure of the Bus, which controls access to RAM across connected devices.
#[derive(Debug)]
pub struct Bus {
    /// The allocated RAM for the Bus.
    ram: RAM,
}

impl Bus {
    /// Construct a new Bus, instantiating its RAM block to all 0s.
    pub fn new() -> Bus {
        Bus {
            ram: RAM { data: [0x0000; RAM_SIZE] },
        }
    }

    /// Read a byte stored in RAM at the given address.
    pub fn read(&self, addr: u16) -> u8 {
        match addr {
            // Note that this matcher will change as we connect devices to the Bus.
            0x0000..=0xFFFF => self.ram.data[addr as usize],
        }
    }

    /// Write a byte value to RAM at the given address.
    pub fn write(&mut self, addr: u16, data: u8) {
        match addr {
            // Note that this matcher will change as we connect devices to the Bus.
            0x0000..=0xFFFF => self.ram.data[addr as usize] = data,
        };
    }
}

/// Custom struct for implmenting the Debug trait on top of a "large" array.
/// Rust only supports out-of-the-box implementation of the Debug trait for arrays of size
/// 32 and smaller. Since we're making something much larger (65536), we need to implement
/// Debug ourselves.
struct RAM {
    data: [u8; RAM_SIZE],
}

impl fmt::Debug for RAM {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        self.data[..].fmt(formatter)
    }
}

impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}
