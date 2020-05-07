/// Basic structure used to define what sort of data we should expect to pull from the ROM.
#[derive(PartialEq, Debug, Clone)]
pub struct NESHeader {
    // The size of PRG (Program) ROM.
    pub prg_rom_size: usize,
    // The size of CHR (Character) ROM.
    pub chr_rom_size: usize,
    // The nametable mirroring mode.
    pub nametable_mirror_mode: NametableMirroringMode,
    // The system type.
    pub system_type: SystemType,
    // The size of PRG (Program) RAM.
    pub prg_ram_size: usize,
}

/// Definition of possible nametable mirror modes.
/// 
/// Reference: https://wiki.nesdev.com/w/index.php/Mirroring#Nametable_Mirroring
#[derive(PartialEq, Debug, Clone)]
pub enum NametableMirroringMode {
    Horizontal,
    Vertical,
    SingleScreenLow,
    SingleScreenHigh,
    FourScreen,
}

/// Definition of possible system types.
/// 
/// Reference: https://wiki.nesdev.com/w/index.php/INES#Flags_7
#[derive(PartialEq, Debug, Clone)]
pub enum SystemType {
    NES,
    NES2,
    VSUnisystem,
    PlayChoice10,
}
