/// Basic structure used to define what sort of data we should expect to pull from the ROM.
#[derive(PartialEq, Debug, Clone)]
pub struct INESHeader {
    // The size of PRG (Program) ROM.
    pub prg_rom_size: usize,
    // The size of CHR (Character) ROM.
    pub chr_rom_size: usize,
    // The nametable mirroring mode.
    pub nametable_mirror_mode: NametableMirroringMode,
    // A flag denoting if a 512-byte trainer exists at $7000-$71FF.
    pub trainer_exists: bool,
    // The mapper number.
    pub mapper: u8,
    // The system type.
    pub system_type: SystemType,
    // The size of PRG (Program) RAM.
    pub prg_ram_size: usize,
    // The TV typs.
    pub tv_type: TVType,
    // A flag denoting if PRG RAM is used. True represents that the PRG RAM is used.
    pub prg_ram_used: bool,
    // A flag denoting if Bus conflicts are present on the board.
    pub bus_conflicts_present: bool,
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
    VSUnisystem,
    PlayChoice10,
}

/// Definition of possible TV types.
/// 
/// Reference: https://wiki.nesdev.com/w/index.php/INES#Flags_9
#[derive(PartialEq, Debug, Clone)]
pub enum TVType {
    NTSC,
    PAL,
    DualCompatible,
}
