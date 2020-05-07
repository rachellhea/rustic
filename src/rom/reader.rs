use crate::rom::model::*;
use nom::{
    IResult,
    bits::{
        bits,
        streaming::take,
    },
    bytes::complete::tag,
    combinator::map_opt,
    number::complete::{
        be_u32,
        be_u8,
    },
    sequence::pair,
};

pub const KILOBYTE_SIZE: usize = 1024;
pub const PRG_ROM_PAGE_SIZE: usize = 16 * KILOBYTE_SIZE;
pub const CHR_ROM_PAGE_SIZE: usize = 8 * KILOBYTE_SIZE;
pub const PRG_RAM_PAGE_SIZE: usize = 8 * KILOBYTE_SIZE;

bitflags! {
    struct Flags6: u8 {
        const VERTICAL_MIRRORING = 1 as u8;
        const BATTERY_BACKED_RAM = (1 << 1) as u8;
        const TRAINER_EXISTS = (1 << 2) as u8;
        const FOUR_SCREEN_MIRRORING = (1 << 3) as u8;
    }
}

impl Into<NametableMirroringMode> for Flags6 {
    fn into(self) -> NametableMirroringMode {
        if self.contains(Flags6::FOUR_SCREEN_MIRRORING) {
            NametableMirroringMode::FourScreen
        } else if self.contains(Flags6::VERTICAL_MIRRORING) {
            NametableMirroringMode::Vertical
        } else {
            NametableMirroringMode::Horizontal
        }
    }
}

bitflags! {
    struct Flags7: u8 {
        const VS_UNISYSTEM = 1 as u8;
        const PLAYCHOICE_10 = (1 << 1) as u8;
        const NES_2_0 = (2 << 2) as u8; // 00001000
    }
}

impl Into<SystemType> for Flags7 {
    fn into(self) -> SystemType {
        if self.contains(Flags7::NES_2_0) {
            SystemType::NES2
        } else if self.contains(Flags7::VS_UNISYSTEM) {
            SystemType::VSUnisystem
        } else if self.contains(Flags7::PLAYCHOICE_10) {
            SystemType::PlayChoice10
        } else {
            SystemType::NES
        }
    }
}

fn parse_flag_bits<T>(input: &[u8], from_bits_fn: fn(u8) -> Option<T>) -> IResult<&[u8], (u8, T)> {
    bits(pair(
        take::<_, _, _, (_, _)>(4usize),
        map_opt(take::<_, _, _, (_, _)>(4usize), from_bits_fn)
    ))(input)
}

fn parse_header(input: &[u8]) -> IResult<&[u8], NESHeader> {
    let (i, _) = tag(b"NES\x1A")(input)?;
    let (i, prg_page_count) = be_u8(i)?;
    let (i, chr_page_count) = be_u8(i)?;
    let (i, (_mapper_lo, flags6)) = parse_flag_bits(i, Flags6::from_bits)?;
    let (i, (_mapper_hi, flags7)) = parse_flag_bits(i, Flags7::from_bits)?;
    let (i, prg_ram_size) = be_u8(i)?;
    let (i, _) = be_u8(i)?; // skip byte 9; not in the official specification (TODO)
    let (i, _) = be_u8(i)?; // skip byte 10; not in the official specification (TODO)
    let (i, _) = be_u8(i)?; // skip byte 11; unused padding
    let (i, _) = be_u32(i)?; // skip bytes 12-15; unused padding

    Ok((i, NESHeader {
        prg_rom_size: prg_page_count as usize,
        chr_rom_size: chr_page_count as usize,
        nametable_mirror_mode: flags6.into(),
        system_type: flags7.into(),
        prg_ram_size: if prg_ram_size > 0 {
            prg_ram_size as usize
        } else { // PRG RAM Size specified as 0 is treated as 1 page for compatibility
            1
        },
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use expectest::prelude::*;

    #[test]
    fn header() {
        // Master reference: https://wiki.nesdev.com/w/index.php/INES
        let data = vec![
            0x4E, // N
            0x45, // E
            0x53, // S
            0x1A, // <EOF>
            0x02, // PRG ROM has 2 pages
            0x01, // CHR ROM has 1 page
            0x31, // Flags6 -> first 4 bits are low nybble of mapper, last 4 map to mirroring mode
            0x00, // Flags7 -> first 4 bits are high nybble of mapper, last 4 map to system type
            0x00, // Flags8 -> 8-bit integer specifying the number of pages in PRG RAM
            0x00, // Flags9 -> lowest bit specifies the TV system (NTSC or PAL)
            0x00, // Flags10 -> bits 1+2 specify TV system (again), bit 4 specifies if PRG RAM present
            0x00, // unused padding
            0x00, // unused padding
            0x00, // unused padding
            0x00, // unused padding
            0x00, // unused padding
        ];

        let rom = match parse_header(&data[0..16]) {
            IResult::Ok((_, rom)) => rom,
            _ => panic!("Failed to parse!"),
        };

        expect!(rom.prg_rom_size).to(be_eq(2));
        expect!(rom.chr_rom_size).to(be_eq(1));
        expect!(rom.prg_ram_size).to(be_eq(1));
        expect!(rom.nametable_mirror_mode).to(be_eq(NametableMirroringMode::Vertical));
        expect!(rom.system_type).to(be_eq(SystemType::NES));
    }
}
