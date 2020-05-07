use crate::rom::model::*;
use nom::{
    IResult,
    bits::{
        bits,
        streaming::take,
    },
    bytes::complete::tag,
    combinator::map_opt,
    multi::count,
    number::complete::be_u8,
    sequence::{
        pair,
        tuple,
    }
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
    }
}

impl Into<SystemType> for Flags7 {
    fn into(self) -> SystemType {
        if self.contains(Flags7::VS_UNISYSTEM) {
            SystemType::VSUnisystem
        } else if self.contains(Flags7::PLAYCHOICE_10) {
            SystemType::PlayChoice10
        } else {
            SystemType::NES
        }
    }
}

bitflags! {
    struct Flags9: u8 {
        const NTSC = 0 as u8;
        const PAL = 1 as u8;
    }
}

impl Into<TVType> for Flags9 {
    fn into(self) -> TVType {
        if self.contains(Flags9::PAL) {
            TVType::PAL
        } else {
            TVType::NTSC
        }
    }
}

bitflags! {
    struct Flags10: u8 {
        const NTSC = 0 as u8;
        // Note that technically Dual-Compatible is represented in byte 10 of the header as
        // *either* 1 or 3. As far as bitflags is concerned though, contains(1) + contains(2)
        // means we would contains(3).
        const DUAL_COMPATIBLE = 1 as u8;
        const PAL = (1 << 1) as u8;
    }
}

impl Into<TVType> for Flags10 {
    fn into(self) -> TVType {
        // Check for dual-compatible first, because 3 technically contains 2.
        if self.contains(Flags10::DUAL_COMPATIBLE) {
            TVType::DualCompatible
        // Now implied that bit 0 is 0, which means it's safe to check for PAL.
        } else if self.contains(Flags10::PAL) {
            TVType::PAL
        // Now implied that bit 1 is 0, which means it must be NTSC.
        } else {
            TVType::NTSC
        }
    }
}

fn parse_flag_bits<T>(input: &[u8], from_bits_fn: fn(u8) -> Option<T>) -> IResult<&[u8], (u8, T)> {
    bits(pair(
        take::<_, _, _, (_, _)>(4usize),
        map_opt(take::<_, _, _, (_, _)>(4usize), from_bits_fn)
    ))(input)
}

fn parse_flags_10(input: &[u8]) -> IResult<&[u8], (u8, u8, u8, u8, Flags10)> {
    bits(tuple((
        take::<_, _, _, (_, _)>(2usize),
        take::<_, _, _, (_, _)>(1usize), // Does the board have bus conflicts? (0: no, 1: yes)
        take::<_, _, _, (_, _)>(1usize), // Is PRG RAM used? (0: present, 1: not)
        take::<_, _, _, (_, _)>(2usize),
        map_opt(take::<_, _, _, (_, _)>(2usize), Flags10::from_bits)
    )))(input)
}

fn parse_ines_header(input: &[u8]) -> IResult<&[u8], INESHeader> {
    let (i, _) = tag(b"NES\x1A")(input)?;
    let (i, prg_page_count) = be_u8(i)?;
    let (i, chr_page_count) = be_u8(i)?;
    let (i, (mapper_lo, flags6)) = parse_flag_bits(i, Flags6::from_bits)?;
    let (i, (mapper_hi, flags7)) = parse_flag_bits(i, Flags7::from_bits)?;
    let (i, prg_ram_size) = be_u8(i)?;
    let (i, (_, flags9)) = parse_flag_bits(i, Flags9::from_bits)?; // Only bit 0 matters
    let (i, (_, bus_conflicts_f10, prg_ram_used_f10, _, flags10)) = parse_flags_10(i)?;

    // Chomp unused padding bytes 11 - 15. Note that this implicitly destroys compatibility
    // with the archaic iNES format (which should be fine). In that format, bytes 7-15 would
    // often be filled with signature strings from an image conversion or auditing tool.
    // The prior parsing of bytes 7-10 up to this point destroys that compatibility, anyways.
    // So I'm not going to fret about it.
    let (i, _) = count(tag(b"\x00"), 5)(i)?;

    let mapper = (mapper_hi << 4) | mapper_lo;
    let tv_type = if TVType::DualCompatible == flags10.into() {
        flags10.into()
    } else {
        flags9.into()
    };

    Ok((i, INESHeader {
        prg_rom_size: prg_page_count as usize,
        chr_rom_size: chr_page_count as usize,
        nametable_mirror_mode: flags6.into(),
        trainer_exists: flags6.contains(Flags6::TRAINER_EXISTS),
        mapper: mapper,
        system_type: flags7.into(),
        prg_ram_size: if prg_ram_size > 0 {
            prg_ram_size as usize
        } else { // PRG RAM Size specified as 0 is treated as 1 page for compatibility
            1
        },
        tv_type: tv_type,
        // Note that we check prg_ram_used_f10 == 0.
        // For some reason, spec says 0 = used, 1 = not used.
        prg_ram_used: flags6.contains(Flags6::BATTERY_BACKED_RAM) || prg_ram_used_f10 == 0,
        bus_conflicts_present: bus_conflicts_f10 != 0,
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
            0x02, // Number of pages in PRG ROM (8-bit integer)
            0x01, // Number of pages in CHR ROM (8-bit integer)
            0b00110001, // Mirroring (0+3), PRG RAM flag (1), trainer flag (2), Mapper low (4..7)
            0b00000000, // System type (0..4), Mapper high (4..7)
            0x00, // Number of pages in PRG RAM (8-bit integer)
            0b00000001, // TV system (0)
            0b00100001, // TV system (0+1), PRG RAM flag (4), board bus conflicts (5)
            0x00, // unused padding
            0x00, // unused padding
            0x00, // unused padding
            0x00, // unused padding
            0x00, // unused padding
        ];

        let rom = match parse_ines_header(&data[0..16]) {
            IResult::Ok((_, rom)) => rom,
            _ => panic!("Failed to parse!"),
        };

        expect!(rom.prg_rom_size).to(be_eq(2));
        expect!(rom.chr_rom_size).to(be_eq(1));
        expect!(rom.nametable_mirror_mode).to(be_eq(NametableMirroringMode::Vertical));
        expect!(rom.trainer_exists).to(be_false());
        expect!(rom.mapper).to(be_eq(0b00000011));
        expect!(rom.system_type).to(be_eq(SystemType::NES));
        expect!(rom.prg_ram_size).to(be_eq(1));
        expect!(rom.tv_type).to(be_eq(TVType::DualCompatible));
        expect!(rom.prg_ram_used).to(be_true());
        expect!(rom.bus_conflicts_present).to(be_true());
    }
}
