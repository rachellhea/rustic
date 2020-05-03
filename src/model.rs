use lazy_static::lazy_static;

/// A basic structure of data defining an instruction supported by the CPU. Each instruction is
/// submitted to the CPU via a single byte which determines the functionality of the instruction
/// in how it loads its data from memory (if any) and how it operates on that data.
/// 
/// There are 56 different operations and 12 different addressing modes. Not all operations support
/// all addressing modes. The input format for the instruction opcode (a byte) allows for 256 total
/// combinations, many of which are unused.
/// 
/// For reference: http://www.obelisk.me.uk/6502/reference.html
#[derive(Debug)]
pub struct Instruction<'a> {
    // The name of the instruction (3 alphabetic characters, all upper-case)
    pub name: &'a str,
    // The number of cycles required for computing this instruction
    pub cycles: u8,
    // The operating mode to use for implementation of this instruction. This needs to be mapped
    // by individual clients to a function pointer which implements that behavior.
    pub op_mode: OperatingMode,
    // The addressing mode to use for implementation of this instruction. This needs to be mapped
    // by individual clients to a function pointer which implements that behavior.
    pub addr_mode: AddressingMode,
}

/// The set of flags which are used to compose the status code bitmask on the CPU.
/// The status code bitmask is updated after every instruction, and thus the bitmask can be used
/// to inform future instructions about prior results, or to control various aspects of functionality,
/// e.g., if the I flag is set (disable interrupts), then IRQ signals can be ignored (but NMI cannot).
#[derive(PartialEq, Copy, Clone, Debug)]
pub enum StatusFlag {
    C = 1,      // Carry Bit
    Z = 1 << 1, // Zero
    I = 1 << 2, // Disable Interrupts
    D = 1 << 3, // Decimal Mode (unused)
    B = 1 << 4, // Break
    U = 1 << 5, // Unused
    V = 1 << 6, // Overflow
    N = 1 << 7, // Negative
}

/// The defined and supported addressing modes. Each of these determines how an instruction retrieves
/// the data to be used for its operation.
#[derive(PartialEq, Copy, Clone, Debug)]
pub enum AddressingMode {
    ABS,
    ABX,
    ABY,
    IMP,
    IMM,
    IND,
    IDX,
    IDY,
    REL,
    ZPG,
    ZPX,
    ZPY
}

/// The defined and supported operating modes. Each of these determines how an instruction operates
/// on its given data.
#[derive(PartialEq, Copy, Clone, Debug)]
pub enum OperatingMode {
    ADC,
    AND,
    ASL,
    BCC,
    BCS,
    BEQ,
    BIT,
    BMI,
    BNE,
    BPL,
    BRK,
    BVC,
    BVS,
    CLC,
    CLD,
    CLI,
    CLV,
    CMP,
    CPX,
    CPY,
    DEC,
    DEX,
    DEY,
    EOR,
    INC,
    INX,
    INY,
    JMP,
    JSR,
    LDA,
    LDX,
    LDY,
    LSR,
    NOP,
    ORA,
    PHA,
    PHP,
    PLA,
    PLP,
    ROL,
    ROR,
    RTI,
    RTS,
    SBC,
    SEC,
    SED,
    SEI,
    STA,
    STX,
    STY,
    TAX,
    TAY,
    TSX,
    TXA,
    TXS,
    TYA,
    XXX
}

lazy_static! {
    /// Static mapping of instruction opcodes (their index in the vector) and some properties about those instructions.
    /// Namely:
    /// * The number of cycles that they consume
    /// * Their operating mode
    /// * Their addressing mode
    pub static ref INSTRUCTIONS: Vec<Instruction<'static>> = vec![
        Instruction { name: "BRK", cycles: 7, op_mode: OperatingMode::BRK, addr_mode: AddressingMode::IMM }, // 0x00
        Instruction { name: "ORA", cycles: 6, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::IDX }, // 0x01
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x02
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x03
        Instruction { name: "XXX", cycles: 3, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x04
        Instruction { name: "ORA", cycles: 3, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::ZPG }, // 0x05
        Instruction { name: "ASL", cycles: 5, op_mode: OperatingMode::ASL, addr_mode: AddressingMode::ZPG }, // 0x06
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x07
        Instruction { name: "PHP", cycles: 3, op_mode: OperatingMode::PHP, addr_mode: AddressingMode::IMP }, // 0x08
        Instruction { name: "ORA", cycles: 2, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::IMM }, // 0x09
        Instruction { name: "ASL", cycles: 2, op_mode: OperatingMode::ASL, addr_mode: AddressingMode::IMP }, // 0x0A
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x0B
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x0C
        Instruction { name: "ORA", cycles: 4, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::ABS }, // 0x0D
        Instruction { name: "ASL", cycles: 6, op_mode: OperatingMode::ASL, addr_mode: AddressingMode::ABS }, // 0x0E
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x0F
        Instruction { name: "BPL", cycles: 2, op_mode: OperatingMode::BPL, addr_mode: AddressingMode::REL }, // 0x10
        Instruction { name: "ORA", cycles: 5, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::IDY }, // 0x11
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x12
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x13
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x14
        Instruction { name: "ORA", cycles: 4, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::ZPX }, // 0x15
        Instruction { name: "ASL", cycles: 6, op_mode: OperatingMode::ASL, addr_mode: AddressingMode::ZPX }, // 0x16
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x17
        Instruction { name: "CLC", cycles: 2, op_mode: OperatingMode::CLC, addr_mode: AddressingMode::IMP }, // 0x18
        Instruction { name: "ORA", cycles: 4, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::ABY }, // 0x19
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x1A
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x1B
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x1C
        Instruction { name: "ORA", cycles: 4, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::ABX }, // 0x1D
        Instruction { name: "ASL", cycles: 7, op_mode: OperatingMode::ASL, addr_mode: AddressingMode::ABX }, // 0x1E
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x1F
        Instruction { name: "JSR", cycles: 6, op_mode: OperatingMode::JSR, addr_mode: AddressingMode::ABS }, // 0x20
        Instruction { name: "AND", cycles: 6, op_mode: OperatingMode::AND, addr_mode: AddressingMode::IDX }, // 0x21
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x22
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x23
        Instruction { name: "BIT", cycles: 3, op_mode: OperatingMode::BIT, addr_mode: AddressingMode::ZPG }, // 0x24
        Instruction { name: "AND", cycles: 3, op_mode: OperatingMode::AND, addr_mode: AddressingMode::ZPG }, // 0x25
        Instruction { name: "ROL", cycles: 5, op_mode: OperatingMode::ROL, addr_mode: AddressingMode::ZPG }, // 0x26
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x27
        Instruction { name: "PLP", cycles: 4, op_mode: OperatingMode::PLP, addr_mode: AddressingMode::IMP }, // 0x28
        Instruction { name: "AND", cycles: 2, op_mode: OperatingMode::AND, addr_mode: AddressingMode::IMM }, // 0x29
        Instruction { name: "ROL", cycles: 2, op_mode: OperatingMode::ROL, addr_mode: AddressingMode::IMP }, // 0x2A
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x2B
        Instruction { name: "BIT", cycles: 4, op_mode: OperatingMode::BIT, addr_mode: AddressingMode::ABS }, // 0x2C
        Instruction { name: "AND", cycles: 4, op_mode: OperatingMode::AND, addr_mode: AddressingMode::ABS }, // 0x2D
        Instruction { name: "ROL", cycles: 6, op_mode: OperatingMode::ROL, addr_mode: AddressingMode::ABS }, // 0x2E
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x2F
        Instruction { name: "BMI", cycles: 2, op_mode: OperatingMode::BMI, addr_mode: AddressingMode::REL }, // 0x30
        Instruction { name: "AND", cycles: 5, op_mode: OperatingMode::AND, addr_mode: AddressingMode::IDY }, // 0x31
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x32
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x33
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x34
        Instruction { name: "AND", cycles: 4, op_mode: OperatingMode::AND, addr_mode: AddressingMode::ZPX }, // 0x35
        Instruction { name: "ROL", cycles: 6, op_mode: OperatingMode::ROL, addr_mode: AddressingMode::ZPX }, // 0x36
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x37
        Instruction { name: "SEC", cycles: 2, op_mode: OperatingMode::SEC, addr_mode: AddressingMode::IMP }, // 0x38
        Instruction { name: "AND", cycles: 4, op_mode: OperatingMode::AND, addr_mode: AddressingMode::ABY }, // 0x39
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x3A
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x3B
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x3C
        Instruction { name: "AND", cycles: 4, op_mode: OperatingMode::AND, addr_mode: AddressingMode::ABX }, // 0x3D
        Instruction { name: "ROL", cycles: 7, op_mode: OperatingMode::ROL, addr_mode: AddressingMode::ABX }, // 0x3E
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x3F
        Instruction { name: "RTI", cycles: 6, op_mode: OperatingMode::RTI, addr_mode: AddressingMode::IMP }, // 0x40
        Instruction { name: "EOR", cycles: 6, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::IDX }, // 0x41
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x42
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x43
        Instruction { name: "XXX", cycles: 3, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x44
        Instruction { name: "EOR", cycles: 3, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::ZPG }, // 0x45
        Instruction { name: "LSR", cycles: 5, op_mode: OperatingMode::LSR, addr_mode: AddressingMode::ZPG }, // 0x46
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x47
        Instruction { name: "PHA", cycles: 3, op_mode: OperatingMode::PHA, addr_mode: AddressingMode::IMP }, // 0x48
        Instruction { name: "EOR", cycles: 2, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::IMM }, // 0x49
        Instruction { name: "LSR", cycles: 2, op_mode: OperatingMode::LSR, addr_mode: AddressingMode::IMP }, // 0x4A
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x4B
        Instruction { name: "JMP", cycles: 3, op_mode: OperatingMode::JMP, addr_mode: AddressingMode::ABS }, // 0x4C
        Instruction { name: "EOR", cycles: 4, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::ABS }, // 0x4D
        Instruction { name: "LSR", cycles: 6, op_mode: OperatingMode::LSR, addr_mode: AddressingMode::ABS }, // 0x4E
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x4F
        Instruction { name: "BVC", cycles: 2, op_mode: OperatingMode::BVC, addr_mode: AddressingMode::REL }, // 0x50
        Instruction { name: "EOR", cycles: 5, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::IDY }, // 0x51
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x52
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x53
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x54
        Instruction { name: "EOR", cycles: 4, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::ZPX }, // 0x55
        Instruction { name: "LSR", cycles: 6, op_mode: OperatingMode::LSR, addr_mode: AddressingMode::ZPX }, // 0x56
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x57
        Instruction { name: "CLI", cycles: 2, op_mode: OperatingMode::CLI, addr_mode: AddressingMode::IMP }, // 0x58
        Instruction { name: "EOR", cycles: 4, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::ABY }, // 0x59
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x5A
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x5B
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x5C
        Instruction { name: "EOR", cycles: 4, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::ABX }, // 0x5D
        Instruction { name: "LSR", cycles: 7, op_mode: OperatingMode::LSR, addr_mode: AddressingMode::ABX }, // 0x5E
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x5F
        Instruction { name: "RTS", cycles: 6, op_mode: OperatingMode::RTS, addr_mode: AddressingMode::IMP }, // 0x60
        Instruction { name: "ADC", cycles: 6, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::IDX }, // 0x61
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x62
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x63
        Instruction { name: "XXX", cycles: 3, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x64
        Instruction { name: "ADC", cycles: 3, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::ZPG }, // 0x65
        Instruction { name: "ROR", cycles: 5, op_mode: OperatingMode::ROR, addr_mode: AddressingMode::ZPG }, // 0x66
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x67
        Instruction { name: "PLA", cycles: 4, op_mode: OperatingMode::PLA, addr_mode: AddressingMode::IMP }, // 0x68
        Instruction { name: "ADC", cycles: 2, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::IMM }, // 0x69
        Instruction { name: "ROR", cycles: 2, op_mode: OperatingMode::ROR, addr_mode: AddressingMode::IMP }, // 0x6A
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x6B
        Instruction { name: "JMP", cycles: 5, op_mode: OperatingMode::JMP, addr_mode: AddressingMode::IND }, // 0x6C
        Instruction { name: "ADC", cycles: 4, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::ABS }, // 0x6D
        Instruction { name: "ROR", cycles: 6, op_mode: OperatingMode::ROR, addr_mode: AddressingMode::ABS }, // 0x6E
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x6F
        Instruction { name: "BVS", cycles: 2, op_mode: OperatingMode::BVS, addr_mode: AddressingMode::REL }, // 0x70
        Instruction { name: "ADC", cycles: 5, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::IDY }, // 0x71
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x72
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x73
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x74
        Instruction { name: "ADC", cycles: 4, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::ZPX }, // 0x75
        Instruction { name: "ROR", cycles: 6, op_mode: OperatingMode::ROR, addr_mode: AddressingMode::ZPX }, // 0x76
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x77
        Instruction { name: "SEI", cycles: 2, op_mode: OperatingMode::SEI, addr_mode: AddressingMode::IMP }, // 0x78
        Instruction { name: "ADC", cycles: 4, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::ABY }, // 0x79
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x7A
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x7B
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x7C
        Instruction { name: "ADC", cycles: 4, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::ABX }, // 0x7D
        Instruction { name: "ROR", cycles: 7, op_mode: OperatingMode::ROR, addr_mode: AddressingMode::ABX }, // 0x7E
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x7F
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x80
        Instruction { name: "STA", cycles: 6, op_mode: OperatingMode::STA, addr_mode: AddressingMode::IDX }, // 0x81
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x82
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x83
        Instruction { name: "STY", cycles: 3, op_mode: OperatingMode::STY, addr_mode: AddressingMode::ZPG }, // 0x84
        Instruction { name: "STA", cycles: 3, op_mode: OperatingMode::STA, addr_mode: AddressingMode::ZPG }, // 0x85
        Instruction { name: "STX", cycles: 3, op_mode: OperatingMode::STX, addr_mode: AddressingMode::ZPG }, // 0x86
        Instruction { name: "XXX", cycles: 3, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x87
        Instruction { name: "DEY", cycles: 2, op_mode: OperatingMode::DEY, addr_mode: AddressingMode::IMP }, // 0x88
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x89
        Instruction { name: "TXA", cycles: 2, op_mode: OperatingMode::TXA, addr_mode: AddressingMode::IMP }, // 0x8A
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x8B
        Instruction { name: "STY", cycles: 4, op_mode: OperatingMode::STY, addr_mode: AddressingMode::ABS }, // 0x8C
        Instruction { name: "STA", cycles: 4, op_mode: OperatingMode::STA, addr_mode: AddressingMode::ABS }, // 0x8D
        Instruction { name: "STX", cycles: 4, op_mode: OperatingMode::STX, addr_mode: AddressingMode::ABS }, // 0x8E
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x8F
        Instruction { name: "BCC", cycles: 2, op_mode: OperatingMode::BCC, addr_mode: AddressingMode::REL }, // 0x90
        Instruction { name: "STA", cycles: 6, op_mode: OperatingMode::STA, addr_mode: AddressingMode::IDY }, // 0x91
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x92
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x93
        Instruction { name: "STY", cycles: 4, op_mode: OperatingMode::STY, addr_mode: AddressingMode::ZPX }, // 0x94
        Instruction { name: "STA", cycles: 4, op_mode: OperatingMode::STA, addr_mode: AddressingMode::ZPX }, // 0x95
        Instruction { name: "STX", cycles: 4, op_mode: OperatingMode::STX, addr_mode: AddressingMode::ZPY }, // 0x96
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x97
        Instruction { name: "TYA", cycles: 2, op_mode: OperatingMode::TYA, addr_mode: AddressingMode::IMP }, // 0x98
        Instruction { name: "STA", cycles: 5, op_mode: OperatingMode::STA, addr_mode: AddressingMode::ABY }, // 0x99
        Instruction { name: "TXS", cycles: 2, op_mode: OperatingMode::TXS, addr_mode: AddressingMode::IMP }, // 0x9A
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x9B
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0x9C
        Instruction { name: "STA", cycles: 5, op_mode: OperatingMode::STA, addr_mode: AddressingMode::ABX }, // 0x9D
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x9E
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0x9F
        Instruction { name: "LDY", cycles: 2, op_mode: OperatingMode::LDY, addr_mode: AddressingMode::IMM }, // 0xA0
        Instruction { name: "LDA", cycles: 6, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::IDX }, // 0xA1
        Instruction { name: "LDX", cycles: 2, op_mode: OperatingMode::LDX, addr_mode: AddressingMode::IMM }, // 0xA2
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xA3
        Instruction { name: "LDY", cycles: 3, op_mode: OperatingMode::LDY, addr_mode: AddressingMode::ZPG }, // 0xA4
        Instruction { name: "LDA", cycles: 3, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::ZPG }, // 0xA5
        Instruction { name: "LDX", cycles: 3, op_mode: OperatingMode::LDX, addr_mode: AddressingMode::ZPG }, // 0xA6
        Instruction { name: "XXX", cycles: 3, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xA7
        Instruction { name: "TAY", cycles: 2, op_mode: OperatingMode::TAY, addr_mode: AddressingMode::IMP }, // 0xA8
        Instruction { name: "LDA", cycles: 2, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::IMM }, // 0xA9
        Instruction { name: "TAX", cycles: 2, op_mode: OperatingMode::TAX, addr_mode: AddressingMode::IMP }, // 0xAA
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xAB
        Instruction { name: "LDY", cycles: 4, op_mode: OperatingMode::LDY, addr_mode: AddressingMode::ABS }, // 0xAC
        Instruction { name: "LDA", cycles: 4, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::ABS }, // 0xAD
        Instruction { name: "LDX", cycles: 4, op_mode: OperatingMode::LDX, addr_mode: AddressingMode::ABS }, // 0xAE
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xAF
        Instruction { name: "BCS", cycles: 2, op_mode: OperatingMode::BCS, addr_mode: AddressingMode::REL }, // 0xB0
        Instruction { name: "LDA", cycles: 5, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::IDY }, // 0xB1
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xB2
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xB3
        Instruction { name: "LDY", cycles: 4, op_mode: OperatingMode::LDY, addr_mode: AddressingMode::ZPX }, // 0xB4
        Instruction { name: "LDA", cycles: 4, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::ZPX }, // 0xB5
        Instruction { name: "LDX", cycles: 4, op_mode: OperatingMode::LDX, addr_mode: AddressingMode::ZPY }, // 0xB6
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xB7
        Instruction { name: "CLV", cycles: 2, op_mode: OperatingMode::CLV, addr_mode: AddressingMode::IMP }, // 0xB8
        Instruction { name: "LDA", cycles: 4, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::ABY }, // 0xB9
        Instruction { name: "TSX", cycles: 2, op_mode: OperatingMode::TSX, addr_mode: AddressingMode::IMP }, // 0xBA
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xBB
        Instruction { name: "LDY", cycles: 4, op_mode: OperatingMode::LDY, addr_mode: AddressingMode::ABX }, // 0xBC
        Instruction { name: "LDA", cycles: 4, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::ABX }, // 0xBD
        Instruction { name: "LDX", cycles: 4, op_mode: OperatingMode::LDX, addr_mode: AddressingMode::ABY }, // 0xBE
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xBF
        Instruction { name: "CPY", cycles: 2, op_mode: OperatingMode::CPY, addr_mode: AddressingMode::IMM }, // 0xC0
        Instruction { name: "CMP", cycles: 6, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::IDX }, // 0xC1
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0xC2
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xC3
        Instruction { name: "CPY", cycles: 3, op_mode: OperatingMode::CPY, addr_mode: AddressingMode::ZPG }, // 0xC4
        Instruction { name: "CMP", cycles: 3, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::ZPG }, // 0xC5
        Instruction { name: "DEC", cycles: 5, op_mode: OperatingMode::DEC, addr_mode: AddressingMode::ZPG }, // 0xC6
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xC7
        Instruction { name: "INY", cycles: 2, op_mode: OperatingMode::INY, addr_mode: AddressingMode::IMP }, // 0xC8
        Instruction { name: "CMP", cycles: 2, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::IMM }, // 0xC9
        Instruction { name: "DEX", cycles: 2, op_mode: OperatingMode::DEX, addr_mode: AddressingMode::IMP }, // 0xCA
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xCB
        Instruction { name: "CPY", cycles: 4, op_mode: OperatingMode::CPY, addr_mode: AddressingMode::ABS }, // 0xCC
        Instruction { name: "CMP", cycles: 4, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::ABS }, // 0xCD
        Instruction { name: "DEC", cycles: 6, op_mode: OperatingMode::DEC, addr_mode: AddressingMode::ABS }, // 0xCE
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xCF
        Instruction { name: "BNE", cycles: 2, op_mode: OperatingMode::BNE, addr_mode: AddressingMode::REL }, // 0xD0
        Instruction { name: "CMP", cycles: 5, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::IDY }, // 0xD1
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xD2
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xD3
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0xD4
        Instruction { name: "CMP", cycles: 4, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::ZPX }, // 0xD5
        Instruction { name: "DEC", cycles: 6, op_mode: OperatingMode::DEC, addr_mode: AddressingMode::ZPX }, // 0xD6
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xD7
        Instruction { name: "CLD", cycles: 2, op_mode: OperatingMode::CLD, addr_mode: AddressingMode::IMP }, // 0xD8
        Instruction { name: "CMP", cycles: 4, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::ABY }, // 0xD9
        Instruction { name: "NOP", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0xDA
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xDB
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0xDC
        Instruction { name: "CMP", cycles: 4, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::ABX }, // 0xDD
        Instruction { name: "DEC", cycles: 7, op_mode: OperatingMode::DEC, addr_mode: AddressingMode::ABX }, // 0xDE
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xDF
        Instruction { name: "CPX", cycles: 2, op_mode: OperatingMode::CPX, addr_mode: AddressingMode::IMM }, // 0xE0
        Instruction { name: "SBC", cycles: 6, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::IDX }, // 0xE1
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0xE2
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xE3
        Instruction { name: "CPX", cycles: 3, op_mode: OperatingMode::CPX, addr_mode: AddressingMode::ZPG }, // 0xE4
        Instruction { name: "SBC", cycles: 3, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::ZPG }, // 0xE5
        Instruction { name: "INC", cycles: 5, op_mode: OperatingMode::INC, addr_mode: AddressingMode::ZPG }, // 0xE6
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xE7
        Instruction { name: "INX", cycles: 2, op_mode: OperatingMode::INX, addr_mode: AddressingMode::IMP }, // 0xE8
        Instruction { name: "SBC", cycles: 2, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::IMM }, // 0xE9
        Instruction { name: "NOP", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0xEA
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::IMP }, // 0xEB
        Instruction { name: "CPX", cycles: 4, op_mode: OperatingMode::CPX, addr_mode: AddressingMode::ABS }, // 0xEC
        Instruction { name: "SBC", cycles: 4, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::ABS }, // 0xED
        Instruction { name: "INC", cycles: 6, op_mode: OperatingMode::INC, addr_mode: AddressingMode::ABS }, // 0xEE
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xEF
        Instruction { name: "BEQ", cycles: 2, op_mode: OperatingMode::BEQ, addr_mode: AddressingMode::REL }, // 0xF0
        Instruction { name: "SBC", cycles: 5, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::IDY }, // 0xF1
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xF2
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xF3
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0xF4
        Instruction { name: "SBC", cycles: 4, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::ZPX }, // 0xF5
        Instruction { name: "INC", cycles: 6, op_mode: OperatingMode::INC, addr_mode: AddressingMode::ZPX }, // 0xF6
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xF7
        Instruction { name: "SED", cycles: 2, op_mode: OperatingMode::SED, addr_mode: AddressingMode::IMP }, // 0xF8
        Instruction { name: "SBC", cycles: 4, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::ABY }, // 0xF9
        Instruction { name: "NOP", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0xFA
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xFB
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP }, // 0xFC
        Instruction { name: "SBC", cycles: 4, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::ABX }, // 0xFD
        Instruction { name: "INC", cycles: 7, op_mode: OperatingMode::INC, addr_mode: AddressingMode::ABX }, // 0xFE
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP }, // 0xFF
    ];
}
