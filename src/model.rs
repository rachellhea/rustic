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
#[derive(PartialEq, Copy, Clone)]
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
#[derive(PartialEq, Copy, Clone)]
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
#[derive(PartialEq, Copy, Clone)]
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
        Instruction { name: "BRK", cycles: 7, op_mode: OperatingMode::BRK, addr_mode: AddressingMode::IMM },
        Instruction { name: "ORA", cycles: 6, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::IDX },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 3, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "ORA", cycles: 3, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::ZPG },
        Instruction { name: "ASL", cycles: 5, op_mode: OperatingMode::ASL, addr_mode: AddressingMode::ZPG },
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "PHP", cycles: 3, op_mode: OperatingMode::PHP, addr_mode: AddressingMode::IMP },
        Instruction { name: "ORA", cycles: 2, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::IMM },
        Instruction { name: "ASL", cycles: 2, op_mode: OperatingMode::ASL, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "ORA", cycles: 4, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::ABS },
        Instruction { name: "ASL", cycles: 6, op_mode: OperatingMode::ASL, addr_mode: AddressingMode::ABS },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "BPL", cycles: 2, op_mode: OperatingMode::BPL, addr_mode: AddressingMode::REL },
        Instruction { name: "ORA", cycles: 5, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::IDY },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "ORA", cycles: 4, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::ZPX },
        Instruction { name: "ASL", cycles: 6, op_mode: OperatingMode::ASL, addr_mode: AddressingMode::ZPX },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "CLC", cycles: 2, op_mode: OperatingMode::CLC, addr_mode: AddressingMode::IMP },
        Instruction { name: "ORA", cycles: 4, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::ABY },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "ORA", cycles: 4, op_mode: OperatingMode::ORA, addr_mode: AddressingMode::ABX },
        Instruction { name: "ASL", cycles: 7, op_mode: OperatingMode::ASL, addr_mode: AddressingMode::ABX },
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "JSR", cycles: 6, op_mode: OperatingMode::JSR, addr_mode: AddressingMode::ABS },
        Instruction { name: "AND", cycles: 6, op_mode: OperatingMode::AND, addr_mode: AddressingMode::IDX },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "BIT", cycles: 3, op_mode: OperatingMode::BIT, addr_mode: AddressingMode::ZPG },
        Instruction { name: "AND", cycles: 3, op_mode: OperatingMode::AND, addr_mode: AddressingMode::ZPG },
        Instruction { name: "ROL", cycles: 5, op_mode: OperatingMode::ROL, addr_mode: AddressingMode::ZPG },
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "PLP", cycles: 4, op_mode: OperatingMode::PLP, addr_mode: AddressingMode::IMP },
        Instruction { name: "AND", cycles: 2, op_mode: OperatingMode::AND, addr_mode: AddressingMode::IMM },
        Instruction { name: "ROL", cycles: 2, op_mode: OperatingMode::ROL, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "BIT", cycles: 4, op_mode: OperatingMode::BIT, addr_mode: AddressingMode::ABS },
        Instruction { name: "AND", cycles: 4, op_mode: OperatingMode::AND, addr_mode: AddressingMode::ABS },
        Instruction { name: "ROL", cycles: 6, op_mode: OperatingMode::ROL, addr_mode: AddressingMode::ABS },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "BMI", cycles: 2, op_mode: OperatingMode::BMI, addr_mode: AddressingMode::REL },
        Instruction { name: "AND", cycles: 5, op_mode: OperatingMode::AND, addr_mode: AddressingMode::IDY },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "AND", cycles: 4, op_mode: OperatingMode::AND, addr_mode: AddressingMode::ZPX },
        Instruction { name: "ROL", cycles: 6, op_mode: OperatingMode::ROL, addr_mode: AddressingMode::ZPX },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "SEC", cycles: 2, op_mode: OperatingMode::SEC, addr_mode: AddressingMode::IMP },
        Instruction { name: "AND", cycles: 4, op_mode: OperatingMode::AND, addr_mode: AddressingMode::ABY },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "AND", cycles: 4, op_mode: OperatingMode::AND, addr_mode: AddressingMode::ABX },
        Instruction { name: "ROL", cycles: 7, op_mode: OperatingMode::ROL, addr_mode: AddressingMode::ABX },
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "RTI", cycles: 6, op_mode: OperatingMode::RTI, addr_mode: AddressingMode::IMP },
        Instruction { name: "EOR", cycles: 6, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::IDX },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 3, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "EOR", cycles: 3, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::ZPG },
        Instruction { name: "LSR", cycles: 5, op_mode: OperatingMode::LSR, addr_mode: AddressingMode::ZPG },
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "PHA", cycles: 3, op_mode: OperatingMode::PHA, addr_mode: AddressingMode::IMP },
        Instruction { name: "EOR", cycles: 2, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::IMM },
        Instruction { name: "LSR", cycles: 2, op_mode: OperatingMode::LSR, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "JMP", cycles: 3, op_mode: OperatingMode::JMP, addr_mode: AddressingMode::ABS },
        Instruction { name: "EOR", cycles: 4, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::ABS },
        Instruction { name: "LSR", cycles: 6, op_mode: OperatingMode::LSR, addr_mode: AddressingMode::ABS },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "BVC", cycles: 2, op_mode: OperatingMode::BVC, addr_mode: AddressingMode::REL },
        Instruction { name: "EOR", cycles: 5, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::IDY },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "EOR", cycles: 4, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::ZPX },
        Instruction { name: "LSR", cycles: 6, op_mode: OperatingMode::LSR, addr_mode: AddressingMode::ZPX },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "CLI", cycles: 2, op_mode: OperatingMode::CLI, addr_mode: AddressingMode::IMP },
        Instruction { name: "EOR", cycles: 4, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::ABY },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "EOR", cycles: 4, op_mode: OperatingMode::EOR, addr_mode: AddressingMode::ABX },
        Instruction { name: "LSR", cycles: 7, op_mode: OperatingMode::LSR, addr_mode: AddressingMode::ABX },
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "RTS", cycles: 6, op_mode: OperatingMode::RTS, addr_mode: AddressingMode::IMP },
        Instruction { name: "ADC", cycles: 6, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::IDX },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 3, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "ADC", cycles: 3, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::ZPG },
        Instruction { name: "ROR", cycles: 5, op_mode: OperatingMode::ROR, addr_mode: AddressingMode::ZPG },
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "PLA", cycles: 4, op_mode: OperatingMode::PLA, addr_mode: AddressingMode::IMP },
        Instruction { name: "ADC", cycles: 2, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::IMM },
        Instruction { name: "ROR", cycles: 2, op_mode: OperatingMode::ROR, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "JMP", cycles: 5, op_mode: OperatingMode::JMP, addr_mode: AddressingMode::IND },
        Instruction { name: "ADC", cycles: 4, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::ABS },
        Instruction { name: "ROR", cycles: 6, op_mode: OperatingMode::ROR, addr_mode: AddressingMode::ABS },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "BVS", cycles: 2, op_mode: OperatingMode::BVS, addr_mode: AddressingMode::REL },
        Instruction { name: "ADC", cycles: 5, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::IDY },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "ADC", cycles: 4, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::ZPX },
        Instruction { name: "ROR", cycles: 6, op_mode: OperatingMode::ROR, addr_mode: AddressingMode::ZPX },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "SEI", cycles: 2, op_mode: OperatingMode::SEI, addr_mode: AddressingMode::IMP },
        Instruction { name: "ADC", cycles: 4, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::ABY },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "ADC", cycles: 4, op_mode: OperatingMode::ADC, addr_mode: AddressingMode::ABX },
        Instruction { name: "ROR", cycles: 7, op_mode: OperatingMode::ROR, addr_mode: AddressingMode::ABX },
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "STA", cycles: 6, op_mode: OperatingMode::STA, addr_mode: AddressingMode::IDX },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "STY", cycles: 3, op_mode: OperatingMode::STY, addr_mode: AddressingMode::ZPG },
        Instruction { name: "STA", cycles: 3, op_mode: OperatingMode::STA, addr_mode: AddressingMode::ZPG },
        Instruction { name: "STX", cycles: 3, op_mode: OperatingMode::STX, addr_mode: AddressingMode::ZPG },
        Instruction { name: "XXX", cycles: 3, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "DEY", cycles: 2, op_mode: OperatingMode::DEY, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "TXA", cycles: 2, op_mode: OperatingMode::TXA, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "STY", cycles: 4, op_mode: OperatingMode::STY, addr_mode: AddressingMode::ABS },
        Instruction { name: "STA", cycles: 4, op_mode: OperatingMode::STA, addr_mode: AddressingMode::ABS },
        Instruction { name: "STX", cycles: 4, op_mode: OperatingMode::STX, addr_mode: AddressingMode::ABS },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "BCC", cycles: 2, op_mode: OperatingMode::BCC, addr_mode: AddressingMode::REL },
        Instruction { name: "STA", cycles: 6, op_mode: OperatingMode::STA, addr_mode: AddressingMode::IDY },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "STY", cycles: 4, op_mode: OperatingMode::STY, addr_mode: AddressingMode::ZPX },
        Instruction { name: "STA", cycles: 4, op_mode: OperatingMode::STA, addr_mode: AddressingMode::ZPX },
        Instruction { name: "STX", cycles: 4, op_mode: OperatingMode::STX, addr_mode: AddressingMode::ZPY },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "TYA", cycles: 2, op_mode: OperatingMode::TYA, addr_mode: AddressingMode::IMP },
        Instruction { name: "STA", cycles: 5, op_mode: OperatingMode::STA, addr_mode: AddressingMode::ABY },
        Instruction { name: "TXS", cycles: 2, op_mode: OperatingMode::TXS, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "STA", cycles: 5, op_mode: OperatingMode::STA, addr_mode: AddressingMode::ABX },
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "LDY", cycles: 2, op_mode: OperatingMode::LDY, addr_mode: AddressingMode::IMM },
        Instruction { name: "LDA", cycles: 6, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::IDX },
        Instruction { name: "LDX", cycles: 2, op_mode: OperatingMode::LDX, addr_mode: AddressingMode::IMM },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "LDY", cycles: 3, op_mode: OperatingMode::LDY, addr_mode: AddressingMode::ZPG },
        Instruction { name: "LDA", cycles: 3, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::ZPG },
        Instruction { name: "LDX", cycles: 3, op_mode: OperatingMode::LDX, addr_mode: AddressingMode::ZPG },
        Instruction { name: "XXX", cycles: 3, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "TAY", cycles: 2, op_mode: OperatingMode::TAY, addr_mode: AddressingMode::IMP },
        Instruction { name: "LDA", cycles: 2, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::IMM },
        Instruction { name: "TAX", cycles: 2, op_mode: OperatingMode::TAX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "LDY", cycles: 4, op_mode: OperatingMode::LDY, addr_mode: AddressingMode::ABS },
        Instruction { name: "LDA", cycles: 4, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::ABS },
        Instruction { name: "LDX", cycles: 4, op_mode: OperatingMode::LDX, addr_mode: AddressingMode::ABS },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "BCS", cycles: 2, op_mode: OperatingMode::BCS, addr_mode: AddressingMode::REL },
        Instruction { name: "LDA", cycles: 5, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::IDY },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "LDY", cycles: 4, op_mode: OperatingMode::LDY, addr_mode: AddressingMode::ZPX },
        Instruction { name: "LDA", cycles: 4, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::ZPX },
        Instruction { name: "LDX", cycles: 4, op_mode: OperatingMode::LDX, addr_mode: AddressingMode::ZPY },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "CLV", cycles: 2, op_mode: OperatingMode::CLV, addr_mode: AddressingMode::IMP },
        Instruction { name: "LDA", cycles: 4, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::ABY },
        Instruction { name: "TSX", cycles: 2, op_mode: OperatingMode::TSX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "LDY", cycles: 4, op_mode: OperatingMode::LDY, addr_mode: AddressingMode::ABX },
        Instruction { name: "LDA", cycles: 4, op_mode: OperatingMode::LDA, addr_mode: AddressingMode::ABX },
        Instruction { name: "LDX", cycles: 4, op_mode: OperatingMode::LDX, addr_mode: AddressingMode::ABY },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "CPY", cycles: 2, op_mode: OperatingMode::CPY, addr_mode: AddressingMode::IMM },
        Instruction { name: "CMP", cycles: 6, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::IDX },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "CPY", cycles: 3, op_mode: OperatingMode::CPY, addr_mode: AddressingMode::ZPG },
        Instruction { name: "CMP", cycles: 3, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::ZPG },
        Instruction { name: "DEC", cycles: 5, op_mode: OperatingMode::DEC, addr_mode: AddressingMode::ZPG },
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "INY", cycles: 2, op_mode: OperatingMode::INY, addr_mode: AddressingMode::IMP },
        Instruction { name: "CMP", cycles: 2, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::IMM },
        Instruction { name: "DEX", cycles: 2, op_mode: OperatingMode::DEX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "CPY", cycles: 4, op_mode: OperatingMode::CPY, addr_mode: AddressingMode::ABS },
        Instruction { name: "CMP", cycles: 4, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::ABS },
        Instruction { name: "DEC", cycles: 6, op_mode: OperatingMode::DEC, addr_mode: AddressingMode::ABS },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "BNE", cycles: 2, op_mode: OperatingMode::BNE, addr_mode: AddressingMode::REL },
        Instruction { name: "CMP", cycles: 5, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::IDY },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "CMP", cycles: 4, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::ZPX },
        Instruction { name: "DEC", cycles: 6, op_mode: OperatingMode::DEC, addr_mode: AddressingMode::ZPX },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "CLD", cycles: 2, op_mode: OperatingMode::CLD, addr_mode: AddressingMode::IMP },
        Instruction { name: "CMP", cycles: 4, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::ABY },
        Instruction { name: "NOP", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "CMP", cycles: 4, op_mode: OperatingMode::CMP, addr_mode: AddressingMode::ABX },
        Instruction { name: "DEC", cycles: 7, op_mode: OperatingMode::DEC, addr_mode: AddressingMode::ABX },
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "CPX", cycles: 2, op_mode: OperatingMode::CPX, addr_mode: AddressingMode::IMM },
        Instruction { name: "SBC", cycles: 6, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::IDX },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "CPX", cycles: 3, op_mode: OperatingMode::CPX, addr_mode: AddressingMode::ZPG },
        Instruction { name: "SBC", cycles: 3, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::ZPG },
        Instruction { name: "INC", cycles: 5, op_mode: OperatingMode::INC, addr_mode: AddressingMode::ZPG },
        Instruction { name: "XXX", cycles: 5, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "INX", cycles: 2, op_mode: OperatingMode::INX, addr_mode: AddressingMode::IMP },
        Instruction { name: "SBC", cycles: 2, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::IMM },
        Instruction { name: "NOP", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::IMP },
        Instruction { name: "CPX", cycles: 4, op_mode: OperatingMode::CPX, addr_mode: AddressingMode::ABS },
        Instruction { name: "SBC", cycles: 4, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::ABS },
        Instruction { name: "INC", cycles: 6, op_mode: OperatingMode::INC, addr_mode: AddressingMode::ABS },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "BEQ", cycles: 2, op_mode: OperatingMode::BEQ, addr_mode: AddressingMode::REL },
        Instruction { name: "SBC", cycles: 5, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::IDY },
        Instruction { name: "XXX", cycles: 2, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 8, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "SBC", cycles: 4, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::ZPX },
        Instruction { name: "INC", cycles: 6, op_mode: OperatingMode::INC, addr_mode: AddressingMode::ZPX },
        Instruction { name: "XXX", cycles: 6, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "SED", cycles: 2, op_mode: OperatingMode::SED, addr_mode: AddressingMode::IMP },
        Instruction { name: "SBC", cycles: 4, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::ABY },
        Instruction { name: "NOP", cycles: 2, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
        Instruction { name: "XXX", cycles: 4, op_mode: OperatingMode::NOP, addr_mode: AddressingMode::IMP },
        Instruction { name: "SBC", cycles: 4, op_mode: OperatingMode::SBC, addr_mode: AddressingMode::ABX },
        Instruction { name: "INC", cycles: 7, op_mode: OperatingMode::INC, addr_mode: AddressingMode::ABX },
        Instruction { name: "XXX", cycles: 7, op_mode: OperatingMode::XXX, addr_mode: AddressingMode::IMP },
    ];
}
