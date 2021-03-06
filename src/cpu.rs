use crate::bus::Bus;
use crate::model::*;

/// Core data structure for emulating a 6502 CPU.
pub struct CPU<'a> {
    /// A hook to the bus, which serves as our interface through which we can read from and write to
    /// memory. This includes retrieving opcodes and data for instruction processing.
    bus: Box<&'a mut Bus>,
    // The accumulator register.
    pub a: u8,
    // The X register.
    pub x: u8,
    // The Y register.
    pub y: u8,
    // The stack pointer.
    pub stk_ptr: u8,
    // The program counter.
    pub p_ctr: u16,
    /// The status code. This is a bitmask composed of a number of flags, each of which serves to
    /// surface some meaning about the state of the CPU and its last executed instruction.
    /// See: model::CPUStatus
    status: CPUStatus,
    // A reference bank for data fetched from memory.
    m: u8,
    // The address (absolute) from which we'll fetch data from memory.
    addr: u16,
    /// The address (relative) from which we'll fetch data from memory.
    /// Not all operations have access to the full memory bank; some can only make jumps from the
    /// current memory address. This register stores that input.
    addr_rel: u16,
    // The current opcode input.
    opcode: u8,
    // The number of cycles left to execute for the current command.
    cycles: u8,
    // Addressing mode for the current instruction.
    addr_mode: Option<AddressingMode>,
    // Operating mode for the current instruction.
    op_mode: Option<OperatingMode>,
}

impl<'a> CPU<'a> {
    /// Construct a new CPU instance with the given Bus hook.
    pub fn new(bus: &'a mut Bus) -> CPU<'a> {
        CPU {
            bus:       Box::new(bus),
            a:         0x00,
            x:         0x00,
            y:         0x00,
            stk_ptr:   0xFD,
            p_ctr:     0x0000,
            status:    CPUStatus::default(),
            m:         0x00,
            addr:      0x0000,
            addr_rel:  0x00,
            opcode:    0x00,
            cycles:    0,
            addr_mode: None,
            op_mode:   None,
        }
    }

    /// Iterate forward one clock cycle. Accept the input associated with that clock
    /// cycle and act on it.
    pub fn clock(&mut self) {
        if self.cycles == 0 {
            self.opcode = self.read(self.p_ctr);
            self.p_ctr += 1;

            let instruction = &INSTRUCTIONS[self.opcode as usize];
            println!("Clock cycle found {:?}", instruction);

            self.cycles = instruction.cycles;
            self.addr_mode = Some(instruction.addr_mode);
            self.op_mode = Some(instruction.op_mode);

            // Execute the instruction. If at least one additional cycle is needed,
            // then increment the cycles counter.
            let addrmode_cycles = get_addressing_mode(instruction.addr_mode)(self);
            let opmode_cycles = get_operating_mode(instruction.op_mode)(self);
            self.cycles += addrmode_cycles & opmode_cycles;
        }

        self.cycles -= 1;
    }

    /// Request an interrupt. This can be ignored if the I flag on the status code is set.
    pub fn interrupt(&mut self) {
        if self.status.contains(CPUStatus::I) {
            return;
        }

        self.interrupt_and_set_address_to(0xFFFE);
    }

    /// Demand an interrupt. This one cannot be ignored.
    /// This code is *mostly* equivalent to irq, with the only difference being the address
    /// that we read to determine the value for the program counter.
    pub fn interrupt_no_mask(&mut self) {
        self.interrupt_and_set_address_to(0xFFFA);
    }

    /// Reset the CPU to a known state:
    /// 
    /// * All registers (A, X, Y, and M) are set to 0
    /// * The stack pointer is set to address 0x00FD
    /// * The status code is set to 00100000 (Unused)
    /// * The program counter is set to the value at address 0xFFFD | 0xFFFC
    /// * The absolute and relative addresses are set to 0
    /// * The cycle count is set to 8 (because this operation takes time)
    pub fn reset(&mut self) {
        self.a = 0x00;
        self.x = 0x00;
        self.y = 0x00;
        self.stk_ptr = 0xFD;
        self.status = CPUStatus::U; // 00100000
        self.addr = 0xFFFC;

        let lo: u16 = self.read(self.addr) as u16;
        let hi: u16 = self.read(self.addr + 1) as u16;
        self.p_ctr = (hi << 8) | lo;

        self.addr = 0x0000;
        self.addr_rel = 0x0000;
        self.m = 0x00;
        self.cycles = 8;
    }

    /// Fetch the data at the currently specified address and save it into the M register.
    /// Strictly speaking, this does not need to return a value. But it's easy, and potentially
    /// convenient.
    pub fn fetch(&mut self) -> u8 {
        // Check the current address mode; if it is not set to IMP, read a value and cache it
        // for later operation by an instruction.
        if self.addr_mode.unwrap_or(AddressingMode::ABS) != AddressingMode::IMP {
            self.m = self.read(self.addr);
        }

        self.m
    }

    // -- Addressing Modes -- //

    /// Absolute addressing. A full 16-bit address is provided as input, with the offset specified
    /// first and the page specified second.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/addressing.html#ABS
    fn abs(&mut self) -> u8 {
        self.read_address_with_value_offset(0)
    }

    /// Absolute addressing, offset by X. This is the same as ABS, but the result address is
    /// incremented by the value in register X.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/addressing.html#ABX
    fn abx(&mut self) -> u8 {
        self.read_address_with_value_offset(self.x as u16)
    }

    /// Absolute addressing, offset by Y. This is the same as ABS, but the result address is
    /// incremented by the value in register Y.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/addressing.html#ABY
    fn aby(&mut self) -> u8 {
        self.read_address_with_value_offset(self.y as u16)
    }

    /// Implicit addressing. Load the accumulator into memory as input. The value may not
    /// actually be used, but that's not the addressing mode's problem.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/addressing.html#IMP
    fn imp(&mut self) -> u8 {
        self.m = self.a;

        0
    }

    /// Immediate addressing. An additional byte is provided as part of the input and should
    /// be treated as data on which to operate, not as part of an address which points to data.
    /// 
    /// e.g.,
    /// ```assembly
    /// LDA #10         ;Load 10 ($0A) into the accumulator
    /// LDX #LO LABEL   ;Load the LSB of a 16 bit address into X
    /// LDY #HI LABEL   ;Load the MSB of a 16 bit address into Y
    /// ```
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/addressing.html#IMM
    fn imm(&mut self) -> u8 {
        self.a = self.p_ctr as u8;
        self.p_ctr += 1;

        0
    }

    /// Indirect addressing. The instruction contains a 16-bit address identifying the location
    /// of the least significant byte of some other 16-bit address in memory which is the true
    /// target of the instruction. This is essentially an implementation of memory pointers.
    /// 
    /// Note that this is ONLY supported by JMP.
    /// 
    /// The original hardware here is actually bugged; we simulate that bug for authenticity.
    /// If the low byte of the supplied address is 0xFF, then reading the high byte of the actual
    /// address requires crossing a page boundary. On the actual chip, what happens is we wrap back
    /// around to the same page, which yields an invalid actual address. Most programmers knew
    /// about this bug and designed around it.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/addressing.html#IND
    fn ind(&mut self) -> u8 {
        let (pointer, _, lo) = self.read_address_from_bus();
        if lo == 0x00FF {
            self.addr = ((self.read(pointer & 0xFF00) as u16) << 8) | self.read(pointer) as u16;
        } else {
            self.addr = ((self.read(pointer + 1) as u16) << 8) | self.read(pointer) as u16;
        }

        0
    }

    /// Indexed indirect addressing. Similar to indirect addressing, but the value in register X
    /// is added to it. This is useful for representing tables of data. In such a case, the address
    /// of the table is what is provided to the instruction, and the X register is added to simulate
    /// indexing (with zero page wrap-around).
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/addressing.html#IDX
    fn idx(&mut self) -> u8 {
        let (p, _, _) = self.read_address_from_bus();
        let pointer = p + self.x as u16;

        let lo: u16 = self.read(pointer & 0x00FF) as u16;
        let hi: u16 = self.read((pointer + 1) & 0x00FF) as u16;
        self.addr = (hi << 8) | lo;

        0
    }

    /// Indirect indexed addressing. Similar to indirect addressing, but does not match the X
    /// complement. Instead of incrementing the address by Y and then loading that address, we
    /// instead load the address and then increment the resulting value by Y.
    /// 
    /// If the incrementing requires a page change, then we signal for an additional clock cycle.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/addressing.html#IDY
    fn idy(&mut self) -> u8 {
        let (pointer, _, _) = self.read_address_from_bus();
    
        let lo: u16 = self.read(pointer & 0x00FF) as u16;
        let hi: u16 = self.read((pointer + 1) & 0x00FF) as u16;
        self.addr = ((hi << 8) | lo) + (self.y as u16);

        ((self.addr & 0xFF00) != (hi << 8)) as u8
    }

    /// Relative addressing. A signed 8-bit relative offset is provided as input to be loaded.
    /// This is used commonly by Branching instructions.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/addressing.html#REL
    fn rel(&mut self) -> u8 {
        self.addr_rel = self.read(self.p_ctr) as u16;
        self.p_ctr += 1;

        // The input byte can be negative, which represents a backward jump. In order to represent
        // that as a full 16-bit address, we just set the page byte of the address to 31 (0xFF).
        // e.g.,
        //      Input byte: 0x83 (0000000010000011)
        //      i & 0x80 = (0000000010000011) & (0000000010000000) = 0000000010000000 > 0
        //      i | 0xFF00 = (10000011) | (1111111100000000) = 1111111110000011 (jump back by 3)
        if self.addr_rel & 0x80 > 0 {
            self.addr_rel |= 0xFF00;
        }

        0
    }

    /// Zero Page addressing with 0 offset. This addressing mode (and its variants ZPX and ZPY)
    /// are useful in that they allow programmers to cut down on byte code, since the only
    /// element of input is an offset on page zero.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/addressing.html#ZPG
    /// Paging Documentation: https://en.wikipedia.org/wiki/Paging
    fn zpg(&mut self) -> u8 {
        self.zero_page(0)
    }

    /// Zero Page addressing offset by the value in the X register.
    /// This is useful for e.g., looping through successive addresses.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/addressing.html#ZPX
    fn zpx(&mut self) -> u8 {
        self.zero_page(self.x)
    }

    /// Zero Page addressing offset by the value in the Y register.
    /// This is useful for e.g., looping through successive addresses.
    /// 
    /// This mode can ONLY be used with instructions LDX and STX.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/addressing.html#ZPY
    fn zpy(&mut self) -> u8 {
        self.zero_page(self.y)
    }

    // -- Operation Modes -- //
    
    /// Add the contents of a memory address to the accumulator, together with the carry bit.
    /// If an overflow occurs, we set the carry bit for enabling multi-byte addition.
    /// 
    /// A,Z,C,N,V = A + M + C
    /// 
    /// Status flags set:
    /// * C - if overflow in bit 7
    /// * Z - if A == 0
    /// * V - if sign bit is incorrect
    /// * N - if bit 7 of A is set
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#ADC
    /// 
    /// Examples:
    /// 
    /// A = 11000000
    /// M = 01100010
    /// C = 0
    /// 
    ///   ------------- This bit takes the carry from the prior bit, which results in
    ///   |             01 + 01 + 00 == 10; since this is bit 7, this would result in
    ///   |             the C flag being set to 1.
    ///   |------------ This bit sums to 10, so carry 1 to the next bit
    ///   vv
    ///   11000000
    /// + 01100010
    /// ----------
    ///  100100010
    ///  ^
    ///  -------------- Overflow here, so C = 1
    /// 
    /// A = 00000000
    /// M = 00000000
    /// C = 1
    /// 
    ///   00000000
    ///   00000000
    /// + 00000001 <- Just to illustrate that C is always treated as bit 0
    /// ----------
    ///   00000001
    /// 
    /// How do we determine the value of the overflow flag? Consider the following truth
    /// table of bit 7 from each value in the computation (ignoring C, which is always 0):
    /// 
    /// | A | M | R || V || A^R | A^M | !(A^M) |
    /// |---|---|---||---||:---:|:---:|:------:|
    /// | 0 | 0 | 0 || 0 ||  0  |  0  |    1   |
    /// | 0 | 0 | 1 || 1 ||  1  |  0  |    1   |
    /// | 0 | 1 | 0 || 0 ||  0  |  1  |    0   |
    /// | 0 | 1 | 1 || 0 ||  1  |  1  |    0   |
    /// | 1 | 0 | 0 || 0 ||  1  |  1  |    0   |
    /// | 1 | 0 | 1 || 0 ||  0  |  1  |    0   |
    /// | 1 | 1 | 0 || 1 ||  1  |  0  |    1   |
    /// | 1 | 1 | 1 || 0 ||  0  |  0  |    1   |
    /// 
    /// Thus, V = (A^R) & !(A^M).
    fn adc(&mut self) -> u8 {
        self.fetch();

        let a = self.a as u16;
        let m = self.m as u16;
        let c = self.status.contains(CPUStatus::C) as u16;

        let r = a + m + c;
        self.set_status_flag(CPUStatus::C, r > 255); // Exceeds the bounds of 8-bit integers
        self.set_status_flag(CPUStatus::Z, (r as u8) == 0); // Check only the 8-bit result
        self.set_status_flag(CPUStatus::N, r & (1 << 7) > 0);

        // See above for explanation on this formula.
        // Remember that we only care about bit 7 of the result!
        self.set_status_flag(CPUStatus::V, (a ^ r) & !(a ^ m) & (1 << 7) != 0);
        self.a = r as u8;

        1
    }

    /// Logical AND.
    /// 
    /// A,Z,N = A & M
    /// 
    /// Status flags set:
    /// * Z - if A == 0
    /// * N - if bit 7 of A is set
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#AND
    fn and(&mut self) -> u8 {
        self.fetch();

        self.a &= self.m;
        self.set_status_flag(CPUStatus::Z, self.a == 0);
        self.set_status_flag(CPUStatus::N, self.a & (1 << 7) > 0);

        1
    }

    /// Arithmetic bit shift left. Shifts the contents of either A or M one bit
    /// to the left (i.e., multiplies by 2). Bit 0 is always set to 0. The result
    /// is either stored back to A (if we are using IMP addressing mode) or
    /// written to memory (otherwise).
    /// 
    /// IMP: A,Z,C,N = M << 1
    /// else: M,Z,C,N = M << 1
    /// 
    /// Status flags set:
    /// * C - the contents of bit 7 before the shift
    /// * Z - if the shift result == 0
    /// * N - if bit 7 of the shift result is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#ASL
    fn asl(&mut self) -> u8 {
        self.fetch();

        let shift = self.m << 1;
        self.set_status_flag(CPUStatus::C, self.m & (1 << 7) > 0);
        self.set_status_flag(CPUStatus::Z, shift as u8 == 0);
        self.set_status_flag(CPUStatus::N, shift & (1 << 7) > 0);

        if self.addr_mode.unwrap_or(AddressingMode::IMP) == AddressingMode::IMP {
            self.a = shift;
        } else {
            self.write(self.addr, shift);
        }

        0
    }

    /// Branch if Carry clear. If the carry flag is clear, then add the relative
    /// address displacement to the program counter and jump to that location.
    /// 
    /// No status flags affected.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#BCC
    fn bcc(&mut self) -> u8 {
        self.branch_on_flag(CPUStatus::C, false)
    }

    /// Branch if Carry set. If the carry flag is set, then add the relative
    /// address displacement to the program counter and jump to that location.
    /// 
    /// No status flags affected.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#BCS
    fn bcs(&mut self) -> u8 {
        self.branch_on_flag(CPUStatus::C, true)
    }

    /// Branch if Zero set. If the zero flag is set, then add the relative
    /// address displacement to the program counter and jump to that location.
    /// 
    /// No status flags affected.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#BEQ
    fn beq(&mut self) -> u8 {
        self.branch_on_flag(CPUStatus::Z, true)
    }

    /// Bit test. Tests if one or more bits are set in a target memory location.
    /// The result of A & M is not stored.
    /// 
    /// A & M, N = M7, V = M6
    /// 
    /// Status flags set:
    /// * Z - if the AND result is 0
    /// * V - equal to bit 6 of M
    /// * N - equal to bit 7 of M
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#BIT
    fn bit(&mut self) -> u8 {
        self.fetch();

        self.set_status_flag(CPUStatus::Z, self.a & self.m == 0);
        self.set_status_flag(CPUStatus::V, self.m & (1 << 6) > 0);
        self.set_status_flag(CPUStatus::N, self.m & (1 << 7) > 0);

        0
    }

    /// Branch if Negative set. If the negative flag is set, then add the relative
    /// address displacement to the program counter and jump to that location.
    /// 
    /// No status flags affected.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#BMI
    fn bmi(&mut self) -> u8 {
        self.branch_on_flag(CPUStatus::N, true)
    }

    /// Branch if Zero clear. If the zero flag is clear, then add the relative
    /// address displacement to the program counter and jump to that location.
    /// 
    /// No status flags affected.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#BNE
    fn bne(&mut self) -> u8 {
        self.branch_on_flag(CPUStatus::Z, false)
    }

    /// Branch if Negative clear. If the negative flag is clear, then add the relative
    /// address displacement to the program counter and jump to that location.
    /// 
    /// No status flags affected.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#BPL
    fn bpl(&mut self) -> u8 {
        self.branch_on_flag(CPUStatus::N, false)
    }

    /// Forces the generation of an Interrupt request. PC and S are pushed onto the
    /// stack, PC is set to the IRQ interrupt vector at $FFFE..F, and the break flag
    /// in S is set to 1.
    /// 
    /// Note that the B flag *technically* does not exist in the status register. It
    /// only exists in the status flag byte pushed to the stack, and that value should
    /// be discarded when it is recovered from the stack.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#BRK
    /// Further reading: http://nesdev.com/the%20%27B%27%20flag%20&%20BRK%20instruction.txt
    fn brk(&mut self) -> u8 {
        self.p_ctr += 1;
        self.push_to_stack((self.p_ctr >> 8) as u8);
        self.push_to_stack(self.p_ctr as u8);

        // Set the status flag byte before pushing to the stack, then reset to false.
        self.set_status_flag(CPUStatus::B, true);
        self.push_to_stack(self.status.bits());
        self.set_status_flag(CPUStatus::B, false);

        let lo = self.read(0xFFFE) as u16;
        let hi = self.read(0xFFFF) as u16;
        self.p_ctr = (hi << 8) | lo;

        0
    }

    /// Branch if Overflow clear. If the overflow flag is clear, then add the relative
    /// address displacement to the program counter and jump to that location.
    /// 
    /// No status flags affected.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#BVC
    fn bvc(&mut self) -> u8 {
        self.branch_on_flag(CPUStatus::V, false)
    }

    /// Branch if Overflow set. If the overflow flag is clear, then add the relative
    /// address displacement to the program counter and jump to that location.
    /// 
    /// No status flags affected.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#BVS
    fn bvs(&mut self) -> u8 {
        self.branch_on_flag(CPUStatus::V, true)
    }

    /// Clear Carry flag.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#CLC
    fn clc(&mut self) -> u8 {
        self.set_status_flag(CPUStatus::C, false);

        0
    }

    /// Clear Decimal flag.
    /// 
    /// From the docs: The state of the decimal flag is uncertain when the CPU
    /// is powered up, and it is not reset when an interrupt is generated. In
    /// either case, the programmer should include an explicit CLD to ensure
    /// that the flag is cleared before performing any addition/subtraction.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#CLD
    fn cld(&mut self) -> u8 {
        self.set_status_flag(CPUStatus::D, false);

        0
    }

    /// Clear Interrupt disabling flag.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#CLI
    fn cli(&mut self) -> u8 {
        self.set_status_flag(CPUStatus::I, false);

        0
    }

    /// Clear Overflow flag.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#CLV
    fn clv(&mut self) -> u8 {
        self.set_status_flag(CPUStatus::V, false);

        0
    }

    /// Compare M and A via subtraction, setting Z and C as appropriate.
    /// 
    /// Z,C,N = A - M
    /// 
    /// Status flags affected:
    /// * C - if A >= M
    /// * Z - if A == M
    /// * N - if bit 7 of the result is set
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#CMP
    fn cmp(&mut self) -> u8 {
        self.fetch();

        let r = self.a - self.m;
        self.set_status_flag(CPUStatus::C, self.a >= self.m);
        self.set_status_flag(CPUStatus::Z, self.a == self.m);
        self.set_status_flag(CPUStatus::N, r & (1 << 7) > 0);

        1
    }

    /// Compare M and A via subtraction, setting Z and C as appropriate.
    /// 
    /// Z,C,N = A - M
    /// 
    /// Status flags affected:
    /// * C - if A >= M
    /// * Z - if A == M
    /// * N - if bit 7 of the result is set
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#CPY
    fn cpx(&mut self) -> u8 {
        self.fetch();

        let r = self.x - self.m;
        self.set_status_flag(CPUStatus::C, self.x >= self.m);
        self.set_status_flag(CPUStatus::Z, self.x == self.m);
        self.set_status_flag(CPUStatus::N, r & (1 << 7) > 0);

        1
    }

    /// Compare M and A via subtraction, setting Z and C as appropriate.
    /// 
    /// Z,C,N = A - M
    /// 
    /// Status flags affected:
    /// * C - if A >= M
    /// * Z - if A == M
    /// * N - if bit 7 of the result is set
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#CPX
    fn cpy(&mut self) -> u8 {
        self.fetch();

        let r = self.y - self.m;
        self.set_status_flag(CPUStatus::C, self.y >= self.m);
        self.set_status_flag(CPUStatus::Z, self.y == self.m);
        self.set_status_flag(CPUStatus::N, r & (1 << 7) > 0);

        1
    }

    /// Decrement a value from memory.
    /// 
    /// M,Z,N = M - 1
    /// 
    /// Status flags set:
    /// * Z - if M == 0
    /// * N - if bit 7 of M is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#DEC
    fn dec(&mut self) -> u8 {
        self.fetch();
        self.m -= 1;
        self.write(self.addr, self.m);

        self.set_status_flag(CPUStatus::Z, self.m == 0);
        self.set_status_flag(CPUStatus::N, self.m & (1 << 7) > 0);
        
        0
    }

    /// Decrement the value in the X register.
    /// 
    /// X,Z,N = X - 1
    /// 
    /// Status flags set:
    /// * Z - if X == 0
    /// * N - if bit 7 of X is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#DEX
    fn dex(&mut self) -> u8 {
        self.x -= 1;
        self.set_status_flag(CPUStatus::Z, self.x == 0);
        self.set_status_flag(CPUStatus::N, self.x & (1 << 7) > 0);

        0
    }

    /// Decrement the value in the Y register.
    /// 
    /// Y,Z,N = Y - 1
    /// 
    /// Status flags set:
    /// * Z - if Y == 0
    /// * N - if bit 7 of Y is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#DEY
    fn dey(&mut self) -> u8 {
        self.y -= 1;
        self.set_status_flag(CPUStatus::Z, self.y == 0);
        self.set_status_flag(CPUStatus::N, self.y & (1 << 7) > 0);

        0
    }

    /// Logical XOR.
    /// 
    /// A,Z,N = A ^ M
    /// 
    /// Status flags set:
    /// * Z - if A == 0
    /// * N - if bit 7 of A is set
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#EOR
    fn eor(&mut self) -> u8 {
        self.fetch();

        self.a ^= self.m;
        self.set_status_flag(CPUStatus::Z, self.a == 0);
        self.set_status_flag(CPUStatus::N, self.a & (1 << 7) > 0);

        1
    }

    /// Increment a value from memory.
    /// 
    /// M,Z,N = M + 1
    /// 
    /// Status flags set:
    /// * Z - if M == 0
    /// * N - if bit 7 of M is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#INC
    fn inc(&mut self) -> u8 {
        self.fetch();
        self.m += 1;
        self.write(self.addr, self.m);

        self.set_status_flag(CPUStatus::Z, self.m == 0);
        self.set_status_flag(CPUStatus::N, self.m & (1 << 7) > 0);
        
        0
    }

    /// Increment the value in the X register.
    /// 
    /// X,Z,N = X + 1
    /// 
    /// Status flags set:
    /// * Z - if X == 0
    /// * N - if bit 7 of X is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#INX
    fn inx(&mut self) -> u8 {
        self.x += 1;
        self.set_status_flag(CPUStatus::Z, self.x == 0);
        self.set_status_flag(CPUStatus::N, self.x & (1 << 7) > 0);

        0
    }

    /// Increment the value in the Y register.
    /// 
    /// Y,Z,N = Y + 1
    /// 
    /// Status flags set:
    /// * Z - if Y == 0
    /// * N - if bit 7 of Y is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#INY
    fn iny(&mut self) -> u8 {
        self.y += 1;
        self.set_status_flag(CPUStatus::Z, self.y == 0);
        self.set_status_flag(CPUStatus::N, self.y & (1 << 7) > 0);

        0
    }

    /// Set the program counter to the loaded address.
    /// 
    /// No status flags set.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#JMP
    fn jmp(&mut self) -> u8 {
        self.p_ctr = self.addr;

        0
    }

    /// Set the program counter to the loaded address, and save
    /// the old program counter on the stack.
    /// 
    /// No status flags set.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#JSR
    fn jsr(&mut self) -> u8 {
        // The addressing mode for any instruction which uses this
        // operating mode should increment the program counter in
        // some form. Thus, we need to decrement it to ensure that
        // we store the correct value.
        self.p_ctr -= 1;

        // Push the bytes of PC onto the stack in high -> low order.
        // Reading back from the stack will be low -> high order.
        self.push_to_stack((self.p_ctr >> 8) as u8);
        self.push_to_stack(self.p_ctr as u8);
        self.p_ctr = self.addr;

        0
    }

    /// Load a byte of memory into A.
    /// 
    /// A,Z,N = M
    /// 
    /// Status flags set:
    /// * Z - if A == 0
    /// * N - if bit 7 of A is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#LDA
    fn lda(&mut self) -> u8 {
        self.fetch();

        self.a = self.m;
        self.set_status_flag(CPUStatus::Z, self.a == 0);
        self.set_status_flag(CPUStatus::N, self.a & (1 << 7) > 0);
        
        1
    }

    /// Load a byte of memory into X.
    /// 
    /// X,Z,N = M
    /// 
    /// Status flags set:
    /// * Z - if X == 0
    /// * N - if bit 7 of X is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#LDX
    fn ldx(&mut self) -> u8 {
        self.fetch();

        self.x = self.m;
        self.set_status_flag(CPUStatus::Z, self.x == 0);
        self.set_status_flag(CPUStatus::N, self.x & (1 << 7) > 0);
        
        1
    }

    /// Load a byte of memory into Y.
    /// 
    /// Y,Z,N = M
    /// 
    /// Status flags set:
    /// * Z - if A == 0
    /// * N - if bit 7 of A is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#LDY
    fn ldy(&mut self) -> u8 {
        self.fetch();

        self.y = self.m;
        self.set_status_flag(CPUStatus::Z, self.y == 0);
        self.set_status_flag(CPUStatus::N, self.y & (1 << 7) > 0);
        
        1
    }

    /// Logical bit shift right. Shifts the contents of either A or M one bit
    /// to the right (i.e., divides by 2). Bit 7 is always set to 0. The result
    /// is either stored back to A (if we are using IMP addressing mode) or
    /// written to memory (otherwise).
    /// 
    /// IMP: A,Z,C,N = M >> 1
    /// else: M,Z,C,N = M >> 1
    /// 
    /// Status flags set:
    /// * C - the contents of bit 0 before the shift
    /// * Z - if the shift result == 0
    /// * N - if bit 7 of the shift result is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#LSR
    fn lsr(&mut self) -> u8 {
        self.fetch();

        let shift = self.m >> 1;
        self.set_status_flag(CPUStatus::C, self.m & 1 > 0);
        self.set_status_flag(CPUStatus::Z, shift as u8 == 0);
        self.set_status_flag(CPUStatus::N, shift & (1 << 7) > 0);

        if self.addr_mode.unwrap_or(AddressingMode::IMP) == AddressingMode::IMP {
            self.a = shift;
        } else {
            self.write(self.addr, shift);
        }

        0
    }

    /// No-op.
    /// 
    /// Note that not all no-ops in the NES world are true no-ops:
    /// https://wiki.nesdev.com/w/index.php/CPU_unofficial_opcodes
    /// ^ TODO: Implement the above ^
    /// 
    /// References: http://www.obelisk.me.uk/6502/reference.html#NOP
    fn nop(&mut self) -> u8 {
        0
    }

    /// Logical OR.
    /// 
    /// A,Z,N = A | M
    /// 
    /// Status flags set:
    /// * Z - if A == 0
    /// * N - if bit 7 of A is set
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#ORA
    fn ora(&mut self) -> u8 {
        self.fetch();

        self.a |= self.m;
        self.set_status_flag(CPUStatus::Z, self.a == 0);
        self.set_status_flag(CPUStatus::N, self.a & (1 << 7) > 0);

        1
    }

    /// Push a copy of the accumulator onto the stack.
    /// 
    /// No status flags set.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#PHA
    /// 6502 Stack reference: https://wiki.nesdev.com/w/index.php/Stack
    fn pha(&mut self) -> u8 {
        self.push_to_stack(self.a);

        0
    }

    /// Push a copy of the status code onto the stack.
    /// 
    /// Note that the B flag is always set in the status code which is pushed.
    /// http://nesdev.com/the%20%27B%27%20flag%20&%20BRK%20instruction.txt
    /// 
    /// No status flags set.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#PHA
    /// 6502 Stack reference: https://wiki.nesdev.com/w/index.php/Stack
    fn php(&mut self) -> u8 {
        self.set_status_flag(CPUStatus::B, true);
        self.push_to_stack(self.status.bits());
        self.set_status_flag(CPUStatus::B, false);

        0
    }

    /// Pull from the stack and assign that value to the accumulator.
    /// 
    /// Status flags set:
    /// * Z - if A == 0
    /// * N - if bit 7 of A is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#PHA
    /// 6502 Stack reference: https://wiki.nesdev.com/w/index.php/Stack
    fn pla(&mut self) -> u8 {
        self.a = self.pull_from_stack();
        self.set_status_flag(CPUStatus::Z, self.a == 0);
        self.set_status_flag(CPUStatus::N, self.a & (1 << 7) > 0);

        0
    }

    /// Pull from the stack and assign that value to the status code.
    /// 
    /// Note that the B flag is always set in the status code which is pushed.
    /// Thus, we need to set it back to false after we retrieve it.
    /// http://nesdev.com/the%20%27B%27%20flag%20&%20BRK%20instruction.txt
    /// 
    /// No status flags directly set; they are set to the values from
    /// the stack.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#PHA
    /// 6502 Stack reference: https://wiki.nesdev.com/w/index.php/Stack
    fn plp(&mut self) -> u8 {
        self.status = CPUStatus::from_bits(self.pull_from_stack())
            .unwrap_or(CPUStatus::default());
        self.set_status_flag(CPUStatus::B, false);

        0
    }

    /// Rotate one bit to the left. Bit 0 is filled with the current value
    /// of the carry flag, and the carry flag is updated with the old bit 7.
    /// 
    /// Status flags set:
    /// * C - the contents of bit 7 before the rotation
    /// * Z - if the rotation result == 0
    /// * N - if bit 7 of the rotation result is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#ROL
    fn rol(&mut self) -> u8 {
        self.fetch();

        let shift = (self.m << 1) | self.status.contains(CPUStatus::C) as u8;
        self.set_status_flag(CPUStatus::C, self.m & (1 << 7) > 0);
        self.set_status_flag(CPUStatus::Z, shift == 0);
        self.set_status_flag(CPUStatus::N, shift & (1 << 7) > 0);
        if self.addr_mode.unwrap_or(AddressingMode::IMP) == AddressingMode::IMP {
            self.a = shift;
        } else {
            self.write(self.addr, shift);
        }

        0
    }

    /// Rotate one bit to the right. Bit 7 is filled with the current value
    /// of the carry flag, and the carry flag is updated with the old bit 0.
    /// 
    /// Status flags set:
    /// * C - the contents of bit 0 before the rotation
    /// * Z - if the rotation result == 0
    /// * N - if bit 7 of the rotation result is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#ROR
    fn ror(&mut self) -> u8 {
        self.fetch();

        let shift = (self.m >> 1) | ((self.status.contains(CPUStatus::C) as u8) << 7);
        self.set_status_flag(CPUStatus::C, self.m & 1 > 0);
        self.set_status_flag(CPUStatus::Z, shift == 0);
        self.set_status_flag(CPUStatus::N, shift & (1 << 7) > 0);
        if self.addr_mode.unwrap_or(AddressingMode::IMP) == AddressingMode::IMP {
            self.a = shift;
        } else {
            self.write(self.addr, shift);
        }

        0
    }
    
    /// Return from an interrupt routine. Pull the processor flags from the stack,
    /// then the program counter.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#RTI
    fn rti(&mut self) -> u8 {
        self.status = CPUStatus::from_bits(self.pull_from_stack())
            .unwrap_or(CPUStatus::default());
        self.set_status_flag(CPUStatus::B, false);
        self.set_status_flag(CPUStatus::U, false); // Just in case.

        let lo = self.pull_from_stack() as u16;
        let hi = self.pull_from_stack() as u16;
        self.p_ctr = (hi << 8) | lo;

        0
    }

    /// Pull the program counter from the value saved on the stack.
    /// 
    /// No status flags set.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#RTS
    fn rts(&mut self) -> u8 {
        let lo = self.pull_from_stack() as u16;
        let hi = self.pull_from_stack() as u16;

        // Add 1 back to the program counter, since it was decremented
        // during JSR.
        self.p_ctr = ((hi << 8) | lo) + 1;

        0
    }

    /// Subtract the contents of a memory address from the accumulator, together with the carry bit.
    /// If an overflow occurs, we clear the carry bit for enabling multi-byte subtraction.
    /// 
    /// A,Z,C,N,V = A - M - (1 - C)
    /// 
    /// Status flags set:
    /// * C - if overflow in bit 7
    /// * Z - if A == 0
    /// * V - if sign bit is incorrect
    /// * N - if bit 7 of A is set
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#SBC
    /// 
    /// Logical breakdown:
    /// 
    /// A - M - (1 - C) = A + (-1)(M + (1 - C))
    ///                 = A + (-1)(M) + (-1)(1 - C)
    ///                 = A + (-M) - 1 + C
    /// 
    /// Inverting a positive number in binary form to its negative magnitude is a twos-complement, e.g.:
    /// 
    ///  10 (decimal) = 00001010
    /// -10 (decimal) = 11110101 + 000000001
    ///               = 11110110 (in unsigned form, 246)
    /// 
    /// Easy check for this is that 246 + 10 = 256, which is the full range of integers representable
    /// with 8 bits. As a further example, if we were to perform an addition in unsigned form and
    /// modulate the result into 8-bit form, then:
    ///
    /// (25 + 246) % 256 = 271 % 256
    ///                  = 15
    ///                  = 25 - 10
    /// 
    /// Now, note above that the formula already has a (- 1) term, which sums with the (+ 1) term of
    /// the twos-complement to 0. Thus:
    /// 
    /// A - M - (1 - C) = A + !M + 1 - 1 + C
    ///                 = A + !M + C
    /// 
    /// Very easy. :)
    fn sbc(&mut self) -> u8 {
        self.fetch();

        // Doing an xor here to make sure we don't flip anything beyond bit-7.
        let not_m = self.m as u16 ^ 0x00FF;
        let a = self.a as u16;
        let c = self.status.contains(CPUStatus::C) as u16;
        
        let r = a + not_m + c;
        self.set_status_flag(CPUStatus::C, r > 255);
        self.set_status_flag(CPUStatus::Z, r == 0);
        self.set_status_flag(CPUStatus::N, r & (1 << 7) > 0);

        // See the documentation for the ADC instruction for an explanation
        // of this logic. Note that we se !M instead of M in the formula.
        self.set_status_flag(CPUStatus::V, (a ^ r) & !(a ^ not_m) & (1 << 7) != 0);
        self.a = r as u8;

        1
    }

    /// Set Carry flag.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#SEC
    fn sec(&mut self) -> u8 {
        self.set_status_flag(CPUStatus::C, true);

        0
    }

    /// Set Decimal flag.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#SED
    fn sed(&mut self) -> u8 {
        self.set_status_flag(CPUStatus::D, true);

        0
    }

    /// Set Interrupt disable flag.
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#SEI
    fn sei(&mut self) -> u8 {
        self.set_status_flag(CPUStatus::I, true);

        0
    }

    /// Store the contents of A into memory.
    /// 
    /// M = A
    /// 
    /// No status flags set.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#STA
    fn sta(&mut self) -> u8 {
        self.write(self.addr, self.a);

        0
    }

    /// Store the contents of X into memory.
    /// 
    /// M = X
    /// 
    /// No status flags set.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#STX
    fn stx(&mut self) -> u8 {
        self.write(self.addr, self.x);

        0
    }

    /// Store the contents of Y into memory.
    /// 
    /// M = Y
    /// 
    /// No status flags set.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#STY
    fn sty(&mut self) -> u8 {
        self.write(self.addr, self.y);

        0
    }

    /// Transfer A to X.
    /// 
    /// X = A
    /// 
    /// Status flags set:
    /// * Z - if X == 0
    /// * N - if bit 7 of X is set
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#TAX
    fn tax(&mut self) -> u8 {
        self.x = self.a;
        self.set_status_flag(CPUStatus::Z, self.x == 0);
        self.set_status_flag(CPUStatus::N, self.x & (1 << 7) > 0);

        0
    }

    /// Transfer A to Y.
    /// 
    /// Y = A
    /// 
    /// Status flags set:
    /// * Z - if Y == 0
    /// * N - if bit 7 of Y is set
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#TAY
    fn tay(&mut self) -> u8 {
        self.y = self.a;
        self.set_status_flag(CPUStatus::Z, self.y == 0);
        self.set_status_flag(CPUStatus::N, self.y & (1 << 7) > 0);

        0
    }

    /// Transfer stack pointer to X.
    /// 
    /// X = S
    /// 
    /// Status flags set:
    /// * Z - if X is 0
    /// * N - if bit 7 of X is set
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#TSX
    fn tsx(&mut self) -> u8 {
        self.x = self.stk_ptr;
        self.set_status_flag(CPUStatus::Z, self.x == 0);
        self.set_status_flag(CPUStatus::N, self.x & (1 << 7) > 0);

        0
    }

    /// Transfer X to A.
    /// 
    /// X = A
    /// 
    /// Status flags set:
    /// * Z - if A == 0
    /// * N - if bit 7 of A is set
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#TXA
    fn txa(&mut self) -> u8 {
        self.a = self.x;
        self.set_status_flag(CPUStatus::Z, self.a == 0);
        self.set_status_flag(CPUStatus::N, self.a & (1 << 7) > 0);

        0
    }

    /// Transfer X to stack pointer.
    /// 
    /// S = X
    /// 
    /// No status flags set.
    /// 
    /// Reference: http://obelisk.me.uk/6502/reference.html#TXS
    fn txs(&mut self) -> u8 {
        self.stk_ptr = self.x;

        0
    }

    /// Transfer Y to A.
    /// 
    /// X = A
    /// 
    /// Status flags set:
    /// * Z - if A == 0
    /// * N - if bit 7 of A is set
    /// 
    /// Reference: http://www.obelisk.me.uk/6502/reference.html#TYA
    fn tya(&mut self) -> u8 {
        self.a = self.y;
        self.set_status_flag(CPUStatus::Z, self.a == 0);
        self.set_status_flag(CPUStatus::N, self.a & (1 << 7) > 0);

        0
    }

    /// Dummy function for unsupported operations. No-op.
    fn xxx(&mut self) -> u8 {
        0
    }

    /// Helper function for addressing modes which access page zero. This allows cutting down
    /// on byte code by specifying only one other byte of input to an instruction.
    /// 
    /// Note (from the docs): The address calculation wraps around if the sum of the base address
    /// and the register exceed $FF. If we repeat the last example but with $FF in the X register
    /// then the accumulator will be loaded from $007F (e.g. $80 + $FF => $7F) and not $017F.
    fn zero_page(&mut self, offset: u8) -> u8 {
        self.addr = self.read(self.p_ctr) as u16 + offset as u16;
        self.addr &= 0x00FF;
        self.p_ctr += 1;

        0
    }

    /// Read an address from the bus, returning the following tuple:
    /// 
    /// (address, high_byte, low_byte)
    /// 
    /// This exposes computational data for any interested callers.
    fn read_address_from_bus(&mut self) -> (u16, u16, u16) {
        let lo: u16 = self.read(self.p_ctr) as u16;
        self.p_ctr += 1;
        let hi: u16 = self.read(self.p_ctr) as u16;
        self.p_ctr += 1;

        ((hi << 8) | lo, hi, lo)
    }

    /// Read an address from the input buffer in two-byte form (offset -> page) and increment the
    /// result with an offset value.
    fn read_address_with_value_offset(&mut self, v: u16) -> u8 {
        let (ptr, hi, _) = self.read_address_from_bus();
        self.addr = ptr + v;
        
        // Check if we have moved to a different page after incrementing. If we have, then we need
        // an additional clock cycle.
        // Note that for the case where v == 0, this is trivially true, as we cannot flip the page.
        (self.addr & 0xFF00 != (hi << 8)) as u8
    }

    /// Interrupt helper. This contains the shared logic used by the interrupt functions
    /// IRQ and NMI (interrupt and interrupt_no_mask, respectively), which only differ in
    /// the address they use to set the program counter.
    fn interrupt_and_set_address_to(&mut self, addr: u16) {
        self.push_to_stack((self.p_ctr >> 8) as u8);
        self.push_to_stack(self.p_ctr as u8);

        self.set_status_flag(CPUStatus::B, false);
        self.set_status_flag(CPUStatus::U, true);
        self.set_status_flag(CPUStatus::I, true);
        self.push_to_stack(self.status.bits());
        
        self.addr = addr;

        let lo: u16 = self.read(self.addr) as u16;
        let hi: u16 = self.read(self.addr + 1) as u16;
        self.p_ctr = (hi << 8) | lo;
        self.cycles = 7;
    }
    
    /// Set the boolean value of a particular flag in the status code.
    fn set_status_flag(&mut self, status_flag: CPUStatus, v: bool) {
        if v {
            self.status |= status_flag;
        } else {
            self.status &= !status_flag;
        }
    }

    /// Read data from the connected Bus.
    fn read(&self, addr: u16) -> u8 {
        self.bus
            .as_ref()
            .read(addr)
    }

    /// Write data to the connected Bus.
    fn write(&mut self, addr: u16, data: u8) {
        self.bus
            .as_mut()
            .write(addr, data);
    }

    /// Short helper function for pushing a value to the stack.
    fn push_to_stack(&mut self, data: u8) {
        self.write(0x0100 + self.stk_ptr as u16, data);
        self.stk_ptr -= 1;
    }

    /// Short helper function for pulling a value from the stack.
    fn pull_from_stack(&mut self) -> u8 {
        self.stk_ptr += 1;
        self.read(0x0100 + self.stk_ptr as u16)
    }

    /// Add the relative address displacement value to the program counter.
    /// 
    /// If a page turn is detected, then we return 1.
    fn add_relative_to_pc(&mut self) -> u8 {
        let current_page = self.p_ctr & 0xFF00;
        self.p_ctr += self.addr_rel as u16;

        ((self.p_ctr & 0xFF00) != current_page) as u8
    }

    /// Short helper to generalize the Branch if X clear/set logic.
    fn branch_on_flag(&mut self, flag: CPUStatus, v: bool) -> u8 {
        if self.status.contains(flag) == v {
            self.add_relative_to_pc();
            1
        } else {
            0
        }
    }
}

/// Function for defining the mapping between addressing modes and the functions on the CPU
/// which implement them.
fn get_addressing_mode<'a>(addr_mode: AddressingMode) -> Box<fn(&mut CPU<'a>) -> u8> {
    match addr_mode {
        AddressingMode::ABS => Box::new(CPU::abs),
        AddressingMode::ABX => Box::new(CPU::abx),
        AddressingMode::ABY => Box::new(CPU::aby),
        AddressingMode::IMP => Box::new(CPU::imp),
        AddressingMode::IMM => Box::new(CPU::imm),
        AddressingMode::IND => Box::new(CPU::ind),
        AddressingMode::IDX => Box::new(CPU::idx),
        AddressingMode::IDY => Box::new(CPU::idy),
        AddressingMode::REL => Box::new(CPU::rel),
        AddressingMode::ZPG => Box::new(CPU::zpg),
        AddressingMode::ZPX => Box::new(CPU::zpx),
        AddressingMode::ZPY => Box::new(CPU::zpy),
    }
}

/// Function for defining the mapping between operating modes and the functions on the CPU
/// which implement them.
fn get_operating_mode<'a>(op_mode: OperatingMode) -> Box<fn(&mut CPU<'a>) -> u8> {
    match op_mode {
        OperatingMode::ADC => Box::new(CPU::adc),
        OperatingMode::AND => Box::new(CPU::and),
        OperatingMode::ASL => Box::new(CPU::asl),
        OperatingMode::BCC => Box::new(CPU::bcc),
        OperatingMode::BCS => Box::new(CPU::bcs),
        OperatingMode::BEQ => Box::new(CPU::beq),
        OperatingMode::BIT => Box::new(CPU::bit),
        OperatingMode::BMI => Box::new(CPU::bmi),
        OperatingMode::BNE => Box::new(CPU::bne),
        OperatingMode::BPL => Box::new(CPU::bpl),
        OperatingMode::BRK => Box::new(CPU::brk),
        OperatingMode::BVC => Box::new(CPU::bvc),
        OperatingMode::BVS => Box::new(CPU::bvs),
        OperatingMode::CLC => Box::new(CPU::clc),
        OperatingMode::CLD => Box::new(CPU::cld),
        OperatingMode::CLI => Box::new(CPU::cli),
        OperatingMode::CLV => Box::new(CPU::clv),
        OperatingMode::CMP => Box::new(CPU::cmp),
        OperatingMode::CPX => Box::new(CPU::cpx),
        OperatingMode::CPY => Box::new(CPU::cpy),
        OperatingMode::DEC => Box::new(CPU::dec),
        OperatingMode::DEX => Box::new(CPU::dex),
        OperatingMode::DEY => Box::new(CPU::dey),
        OperatingMode::EOR => Box::new(CPU::eor),
        OperatingMode::INC => Box::new(CPU::inc),
        OperatingMode::INX => Box::new(CPU::inx),
        OperatingMode::INY => Box::new(CPU::iny),
        OperatingMode::JMP => Box::new(CPU::jmp),
        OperatingMode::JSR => Box::new(CPU::jsr),
        OperatingMode::LDA => Box::new(CPU::lda),
        OperatingMode::LDX => Box::new(CPU::ldx),
        OperatingMode::LDY => Box::new(CPU::ldy),
        OperatingMode::LSR => Box::new(CPU::lsr),
        OperatingMode::NOP => Box::new(CPU::nop),
        OperatingMode::ORA => Box::new(CPU::ora),
        OperatingMode::PHA => Box::new(CPU::pha),
        OperatingMode::PHP => Box::new(CPU::php),
        OperatingMode::PLA => Box::new(CPU::pla),
        OperatingMode::PLP => Box::new(CPU::plp),
        OperatingMode::ROL => Box::new(CPU::rol),
        OperatingMode::ROR => Box::new(CPU::ror),
        OperatingMode::RTI => Box::new(CPU::rti),
        OperatingMode::RTS => Box::new(CPU::rts),
        OperatingMode::SBC => Box::new(CPU::sbc),
        OperatingMode::SEC => Box::new(CPU::sec),
        OperatingMode::SED => Box::new(CPU::sed),
        OperatingMode::SEI => Box::new(CPU::sei),
        OperatingMode::STA => Box::new(CPU::sta),
        OperatingMode::STX => Box::new(CPU::stx),
        OperatingMode::STY => Box::new(CPU::sty),
        OperatingMode::TAX => Box::new(CPU::tax),
        OperatingMode::TAY => Box::new(CPU::tay),
        OperatingMode::TSX => Box::new(CPU::tsx),
        OperatingMode::TXA => Box::new(CPU::txa),
        OperatingMode::TXS => Box::new(CPU::txs),
        OperatingMode::TYA => Box::new(CPU::tya),
        OperatingMode::XXX => Box::new(CPU::xxx),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use expectest::prelude::*;

    #[test]
    fn abs() {
        let mut bus = Bus::new();
        bus.write(0x0001, 0x01);
        let mut cpu = CPU::new(&mut bus);

        let v = cpu.abs();
        expect!(v).to(be_eq(0));
        expect!(cpu.addr).to(be_eq(0x0100));
        expect!(cpu.p_ctr).to(be_eq(0x0002));
    }

    #[test]
    fn abx() {
        let mut bus = Bus::new();
        bus.write(0x0001, 0x01);

        let mut cpu = CPU::new(&mut bus);
        cpu.x = 0x04;

        let v = cpu.abx();
        expect!(v).to(be_eq(0));
        expect!(cpu.addr).to(be_eq(0x0104));
        expect!(cpu.p_ctr).to(be_eq(0x0002));
    }

    #[test]
    fn abx_with_page_turn() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0xFF);
        bus.write(0x0001, 0x01);

        let mut cpu = CPU::new(&mut bus);
        cpu.x = 0x04;

        let v = cpu.abx();
        expect!(v).to(be_eq(1));
        expect!(cpu.addr).to(be_eq(0x0203));
        expect!(cpu.p_ctr).to(be_eq(0x0002));
    }

    #[test]
    fn aby() {
        let mut bus = Bus::new();
        bus.write(0x0001, 0x01);

        let mut cpu = CPU::new(&mut bus);
        cpu.y = 0x04;

        let v = cpu.aby();
        expect!(v).to(be_eq(0));
        expect!(cpu.addr).to(be_eq(0x0104));
        expect!(cpu.p_ctr).to(be_eq(0x0002));
    }

    #[test]
    fn aby_with_page_turn() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0xFF);
        bus.write(0x0001, 0x01);

        let mut cpu = CPU::new(&mut bus);
        cpu.y = 0x04;

        let v = cpu.aby();
        expect!(v).to(be_eq(1));
        expect!(cpu.addr).to(be_eq(0x0203));
        expect!(cpu.p_ctr).to(be_eq(0x0002));
    }

    #[test]
    fn imp() {
        let mut bus = Bus::new();

        let mut cpu = CPU::new(&mut bus);
        cpu.a = 0x10;

        let v = cpu.imp();
        expect!(v).to(be_eq(0));
        expect!(cpu.m).to(be_eq(cpu.a));
    }

    #[test]
    fn imm() {
        let mut bus = Bus::new();

        let mut cpu = CPU::new(&mut bus);
        cpu.p_ctr = 0x00FF;

        let v = cpu.imm();
        expect!(v).to(be_eq(0));
        expect!(cpu.a).to(be_eq(0x00FF));
        expect!(cpu.p_ctr).to(be_eq(0x0100));
    }

    #[test]
    fn ind() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0xFE);
        bus.write(0x0001, 0xDD);
        bus.write(0xDDFE, 0x0D);
        bus.write(0xDDFF, 0xD0);

        let mut cpu = CPU::new(&mut bus);
        
        let v = cpu.ind();
        expect!(v).to(be_eq(0));
        expect!(cpu.addr).to(be_eq(0xD00D));
        expect!(cpu.p_ctr).to(be_eq(0x0002));
    }

    #[test]
    fn ind_low_byte_is_ff() {
        // We simulate the hardware bug here; reading at PC gives us
        // the address 0xDDFF. The hardware here would only increment
        // the lower byte of the address without spilling that into the
        // overflow of the resulting address. Thus, the high byte will
        // originate from address 0xDD00, NOT 0xDE00 (as math says)
        let mut bus = Bus::new();
        bus.write(0x0000, 0xFF);
        bus.write(0x0001, 0xDD);
        bus.write(0xDDFF, 0x0D);
        bus.write(0xDD00, 0xD0);

        let mut cpu = CPU::new(&mut bus);
        
        let v = cpu.ind();
        expect!(v).to(be_eq(0));
        expect!(cpu.addr).to(be_eq(0xD00D));
        expect!(cpu.p_ctr).to(be_eq(0x0002));
    }

    #[test]
    fn idx() {
        let mut bus = Bus::new();
        bus.write(0x0100, 0xFF);
        bus.write(0x0101, 0xDD);
        bus.write(0x0000, 0x0D);
        bus.write(0x0001, 0xD0);

        let mut cpu = CPU::new(&mut bus);
        cpu.x = 0x01;
        cpu.p_ctr = 0x0100;

        let v = cpu.idx();
        expect!(v).to(be_eq(0));
        expect!(cpu.addr).to(be_eq(0xD00D));
        expect!(cpu.p_ctr).to(be_eq(0x0102));
    }

    #[test]
    fn idy() {
        let mut bus = Bus::new();
        bus.write(0x0100, 0xFE);
        bus.write(0x0101, 0xDD);
        bus.write(0x00FE, 0x0D);
        bus.write(0x00FF, 0xD0);

        let mut cpu = CPU::new(&mut bus);
        cpu.y = 0x01;
        cpu.p_ctr = 0x0100;

        let v = cpu.idy();
        expect!(v).to(be_eq(0));
        expect!(cpu.addr).to(be_eq(0xD00E));
        expect!(cpu.p_ctr).to(be_eq(0x0102));
    }

    #[test]
    fn idy_page_change() {
        let mut bus = Bus::new();
        bus.write(0x0100, 0xFE);
        bus.write(0x0101, 0xDD);
        bus.write(0x00FE, 0xFF);
        bus.write(0x00FF, 0xD0);

        let mut cpu = CPU::new(&mut bus);
        cpu.y = 0x01;
        cpu.p_ctr = 0x0100;

        let v = cpu.idy();
        expect!(v).to(be_eq(1));
        expect!(cpu.addr).to(be_eq(0xD100));
        expect!(cpu.p_ctr).to(be_eq(0x0102));
    }

    #[test]
    fn rel_forward() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0x10);

        let mut cpu = CPU::new(&mut bus);
        
        let v = cpu.rel();
        expect!(v).to(be_eq(0));
        expect!(cpu.addr_rel).to(be_eq(0x0010));
        expect!(cpu.p_ctr).to(be_eq(0x0001));
    }

    #[test]
    fn rel_backward() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0xF0);

        let mut cpu = CPU::new(&mut bus);
        
        let v = cpu.rel();
        expect!(v).to(be_eq(0));
        expect!(cpu.addr_rel).to(be_eq(0xFFF0));
        expect!(cpu.p_ctr).to(be_eq(0x0001));
    }

    #[test]
    fn zpg() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0xFF);

        let mut cpu = CPU::new(&mut bus);

        let v = cpu.zpg();
        expect!(v).to(be_eq(0));
        expect!(cpu.addr).to(be_eq(0x00FF));
        expect!(cpu.p_ctr).to(be_eq(0x0001));
    }

    #[test]
    fn zpx() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0xFE);

        let mut cpu = CPU::new(&mut bus);
        cpu.x = 0x01;

        let v = cpu.zpx();
        expect!(v).to(be_eq(0));
        expect!(cpu.addr).to(be_eq(0x00FF));
        expect!(cpu.p_ctr).to(be_eq(0x0001));
    }

    #[test]
    fn zpx_page_change() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0xFF);

        let mut cpu = CPU::new(&mut bus);
        cpu.x = 0x01;

        let v = cpu.zpx();
        expect!(v).to(be_eq(0));
        expect!(cpu.addr).to(be_eq(0x0000));
        expect!(cpu.p_ctr).to(be_eq(0x0001));
    }

    #[test]
    fn zpy() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0xFE);

        let mut cpu = CPU::new(&mut bus);
        cpu.y = 0x01;

        let v = cpu.zpy();
        expect!(v).to(be_eq(0));
        expect!(cpu.addr).to(be_eq(0x00FF));
        expect!(cpu.p_ctr).to(be_eq(0x0001));
    }

    #[test]
    fn zpy_page_change() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0xFF);

        let mut cpu = CPU::new(&mut bus);
        cpu.y = 0x01;

        let v = cpu.zpy();
        expect!(v).to(be_eq(0));
        expect!(cpu.addr).to(be_eq(0x0000));
        expect!(cpu.p_ctr).to(be_eq(0x0001));
    }

    #[test]
    fn and() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0b00000000);

        let mut cpu = CPU::new(&mut bus);
        cpu.a = 0b10101010;
        cpu.set_status_flag(CPUStatus::N, true);

        let v = cpu.and();
        expect!(v).to(be_eq(1));
        expect!(cpu.a).to(be_eq(0b00000000));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn bit() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0b00000000);

        let mut cpu = CPU::new(&mut bus);
        cpu.a = 0b11111111;
        cpu.set_status_flag(CPUStatus::V, true);
        cpu.set_status_flag(CPUStatus::N, true);

        let v = cpu.bit();
        expect!(v).to(be_eq(0));
        expect!(cpu.a).to(be_eq(0b11111111)); // unchanged
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::V)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn eor() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0b11111111);

        let mut cpu = CPU::new(&mut bus);
        cpu.a = 0b11111111;
        cpu.set_status_flag(CPUStatus::N, true);

        let v = cpu.eor();
        expect!(v).to(be_eq(1));
        expect!(cpu.a).to(be_eq(0b00000000));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn ora() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0b00000000);

        let mut cpu = CPU::new(&mut bus);
        cpu.a = 0b00000000;
        cpu.set_status_flag(CPUStatus::N, true);

        let v = cpu.ora();
        expect!(v).to(be_eq(1));
        expect!(cpu.a).to(be_eq(0b00000000));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn fetch_imp_mode() {
        // Note: this is only a unit test at present because we need to
        // be able to verify the value in the M register to ensure that
        // it did not change. There is no other way to verify this at
        // present, due to the lack of instruction implementations. Once
        // all the instructions are implemented, there should be a good
        // candidate for turning this into an integration test.
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.m = 0xFF;
        cpu.addr_mode = Some(AddressingMode::IMP);
        
        let fetched = cpu.fetch();
        expect!(fetched).to(be_eq(cpu.m));
        expect!(cpu.m).to(be_eq(0xFF));
    }

    #[test]
    fn tax() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.x = 0xFF;
        cpu.a = 0x00;

        let v = cpu.tax();
        expect!(v).to(be_eq(0));
        expect!(cpu.x).to(be_eq(0x00));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn tay() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.y = 0xFF;
        cpu.a = 0x00;

        let v = cpu.tay();
        expect!(v).to(be_eq(0));
        expect!(cpu.y).to(be_eq(0x00));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn txa() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.x = 0x00;
        cpu.a = 0xFF;

        let v = cpu.txa();
        expect!(v).to(be_eq(0));
        expect!(cpu.a).to(be_eq(0x00));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }
    
    #[test]
    fn tya() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.y = 0x00;
        cpu.a = 0xFF;

        let v = cpu.tya();
        expect!(v).to(be_eq(0));
        expect!(cpu.a).to(be_eq(0x00));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }
    
    #[test]
    fn lda() {
        let mut bus = Bus::new();
        bus.write(0x00FF, 0x00);

        let mut cpu = CPU::new(&mut bus);
        cpu.addr = 0x00FF;
        cpu.a = 0xFF;

        let v = cpu.lda();
        expect!(v).to(be_eq(1));
        expect!(cpu.a).to(be_eq(0x00));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn ldx() {
        let mut bus = Bus::new();
        bus.write(0x00FF, 0x00);

        let mut cpu = CPU::new(&mut bus);
        cpu.addr = 0x00FF;
        cpu.x = 0xFF;

        let v = cpu.ldx();
        expect!(v).to(be_eq(1));
        expect!(cpu.x).to(be_eq(0x00));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn ldy() {
        let mut bus = Bus::new();
        bus.write(0x00FF, 0x00);

        let mut cpu = CPU::new(&mut bus);
        cpu.addr = 0x00FF;
        cpu.y = 0xFF;

        let v = cpu.ldy();
        expect!(v).to(be_eq(1));
        expect!(cpu.y).to(be_eq(0x00));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn sta() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr = 0x00FF;
        cpu.a = 0xFF;

        let v = cpu.sta();
        expect!(v).to(be_eq(0));
        expect!(bus.read(0x00FF)).to(be_eq(0xFF));
    }

    #[test]
    fn stx() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr = 0x00FF;
        cpu.x = 0xFF;

        let v = cpu.stx();
        expect!(v).to(be_eq(0));
        expect!(bus.read(0x00FF)).to(be_eq(0xFF));
    }

    #[test]
    fn sty() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr = 0x00FF;
        cpu.y = 0xFF;

        let v = cpu.sty();
        expect!(v).to(be_eq(0));
        expect!(bus.read(0x00FF)).to(be_eq(0xFF));
    }

    #[test]
    fn tsx() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.x = 0xFF;

        let v = cpu.tsx();
        expect!(v).to(be_eq(0));
        expect!(cpu.stk_ptr).to(be_eq(cpu.x));
    }

    #[test]
    fn txs() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.stk_ptr = 0xFF;

        let v = cpu.tsx();
        expect!(v).to(be_eq(0));
        expect!(cpu.stk_ptr).to(be_eq(cpu.x));
    }

    #[test]
    fn pha() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.reset(); // stack pointer is set to 0xFD here
        cpu.a = 0xFF;

        let v = cpu.pha();
        expect!(v).to(be_eq(0));
        expect!(cpu.stk_ptr).to(be_eq(0xFC));
        expect!(cpu.read(0x01FD)).to(be_eq(cpu.a));
    }

    #[test]
    fn php() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.reset(); // stack pointer is set to 0xFD here
        cpu.status = CPUStatus::from_bits(0b10101010).unwrap_or(CPUStatus::X);

        let v = cpu.php();
        expect!(v).to(be_eq(0));
        expect!(cpu.stk_ptr).to(be_eq(0xFC));
        expect!(cpu.read(0x01FD)).to(be_eq(cpu.status.bits() + (1 << 4))); // B is always set
    }

    #[test]
    fn pla() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.reset(); // stack pointer is set to 0xFD here
        cpu.push_to_stack(0xFF); // stack: 0xFD => 0xFF, stack pointer is now 0xFC

        let v = cpu.pla();
        expect!(v).to(be_eq(0));
        expect!(cpu.stk_ptr).to(be_eq(0xFD)); // came back to 0xFD after pull
        expect!(cpu.read(0x01FD)).to(be_eq(cpu.a));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_true());
    }

    #[test]
    fn plp() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.reset(); // stack pointer is set to 0xFD here
        cpu.push_to_stack(0b11111111); // stack: 0xFD => 0xFF, stack pointer is now 0xFC

        let v = cpu.plp();
        expect!(v).to(be_eq(0));
        expect!(cpu.stk_ptr).to(be_eq(0xFD)); // came back to 0xFD after pull
        expect!(cpu.read(0x01FD)).to(be_eq(cpu.status.bits() + (1 << 4)));
    }

    #[test]
    fn inc() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0x7F);

        let mut cpu = CPU::new(&mut bus);
        cpu.addr = 0x0000;
        
        let v = cpu.inc();
        expect!(v).to(be_eq(0));
        expect!(cpu.m).to(be_eq(0x80));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_true());
    }

    #[test]
    fn inx() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.x = 0x7F;
        
        let v = cpu.inx();
        expect!(v).to(be_eq(0));
        expect!(cpu.x).to(be_eq(0x80));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_true());
    }

    #[test]
    fn iny() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.y = 0x7F;
        
        let v = cpu.iny();
        expect!(v).to(be_eq(0));
        expect!(cpu.y).to(be_eq(0x80));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_true());
    }

    #[test]
    fn dec() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0x80);

        let mut cpu = CPU::new(&mut bus);
        cpu.addr = 0x0000;
        
        let v = cpu.dec();
        expect!(v).to(be_eq(0));
        expect!(cpu.m).to(be_eq(0x7F));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn dex() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.x = 0x80;
        
        let v = cpu.dex();
        expect!(v).to(be_eq(0));
        expect!(cpu.x).to(be_eq(0x7F));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn dey() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.y = 0x80;
        
        let v = cpu.dey();
        expect!(v).to(be_eq(0));
        expect!(cpu.y).to(be_eq(0x7F));
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn asl() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0b10101010);

        let mut cpu = CPU::new(&mut bus);
        cpu.addr_mode = Some(AddressingMode::ABS);

        let v = cpu.asl();
        expect!(v).to(be_eq(0));
        expect!(cpu.read(0x0000)).to(be_eq(0b01010100));
        expect!(cpu.status.contains(CPUStatus::C)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn asl_imp() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.a = 0b10101010;
        cpu.m = 0b10101010;
        cpu.addr_mode = Some(AddressingMode::IMP);

        let v = cpu.asl();
        expect!(v).to(be_eq(0));
        expect!(cpu.a).to(be_eq(0b01010100));
        expect!(cpu.status.contains(CPUStatus::C)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn lsr() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0b10101010);

        let mut cpu = CPU::new(&mut bus);
        cpu.addr_mode = Some(AddressingMode::ABS);

        let v = cpu.lsr();
        expect!(v).to(be_eq(0));
        expect!(cpu.read(0x0000)).to(be_eq(0b01010101));
        expect!(cpu.status.contains(CPUStatus::C)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn lsr_imp() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.a = 0b10101010;
        cpu.m = 0b10101010;
        cpu.addr_mode = Some(AddressingMode::IMP);

        let v = cpu.lsr();
        expect!(v).to(be_eq(0));
        expect!(cpu.a).to(be_eq(0b01010101));
        expect!(cpu.status.contains(CPUStatus::C)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn rol() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0b10101010);

        let mut cpu = CPU::new(&mut bus);
        cpu.set_status_flag(CPUStatus::C, true);
        cpu.addr_mode = Some(AddressingMode::ABS);

        let v = cpu.rol();
        expect!(v).to(be_eq(0));
        expect!(cpu.read(0x0000)).to(be_eq(0b01010101));
        expect!(cpu.status.contains(CPUStatus::C)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn rol_imp() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.a = 0b10101010;
        cpu.m = 0b10101010;
        cpu.set_status_flag(CPUStatus::C, true);
        cpu.addr_mode = Some(AddressingMode::IMP);

        let v = cpu.rol();
        expect!(v).to(be_eq(0));
        expect!(cpu.a).to(be_eq(0b01010101));
        expect!(cpu.status.contains(CPUStatus::C)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn ror() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0b10101010);

        let mut cpu = CPU::new(&mut bus);
        cpu.set_status_flag(CPUStatus::C, true);
        cpu.addr_mode = Some(AddressingMode::ABS);

        let v = cpu.ror();
        expect!(v).to(be_eq(0));
        expect!(cpu.read(0x0000)).to(be_eq(0b11010101));
        expect!(cpu.status.contains(CPUStatus::C)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_true());
    }

    #[test]
    fn ror_imp() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.a = 0b10101010;
        cpu.m = 0b10101010;
        cpu.set_status_flag(CPUStatus::C, true);
        cpu.addr_mode = Some(AddressingMode::IMP);

        let v = cpu.ror();
        expect!(v).to(be_eq(0));
        expect!(cpu.a).to(be_eq(0b11010101));
        expect!(cpu.status.contains(CPUStatus::C)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_true());
    }

    #[test]
    fn jmp() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.p_ctr = 0xFFFF;
        cpu.addr = 0x9999;

        let v = cpu.jmp();
        expect!(v).to(be_eq(0));
        expect!(cpu.p_ctr).to(be_eq(cpu.addr));
        expect!(cpu.p_ctr).to(be_eq(0x9999));
    }

    #[test]
    fn jsr() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.p_ctr = 0xFFDE;
        cpu.addr = 0x9999;
        cpu.stk_ptr = 0xFD;

        let v = cpu.jsr();
        expect!(v).to(be_eq(0));
        expect!(cpu.p_ctr).to(be_eq(cpu.addr));
        expect!(cpu.p_ctr).to(be_eq(0x9999));
        expect!(cpu.stk_ptr).to(be_eq(0xFB));
        expect!(cpu.read(0x01FD)).to(be_eq(0xFF));
        expect!(cpu.read(0x01FC)).to(be_eq(0xDD));
    }

    #[test]
    fn rts() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.p_ctr = 0x9999;
        cpu.stk_ptr = 0xFD;
        cpu.push_to_stack(0xFF);
        cpu.push_to_stack(0xDD);

        let v = cpu.rts();
        expect!(v).to(be_eq(0));
        expect!(cpu.p_ctr).to(be_eq(0xFFDE));
        expect!(cpu.stk_ptr).to(be_eq(0xFD));
    }

    #[test]
    fn add_relative_to_pc() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr_rel = 0x0010;
        cpu.p_ctr = 0x0000;

        let v = cpu.add_relative_to_pc();
        expect!(v).to(be_eq(0));
        expect!(cpu.p_ctr).to(be_eq(0x0010));
    }

    #[test]
    fn add_relative_to_pc_with_page_turn() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr_rel = 0x0100;
        cpu.p_ctr = 0x0000;

        let v = cpu.add_relative_to_pc();
        expect!(v).to(be_eq(1));
        expect!(cpu.p_ctr).to(be_eq(0x0100));
    }

    #[test]
    fn bcc() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr_rel = 0x0010;
        cpu.p_ctr = 0x0000;
        cpu.set_status_flag(CPUStatus::C, false);

        let v = cpu.bcc();
        expect!(v).to(be_eq(1));
        expect!(cpu.p_ctr).to(be_eq(0x0010));
    }

    #[test]
    fn bcs() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr_rel = 0x0010;
        cpu.p_ctr = 0x0000;
        cpu.set_status_flag(CPUStatus::C, true);

        let v = cpu.bcs();
        expect!(v).to(be_eq(1));
        expect!(cpu.p_ctr).to(be_eq(0x0010));
    }

    #[test]
    fn beq() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr_rel = 0x0010;
        cpu.p_ctr = 0x0000;
        cpu.set_status_flag(CPUStatus::Z, true);

        let v = cpu.beq();
        expect!(v).to(be_eq(1));
        expect!(cpu.p_ctr).to(be_eq(0x0010));
    }

    #[test]
    fn bmi() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr_rel = 0x0010;
        cpu.p_ctr = 0x0000;
        cpu.set_status_flag(CPUStatus::N, true);

        let v = cpu.bmi();
        expect!(v).to(be_eq(1));
        expect!(cpu.p_ctr).to(be_eq(0x0010));
    }

    #[test]
    fn bne() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr_rel = 0x0010;
        cpu.p_ctr = 0x0000;
        cpu.set_status_flag(CPUStatus::Z, false);

        let v = cpu.bne();
        expect!(v).to(be_eq(1));
        expect!(cpu.p_ctr).to(be_eq(0x0010));
    }

    #[test]
    fn bpl() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr_rel = 0x0010;
        cpu.p_ctr = 0x0000;
        cpu.set_status_flag(CPUStatus::N, false);

        let v = cpu.bpl();
        expect!(v).to(be_eq(1));
        expect!(cpu.p_ctr).to(be_eq(0x0010));
    }

    #[test]
    fn bvc() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr_rel = 0x0010;
        cpu.p_ctr = 0x0000;
        cpu.set_status_flag(CPUStatus::V, false);

        let v = cpu.bvc();
        expect!(v).to(be_eq(1));
        expect!(cpu.p_ctr).to(be_eq(0x0010));
    }

    #[test]
    fn bvs() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr_rel = 0x0010;
        cpu.p_ctr = 0x0000;
        cpu.set_status_flag(CPUStatus::V, true);

        let v = cpu.bvs();
        expect!(v).to(be_eq(1));
        expect!(cpu.p_ctr).to(be_eq(0x0010));
    }

    #[test]
    fn brk() {
        let mut bus = Bus::new();
        bus.write(0xFFFE, 0x0D);
        bus.write(0xFFFF, 0xD0);

        let mut cpu = CPU::new(&mut bus);
        cpu.p_ctr = 0xDDFE;

        // B is not set, but will be in the stack
        cpu.status = CPUStatus::from_bits(0b10101010).unwrap_or(CPUStatus::X);

        let v = cpu.brk();
        expect!(v).to(be_eq(0));
        expect!(cpu.p_ctr).to(be_eq(0xD00D));
        expect!(cpu.pull_from_stack()).to(be_eq(0b10111010));
        expect!(cpu.pull_from_stack()).to(be_eq(0xFF));
        expect!(cpu.pull_from_stack()).to(be_eq(0xDD));
    }

    #[test]
    fn rti() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.push_to_stack(0xDD);
        cpu.push_to_stack(0xFE);
        cpu.push_to_stack(0b10111010); // B is set in stack

        let v = cpu.rti();
        expect!(v).to(be_eq(0));
        expect!(cpu.p_ctr).to(be_eq(0xDDFE));
        expect!(cpu.status.bits()).to(be_eq(0b10001010)); // B and U not set
    }

    #[test]
    fn cmp() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.a = 0xFF;
        cpu.write(0x0000, 0xFF);

        let v = cpu.cmp();
        expect!(v).to(be_eq(1));
        expect!(cpu.status.contains(CPUStatus::C)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
        fn cpx() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.x = 0xFF;
        cpu.write(0x0000, 0xFF);

        let v = cpu.cpx();
        expect!(v).to(be_eq(1));
        expect!(cpu.status.contains(CPUStatus::C)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn cpy() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.y = 0xFF;
        cpu.write(0x0000, 0xFF);

        let v = cpu.cpy();
        expect!(v).to(be_eq(1));
        expect!(cpu.status.contains(CPUStatus::C)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn adc() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr_mode = Some(AddressingMode::IMP);
        cpu.a = 0b11000000; // 192
        cpu.m = 0b01100010; // 98
        cpu.set_status_flag(CPUStatus::C, true);

        let v = cpu.adc();
        expect!(v).to(be_eq(1));
        expect!(cpu.a).to(be_eq(0b00100011));
        expect!(cpu.status.contains(CPUStatus::C)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::V)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }

    #[test]
    fn sbc() {
        let mut bus = Bus::new();
        let mut cpu = CPU::new(&mut bus);
        cpu.addr_mode = Some(AddressingMode::IMP);
        cpu.a = 0b11000000; // 192
        cpu.m = 0b01100010; // 98
        cpu.set_status_flag(CPUStatus::C, true);

        let v = cpu.sbc();
        expect!(v).to(be_eq(1));
        expect!(cpu.a).to(be_eq(0b01011110));
        expect!(cpu.status.contains(CPUStatus::C)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::Z)).to(be_false());
        expect!(cpu.status.contains(CPUStatus::V)).to(be_true());
        expect!(cpu.status.contains(CPUStatus::N)).to(be_false());
    }
}