use crate::bus::Bus;
use crate::model::{Instruction, StatusFlag};

/// Core data structure for emulating a 6502 CPU.
pub struct CPU<'a> {
    /// A hook to the bus, which serves as our interface through which we can read from and write to
    /// memory. This includes retrieving opcodes and data for instruction processing.
    bus: Box<&'a mut Bus>,
    /// The status code. This is a bitmask composed of a number of flags, each of which serves to
    /// surface some meaning about the state of the CPU and its last executed instruction.
    /// See: model::StatusFlag
    pub stat: u8,
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
    cycles: u8
}

impl<'a> CPU<'a> {
    /// Construct a new CPU instance with the given Bus hook.
    pub fn new(bus: &'a mut Bus) -> CPU<'a> {
        CPU {
            bus:      Box::new(bus),
            stat:     0x00,
            a:        0x00,
            x:        0x00,
            y:        0x00,
            stk_ptr:  0xFD,
            p_ctr:    0x0000,
            m:        0x00,
            addr:     0x0000,
            addr_rel: 0x00,
            opcode:   0x00,
            cycles:   0,
        }
    }

    /// Iterate forward one clock cycle. Accept the input associated with that clock
    /// cycle and act on it.
    pub fn clock(&mut self) {
        if self.cycles == 0 {
            self.opcode = self.read(self.p_ctr);
            self.p_ctr += 1;

            let instruction = self.interpret_instruction();
            self.cycles = instruction.cycles;

            // Execute the instruction. If at least one additional cycle is needed,
            // then increment the cycles counter.
            let addrmode_cycles = (instruction.address_mode)(self);
            self.cycles += addrmode_cycles;
        }

        self.cycles -= 1;
    }

    /// Retrieve the boolean value of a particular flag in the status code.
    pub fn get_status_flag(&self, status_flag: StatusFlag) -> bool {
        (self.stat & (status_flag as u8)) != 0
    }
    
    /// Convenience function for flipping on the interrupt flag, as it is impossible to do
    /// so through other public library functions.
    pub fn set_interrupt_flag(&mut self, v: bool) {
        self.set_status_flag(StatusFlag::I, v);
    }

    /// Request an interrupt. This can be ignored if the I flag on the status code is set.
    pub fn interrupt(&mut self) {
        if self.get_status_flag(StatusFlag::I) {
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
        self.stat = StatusFlag::U as u8; // 00100000
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
        if (*self.interpret_instruction().address_mode) as usize != CPU::imp as usize {
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
    
    /// Add M to A with Carry
    fn adc(&mut self) -> u8 {
        0
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
        self.set_status_flag(StatusFlag::Z, self.a == 0);
        self.set_status_flag(StatusFlag::N, self.a & (1 << 7) > 0);

        1
    }

    /// Shift Left One Bit (M or A)
    fn asl(&mut self) -> u8 {
        0
    }

    /// Branch on Carry Clear
    fn bcc(&mut self) -> u8 {
        0
    }

    /// Branch on Carry Set
    fn bcs(&mut self) -> u8 {
        0
    }

    /// Branch on Result Zero
    fn beq(&mut self) -> u8 {
        0
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

        self.set_status_flag(StatusFlag::Z, self.a & self.m == 0);
        self.set_status_flag(StatusFlag::V, self.m & (1 << 6) > 0);
        self.set_status_flag(StatusFlag::N, self.m & (1 << 7) > 0);

        0
    }

    /// Branch on Result Minus
    fn bmi(&mut self) -> u8 {
        0
    }

    /// Branch on Result not Zero
    fn bne(&mut self) -> u8 {
        0
    }

    /// Branch on Result Plus
    fn bpl(&mut self) -> u8 {
        0
    }

    /// Force Break
    fn brk(&mut self) -> u8 {
        0
    }

    /// Branch on Overflow Clear
    fn bvc(&mut self) -> u8 {
        0
    }

    /// Branch on Overflow Set
    fn bvs(&mut self) -> u8 {
        0
    }

    /// Clear Carry Flag
    fn clc(&mut self) -> u8 {
        0
    }

    /// Clear Decimal Mode
    fn cld(&mut self) -> u8 {
        0
    }

    /// Clear interrupt Disable Bit
    fn cli(&mut self) -> u8 {
        0
    }

    /// Clear Overflow Flag
    fn clv(&mut self) -> u8 {
        0
    }

    /// Compare M and A
    fn cmp(&mut self) -> u8 {
        0
    }

    /// Compare M and X
    fn cpx(&mut self) -> u8 {
        0
    }

    /// Compare M and Y
    fn cpy(&mut self) -> u8 {
        0
    }

    /// Decrement M by One
    fn dec(&mut self) -> u8 {
        0
    }

    /// Decrement X by One
    fn dex(&mut self) -> u8 {
        0
    }

    /// Decrement Y by One
    fn dey(&mut self) -> u8 {
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
        self.set_status_flag(StatusFlag::Z, self.a == 0);
        self.set_status_flag(StatusFlag::N, self.a & (1 << 7) > 0);

        1
    }

    /// Increment M by One
    fn inc(&mut self) -> u8 {
        0
    }

    /// Increment X by One
    fn inx(&mut self) -> u8 {
        0
    }

    /// Increment Y by One
    fn iny(&mut self) -> u8 {
        0
    }

    /// Jump to Location
    fn jmp(&mut self) -> u8 {
        0
    }

    /// Jump to Location Save Return Address
    fn jsr(&mut self) -> u8 {
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
        self.set_status_flag(StatusFlag::Z, self.a == 0);
        self.set_status_flag(StatusFlag::N, self.a & (1 << 7) > 0);
        
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
        self.set_status_flag(StatusFlag::Z, self.x == 0);
        self.set_status_flag(StatusFlag::N, self.x & (1 << 7) > 0);
        
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
        self.set_status_flag(StatusFlag::Z, self.y == 0);
        self.set_status_flag(StatusFlag::N, self.y & (1 << 7) > 0);
        
        1
    }

    /// Shift Right One Bit (M or A)
    fn lsr(&mut self) -> u8 {
        0
    }

    /// No Operation
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
        self.set_status_flag(StatusFlag::Z, self.a == 0);
        self.set_status_flag(StatusFlag::N, self.a & (1 << 7) > 0);

        1
    }

    /// Push A on Stack
    fn pha(&mut self) -> u8 {
        0
    }

    /// Push Processor Status on Stack
    fn php(&mut self) -> u8 {
        0
    }

    /// Pull A from Stack
    fn pla(&mut self) -> u8 {
        0
    }

    /// Pull Processor Status from Stack
    fn plp(&mut self) -> u8 {
        0
    }

    /// Rotate One Bit Left (M or A)
    fn rol(&mut self) -> u8 {
        0
    }

    /// Rotate One Bit Right (M or A)
    fn ror(&mut self) -> u8 {
        0
    }

    /// Return from Interrupt
    fn rti(&mut self) -> u8 {
        0
    }

    /// Return from Subroutine
    fn rts(&mut self) -> u8 {
        0
    }

    /// Subtract M from A with Borrow
    fn sbc(&mut self) -> u8 {
        0
    }

    /// Set Carry Flag
    fn sec(&mut self) -> u8 {
        0
    }

    /// Set Decimal Mode
    fn sed(&mut self) -> u8 {
        0
    }

    /// Set Interrupt Disable Status
    fn sei(&mut self) -> u8 {
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
        self.set_status_flag(StatusFlag::Z, self.x == 0);
        self.set_status_flag(StatusFlag::N, self.x & (1 << 7) > 0);

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
        self.set_status_flag(StatusFlag::Z, self.y == 0);
        self.set_status_flag(StatusFlag::N, self.y & (1 << 7) > 0);

        0
    }

    /// Transfer Stack Pointer to X
    fn tsx(&mut self) -> u8 {
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
        self.set_status_flag(StatusFlag::Z, self.a == 0);
        self.set_status_flag(StatusFlag::N, self.a & (1 << 7) > 0);

        0
    }

    /// Transfer X to Stack Pointer
    fn txs(&mut self) -> u8 {
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
        self.set_status_flag(StatusFlag::Z, self.a == 0);
        self.set_status_flag(StatusFlag::N, self.a & (1 << 7) > 0);

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
        self.write(0x0100 + self.stk_ptr as u16, (self.p_ctr >> 8) as u8);
        self.stk_ptr -= 1;
        self.write(0x0100 + self.stk_ptr as u16, self.p_ctr as u8);
        self.stk_ptr -= 1;

        self.set_status_flag(StatusFlag::B, false);
        self.set_status_flag(StatusFlag::U, true);
        self.set_status_flag(StatusFlag::I, true);
        self.write(0x0100 + self.stk_ptr as u16, self.stat);
        self.stk_ptr -= 1;
        self.addr = addr;

        let lo: u16 = self.read(self.addr) as u16;
        let hi: u16 = self.read(self.addr) as u16;
        self.p_ctr = (hi << 8) | lo;
        self.cycles = 7;
    }
    
    /// Set the boolean value of a particular flag in the status code.
    fn set_status_flag(&mut self, status_flag: StatusFlag, v: bool) {
        if v {
            self.stat |= status_flag as u8;
        } else {
            self.stat &= !(status_flag as u8);
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

    /// Instruction interpreter. This reads the opcode set on the CPU and returns some data about
    /// the instruction that is associated with that opcode. Each instruction contains:
    /// 
    /// 1. The number of clock cycles required by the instruction.
    /// 2. A function which implements the data addressing mode behavior of the instruction.
    /// 
    /// Ideally, this wouldn't be as large as it is; unfortunately, unsupported opcodes still
    /// define a known number of cycles, so they need to be mapped for correctness. 
    fn interpret_instruction(&self) -> Instruction<'a> {
        match self.opcode {
            0x00 => Instruction { name: "BRK", cycles: 7, operate: Box::new(CPU::brk), address_mode: Box::new(CPU::imm) },
            0x01 => Instruction { name: "ORA", cycles: 6, operate: Box::new(CPU::ora), address_mode: Box::new(CPU::idx) },
            0x02 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x03 => Instruction { name: "XXX", cycles: 8, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x04 => Instruction { name: "XXX", cycles: 3, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x05 => Instruction { name: "ORA", cycles: 3, operate: Box::new(CPU::ora), address_mode: Box::new(CPU::zpg) },
            0x06 => Instruction { name: "ASL", cycles: 5, operate: Box::new(CPU::asl), address_mode: Box::new(CPU::zpg) },
            0x07 => Instruction { name: "XXX", cycles: 5, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x08 => Instruction { name: "PHP", cycles: 3, operate: Box::new(CPU::php), address_mode: Box::new(CPU::imp) },
            0x09 => Instruction { name: "ORA", cycles: 2, operate: Box::new(CPU::ora), address_mode: Box::new(CPU::imm) },
            0x0A => Instruction { name: "ASL", cycles: 2, operate: Box::new(CPU::asl), address_mode: Box::new(CPU::imp) },
            0x0B => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x0C => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x0D => Instruction { name: "ORA", cycles: 4, operate: Box::new(CPU::ora), address_mode: Box::new(CPU::abs) },
            0x0E => Instruction { name: "ASL", cycles: 6, operate: Box::new(CPU::asl), address_mode: Box::new(CPU::abs) },
            0x0F => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x10 => Instruction { name: "BPL", cycles: 2, operate: Box::new(CPU::bpl), address_mode: Box::new(CPU::rel) },
            0x11 => Instruction { name: "ORA", cycles: 5, operate: Box::new(CPU::ora), address_mode: Box::new(CPU::idy) },
            0x12 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x13 => Instruction { name: "XXX", cycles: 8, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x14 => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x15 => Instruction { name: "ORA", cycles: 4, operate: Box::new(CPU::ora), address_mode: Box::new(CPU::zpx) },
            0x16 => Instruction { name: "ASL", cycles: 6, operate: Box::new(CPU::asl), address_mode: Box::new(CPU::zpx) },
            0x17 => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x18 => Instruction { name: "CLC", cycles: 2, operate: Box::new(CPU::clc), address_mode: Box::new(CPU::imp) },
            0x19 => Instruction { name: "ORA", cycles: 4, operate: Box::new(CPU::ora), address_mode: Box::new(CPU::aby) },
            0x1A => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x1B => Instruction { name: "XXX", cycles: 7, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x1C => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x1D => Instruction { name: "ORA", cycles: 4, operate: Box::new(CPU::ora), address_mode: Box::new(CPU::abx) },
            0x1E => Instruction { name: "ASL", cycles: 7, operate: Box::new(CPU::asl), address_mode: Box::new(CPU::abx) },
            0x1F => Instruction { name: "XXX", cycles: 7, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x20 => Instruction { name: "JSR", cycles: 6, operate: Box::new(CPU::jsr), address_mode: Box::new(CPU::abs) },
            0x21 => Instruction { name: "AND", cycles: 6, operate: Box::new(CPU::and), address_mode: Box::new(CPU::idx) },
            0x22 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x23 => Instruction { name: "XXX", cycles: 8, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x24 => Instruction { name: "BIT", cycles: 3, operate: Box::new(CPU::bit), address_mode: Box::new(CPU::zpg) },
            0x25 => Instruction { name: "AND", cycles: 3, operate: Box::new(CPU::and), address_mode: Box::new(CPU::zpg) },
            0x26 => Instruction { name: "ROL", cycles: 5, operate: Box::new(CPU::rol), address_mode: Box::new(CPU::zpg) },
            0x27 => Instruction { name: "XXX", cycles: 5, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x28 => Instruction { name: "PLP", cycles: 4, operate: Box::new(CPU::plp), address_mode: Box::new(CPU::imp) },
            0x29 => Instruction { name: "AND", cycles: 2, operate: Box::new(CPU::and), address_mode: Box::new(CPU::imm) },
            0x2A => Instruction { name: "ROL", cycles: 2, operate: Box::new(CPU::rol), address_mode: Box::new(CPU::imp) },
            0x2B => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x2C => Instruction { name: "BIT", cycles: 4, operate: Box::new(CPU::bit), address_mode: Box::new(CPU::abs) },
            0x2D => Instruction { name: "AND", cycles: 4, operate: Box::new(CPU::and), address_mode: Box::new(CPU::abs) },
            0x2E => Instruction { name: "ROL", cycles: 6, operate: Box::new(CPU::rol), address_mode: Box::new(CPU::abs) },
            0x2F => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x30 => Instruction { name: "BMI", cycles: 2, operate: Box::new(CPU::bmi), address_mode: Box::new(CPU::rel) },
            0x31 => Instruction { name: "AND", cycles: 5, operate: Box::new(CPU::and), address_mode: Box::new(CPU::idy) },
            0x32 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x33 => Instruction { name: "XXX", cycles: 8, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x34 => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x35 => Instruction { name: "AND", cycles: 4, operate: Box::new(CPU::and), address_mode: Box::new(CPU::zpx) },
            0x36 => Instruction { name: "ROL", cycles: 6, operate: Box::new(CPU::rol), address_mode: Box::new(CPU::zpx) },
            0x37 => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x38 => Instruction { name: "SEC", cycles: 2, operate: Box::new(CPU::sec), address_mode: Box::new(CPU::imp) },
            0x39 => Instruction { name: "AND", cycles: 4, operate: Box::new(CPU::and), address_mode: Box::new(CPU::aby) },
            0x3A => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x3B => Instruction { name: "XXX", cycles: 7, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x3C => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x3D => Instruction { name: "AND", cycles: 4, operate: Box::new(CPU::and), address_mode: Box::new(CPU::abx) },
            0x3E => Instruction { name: "ROL", cycles: 7, operate: Box::new(CPU::rol), address_mode: Box::new(CPU::abx) },
            0x3F => Instruction { name: "XXX", cycles: 7, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x40 => Instruction { name: "RTI", cycles: 6, operate: Box::new(CPU::rti), address_mode: Box::new(CPU::imp) },
            0x41 => Instruction { name: "EOR", cycles: 6, operate: Box::new(CPU::eor), address_mode: Box::new(CPU::idx) },
            0x42 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x43 => Instruction { name: "XXX", cycles: 8, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x44 => Instruction { name: "XXX", cycles: 3, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x45 => Instruction { name: "EOR", cycles: 3, operate: Box::new(CPU::eor), address_mode: Box::new(CPU::zpg) },
            0x46 => Instruction { name: "LSR", cycles: 5, operate: Box::new(CPU::lsr), address_mode: Box::new(CPU::zpg) },
            0x47 => Instruction { name: "XXX", cycles: 5, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x48 => Instruction { name: "PHA", cycles: 3, operate: Box::new(CPU::pha), address_mode: Box::new(CPU::imp) },
            0x49 => Instruction { name: "EOR", cycles: 2, operate: Box::new(CPU::eor), address_mode: Box::new(CPU::imm) },
            0x4A => Instruction { name: "LSR", cycles: 2, operate: Box::new(CPU::lsr), address_mode: Box::new(CPU::imp) },
            0x4B => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x4C => Instruction { name: "JMP", cycles: 3, operate: Box::new(CPU::jmp), address_mode: Box::new(CPU::abs) },
            0x4D => Instruction { name: "EOR", cycles: 4, operate: Box::new(CPU::eor), address_mode: Box::new(CPU::abs) },
            0x4E => Instruction { name: "LSR", cycles: 6, operate: Box::new(CPU::lsr), address_mode: Box::new(CPU::abs) },
            0x4F => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x50 => Instruction { name: "BVC", cycles: 2, operate: Box::new(CPU::bvc), address_mode: Box::new(CPU::rel) },
            0x51 => Instruction { name: "EOR", cycles: 5, operate: Box::new(CPU::eor), address_mode: Box::new(CPU::idy) },
            0x52 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x53 => Instruction { name: "XXX", cycles: 8, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x54 => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x55 => Instruction { name: "EOR", cycles: 4, operate: Box::new(CPU::eor), address_mode: Box::new(CPU::zpx) },
            0x56 => Instruction { name: "LSR", cycles: 6, operate: Box::new(CPU::lsr), address_mode: Box::new(CPU::zpx) },
            0x57 => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x58 => Instruction { name: "CLI", cycles: 2, operate: Box::new(CPU::cli), address_mode: Box::new(CPU::imp) },
            0x59 => Instruction { name: "EOR", cycles: 4, operate: Box::new(CPU::eor), address_mode: Box::new(CPU::aby) },
            0x5A => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x5B => Instruction { name: "XXX", cycles: 7, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x5C => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x5D => Instruction { name: "EOR", cycles: 4, operate: Box::new(CPU::eor), address_mode: Box::new(CPU::abx) },
            0x5E => Instruction { name: "LSR", cycles: 7, operate: Box::new(CPU::lsr), address_mode: Box::new(CPU::abx) },
            0x5F => Instruction { name: "XXX", cycles: 7, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x60 => Instruction { name: "RTS", cycles: 6, operate: Box::new(CPU::rts), address_mode: Box::new(CPU::imp) },
            0x61 => Instruction { name: "ADC", cycles: 6, operate: Box::new(CPU::adc), address_mode: Box::new(CPU::idx) },
            0x62 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x63 => Instruction { name: "XXX", cycles: 8, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x64 => Instruction { name: "XXX", cycles: 3, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x65 => Instruction { name: "ADC", cycles: 3, operate: Box::new(CPU::adc), address_mode: Box::new(CPU::zpg) },
            0x66 => Instruction { name: "ROR", cycles: 5, operate: Box::new(CPU::ror), address_mode: Box::new(CPU::zpg) },
            0x67 => Instruction { name: "XXX", cycles: 5, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x68 => Instruction { name: "PLA", cycles: 4, operate: Box::new(CPU::pla), address_mode: Box::new(CPU::imp) },
            0x69 => Instruction { name: "ADC", cycles: 2, operate: Box::new(CPU::adc), address_mode: Box::new(CPU::imm) },
            0x6A => Instruction { name: "ROR", cycles: 2, operate: Box::new(CPU::ror), address_mode: Box::new(CPU::imp) },
            0x6B => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x6C => Instruction { name: "JMP", cycles: 5, operate: Box::new(CPU::jmp), address_mode: Box::new(CPU::ind) },
            0x6D => Instruction { name: "ADC", cycles: 4, operate: Box::new(CPU::adc), address_mode: Box::new(CPU::abs) },
            0x6E => Instruction { name: "ROR", cycles: 6, operate: Box::new(CPU::ror), address_mode: Box::new(CPU::abs) },
            0x6F => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x70 => Instruction { name: "BVS", cycles: 2, operate: Box::new(CPU::bvs), address_mode: Box::new(CPU::rel) },
            0x71 => Instruction { name: "ADC", cycles: 5, operate: Box::new(CPU::adc), address_mode: Box::new(CPU::idy) },
            0x72 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x73 => Instruction { name: "XXX", cycles: 8, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x74 => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x75 => Instruction { name: "ADC", cycles: 4, operate: Box::new(CPU::adc), address_mode: Box::new(CPU::zpx) },
            0x76 => Instruction { name: "ROR", cycles: 6, operate: Box::new(CPU::ror), address_mode: Box::new(CPU::zpx) },
            0x77 => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x78 => Instruction { name: "SEI", cycles: 2, operate: Box::new(CPU::sei), address_mode: Box::new(CPU::imp) },
            0x79 => Instruction { name: "ADC", cycles: 4, operate: Box::new(CPU::adc), address_mode: Box::new(CPU::aby) },
            0x7A => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x7B => Instruction { name: "XXX", cycles: 7, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x7C => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x7D => Instruction { name: "ADC", cycles: 4, operate: Box::new(CPU::adc), address_mode: Box::new(CPU::abx) },
            0x7E => Instruction { name: "ROR", cycles: 7, operate: Box::new(CPU::ror), address_mode: Box::new(CPU::abx) },
            0x7F => Instruction { name: "XXX", cycles: 7, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x80 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x81 => Instruction { name: "STA", cycles: 6, operate: Box::new(CPU::sta), address_mode: Box::new(CPU::idx) },
            0x82 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x83 => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x84 => Instruction { name: "STY", cycles: 3, operate: Box::new(CPU::sty), address_mode: Box::new(CPU::zpg) },
            0x85 => Instruction { name: "STA", cycles: 3, operate: Box::new(CPU::sta), address_mode: Box::new(CPU::zpg) },
            0x86 => Instruction { name: "STX", cycles: 3, operate: Box::new(CPU::stx), address_mode: Box::new(CPU::zpg) },
            0x87 => Instruction { name: "XXX", cycles: 3, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x88 => Instruction { name: "DEY", cycles: 2, operate: Box::new(CPU::dey), address_mode: Box::new(CPU::imp) },
            0x89 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x8A => Instruction { name: "TXA", cycles: 2, operate: Box::new(CPU::txa), address_mode: Box::new(CPU::imp) },
            0x8B => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x8C => Instruction { name: "STY", cycles: 4, operate: Box::new(CPU::sty), address_mode: Box::new(CPU::abs) },
            0x8D => Instruction { name: "STA", cycles: 4, operate: Box::new(CPU::sta), address_mode: Box::new(CPU::abs) },
            0x8E => Instruction { name: "STX", cycles: 4, operate: Box::new(CPU::stx), address_mode: Box::new(CPU::abs) },
            0x8F => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x90 => Instruction { name: "BCC", cycles: 2, operate: Box::new(CPU::bcc), address_mode: Box::new(CPU::rel) },
            0x91 => Instruction { name: "STA", cycles: 6, operate: Box::new(CPU::sta), address_mode: Box::new(CPU::idy) },
            0x92 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x93 => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x94 => Instruction { name: "STY", cycles: 4, operate: Box::new(CPU::sty), address_mode: Box::new(CPU::zpx) },
            0x95 => Instruction { name: "STA", cycles: 4, operate: Box::new(CPU::sta), address_mode: Box::new(CPU::zpx) },
            0x96 => Instruction { name: "STX", cycles: 4, operate: Box::new(CPU::stx), address_mode: Box::new(CPU::zpy) },
            0x97 => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x98 => Instruction { name: "TYA", cycles: 2, operate: Box::new(CPU::tya), address_mode: Box::new(CPU::imp) },
            0x99 => Instruction { name: "STA", cycles: 5, operate: Box::new(CPU::sta), address_mode: Box::new(CPU::aby) },
            0x9A => Instruction { name: "TXS", cycles: 2, operate: Box::new(CPU::txs), address_mode: Box::new(CPU::imp) },
            0x9B => Instruction { name: "XXX", cycles: 5, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x9C => Instruction { name: "XXX", cycles: 5, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0x9D => Instruction { name: "STA", cycles: 5, operate: Box::new(CPU::sta), address_mode: Box::new(CPU::abx) },
            0x9E => Instruction { name: "XXX", cycles: 5, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0x9F => Instruction { name: "XXX", cycles: 5, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xA0 => Instruction { name: "LDY", cycles: 2, operate: Box::new(CPU::ldy), address_mode: Box::new(CPU::imm) },
            0xA1 => Instruction { name: "LDA", cycles: 6, operate: Box::new(CPU::lda), address_mode: Box::new(CPU::idx) },
            0xA2 => Instruction { name: "LDX", cycles: 2, operate: Box::new(CPU::ldx), address_mode: Box::new(CPU::imm) },
            0xA3 => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xA4 => Instruction { name: "LDY", cycles: 3, operate: Box::new(CPU::ldy), address_mode: Box::new(CPU::zpg) },
            0xA5 => Instruction { name: "LDA", cycles: 3, operate: Box::new(CPU::lda), address_mode: Box::new(CPU::zpg) },
            0xA6 => Instruction { name: "LDX", cycles: 3, operate: Box::new(CPU::ldx), address_mode: Box::new(CPU::zpg) },
            0xA7 => Instruction { name: "XXX", cycles: 3, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xA8 => Instruction { name: "TAY", cycles: 2, operate: Box::new(CPU::tay), address_mode: Box::new(CPU::imp) },
            0xA9 => Instruction { name: "LDA", cycles: 2, operate: Box::new(CPU::lda), address_mode: Box::new(CPU::imm) },
            0xAA => Instruction { name: "TAX", cycles: 2, operate: Box::new(CPU::tax), address_mode: Box::new(CPU::imp) },
            0xAB => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xAC => Instruction { name: "LDY", cycles: 4, operate: Box::new(CPU::ldy), address_mode: Box::new(CPU::abs) },
            0xAD => Instruction { name: "LDA", cycles: 4, operate: Box::new(CPU::lda), address_mode: Box::new(CPU::abs) },
            0xAE => Instruction { name: "LDX", cycles: 4, operate: Box::new(CPU::ldx), address_mode: Box::new(CPU::abs) },
            0xAF => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xB0 => Instruction { name: "BCS", cycles: 2, operate: Box::new(CPU::bcs), address_mode: Box::new(CPU::rel) },
            0xB1 => Instruction { name: "LDA", cycles: 5, operate: Box::new(CPU::lda), address_mode: Box::new(CPU::idy) },
            0xB2 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xB3 => Instruction { name: "XXX", cycles: 5, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xB4 => Instruction { name: "LDY", cycles: 4, operate: Box::new(CPU::ldy), address_mode: Box::new(CPU::zpx) },
            0xB5 => Instruction { name: "LDA", cycles: 4, operate: Box::new(CPU::lda), address_mode: Box::new(CPU::zpx) },
            0xB6 => Instruction { name: "LDX", cycles: 4, operate: Box::new(CPU::ldx), address_mode: Box::new(CPU::zpy) },
            0xB7 => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xB8 => Instruction { name: "CLV", cycles: 2, operate: Box::new(CPU::clv), address_mode: Box::new(CPU::imp) },
            0xB9 => Instruction { name: "LDA", cycles: 4, operate: Box::new(CPU::lda), address_mode: Box::new(CPU::aby) },
            0xBA => Instruction { name: "TSX", cycles: 2, operate: Box::new(CPU::tsx), address_mode: Box::new(CPU::imp) },
            0xBB => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xBC => Instruction { name: "LDY", cycles: 4, operate: Box::new(CPU::ldy), address_mode: Box::new(CPU::abx) },
            0xBD => Instruction { name: "LDA", cycles: 4, operate: Box::new(CPU::lda), address_mode: Box::new(CPU::abx) },
            0xBE => Instruction { name: "LDX", cycles: 4, operate: Box::new(CPU::ldx), address_mode: Box::new(CPU::aby) },
            0xBF => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xC0 => Instruction { name: "CPY", cycles: 2, operate: Box::new(CPU::cpy), address_mode: Box::new(CPU::imm) },
            0xC1 => Instruction { name: "CMP", cycles: 6, operate: Box::new(CPU::cmp), address_mode: Box::new(CPU::idx) },
            0xC2 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0xC3 => Instruction { name: "XXX", cycles: 8, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xC4 => Instruction { name: "CPY", cycles: 3, operate: Box::new(CPU::cpy), address_mode: Box::new(CPU::zpg) },
            0xC5 => Instruction { name: "CMP", cycles: 3, operate: Box::new(CPU::cmp), address_mode: Box::new(CPU::zpg) },
            0xC6 => Instruction { name: "DEC", cycles: 5, operate: Box::new(CPU::dec), address_mode: Box::new(CPU::zpg) },
            0xC7 => Instruction { name: "XXX", cycles: 5, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xC8 => Instruction { name: "INY", cycles: 2, operate: Box::new(CPU::iny), address_mode: Box::new(CPU::imp) },
            0xC9 => Instruction { name: "CMP", cycles: 2, operate: Box::new(CPU::cmp), address_mode: Box::new(CPU::imm) },
            0xCA => Instruction { name: "DEX", cycles: 2, operate: Box::new(CPU::dex), address_mode: Box::new(CPU::imp) },
            0xCB => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xCC => Instruction { name: "CPY", cycles: 4, operate: Box::new(CPU::cpy), address_mode: Box::new(CPU::abs) },
            0xCD => Instruction { name: "CMP", cycles: 4, operate: Box::new(CPU::cmp), address_mode: Box::new(CPU::abs) },
            0xCE => Instruction { name: "DEC", cycles: 6, operate: Box::new(CPU::dec), address_mode: Box::new(CPU::abs) },
            0xCF => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xD0 => Instruction { name: "BNE", cycles: 2, operate: Box::new(CPU::bne), address_mode: Box::new(CPU::rel) },
            0xD1 => Instruction { name: "CMP", cycles: 5, operate: Box::new(CPU::cmp), address_mode: Box::new(CPU::idy) },
            0xD2 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xD3 => Instruction { name: "XXX", cycles: 8, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xD4 => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0xD5 => Instruction { name: "CMP", cycles: 4, operate: Box::new(CPU::cmp), address_mode: Box::new(CPU::zpx) },
            0xD6 => Instruction { name: "DEC", cycles: 6, operate: Box::new(CPU::dec), address_mode: Box::new(CPU::zpx) },
            0xD7 => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xD8 => Instruction { name: "CLD", cycles: 2, operate: Box::new(CPU::cld), address_mode: Box::new(CPU::imp) },
            0xD9 => Instruction { name: "CMP", cycles: 4, operate: Box::new(CPU::cmp), address_mode: Box::new(CPU::aby) },
            0xDA => Instruction { name: "NOP", cycles: 2, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0xDB => Instruction { name: "XXX", cycles: 7, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xDC => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0xDD => Instruction { name: "CMP", cycles: 4, operate: Box::new(CPU::cmp), address_mode: Box::new(CPU::abx) },
            0xDE => Instruction { name: "DEC", cycles: 7, operate: Box::new(CPU::dec), address_mode: Box::new(CPU::abx) },
            0xDF => Instruction { name: "XXX", cycles: 7, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xE0 => Instruction { name: "CPX", cycles: 2, operate: Box::new(CPU::cpx), address_mode: Box::new(CPU::imm) },
            0xE1 => Instruction { name: "SBC", cycles: 6, operate: Box::new(CPU::sbc), address_mode: Box::new(CPU::idx) },
            0xE2 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0xE3 => Instruction { name: "XXX", cycles: 8, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xE4 => Instruction { name: "CPX", cycles: 3, operate: Box::new(CPU::cpx), address_mode: Box::new(CPU::zpg) },
            0xE5 => Instruction { name: "SBC", cycles: 3, operate: Box::new(CPU::sbc), address_mode: Box::new(CPU::zpg) },
            0xE6 => Instruction { name: "INC", cycles: 5, operate: Box::new(CPU::inc), address_mode: Box::new(CPU::zpg) },
            0xE7 => Instruction { name: "XXX", cycles: 5, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xE8 => Instruction { name: "INX", cycles: 2, operate: Box::new(CPU::inx), address_mode: Box::new(CPU::imp) },
            0xE9 => Instruction { name: "SBC", cycles: 2, operate: Box::new(CPU::sbc), address_mode: Box::new(CPU::imm) },
            0xEA => Instruction { name: "NOP", cycles: 2, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0xEB => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::sbc), address_mode: Box::new(CPU::imp) },
            0xEC => Instruction { name: "CPX", cycles: 4, operate: Box::new(CPU::cpx), address_mode: Box::new(CPU::abs) },
            0xED => Instruction { name: "SBC", cycles: 4, operate: Box::new(CPU::sbc), address_mode: Box::new(CPU::abs) },
            0xEE => Instruction { name: "INC", cycles: 6, operate: Box::new(CPU::inc), address_mode: Box::new(CPU::abs) },
            0xEF => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xF0 => Instruction { name: "BEQ", cycles: 2, operate: Box::new(CPU::beq), address_mode: Box::new(CPU::rel) },
            0xF1 => Instruction { name: "SBC", cycles: 5, operate: Box::new(CPU::sbc), address_mode: Box::new(CPU::idy) },
            0xF2 => Instruction { name: "XXX", cycles: 2, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xF3 => Instruction { name: "XXX", cycles: 8, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xF4 => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0xF5 => Instruction { name: "SBC", cycles: 4, operate: Box::new(CPU::sbc), address_mode: Box::new(CPU::zpx) },
            0xF6 => Instruction { name: "INC", cycles: 6, operate: Box::new(CPU::inc), address_mode: Box::new(CPU::zpx) },
            0xF7 => Instruction { name: "XXX", cycles: 6, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xF8 => Instruction { name: "SED", cycles: 2, operate: Box::new(CPU::sed), address_mode: Box::new(CPU::imp) },
            0xF9 => Instruction { name: "SBC", cycles: 4, operate: Box::new(CPU::sbc), address_mode: Box::new(CPU::aby) },
            0xFA => Instruction { name: "NOP", cycles: 2, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0xFB => Instruction { name: "XXX", cycles: 7, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
            0xFC => Instruction { name: "XXX", cycles: 4, operate: Box::new(CPU::nop), address_mode: Box::new(CPU::imp) },
            0xFD => Instruction { name: "SBC", cycles: 4, operate: Box::new(CPU::sbc), address_mode: Box::new(CPU::abx) },
            0xFE => Instruction { name: "INC", cycles: 7, operate: Box::new(CPU::inc), address_mode: Box::new(CPU::abx) },
            0xFF => Instruction { name: "XXX", cycles: 7, operate: Box::new(CPU::xxx), address_mode: Box::new(CPU::imp) },
        }
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
        cpu.set_status_flag(StatusFlag::N, true);

        let v = cpu.and();
        expect!(v).to(be_eq(1));
        expect!(cpu.a).to(be_eq(0b00000000));
        expect!(cpu.get_status_flag(StatusFlag::Z)).to(be_true());
        expect!(cpu.get_status_flag(StatusFlag::N)).to(be_false());
    }

    #[test]
    fn bit() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0b00000000);

        let mut cpu = CPU::new(&mut bus);
        cpu.a = 0b11111111;
        cpu.set_status_flag(StatusFlag::V, true);
        cpu.set_status_flag(StatusFlag::N, true);

        let v = cpu.bit();
        expect!(v).to(be_eq(0));
        expect!(cpu.a).to(be_eq(0b11111111)); // unchanged
        expect!(cpu.get_status_flag(StatusFlag::Z)).to(be_true());
        expect!(cpu.get_status_flag(StatusFlag::V)).to(be_false());
        expect!(cpu.get_status_flag(StatusFlag::N)).to(be_false());
    }

    #[test]
    fn eor() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0b11111111);

        let mut cpu = CPU::new(&mut bus);
        cpu.a = 0b11111111;
        cpu.set_status_flag(StatusFlag::N, true);

        let v = cpu.eor();
        expect!(v).to(be_eq(1));
        expect!(cpu.a).to(be_eq(0b00000000));
        expect!(cpu.get_status_flag(StatusFlag::Z)).to(be_true());
        expect!(cpu.get_status_flag(StatusFlag::N)).to(be_false());
    }

    #[test]
    fn ora() {
        let mut bus = Bus::new();
        bus.write(0x0000, 0b00000000);

        let mut cpu = CPU::new(&mut bus);
        cpu.a = 0b00000000;
        cpu.set_status_flag(StatusFlag::N, true);

        let v = cpu.ora();
        expect!(v).to(be_eq(1));
        expect!(cpu.a).to(be_eq(0b00000000));
        expect!(cpu.get_status_flag(StatusFlag::Z)).to(be_true());
        expect!(cpu.get_status_flag(StatusFlag::N)).to(be_false());
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
        cpu.opcode = 0x0A; // ASL with IMP addressing
        
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
        expect!(cpu.get_status_flag(StatusFlag::Z)).to(be_true());
        expect!(cpu.get_status_flag(StatusFlag::N)).to(be_false());
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
        expect!(cpu.get_status_flag(StatusFlag::Z)).to(be_true());
        expect!(cpu.get_status_flag(StatusFlag::N)).to(be_false());
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
        expect!(cpu.get_status_flag(StatusFlag::Z)).to(be_true());
        expect!(cpu.get_status_flag(StatusFlag::N)).to(be_false());
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
        expect!(cpu.get_status_flag(StatusFlag::Z)).to(be_true());
        expect!(cpu.get_status_flag(StatusFlag::N)).to(be_false());
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
        expect!(cpu.get_status_flag(StatusFlag::Z)).to(be_true());
        expect!(cpu.get_status_flag(StatusFlag::N)).to(be_false());
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
        expect!(cpu.get_status_flag(StatusFlag::Z)).to(be_true());
        expect!(cpu.get_status_flag(StatusFlag::N)).to(be_false());
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
        expect!(cpu.get_status_flag(StatusFlag::Z)).to(be_true());
        expect!(cpu.get_status_flag(StatusFlag::N)).to(be_false());
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
}