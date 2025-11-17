const std = @import("std");

const rom = @import("rom.zig");
const Bus = @import("bus.zig").Bus;
const opcodes = @import("opcodes.zig");
const Controllers = @import("controller.zig").Controllers;
const AdressingMode = opcodes.AdressingMode;

// =========================================
//               NES Adress Space
// =========================================
//  _______________ $10000  _______________
// | PRG-ROM       |       |               |
// | Upper Bank    |       |               |
// |_ _ _ _ _ _ _ _| $C000 | PRG-ROM       |
// | PRG-ROM       |       |               |
// | Lower Bank    |       |               |
// |_______________| $8000 |_______________|
// | SRAM          |       | SRAM          |
// |_______________| $6000 |_______________|
// | Expansion ROM |       | Expansion ROM |
// |_______________| $4020 |_______________|
// | I/O Registers |       |               |
// |_ _ _ _ _ _ _ _| $4000 |               |
// | Mirrors       |       | I/O Registers |
// | $2000-$2007   |       |               |
// |_ _ _ _ _ _ _ _| $2008 |               |
// | I/O Registers |       |               |
// |_______________| $2000 |_______________|
// | Mirrors       |       |               |
// | $0000-$07FF   |       |               |
// |_ _ _ _ _ _ _ _| $0800 |               |
// | RAM           |       | RAM           |
// |_ _ _ _ _ _ _ _| $0200 |               |
// | Stack         |       |               |
// |_ _ _ _ _ _ _ _| $0100 |               |
// | Zero Page     |       |               |
// |_______________| $0000 |_______________|

const RAM: u16 = 0x0000;
const RAM_MIRRORS_END: u16 = 0x2000;
const PPU_REGISTERS: u16 = 0x2000;
const PPU_REGISTERS_MIRRORS_END: u16 = 0x4000;

pub const ProcessorStatus = packed struct(u8) {
    /// The carry flag is set if the last operation caused an overflow from bit 7 of the result or an underflow
    /// from bit 0. This condition is set during arithmetic, comparison and during logical shifts. It can be explicitly
    /// set using the 'Set Carry Flag' (`SEC`) instruction and cleared with 'Clear Carry Flag' (`CLC`).
    carry_flag: bool = false,

    /// The zero flag is set if the result of the last operation as was zero.
    zero_flag: bool = false,

    /// The interrupt disable flag is set if the program has executed a 'Set Interrupt Disable' (`SEI`) instruction.
    /// While this flag is set the processor will not respond to interrupts from devices until it is cleared by a
    /// 'Clear Interrupt Disable' (`CLI`) instruction.
    interrupt_disable: bool = false,

    /// While the decimal mode flag is set the processor will obey the rules of Binary Coded Decimal (`BCD`) arithmetic
    /// during addition and subtraction. The flag can be explicitly set using 'Set Decimal Flag' (`SED`) and cleared
    /// with 'Clear Decimal Flag' (`CLD`).
    decimal_mode: bool = false,

    /// The break command bit is set when a instruction has been executed and an interrupt has been generated
    /// to process it.
    break_command: bool = false,
    break2: bool = false,

    /// The overflow flag is set during arithmetic operations if the result has yielded an invalid 2's complement
    /// result (e.g. adding to positive numbers and ending up with a negative result: 64 + 64 => -128). It is
    /// determined by looking at the carry between bits 6 and 7 and between bit 7 and the carry flag.
    overflow_flag: bool = false,

    /// The negative flag is set if the result of the last operation had bit 7 set to a one.
    negative_flag: bool = false,
};

pub const CPU = struct {
    /// *Program Counter*: points to the next instruction to be executed. The value of program counter is modified
    /// automatically as instructions are executed.
    ///
    /// The value of the program counter can be modified by executing a jump, a relative branch or a subroutine call to
    /// another memory address or by returning from a subroutine or interrupt.
    pc: u16,
    /// *Stack Pointer*: The processor supports a 256 byte stack located between `$0100` and `$01FF`. The stack pointer
    /// is an 8 bit register and holds the low 8 bits of the next free location on the stack. The location of the stack
    /// is fixed and cannot be moved.
    ///
    /// Pushing bytes to the stack causes the stack pointer to be decremented. Conversely pulling bytes causes it to be
    /// incremented.
    /// The CPU does not detect if the stack is overflowed by excessive pushing or pulling operations and will most
    /// likely result in the program crashing.
    sp: u8,
    /// *Accumulator*: used for all arithmetic and logical operations (with the exception of increments and decrements).
    /// The contents of the accumulator can be stored and retrieved either from memory or the stack.
    register_a: u8,
    /// *Index Register X*: most commonly used to hold counters or offsets for accessing memory. The value of the X
    /// register can be loaded and saved in memory, compared with values held in memory or incremented and decremented.
    ///
    /// The X register has one special function. It can be used to get a copy of the stack pointer or change its value.
    register_x: u8,
    /// *Index Register Y*: The Y register is similar to the X register in that it is available for holding counter or
    /// offsets memory access and supports the same set of memory load, save and compare operations as wells as
    /// increments and decrements. It has no special functions.
    register_y: u8,
    /// *Processor Status*: As instructions are executed a set of processor flags are set or clear to record the
    /// results of the operation. Each flag has a single bit within the register.
    status: ProcessorStatus,

    bus: *Bus,

    const Self = @This();

    const STACK_TOP: u8 = 0xFD;
    const STACK_START: u16 = 0x0100;
    const STACK_END: u16 = 0x01FF;

    const InterruptType = enum {
        NMI,
        IRQ,
    };
    const Interrupt = struct {
        type: InterruptType,
        vector_addr: u16,
        flags: ProcessorStatus,
        cycles: u8,
    };
    pub const IRQ = Interrupt{
        .type = .IRQ,
        .vector_addr = 0xFFFE,
        .flags = .{ .break_command = false, .break2 = true },
        .cycles = 7,
    };
    pub const NMI = Interrupt{
        .type = .NMI,
        .vector_addr = 0xFFFA,
        .flags = .{ .break_command = false, .break2 = true },
        .cycles = 8,
    };

    pub fn init(bus: *Bus) CPU {
        var initial_status = ProcessorStatus{};
        initial_status.interrupt_disable = true;
        initial_status.break2 = true;

        return .{
            .sp = STACK_TOP,
            .pc = 0x8000,
            .register_a = 0,
            .register_x = 0,
            .register_y = 0,
            .status = initial_status,
            .bus = bus,
        };
    }

    fn update_zero_and_negative_flags(self: *Self, result: u8) void {
        self.status.zero_flag = result == 0;
        // Check if bit 7 is set
        self.status.negative_flag = result & 0b1000_0000 != 0;
    }

    pub fn mem_read(self: *Self, addr: u16) u8 {
        return self.bus.mem_read(addr);
    }

    pub fn mem_write(self: *Self, addr: u16, data: u8) void {
        self.bus.mem_write(addr, data);
    }

    pub fn reset(self: *Self) void {
        self.register_a = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.sp = STACK_TOP;

        self.status = .{};
        self.status.interrupt_disable = true;
        self.status.break2 = true;

        self.pc = self.bus.mem_read_u16(0xFFFC);
    }

    fn stack_push(self: *Self, data: u8) void {
        var addr = STACK_START + self.sp;
        self.mem_write(addr, data);
        addr -%= 1;

        if (addr < STACK_START) {
            std.log.warn("stack overflow while pushing! addr: 0x{X:04}", .{addr});
        }
        self.sp = @truncate(addr);
    }

    fn stack_pop(self: *Self) u8 {
        self.sp +%= 1;
        const addr = STACK_START + self.sp;
        const data = self.mem_read(addr);

        if (addr > STACK_END) {
            std.log.warn("stack overflow while poping!", .{});
        }

        return data;
    }

    fn stack_push_u16(self: *Self, data: u16) void {
        const hi: u8 = @truncate(data >> 8);
        const lo: u8 = @truncate(data);

        self.stack_push(hi);
        self.stack_push(lo);
    }

    fn stack_pop_u16(self: *Self) u16 {
        const lo = self.stack_pop();
        const hi = self.stack_pop();
        return @as(u16, hi) << 8 | lo;
    }

    fn operand_address(self: *Self, opcode: opcodes.OpCode) u16 {
        const has_page_cross_penalty = opcode.has_page_cross_penalty();
        return switch (opcode.addressing_mode()) {
            AdressingMode.Immediate => self.pc,
            AdressingMode.ZeroPage => self.mem_read(self.pc),
            AdressingMode.ZeroPageX => {
                const lo = self.mem_read(self.pc);
                _ = self.mem_read(lo); // dummy read
                return lo +% self.register_x;
            },
            AdressingMode.ZeroPageY => {
                const lo = self.mem_read(self.pc);
                _ = self.mem_read(lo); // dummy read
                return lo +% self.register_y;
            },
            AdressingMode.Absolute => self.bus.mem_read_u16(self.pc),
            AdressingMode.AbsoluteX => {
                const lo = self.bus.mem_read(self.pc);
                const hi: u16 = self.bus.mem_read(self.pc + 1);
                const result = @addWithOverflow(lo, self.register_x);

                // Only do dummy read if the page or crossed or if it's a RMW instruction.
                // RMW instructions don't have page cross penalty so we can use that to identify them.
                if (result[1] == 1 or !has_page_cross_penalty) {
                    _ = self.bus.mem_read((hi << 8) | result[0]); // dummy read
                }

                if (result[1] == 1) {
                    if (has_page_cross_penalty) self.bus.cycles += 1;
                    return (@as(u16, (hi +% 1)) << 8) | result[0];
                } else {
                    return (hi << 8) | result[0];
                }
            },
            AdressingMode.AbsoluteY => {
                const lo = self.bus.mem_read(self.pc);
                const hi: u16 = self.bus.mem_read(self.pc + 1);
                const result = @addWithOverflow(lo, self.register_y);

                // Only do dummy read if the page or crossed or if it's a RMW instruction.
                // RMW instructions don't have page cross penalty so we can use that to identify them.
                if (result[1] == 1 or !has_page_cross_penalty) {
                    _ = self.bus.mem_read((hi << 8) | result[0]); // dummy read
                }

                if (result[1] == 1) {
                    if (has_page_cross_penalty) self.bus.cycles += 1;
                    return (@as(u16, (hi +% 1)) << 8) | result[0];
                } else {
                    return (hi << 8) | result[0];
                }
            },
            AdressingMode.Relative => {
                const offset: i8 = @bitCast(self.mem_read(self.pc));
                _ = self.mem_read(self.pc + 1); // dummy read
                const jump_addr: u32 = @bitCast(@as(i32, self.pc) +% 1 +% offset);
                if (jump_addr & 0xFF00 != (self.pc + 1) & 0xFF00) {
                    self.bus.cycles += 1;
                }
                return @truncate(jump_addr);
            },
            AdressingMode.Indirect => {
                const ptr_addr = self.bus.mem_read_u16(self.pc);

                // NOTE - 6502 bug mode with with page boundary:
                // if address $3000 contains $40, $30FF contains $80, and $3100 contains $50,
                // the result of JMP ($30FF) will be a transfer of control to $4080 rather than $5080 as you intended
                // i.e. the 6502 took the low byte of the address from $30FF and the high byte from $3000
                if (ptr_addr & 0xFF == 0xFF) {
                    const lo = self.mem_read(ptr_addr);
                    const hi = self.mem_read(ptr_addr & 0xFF00);

                    return @as(u16, hi) << 8 | lo;
                } else {
                    return self.bus.mem_read_u16(ptr_addr);
                }
            },
            AdressingMode.IndirectX => {
                const base = self.mem_read(self.pc);
                _ = self.mem_read(self.pc); // dummy read

                const ptr = base +% self.register_x;
                const lo = self.mem_read(ptr);
                const hi = self.mem_read(ptr +% 1);

                return @as(u16, hi) << 8 | lo;
            },
            AdressingMode.IndirectY => {
                const base = self.mem_read(self.pc);
                const lo = self.mem_read(base);
                const hi = self.mem_read(base +% 1);

                const result = @addWithOverflow(lo, self.register_y);
                if (result[1] == 1) {
                    if (has_page_cross_penalty) self.bus.cycles += 1;
                    _ = self.mem_read(@as(u16, hi) << 8 | result[0]); // dummy read
                    return @as(u16, hi +% 1) << 8 | result[0];
                } else {
                    return @as(u16, hi) << 8 | result[0];
                }
            },
            AdressingMode.Implicit => unreachable,
        };
    }

    fn branch(self: *Self, opcode: opcodes.OpCode, condition: bool) void {
        if (condition) {
            self.bus.cycles += 1;
            const jump_addr = self.operand_address(opcode);
            self.pc = jump_addr;
        }
    }

    fn compare(self: *Self, opcode: opcodes.OpCode, register: u8) void {
        const addr = self.operand_address(opcode);
        const data = self.mem_read(addr);
        const result = register -% data;

        self.status.carry_flag = register >= data;
        self.update_zero_and_negative_flags(result);
    }

    fn cmp(self: *Self, opcode: opcodes.OpCode) void {
        self.compare(opcode, self.register_a);
    }

    fn dec(self: *Self, opcode: opcodes.OpCode) void {
        const addr = self.operand_address(opcode);
        const value = self.mem_read(addr);
        const result = value -% 1;

        self.mem_write(addr, value); // dummy write

        self.mem_write(addr, result);
        self.update_zero_and_negative_flags(result);
    }

    fn adc(self: *Self, value: u8) void {
        const sum: u16 = @as(u16, self.register_a) + @as(u16, value) + @as(u16, @intFromBool(self.status.carry_flag));
        self.status.carry_flag = sum > 0xFF;

        const result: u8 = @truncate(sum);

        self.status.overflow_flag = (value ^ result) & (result ^ self.register_a) & 0x80 != 0;

        self.register_a = result;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn sbc(self: *Self, value: u8) void {
        self.adc(@as(u8, @bitCast(-%@as(i8, @bitCast(value)) -% 1)));
    }

    fn @"and"(self: *Self, opcode: opcodes.OpCode, register: u8) u8 {
        const addr = self.operand_address(opcode);
        const data = self.mem_read(addr);

        const result = register & data;
        self.update_zero_and_negative_flags(result);

        return result;
    }

    fn lsr_value(self: *Self, value: u8) u8 {
        self.status.carry_flag = value & 1 != 0;
        const result = value >> 1;
        self.update_zero_and_negative_flags(result);

        return result;
    }

    fn lsr(self: *Self, opcode: opcodes.OpCode) void {
        switch (opcode.addressing_mode()) {
            .Implicit => {
                self.register_a = self.lsr_value(self.register_a);
            },
            else => {
                const addr = self.operand_address(opcode);
                const value = self.mem_read(addr);

                self.mem_write(addr, value); // dummy write
                self.mem_write(addr, self.lsr_value(value));
            },
        }
    }

    fn eor(self: *Self, opcode: opcodes.OpCode) void {
        const data = self.mem_read(self.operand_address(opcode));
        self.register_a ^= data;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn asl(self: *Self, opcode: opcodes.OpCode) void {
        switch (opcode.addressing_mode()) {
            .Implicit => {
                const result = @mulWithOverflow(self.register_a, 2);

                self.register_a = result[0];
                self.status.carry_flag = result[1] == 1;
                self.update_zero_and_negative_flags(self.register_a);
            },
            else => {
                const addr = self.operand_address(opcode);
                const value = self.mem_read(addr);

                self.mem_write(addr, value); // dummy write

                const result = @mulWithOverflow(value, 2);

                self.mem_write(addr, result[0]);
                self.status.carry_flag = result[1] == 1;
                self.update_zero_and_negative_flags(result[0]);
            },
        }
    }

    fn rol(self: *Self, opcode: opcodes.OpCode) void {
        switch (opcode.addressing_mode()) {
            .Implicit => {
                const is_bit7_set = self.register_a & (1 << 7) != 0;

                self.register_a <<= 1;
                // set bit 0 to the value of carry flag
                self.register_a = self.register_a | @intFromBool(self.status.carry_flag);
                self.status.carry_flag = is_bit7_set;

                self.update_zero_and_negative_flags(self.register_a);
            },
            else => {
                const addr = self.operand_address(opcode);
                var value = self.mem_read(addr);
                const is_bit7_set = value & (1 << 7) != 0;

                self.mem_write(addr, value); // dummy write

                value <<= 1;
                // set bit 0 to the value of carry flag
                value = value | @intFromBool(self.status.carry_flag);
                self.mem_write(addr, value);
                self.status.carry_flag = is_bit7_set;

                self.update_zero_and_negative_flags(value);
            },
        }
    }

    fn ror(self: *Self, opcode: opcodes.OpCode) void {
        switch (opcode.addressing_mode()) {
            .Implicit => {
                const is_bit0_set = self.register_a & 1 != 0;

                self.register_a >>= 1;
                // set bit 7 to the value of carry flag
                self.register_a = (self.register_a & ~@as(u8, 0x80)) | @as(u8, @intFromBool(self.status.carry_flag)) << 7;
                self.status.carry_flag = is_bit0_set;

                self.update_zero_and_negative_flags(self.register_a);
            },
            else => {
                const addr = self.operand_address(opcode);
                var value = self.mem_read(addr);
                const is_bit0_set = value & 1 != 0;

                self.mem_write(addr, value); // dummy write

                value >>= 1;
                // set bit 7 to the value of carry flag
                value = (value & ~@as(u8, 0x80)) | @as(u8, @intFromBool(self.status.carry_flag)) << 7;
                self.mem_write(addr, value);
                self.status.carry_flag = is_bit0_set;

                self.update_zero_and_negative_flags(value);
            },
        }
    }

    fn ora(self: *Self, opcode: opcodes.OpCode) void {
        const value = self.mem_read(self.operand_address(opcode));
        self.register_a |= value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn inc(self: *Self, opcode: opcodes.OpCode) u8 {
        const addr = self.operand_address(opcode);
        const value = self.mem_read(addr);
        const result = value +% 1;

        self.mem_write(addr, value); // dummy write

        self.mem_write(addr, result);
        self.update_zero_and_negative_flags(result);

        return result;
    }

    fn lda(self: *Self, opcode: opcodes.OpCode) void {
        const addr = self.operand_address(opcode);
        const value = self.mem_read(addr);
        self.register_a = value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn ldx(self: *Self, opcode: opcodes.OpCode) void {
        const addr = self.operand_address(opcode);
        const value = self.mem_read(addr);
        self.register_x = value;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn txa(self: *Self) void {
        self.register_a = self.register_x;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn run_instructions(self: *Self, codes: []const u8) void {
        const start_addr = self.pc;
        while (self.pc < start_addr + codes.len) {
            self.tick();
        }
    }

    pub fn tick(self: *Self) void {
        const code = self.mem_read(self.pc);
        const opcode = opcodes.OP_CODES[code];
        self.pc += 1;
        const old_pc = self.pc;

        switch (opcode) {
            .ADC => {
                // NOTE: ignoring decimal mode
                const addr = self.operand_address(opcode);
                const data = self.mem_read(addr);
                self.adc(data);
            },
            .SBC => {
                // NOTE: ignoring decimal mode
                const addr = self.operand_address(opcode);
                const data = self.mem_read(addr);
                self.sbc(data);
            },
            .AND => {
                self.register_a = self.@"and"(opcode, self.register_a);
            },
            .EOR => self.eor(opcode),
            .ASL => self.asl(opcode),
            .LSR => self.lsr(opcode),
            .ROL => self.rol(opcode),
            .ROR => self.ror(opcode),
            .ORA => self.ora(opcode),
            .LDA => self.lda(opcode),
            .LDX => self.ldx(opcode),
            .LDY => {
                const addr = self.operand_address(opcode);
                const value = self.mem_read(addr);
                self.register_y = value;
                self.update_zero_and_negative_flags(self.register_y);
            },
            .TAX => {
                self.register_x = self.register_a;
                self.update_zero_and_negative_flags(self.register_x);
            },
            .TAY => {
                self.register_y = self.register_a;
                self.update_zero_and_negative_flags(self.register_y);
            },
            .TXA => self.txa(),
            .TYA => {
                self.register_a = self.register_y;
                self.update_zero_and_negative_flags(self.register_a);
            },
            .TSX => {
                self.register_x = self.sp;
                self.update_zero_and_negative_flags(self.register_x);
            },
            .TXS => {
                self.sp = self.register_x;
            },
            .INX => {
                self.register_x +%= 1;
                self.update_zero_and_negative_flags(self.register_x);
            },
            .INY => {
                self.register_y +%= 1;
                self.update_zero_and_negative_flags(self.register_y);
            },
            .INC => {
                _ = self.inc(opcode);
            },
            .STA => {
                const addr = self.operand_address(opcode);
                self.mem_write(addr, self.register_a);
            },
            .STX => {
                const addr = self.operand_address(opcode);
                self.mem_write(addr, self.register_x);
            },
            .STY => {
                const addr = self.operand_address(opcode);
                self.mem_write(addr, self.register_y);
            },
            .CMP => self.cmp(opcode),
            .CPX => self.compare(opcode, self.register_x),
            .CPY => self.compare(opcode, self.register_y),
            .BNE => self.branch(opcode, !self.status.zero_flag),
            .BEQ => self.branch(opcode, self.status.zero_flag),
            .BCC => self.branch(opcode, !self.status.carry_flag),
            .BCS => self.branch(opcode, self.status.carry_flag),
            .BPL => self.branch(opcode, !self.status.negative_flag),
            .BMI => self.branch(opcode, self.status.negative_flag),
            .BVC => self.branch(opcode, !self.status.overflow_flag),
            .BVS => self.branch(opcode, self.status.overflow_flag),
            .BIT => {
                const data = self.mem_read(self.operand_address(opcode));

                self.status.zero_flag = self.register_a & data == 0;
                self.status.overflow_flag = data & 0b0100_0000 != 0;
                self.status.negative_flag = data & 0b1000_0000 != 0;
            },
            .CLC => self.status.carry_flag = false,
            .CLD => self.status.decimal_mode = false,
            .SED => self.status.decimal_mode = true,
            .CLI => self.status.interrupt_disable = false,
            .CLV => self.status.overflow_flag = false,
            .SEI => self.status.interrupt_disable = true,
            .SEC => self.status.carry_flag = true,
            .DEC => self.dec(opcode),
            .JMP => {
                const jmp_addr = self.operand_address(opcode);
                self.pc = jmp_addr;
            },
            .JSR => {
                self.stack_push_u16(self.pc + 2 - 1);
                const jmp_addr = self.operand_address(opcode);
                self.pc = jmp_addr;
            },
            .RTS => {
                self.pc = self.stack_pop_u16() + 1;
            },
            .RTI => {
                const current_status = self.status;
                self.status = @bitCast(self.stack_pop());
                self.status.break_command = current_status.break_command;
                self.status.break2 = current_status.break2;

                self.pc = self.stack_pop_u16();
            },
            .DEX => {
                self.register_x -%= 1;
                self.update_zero_and_negative_flags(self.register_x);
            },
            .DEY => {
                self.register_y -%= 1;
                self.update_zero_and_negative_flags(self.register_y);
            },
            .PHA => self.stack_push(self.register_a),
            .PLA => {
                self.register_a = self.stack_pop();
                self.update_zero_and_negative_flags(self.register_a);
            },
            .PHP => {
                var copy_status = self.status;
                copy_status.break_command = true;
                copy_status.break2 = true;
                self.stack_push(@bitCast(copy_status));
            },
            .PLP => {
                const current_status = self.status;
                self.status = @bitCast(self.stack_pop());
                self.status.break_command = current_status.break_command;
                self.status.break2 = current_status.break2;
            },
            .BRK => {
                // skips the following byte
                self.pc += 1;

                var status = self.status;
                status.break2 = true;
                status.break_command = true;
                self.stack_push_u16(self.pc);
                self.stack_push(@bitCast(status));
                self.pc = self.bus.mem_read_u16(0xFFFE);
                self.status.interrupt_disable = true;
            },

            // UNOFFICIAL OPCODES:
            .ANC => {
                self.register_a = self.@"and"(opcode, self.register_a);
                self.status.carry_flag = self.register_a & 0x80 != 0;
            },
            .AXS => {
                const value = self.mem_read(self.operand_address(opcode));
                self.register_x &= self.register_a;
                self.status.carry_flag = self.register_x >= value;
                self.register_x -%= value;

                self.update_zero_and_negative_flags(self.register_x);
            },
            .ARR => {
                const addr = self.operand_address(opcode);
                const data = self.mem_read(addr);

                self.register_a &= data;

                self.register_a >>= 1;
                self.register_a = (self.register_a & ~@as(u8, 0x80)) | @as(u8, @intFromBool(self.status.carry_flag)) << 7;

                const bit5 = (self.register_a >> 5) & 1 != 0;
                const bit6 = (self.register_a >> 6) & 1 != 0;

                if (bit5 and bit6) { // Both bits are 1: set C, clear V
                    self.status.carry_flag = true;
                    self.status.overflow_flag = false;
                } else if (!bit5 and !bit6) { // Both bits are 0: clear C and V
                    self.status.carry_flag = false;
                    self.status.overflow_flag = false;
                } else if (bit5 and !bit6) { // Only bit 5 is 1: set V, clear C
                    self.status.carry_flag = false;
                    self.status.overflow_flag = true;
                } else { // Only bit 6 is 1: set C and V
                    self.status.carry_flag = true;
                    self.status.overflow_flag = true;
                }

                self.update_zero_and_negative_flags(self.register_a);
            },
            .ALR => self.register_a = self.lsr_value(self.@"and"(opcode, self.register_a)),
            .ATX => {
                const addr = self.operand_address(opcode);
                const data = self.mem_read(addr);

                self.register_a = data;
                self.register_x = data;
                self.update_zero_and_negative_flags(data);
            },
            .AXA => {
                const addr = self.operand_address(opcode);
                const result = self.register_x & self.register_a & 7;
                self.mem_write(addr, result);
            },
            .SAX => {
                const addr = self.operand_address(opcode);
                const result = self.register_a & self.register_x;
                self.mem_write(addr, result);
            },
            .DCP => {
                self.dec(opcode);
                self.cmp(opcode);
            },
            .ISC => self.sbc(self.inc(opcode)),
            .LAS => {
                const value = self.mem_read(self.operand_address(opcode));
                const result = value & self.sp;

                self.register_a = result;
                self.register_x = result;
                self.sp = result;

                self.update_zero_and_negative_flags(result);
            },
            .LAX => {
                const addr = self.operand_address(opcode);
                const value = self.mem_read(addr);

                self.register_a = value;
                self.update_zero_and_negative_flags(self.register_a);

                self.register_x = value;
                self.update_zero_and_negative_flags(self.register_x);
            },
            .RLA => {
                self.rol(opcode);
                self.register_a = self.@"and"(opcode, self.register_a);
            },
            .RRA => {
                self.ror(opcode);
                self.adc(self.mem_read(self.operand_address(opcode)));
            },
            .SLO => {
                self.asl(opcode);
                self.ora(opcode);
            },
            .SRE => {
                self.lsr(opcode);
                self.eor(opcode);
            },
            .SXA => {
                const addr = self.operand_address(opcode);
                const result = self.register_x & @as(u8, @truncate(addr >> 8)) + 1;

                const final_addr_hi = @as(u8, @truncate(addr >> 8)) & self.register_x;
                const final_addr_lo = @as(u8, @truncate(addr));
                const final_addr = (@as(u16, final_addr_hi) << 8) | final_addr_lo;

                self.mem_write(final_addr, result);
            },
            .SYA => {
                const addr = self.operand_address(opcode);
                const result = self.register_y & @as(u8, @truncate(addr >> 8)) + 1;

                const final_addr_hi = @as(u8, @truncate(addr >> 8)) & self.register_y;
                const final_addr_lo = @as(u8, @truncate(addr));
                const final_addr = (@as(u16, final_addr_hi) << 8) | final_addr_lo;

                self.mem_write(final_addr, result);
            },
            .XAA => {
                self.txa();
                self.register_a = self.@"and"(opcode, self.register_a);
            },
            .XAS => {
                const addr = self.operand_address(opcode);
                self.sp = self.register_x & self.register_a;
                const result = self.sp & @as(u8, @truncate(addr >> 8)) + 1;
                self.mem_write(addr, result);
            },
            .TOP, .DOP => {
                // No-op instructions, read argument and discard it.
                _ = self.operand_address(opcode);
            },
            .NOP, .KIL => {
                // Do nothing! Any arguments are ignored.
            },
        }

        // For instructions that have implicit adressing, the CPU will read the next byte of memory and then discard it.
        if (opcode.addressing_mode() == .Implicit) {
            _ = self.mem_read(old_pc); // dummy read
        }

        if (self.pc == old_pc) {
            self.pc += opcode.size() - 1;
        }

        self.bus.cycles += opcode.cycles();
    }

    pub fn interrupt(self: *Self, int: Interrupt) void {
        if (int.type == .IRQ and self.status.interrupt_disable) {
            return;
        }
        self.stack_push_u16(self.pc);
        var status = self.status;
        status.break_command = int.flags.break_command;
        status.break2 = int.flags.break2;

        self.stack_push(@bitCast(status));
        self.status.interrupt_disable = true;

        self.pc = self.bus.mem_read_u16(int.vector_addr);

        self.bus.cycles +%= int.cycles;
    }
};

test "0x00: BRK Force Interrupt" {
    const alloc = std.testing.allocator;
    //                          SEI   SEC   BRK   BCS         CLC   NOP
    const instructions = [_]u8{ 0x78, 0x38, 0x00, 0x6C, 0x02, 0x18, 0x1A };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    test_rom.prg_rom[0xFFFE] = 0x05;
    test_rom.prg_rom[0xFFFF] = 0x80;

    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.mem_write(0x1111, 0x06);
    cpu.mem_write(0x1112, 0x80);
    cpu.run_instructions(&instructions);

    // The carry flag is clear indicates that we jumped to the IRQ vector, otherwise the carry flag would be set.
    try std.testing.expect(!cpu.status.carry_flag);
}

test "0xA9: LDA immediate load data" {
    const alloc = std.testing.allocator;
    //                          LDA
    const instructions = [_]u8{ 0xA9, 0x05 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x05, cpu.register_a);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0xA9: LDA zero flag" {
    const alloc = std.testing.allocator;
    //                          LDA
    const instructions = [_]u8{ 0xA9, 0x00 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x00, cpu.register_a);
    try std.testing.expect(cpu.status.zero_flag);
}

test "0xAA: TAX copies register A contents to X" {
    const alloc = std.testing.allocator;
    //                          LDA        TAX
    const instructions = [_]u8{ 0xA9, 0xA, 0xAA };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(10, cpu.register_x);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0xA8: TAY Transfer Accumulator to Y" {
    const alloc = std.testing.allocator;
    //                          LDA         TAY
    const instructions = [_]u8{ 0xA9, 0x69, 0xA8 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x69, cpu.register_y);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0xBA: TSX Transfer Stack Pointer to X" {
    const alloc = std.testing.allocator;
    //                          TSX
    const instructions = [_]u8{0xBA};
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.sp = 0x05;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x05, cpu.register_x);
}

test "0x8A: TXA Transfer X to Accumulator" {
    const alloc = std.testing.allocator;
    //                          LDX         TXA
    const instructions = [_]u8{ 0xA2, 0x69, 0x8A };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x69, cpu.register_x);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);

    //                           LDX         TXA
    const instructions2 = [_]u8{ 0xA2, 0x00, 0x8A };
    var test_rom2 = rom.TestRom.init(alloc, &instructions2);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.run_instructions(&instructions2);

    try std.testing.expectEqual(0x00, cpu2.register_x);
    try std.testing.expect(cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
}

test "0x98: TYA Transfer Y to Accumulator" {
    const alloc = std.testing.allocator;
    //                          LDY         TYA
    const instructions = [_]u8{ 0xA0, 0x69, 0x98 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x69, cpu.register_a);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0x9A: TXS Transfer X to Stack Pointer" {
    const alloc = std.testing.allocator;
    //                          LDX         TXS
    const instructions = [_]u8{ 0xA2, 0x69, 0x9A };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x69, cpu.sp);
}

test "4 ops (LDA, TAX, INX,) working together" {
    const alloc = std.testing.allocator;
    //                          LDA        LDA         TAX   INX
    const instructions = [_]u8{ 0xA9, 0xA, 0xA9, 0xC0, 0xAA, 0xE8 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0xC0, cpu.register_a);
    try std.testing.expectEqual(0xC1, cpu.register_x);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(cpu.status.negative_flag);
}

test "INX overflow" {
    const alloc = std.testing.allocator;
    //                          LDA         TAX   INX   INX
    const instructions = [_]u8{ 0xA9, 0xFF, 0xAA, 0xE8, 0xE8 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_x);
}

test "0xA5: LDA from memory" {
    const alloc = std.testing.allocator;
    //                          LDA
    const instructions = [_]u8{ 0xA5, 0x10 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.mem_write(0x10, 0x55);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x55, cpu.register_a);
}

test "0xC9: CMP equal values" {
    const alloc = std.testing.allocator;
    //                          LDA         CMP
    const instructions = [_]u8{ 0xA9, 0x01, 0xC9, 0x01 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.mem_write(0x10, 0x55);
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.zero_flag);
}

test "0xC9: CMP different values" {
    const alloc = std.testing.allocator;
    //                          LDA         CMP
    const instructions = [_]u8{ 0xA9, 0x01, 0xC9, 0x03 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.mem_write(0x10, 0x55);
    cpu.run_instructions(&instructions);

    try std.testing.expect(!cpu.status.zero_flag);
}

test "0x69: ADC add with carry - no overflow" {
    const alloc = std.testing.allocator;
    //                          LDA         TAX         ADC
    const instructions = [_]u8{ 0xa9, 0xc0, 0xaa, 0xe8, 0x69, 0xc4 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x84, cpu.register_a);
    try std.testing.expectEqual(0xc1, cpu.register_x);
    try std.testing.expect(!cpu.status.overflow_flag);
    try std.testing.expect(cpu.status.carry_flag);
}

test "0x65: ADC add with carry - overflow" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         ADC
    const instructions = [_]u8{ 0xa9, 0x80, 0x85, 0x01, 0x65, 0x01 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu.register_a);
    try std.testing.expectEqual(0, cpu.register_x);
    try std.testing.expect(cpu.status.overflow_flag);
    try std.testing.expect(cpu.status.carry_flag);
}

test "0xE9: SBC Subtract with Carry - no overflow" {
    const alloc = std.testing.allocator;
    //                          LDA         SBC
    const instructions = [_]u8{ 0xa9, 0xF0, 0xE9, 0x50 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x9F, cpu.register_a);
    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.overflow_flag);
    try std.testing.expect(!cpu.status.zero_flag);
}

test "0xE9: SBC Subtract with Carry - overflow" {
    const alloc = std.testing.allocator;
    //                          LDA         SBC
    const instructions = [_]u8{ 0xa9, 0xD0, 0xE9, 0x70 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x5F, cpu.register_a);
    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(cpu.status.overflow_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);
}

test "0x29: logical AND - true result" {
    const alloc = std.testing.allocator;
    //                          LDA         AND
    const instructions = [_]u8{ 0xA9, 0x01, 0x29, 0x01 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x01, cpu.register_a);
}

test "0x29: logical AND - false result" {
    const alloc = std.testing.allocator;
    //                          LDA         AND
    const instructions = [_]u8{ 0xA9, 0x01, 0x29, 0x00 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu.register_a);
    try std.testing.expect(cpu.status.zero_flag);
}

test "0x0A: ASL Arithmetic Shift Left - carry flag set" {
    const alloc = std.testing.allocator;
    //                          LDA         ASL
    const instructions = [_]u8{ 0xA9, 0x80, 0x0A };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu.register_a);
    try std.testing.expect(cpu.status.carry_flag);
}

test "0x0A: ASL Arithmetic Shift Left - carry flag not set" {
    const alloc = std.testing.allocator;
    //                          LDA         ASL
    const instructions = [_]u8{ 0xA9, 0x01, 0x0A };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
}

test "0x06: ASL Arithmetic Shift Left - read value from memory" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         ASL
    const instructions = [_]u8{ 0xA9, 0x02, 0x85, 0xFF, 0x06, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(4, cpu.mem_read(0xFF));
    try std.testing.expect(!cpu.status.carry_flag);
}

test "0x4A: LSR Logical Shift Right - Accumulator" {
    const alloc = std.testing.allocator;
    //                          LDA         LSR
    const instructions = [_]u8{ 0xA9, 0x02, 0x4A };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);

    //                           LDA         LSR
    const instructions2 = [_]u8{ 0xA9, 0x03, 0x4A };
    var test_rom2 = rom.TestRom.init(alloc, &instructions2);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.run_instructions(&instructions2);

    try std.testing.expectEqual(1, cpu2.register_a);
    try std.testing.expect(cpu2.status.carry_flag);
}

test "0x46: LSR Logical Shift Right - read value from memory" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         LSR
    const instructions = [_]u8{ 0xA9, 0x02, 0x85, 0xFF, 0x46, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.mem_read(0xFF));
    try std.testing.expect(!cpu.status.carry_flag);

    //                           LDA         STA         LSR
    const instructions2 = [_]u8{ 0xA9, 0x03, 0x85, 0xFF, 0x46, 0xFF };
    var test_rom2 = rom.TestRom.init(alloc, &instructions2);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.run_instructions(&instructions2);

    try std.testing.expectEqual(1, cpu2.mem_read(0xFF));
    try std.testing.expect(cpu2.status.carry_flag);
}

test "0x09: ORA Logical Inclusive OR" {
    const alloc = std.testing.allocator;
    //                          LDA         ORA
    const instructions = [_]u8{ 0xA9, 0x02, 0x09, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0xFF, cpu.register_a);
}

test "0x2A: ROL Rotate Left" {
    const alloc = std.testing.allocator;
    //                          LDA         SEC   ROL
    const instructions = [_]u8{ 0xA9, 0x01, 0x38, 0x2A };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(3, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);

    //                           LDA         ROL
    const instructions2 = [_]u8{ 0xA9, 0x80, 0x2A };
    var test_rom2 = rom.TestRom.init(alloc, &instructions2);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.run_instructions(&instructions2);

    try std.testing.expectEqual(0, cpu2.register_a);
    try std.testing.expect(cpu2.status.carry_flag);
    try std.testing.expect(cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);

    //                           LDA         ROL
    const instructions3 = [_]u8{ 0xA9, 0x40, 0x2A };
    var test_rom3 = rom.TestRom.init(alloc, &instructions3);
    defer test_rom3.deinit();
    var bus3 = Bus.init(&test_rom3.rom, undefined, undefined);
    var cpu3 = CPU.init(&bus3);
    cpu3.run_instructions(&instructions3);

    try std.testing.expectEqual(0x80, cpu3.register_a);
    try std.testing.expect(!cpu3.status.carry_flag);
    try std.testing.expect(!cpu3.status.zero_flag);
    try std.testing.expect(cpu3.status.negative_flag);
}

test "0x26: ROL Rotate Left - read value from memory" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         SEC   ROL
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0xFF, 0x38, 0x26, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(3, cpu.mem_read(0xFF));
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);

    //                           LDA         STA         ROL
    const instructions2 = [_]u8{ 0xA9, 0x80, 0x85, 0xFF, 0x26, 0xFF };
    var test_rom2 = rom.TestRom.init(alloc, &instructions2);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.run_instructions(&instructions2);

    try std.testing.expectEqual(0, cpu2.mem_read(0xFF));
    try std.testing.expect(cpu2.status.carry_flag);
    try std.testing.expect(cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
}

test "0x6A: ROR Rotate Right" {
    const alloc = std.testing.allocator;
    //                          LDA         SEC   ROR
    const instructions = [_]u8{ 0xA9, 0x01, 0x38, 0x6A };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x80, cpu.register_a);
    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);

    //                           LDA         ROR
    const instructions2 = [_]u8{ 0xA9, 0x1, 0x6A };
    var test_rom2 = rom.TestRom.init(alloc, &instructions2);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.run_instructions(&instructions2);

    try std.testing.expectEqual(0, cpu2.register_a);
    try std.testing.expect(cpu2.status.carry_flag);
    try std.testing.expect(cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);

    //                           LDA         ROR
    const instructions3 = [_]u8{ 0xA9, 0x80, 0x6A };
    var test_rom3 = rom.TestRom.init(alloc, &instructions3);
    defer test_rom3.deinit();
    var bus3 = Bus.init(&test_rom3.rom, undefined, undefined);
    var cpu3 = CPU.init(&bus3);
    cpu3.run_instructions(&instructions3);

    try std.testing.expectEqual(0x40, cpu3.register_a);
    try std.testing.expect(!cpu3.status.carry_flag);
    try std.testing.expect(!cpu3.status.zero_flag);
    try std.testing.expect(!cpu3.status.negative_flag);
}

test "0x66: ROR Rotate Right - Read Value from Memory" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         SEC   ROR
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0xFF, 0x38, 0x66, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x80, cpu.mem_read(0xFF));
    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);

    //                           LDA        STA         ROR
    const instructions2 = [_]u8{ 0xA9, 0x1, 0x85, 0xFF, 0x66, 0xFF };
    var test_rom2 = rom.TestRom.init(alloc, &instructions2);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.run_instructions(&instructions2);

    try std.testing.expectEqual(0, cpu2.mem_read(0xFF));
    try std.testing.expect(cpu2.status.carry_flag);
    try std.testing.expect(cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);

    //                           LDA         STA         ROR
    const instructions3 = [_]u8{ 0xA9, 0x80, 0x85, 0xFF, 0x66, 0xFF };
    var test_rom3 = rom.TestRom.init(alloc, &instructions3);
    defer test_rom3.deinit();
    var bus3 = Bus.init(&test_rom3.rom, undefined, undefined);
    var cpu3 = CPU.init(&bus3);
    cpu3.run_instructions(&instructions3);

    try std.testing.expectEqual(0x40, cpu3.mem_read(0xFF));
    try std.testing.expect(!cpu3.status.carry_flag);
    try std.testing.expect(!cpu3.status.zero_flag);
    try std.testing.expect(!cpu3.status.negative_flag);
}

test "0xD0: BNE jump if not equal - skip INX instruction" {
    const alloc = std.testing.allocator;
    //                          LDA         CMP         BNE         INX
    const instructions = [_]u8{ 0xA9, 0x01, 0xC9, 0x03, 0xD0, 0x01, 0xE8 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.mem_write(0x10, 0x55);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu.register_x);
}

test "0xD0: BNE jump if not equal - execute INX instruction" {
    const alloc = std.testing.allocator;
    //                          LDA         CMP         BNE         INX
    const instructions = [_]u8{ 0xA9, 0x01, 0xC9, 0x01, 0xD0, 0x01, 0xE8 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.mem_write(0x10, 0x55);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_x);
}

test "0x90: BCC Branch if Carry Clear - branch" {
    const alloc = std.testing.allocator;
    //                          LDA         ASL   BCC         LDA
    const instructions = [_]u8{ 0xA9, 0x01, 0x0A, 0x90, 0x02, 0xA9, 0x03 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    // if the register A contains the value 2 that means we did branch, otherwise we would have the value 3.
    try std.testing.expectEqual(2, cpu.register_a);
}

test "0x90: BCC Branch if Carry Clear - no branch" {
    const alloc = std.testing.allocator;
    //                          LDA         ASL   BCC         LDA
    const instructions = [_]u8{ 0xA9, 0x80, 0x0A, 0x90, 0x02, 0xA9, 0x03 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    // if the register A contains the value 3 that means we didn't branch, otherwise we would have the value 0.
    try std.testing.expectEqual(3, cpu.register_a);
}

test "0xB0: BCS Branch if Carry Set - branch" {
    const alloc = std.testing.allocator;
    //                          LDA         ASL   BCS         LDA
    const instructions = [_]u8{ 0xA9, 0x80, 0x0A, 0xB0, 0x02, 0xA9, 0x03 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    // if the register A contains the value 0 that means we did branch, otherwise we would have the value 3.
    try std.testing.expectEqual(0, cpu.register_a);
}

test "0xB0: BCS Branch if Carry Set - no branch" {
    const alloc = std.testing.allocator;
    //                          LDA         ASL   BCS         LDA
    const instructions = [_]u8{ 0xA9, 0x01, 0x0A, 0xB0, 0x02, 0xA9, 0x03 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    // if the register A contains the value 3 that means we did branch, otherwise we would have the value 1.
    try std.testing.expectEqual(3, cpu.register_a);
}

test "0xF0: BEQ Branch if Equal - branch" {
    const alloc = std.testing.allocator;
    //                          LDA         CMP         BEQ         LDA
    const instructions = [_]u8{ 0xA9, 0x01, 0xC9, 0x01, 0xF0, 0x02, 0xA9, 0x03 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    // if the register A contains the value 1 that means we did branch, otherwise we would have the value 3.
    try std.testing.expectEqual(1, cpu.register_a);
}

test "0xF0: BEQ Branch if Equal - no branch" {
    const alloc = std.testing.allocator;
    //                          LDA         CMP         BEQ         LDA
    const instructions = [_]u8{ 0xA9, 0x01, 0xC9, 0x02, 0xF0, 0x02, 0xA9, 0x03 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    // if the register A contains the value 3 that means we did branch, otherwise we would have the value 1.
    try std.testing.expectEqual(3, cpu.register_a);
}

test "0x30: BMI Branch if Minus - branch" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         LDA         BIT         BMI         LDA
    const instructions = [_]u8{ 0xA9, 0x81, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x30, 0x02, 0xA9, 0x04 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    // if the register A contains the value 1 that means we did branch, otherwise we would have the value 4.
    try std.testing.expectEqual(1, cpu.register_a);
}

test "0x30: BMI Branch if Minus - no branch" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         LDA         BIT         BMI         LDA
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x30, 0x02, 0xA9, 0x04 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    // if the register A contains the value 4 that means we didn't branch, otherwise we would have the value 1.
    try std.testing.expectEqual(4, cpu.register_a);
}

test "0x10: BPL Branch if Positive - no branch" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         LDA         BIT         BPL         LDA
    const instructions = [_]u8{ 0xA9, 0x81, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x10, 0x02, 0xA9, 0x04 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    // if the register A contains the value 4 that means we didn't branch, otherwise we would have the value 1.
    try std.testing.expectEqual(4, cpu.register_a);
}

test "0x10: BPL Branch if Positive - branch" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         LDA         BIT         BPL         LDA
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x10, 0x02, 0xA9, 0x04 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    // if the register A contains the value 1 that means we did branch, otherwise we would have the value 4.
    try std.testing.expectEqual(1, cpu.register_a);
}

test "0x50: BVC Branch if Overflow Clear - no branch" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         LDA         BIT         BVC         LDA
    const instructions = [_]u8{ 0xA9, 0x40, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x50, 0x02, 0xA9, 0x04 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    // if the register A contains the value 4 that means we didn't branch, otherwise we would have the value 1.
    try std.testing.expectEqual(4, cpu.register_a);
}

test "0x50: BVC Branch if Overflow Clear  - branch" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         LDA         BIT         BVC         LDA
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x50, 0x02, 0xA9, 0x04 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    // if the register A contains the value 1 that means we did branch, otherwise we would have the value 4.
    try std.testing.expectEqual(1, cpu.register_a);
}

test "0x70: BVS Branch if Overflow Set - branch" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         LDA         BIT         BVS         LDA
    const instructions = [_]u8{ 0xA9, 0x40, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x70, 0x02, 0xA9, 0x04 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    // if the register A contains the value 1 that means we did branch, otherwise we would have the value 4.
    try std.testing.expectEqual(1, cpu.register_a);
}

test "0x70: BVS Branch if Overflow Set - no branch" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         LDA         BIT         BVS         LDA
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x70, 0x02, 0xA9, 0x04 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    // if the register A contains the value 4 that means we didn't branch, otherwise we would have the value 1.
    try std.testing.expectEqual(4, cpu.register_a);
}

test "0x24: BIT test - set N,V,Z flags" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         LDA         BIT
    const instructions = [_]u8{ 0xA9, 0xC0, 0x85, 0xFF, 0xA9, 0x02, 0x24, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu.register_a);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(cpu.status.overflow_flag);
    try std.testing.expect(cpu.status.zero_flag);
}

test "0x24: BIT test - set N,V flags" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         LDA         BIT
    const instructions = [_]u8{ 0xA9, 0xC2, 0x85, 0xFF, 0xA9, 0x02, 0x24, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu.register_a);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(cpu.status.overflow_flag);
    try std.testing.expect(!cpu.status.zero_flag);
}

test "0x24: BIT test - set N flags" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         LDA         BIT
    const instructions = [_]u8{ 0xA9, 0x81, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_a);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.overflow_flag);
    try std.testing.expect(!cpu.status.zero_flag);
}

test "0x18: CLC Clear Carry Flag" {
    const alloc = std.testing.allocator;
    //                          LDA         ASL   CLC
    const instructions = [_]u8{ 0xA9, 0x80, 0x0A, 0x18 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expect(!cpu.status.carry_flag);
}

test "0xF8: SED Set Decimal Flag" {
    const alloc = std.testing.allocator;
    //                          SED
    const instructions = [_]u8{0xF8};
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.decimal_mode);
}

test "0xD8: CLD Clear Decimal Flag" {
    const alloc = std.testing.allocator;
    //                          SED   CLD
    const instructions = [_]u8{ 0xF8, 0xD8 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expect(!cpu.status.decimal_mode);
}

test "0x78: SEI Set Interrupt Disable" {
    const alloc = std.testing.allocator;
    //                          SEI
    const instructions = [_]u8{0x78};
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.interrupt_disable);
}

test "0x38: SEC Set Carry Flag" {
    const alloc = std.testing.allocator;
    //                          SEC
    const instructions = [_]u8{0x38};
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.carry_flag);
}

test "0x58: CLI Clear Interrupt Disable" {
    const alloc = std.testing.allocator;
    //                          SEI   CLI
    const instructions = [_]u8{ 0x78, 0x58 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expect(!cpu.status.interrupt_disable);
}

test "0xB8: CLV Clear Overflow Flag" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         BIT         CLV
    const instructions = [_]u8{ 0xA9, 0xC2, 0x85, 0xFF, 0x24, 0xFF, 0xB8 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expect(!cpu.status.overflow_flag);
}

test "0xA2: LDX Load X Register" {
    const alloc = std.testing.allocator;
    //                          LDX
    const instructions = [_]u8{ 0xA2, 0x01 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_x);
}

test "0xA0: LDY Load Y Register" {
    const alloc = std.testing.allocator;
    //                          LDY
    const instructions = [_]u8{ 0xA0, 0x01 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_y);
}

test "0xE0: CPX Compare X Register" {
    const alloc = std.testing.allocator;
    //                          LDX         CPX
    const instructions = [_]u8{ 0xA2, 0x01, 0xE0, 0x01 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.zero_flag);
    try std.testing.expect(cpu.status.carry_flag);

    //                           LDX         CPX
    const instructions2 = [_]u8{ 0xA2, 0x01, 0xE0, 0x02 };
    var test_rom2 = rom.TestRom.init(alloc, &instructions2);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.run_instructions(&instructions2);

    try std.testing.expect(cpu2.status.negative_flag);
}

test "0xC0: CPY Compare Y Register" {
    const alloc = std.testing.allocator;
    //                          LDY         CPY
    const instructions = [_]u8{ 0xA0, 0x01, 0xC0, 0x01 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.zero_flag);
    try std.testing.expect(cpu.status.carry_flag);

    //                           LDY         CPY
    const instructions2 = [_]u8{ 0xA0, 0x01, 0xC0, 0x02 };
    var test_rom2 = rom.TestRom.init(alloc, &instructions2);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.run_instructions(&instructions2);

    try std.testing.expect(cpu2.status.negative_flag);
}

test "0xC6: DEC Decrement Memory" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         DEC
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0xFF, 0xC6, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expectEqual(0, cpu.mem_read(0xff));

    //                           LDA         STA         DEC
    const instructions2 = [_]u8{ 0xA9, 0x02, 0x85, 0xFF, 0xC6, 0xFF };
    var test_rom2 = rom.TestRom.init(alloc, &instructions2);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.run_instructions(&instructions2);

    try std.testing.expect(!cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
    try std.testing.expectEqual(1, cpu2.mem_read(0xff));

    //                           LDA         STA         DEC
    const instructions3 = [_]u8{ 0xA9, 0x81, 0x85, 0xFF, 0xC6, 0xFF };
    var test_rom3 = rom.TestRom.init(alloc, &instructions3);
    defer test_rom3.deinit();
    var bus3 = Bus.init(&test_rom3.rom, undefined, undefined);
    var cpu3 = CPU.init(&bus3);
    cpu3.run_instructions(&instructions3);

    try std.testing.expect(!cpu3.status.zero_flag);
    try std.testing.expect(cpu3.status.negative_flag);
    try std.testing.expectEqual(0x80, cpu3.mem_read(0xff));
}

test "0xCA: DEX Decrement X Register" {
    const alloc = std.testing.allocator;
    //                          LDX         DEX
    const instructions = [_]u8{ 0xA2, 0x02, 0xCA };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expectEqual(1, cpu.register_x);

    //                           LDX         DEX
    const instructions2 = [_]u8{ 0xA2, 0x01, 0xCA };
    var test_rom2 = rom.TestRom.init(alloc, &instructions2);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.run_instructions(&instructions2);

    try std.testing.expect(cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
    try std.testing.expectEqual(0, cpu2.register_x);

    //                           LDX         DEX
    const instructions3 = [_]u8{ 0xA2, 0x81, 0xCA };
    var test_rom3 = rom.TestRom.init(alloc, &instructions3);
    defer test_rom3.deinit();
    var bus3 = Bus.init(&test_rom3.rom, undefined, undefined);
    var cpu3 = CPU.init(&bus3);
    cpu3.run_instructions(&instructions3);

    try std.testing.expect(!cpu3.status.zero_flag);
    try std.testing.expect(cpu3.status.negative_flag);
    try std.testing.expectEqual(0x80, cpu3.register_x);

    //                           LDX         DEX
    const instructions4 = [_]u8{ 0xA2, 0x00, 0xCA };
    var test_rom4 = rom.TestRom.init(alloc, &instructions4);
    defer test_rom4.deinit();
    var bus4 = Bus.init(&test_rom4.rom, undefined, undefined);
    var cpu4 = CPU.init(&bus4);
    cpu4.run_instructions(&instructions4);

    try std.testing.expect(!cpu4.status.zero_flag);
    try std.testing.expect(cpu4.status.negative_flag);
    try std.testing.expectEqual(0xFF, cpu4.register_x);
}

test "0x88: DEY Decrement Y Register" {
    const alloc = std.testing.allocator;
    //                          LDY         DEY
    const instructions = [_]u8{ 0xA0, 0x02, 0x88 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expectEqual(1, cpu.register_y);

    //                           LDY         DEY
    const instructions2 = [_]u8{ 0xA0, 0x01, 0x88 };
    var test_rom2 = rom.TestRom.init(alloc, &instructions2);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.run_instructions(&instructions2);

    try std.testing.expect(cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
    try std.testing.expectEqual(0, cpu2.register_y);

    //                           LDY         DEY
    const instructions3 = [_]u8{ 0xA0, 0x81, 0x88 };
    var test_rom3 = rom.TestRom.init(alloc, &instructions3);
    defer test_rom3.deinit();
    var bus3 = Bus.init(&test_rom3.rom, undefined, undefined);
    var cpu3 = CPU.init(&bus3);
    cpu3.run_instructions(&instructions3);

    try std.testing.expect(!cpu3.status.zero_flag);
    try std.testing.expect(cpu3.status.negative_flag);
    try std.testing.expectEqual(0x80, cpu3.register_y);
}

test "0x49: EOR Exclusive OR" {
    const alloc = std.testing.allocator;
    //                          LDA         EOR
    const instructions = [_]u8{ 0xA9, 0x01, 0x49, 0x01 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);

    //                           LDA         EOR
    const instructions2 = [_]u8{ 0xA9, 0x01, 0x49, 0x80 };
    var test_rom2 = rom.TestRom.init(alloc, &instructions2);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.run_instructions(&instructions2);

    try std.testing.expect(!cpu2.status.zero_flag);
    try std.testing.expect(cpu2.status.negative_flag);
}

test "0xE8: INX increments X register by 1" {
    const alloc = std.testing.allocator;
    //                          LDA         TAX   INX
    const instructions = [_]u8{ 0xA9, 0x01, 0xAA, 0xE8 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu.register_x);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0xE6: INC Increment Memory" {
    const alloc = std.testing.allocator;
    //                          LDA         STA         INC         LDA
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0x10, 0xE6, 0x10, 0xA5, 0x10 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu.register_a);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0xC8: INY Increment Y Register" {
    const alloc = std.testing.allocator;
    //                          LDY         INY
    const instructions = [_]u8{ 0xA0, 0x01, 0xC8 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu.register_y);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0x6C: JMP Jump to address" {
    const alloc = std.testing.allocator;
    //                          LDA         JMP               LDA
    const instructions = [_]u8{ 0xA9, 0x42, 0x6C, 0x11, 0x11, 0xA9, 0x69 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.mem_write(0x1111, 0x07);
    cpu.mem_write(0x1112, 0x80);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x8007, cpu.pc);
    // if the register A contains the value 0x42 that means we jumped, otherwise we would have the value 0x69.
    try std.testing.expectEqual(0x42, cpu.register_a);
}

test "0x48: PHA Push Accumulator to Stack" {
    const alloc = std.testing.allocator;
    //                          LDA         PHA   LDA         PHA   LDA         PHA
    const instructions = [_]u8{ 0xA9, 0x01, 0x48, 0xA9, 0x02, 0x48, 0xA9, 0x03, 0x48 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x03, cpu.stack_pop());
    try std.testing.expectEqual(0x02, cpu.stack_pop());
    try std.testing.expectEqual(0x01, cpu.stack_pop());
}

test "0x68: PLA Pop Accumulator from Stack" {
    const alloc = std.testing.allocator;
    //                          LDA         PHA   LDA         PLA
    const instructions = [_]u8{ 0xA9, 0x01, 0x48, 0xA9, 0x02, 0x68 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x01, cpu.register_a);
}

test "0x20: JSR Jump to Subroutine" {
    const alloc = std.testing.allocator;
    //                          JSR               LDX         NOP   NOP   LDX
    const instructions = [_]u8{ 0x20, 0x05, 0x80, 0xA2, 0x01, 0x1A, 0x1A, 0xA2, 0x03 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.tick();
    try std.testing.expectEqual(0x8005, cpu.pc);

    cpu.tick();
    cpu.tick();
    cpu.tick();
    try std.testing.expectEqual(3, cpu.register_x);
}

test "0x60: RTS Return from Subroutine" {
    const alloc = std.testing.allocator;
    //                          JSR               JSR               JSR               LDX         RTS   INX   RTS
    const instructions = [_]u8{ 0x20, 0x09, 0x80, 0x20, 0x0C, 0x80, 0x20, 0x0E, 0x80, 0xA2, 0x00, 0x60, 0xE8, 0x60 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_x);
    try std.testing.expectEqual(0x800E, cpu.pc);
}

test "0x40: RTI Return from Interrupt" {
    const alloc = std.testing.allocator;
    //                          RTI
    const instructions = [_]u8{0x40};
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.status.carry_flag = true;
    cpu.status.negative_flag = true;

    cpu.stack_push_u16(cpu.pc + 1);
    cpu.stack_push(@bitCast(cpu.status));

    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expectEqual(0x8001, cpu.pc);
}

test "0x08: PHP Push Processor Status" {
    const alloc = std.testing.allocator;
    //                          SED   SEI   PHP
    const instructions = [_]u8{ 0xF8, 0x78, 0x08 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    const status: ProcessorStatus = @bitCast(cpu.stack_pop());
    try std.testing.expect(status.decimal_mode);
    try std.testing.expect(status.interrupt_disable);
}

test "0x28: PLP Pull Processor Status" {
    const alloc = std.testing.allocator;
    //                          SED   SEI   PHP   PLP
    const instructions = [_]u8{ 0xF8, 0x78, 0x08, 0x28 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.decimal_mode);
    try std.testing.expect(cpu.status.interrupt_disable);
}

test "0x85: STA Store Accumulator" {
    const alloc = std.testing.allocator;
    //                          LDA         STA
    const instructions = [_]u8{ 0xA9, 0x69, 0x85, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x69, cpu.mem_read(0x00FF));
}

test "0x86: STX Store X Register" {
    const alloc = std.testing.allocator;
    //                          LDX         STX
    const instructions = [_]u8{ 0xA2, 0x69, 0x86, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x69, cpu.mem_read(0x00FF));
}

test "0x84: STY Store Y Register" {
    const alloc = std.testing.allocator;
    //                          LDY         STY
    const instructions = [_]u8{ 0xA0, 0x69, 0x84, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x69, cpu.mem_read(0x00FF));
}

test "0x0B: ANC" {
    const alloc = std.testing.allocator;
    //                          LDA         ANC
    const instructions = [_]u8{ 0xA9, 0x80, 0x0B, 0x80 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x80, cpu.register_a);
    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);
}

test "0x2B: ANC" {
    const alloc = std.testing.allocator;
    //                          LDA         ANC
    const instructions = [_]u8{ 0xA9, 0x00, 0x2B, 0x80 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x00, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(cpu.status.zero_flag);
}

test "0xCB: AXS" {
    const alloc = std.testing.allocator;
    //                          AXS
    const instructions = [_]u8{ 0xCB, 0x05 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_a = 1;
    cpu.register_x = 3;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(252, cpu.register_x);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(cpu.status.negative_flag);
}

test "0x6B: ARR" {
    const alloc = std.testing.allocator;
    //                          ARR
    const instructions = [_]u8{ 0x6B, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_a = 0b1100_0000;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(96, cpu.register_a);
    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.overflow_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);

    var test_rom2 = rom.TestRom.init(alloc, &instructions);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.pc = 0x8000;
    cpu2.register_a = 0b0000_0001;
    cpu2.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu2.register_a);
    try std.testing.expect(!cpu2.status.carry_flag);
    try std.testing.expect(!cpu2.status.overflow_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
    try std.testing.expect(cpu2.status.zero_flag);

    var test_rom3 = rom.TestRom.init(alloc, &instructions);
    defer test_rom3.deinit();
    var bus3 = Bus.init(&test_rom3.rom, undefined, undefined);
    var cpu3 = CPU.init(&bus3);
    cpu3.pc = 0x8000;
    cpu3.register_a = 0b0100_0000;
    cpu3.run_instructions(&instructions);

    try std.testing.expectEqual(32, cpu3.register_a);
    try std.testing.expect(!cpu3.status.carry_flag);
    try std.testing.expect(cpu3.status.overflow_flag);
    try std.testing.expect(!cpu3.status.negative_flag);
    try std.testing.expect(!cpu3.status.zero_flag);

    var test_rom4 = rom.TestRom.init(alloc, &instructions);
    defer test_rom4.deinit();
    var bus4 = Bus.init(&test_rom4.rom, undefined, undefined);
    var cpu4 = CPU.init(&bus4);
    cpu4.pc = 0x8000;
    cpu4.register_a = 0b1000_0000;
    cpu4.run_instructions(&instructions);

    try std.testing.expectEqual(64, cpu4.register_a);
    try std.testing.expect(cpu4.status.carry_flag);
    try std.testing.expect(cpu4.status.overflow_flag);
    try std.testing.expect(!cpu4.status.negative_flag);
    try std.testing.expect(!cpu4.status.zero_flag);
}

test "0x4B: ALR" {
    const alloc = std.testing.allocator;
    //                          ALR
    const instructions = [_]u8{ 0x4B, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_a = 0b0000_0010;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);

    var test_rom2 = rom.TestRom.init(alloc, &instructions);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.pc = 0x8000;
    cpu2.register_a = 0b0000_0001;
    cpu2.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu2.register_a);
    try std.testing.expect(cpu2.status.carry_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
    try std.testing.expect(cpu2.status.zero_flag);
}

test "0xAB: ATX" {
    const alloc = std.testing.allocator;
    //                          ATX
    const instructions = [_]u8{ 0xAB, 0x2 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_a = 2;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu.register_x);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);

    var test_rom2 = rom.TestRom.init(alloc, &instructions);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.pc = 0x8000;
    cpu2.register_a = 1;
    cpu2.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu2.register_x);
    try std.testing.expect(!cpu2.status.negative_flag);
    try std.testing.expect(!cpu2.status.zero_flag);
}

test "0x9F: AXA" {
    const alloc = std.testing.allocator;
    //                          AXA
    const instructions = [_]u8{ 0x9F, 0x00, 0x10 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_a = 1;
    cpu.register_x = 3;
    cpu.register_y = 0x92;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.mem_read(0x1092));

    var test_rom2 = rom.TestRom.init(alloc, &instructions);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.pc = 0x8000;
    cpu2.register_a = 4;
    cpu2.register_x = 1;
    cpu2.register_y = 0x92;
    cpu2.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu2.mem_read(0x1092));
}

test "0x87: SAX" {
    const alloc = std.testing.allocator;
    //                          SAX
    const instructions = [_]u8{ 0x87, 0x05 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_x = 0x80;
    cpu.register_a = 0x80;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x80, cpu.mem_read(0x05));
}

test "0xC7: DCP" {
    const alloc = std.testing.allocator;
    //                          DCP
    const instructions = [_]u8{ 0xC7, 0x69 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.mem_write(0x69, 1);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu.mem_read(0x69));
    try std.testing.expect(cpu.status.carry_flag);

    var test_rom2 = rom.TestRom.init(alloc, &instructions);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.pc = 0x8000;
    cpu2.mem_write(0x69, 0);
    cpu2.run_instructions(&instructions);

    try std.testing.expectEqual(255, cpu2.mem_read(0x69));
    try std.testing.expect(!cpu2.status.carry_flag);
}

test "0xE2: DOP" {
    const alloc = std.testing.allocator;
    //                          DOP
    const instructions = [_]u8{ 0xE2, 0x69 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.run_instructions(&instructions);
}

test "0xE7: ISC" {
    const alloc = std.testing.allocator;
    //                          ISC
    const instructions = [_]u8{ 0xE7, 0x69 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_a = 1;
    cpu.mem_write(0x69, 2);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(3, cpu.mem_read(0x69));
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.overflow_flag);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);

    var test_rom2 = rom.TestRom.init(alloc, &instructions);
    defer test_rom2.deinit();
    var bus2 = Bus.init(&test_rom2.rom, undefined, undefined);
    var cpu2 = CPU.init(&bus2);
    cpu2.pc = 0x8000;
    cpu2.register_a = 5;
    cpu2.mem_write(0x69, 2);
    cpu2.run_instructions(&instructions);

    try std.testing.expect(cpu2.status.carry_flag);
    try std.testing.expect(!cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
    try std.testing.expect(!cpu2.status.overflow_flag);
}

test "0xBB: LAS" {
    const alloc = std.testing.allocator;
    //                          LAS
    const instructions = [_]u8{ 0xBB, 0x00, 0x10 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_y = 0x42;
    cpu.mem_write(0x1042, 8);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(8, cpu.sp);
    try std.testing.expectEqual(8, cpu.register_a);
    try std.testing.expectEqual(8, cpu.register_x);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);
}

test "0xA7: LAX" {
    const alloc = std.testing.allocator;
    //                          LAX
    const instructions = [_]u8{ 0xA7, 0xDF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.mem_write(0xDF, 42);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(42, cpu.register_a);
    try std.testing.expectEqual(42, cpu.register_x);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);
}

test "0x27: RLA" {
    const alloc = std.testing.allocator;
    //                          RLA
    const instructions = [_]u8{ 0x27, 0xDF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_a = 5;
    cpu.mem_write(0xDF, 42);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(4, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0x67: RRA" {
    const alloc = std.testing.allocator;
    //                          RRA
    const instructions = [_]u8{ 0x67, 0xDF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.mem_write(0xDF, 42);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(21, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.overflow_flag);
}

test "0x07: SLO" {
    const alloc = std.testing.allocator;
    //                          SLO
    const instructions = [_]u8{ 0x07, 0xDF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_a = 5;
    cpu.mem_write(0xDF, 42);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(85, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0x47: SRE" {
    const alloc = std.testing.allocator;
    //                          SRE
    const instructions = [_]u8{ 0x47, 0xDF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_a = 5;
    cpu.mem_write(0xDF, 42);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(16, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0x9E: SXA" {
    const alloc = std.testing.allocator;
    //                          SXA
    const instructions = [_]u8{ 0x9E, 0x00, 0x10 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_x = 5;
    cpu.register_y = 0x05;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.mem_read(0x1005));
}

test "0x9C: SYA" {
    const alloc = std.testing.allocator;
    //                          SYA
    const instructions = [_]u8{ 0x9C, 0x00, 0x10 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_x = 0x05;
    cpu.register_y = 5;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.mem_read(0x1005));
}

test "0x8B: XAA" {
    const alloc = std.testing.allocator;
    //                          XAA
    const instructions = [_]u8{ 0x8B, 0xFF };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_x = 5;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(5, cpu.register_a);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0x9B: XAS" {
    const alloc = std.testing.allocator;
    //                          XAS
    const instructions = [_]u8{ 0x9B, 0x00, 0x10 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.register_y = 5;
    cpu.register_x = 2;
    cpu.register_a = 2;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu.mem_read(0x1005));
    try std.testing.expectEqual(2, cpu.sp);
}

test "0xBD: LDA AbsoluteX - page cross" {
    const alloc = std.testing.allocator;
    // This test demonstrates the correct address calculation when an indexed
    // addressing mode crosses a page boundary.
    // On a real 6502, this operation
    // would incur an extra clock cycle penalty.
    //                          LDA   $02F0,X
    const instructions = [_]u8{ 0xBD, 0xF0, 0x02 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    // Set up the memory and registers for the page cross.
    // The base address is $02F0 (near the end of page 2).
    // We'll add an offset of $20 using the X register.
    // The final address will be $02F0 + $20 = $0310.
    // This crosses the boundary from page $02 to page $03.
    cpu.register_x = 0x20;
    cpu.mem_write(0x0310, 0x42);
    // Place the value to be loaded at the target address.

    cpu.run_instructions(&instructions);

    // Verify that the accumulator was loaded with the correct value
    // from the address in the next memory page, and flags are set correctly.
    try std.testing.expectEqual(0x42, cpu.register_a);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0xD0: BNE cycle count - branch not taken" {
    const alloc = std.testing.allocator;
    //                          LDA         CMP         BNE
    const instructions = [_]u8{ 0xA9, 0x01, 0xC9, 0x01, 0xD0, 0x05 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);

    const initial_cycles = bus.cycles;
    cpu.run_instructions(&instructions);
    const cycles_used = bus.cycles - initial_cycles;

    // LDA immediate: 2 cycles
    // CMP immediate: 2 cycles
    // BNE not taken: 2 cycles (base cost)
    try std.testing.expectEqual(6, cycles_used);
}

test "0xD0: BNE cycle count - branch taken, same page" {
    const alloc = std.testing.allocator;
    //                          LDA         CMP         BNE         INX   INX
    const instructions = [_]u8{ 0xA9, 0x01, 0xC9, 0x02, 0xD0, 0x02, 0xE8, 0xE8 };
    var test_rom = rom.TestRom.init(alloc, &instructions);
    defer test_rom.deinit();
    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);

    const initial_cycles = bus.cycles;
    cpu.run_instructions(&instructions);
    const cycles_used = bus.cycles - initial_cycles;

    // LDA immediate: 2 cycles
    // CMP immediate: 2 cycles
    // BNE taken (same page): 2 + 1 = 3 cycles
    try std.testing.expectEqual(7, cycles_used);
}

test "0xF0: BEQ cycle count - branch taken, page crossed backward" {
    const alloc = std.testing.allocator;
    var test_rom = rom.TestRom.init(alloc, &[_]u8{});
    defer test_rom.deinit();

    // Set up instructions at $8105 (start of page)
    // BEQ with negative offset to cross back to previous page
    test_rom.prg_rom[0x8100] = 0xA9; // LDA immediate
    test_rom.prg_rom[0x8101] = 0x01;
    test_rom.prg_rom[0x8102] = 0xC9; // CMP immediate
    test_rom.prg_rom[0x8103] = 0x01;
    test_rom.prg_rom[0x8104] = 0xF0; // BEQ
    test_rom.prg_rom[0x8105] = 0xF8; // -8 (branches back to $8103, crossing page)

    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.pc = 0x8100;

    // Execute LDA and CMP
    cpu.tick(); // LDA
    cpu.tick(); // CMP

    const before_branch = bus.cycles;
    cpu.tick(); // BEQ
    const branch_cycles = bus.cycles - before_branch;

    // BEQ with page cross: 2 (base) + 1 (branch taken) + 1 (page cross) = 4 cycles
    try std.testing.expectEqual(4, branch_cycles);
    try std.testing.expectEqual(0x80FE, cpu.pc);
}

test "0x90: BCC cycle count - branch taken, page crossed backward" {
    const alloc = std.testing.allocator;
    var test_rom = rom.TestRom.init(alloc, &[_]u8{});
    defer test_rom.deinit();

    test_rom.prg_rom[0xF000] = 0x90; // BCC
    test_rom.prg_rom[0xF001] = 0b1111_1100; // -4

    var bus = Bus.init(&test_rom.rom, undefined, undefined);
    var cpu = CPU.init(&bus);
    cpu.pc = 0xF000;
    cpu.status.carry_flag = false;

    const pc_before = cpu.pc;
    cpu.tick();

    try std.testing.expectEqual(cpu.pc, pc_before + 2 - 4);
    try std.testing.expectEqual(4, bus.cycles);
}
