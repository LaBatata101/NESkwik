const std = @import("std");

const rom = @import("rom.zig");

const PPU = @import("ppu.zig").PPU;
const Rom = rom.Rom;
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

const ProcessorStatus = packed struct(u8) {
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

    /// The break command bit is set when a BRK instruction has been executed and an interrupt has been generated
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

    /// The CPU has `0x0000..0x2000` of addressing space reserved for RAM space. The RAM `0x000..0x0800` (2KiB) is
    /// mirrored three times:
    ///   - `0x800..0x1000`
    ///   - `0x1000..0x1800`
    ///   - `0x1800..0x2000`
    /// This means that there is no difference in accessing memory addresses at `0x0000` or `0x0800` or
    /// `0x1000` or `0x1800` for reads or writes.
    ram: [2048]u8,
    rom: Rom,
    ppu: PPU,
    controllers: Controllers,

    cycles: u64,
    next_interrupt: u64,

    program_start_addr: u16,

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
    const IRQ = Interrupt{
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

    pub fn init(cartridge: Rom) CPU {
        var initial_status = ProcessorStatus{};
        initial_status.interrupt_disable = true;
        initial_status.break2 = true;

        var cpu = CPU{
            .sp = STACK_TOP,
            .pc = 0x8000,
            .register_a = 0,
            .register_x = 0,
            .register_y = 0,
            .status = initial_status,
            .program_start_addr = 0x8000,
            .rom = cartridge,
            .controllers = Controllers.init(),
            .ppu = PPU.init(cartridge.chr_rom, cartridge.mirroring),
            .cycles = 0,
            .next_interrupt = 0,
            .ram = [_]u8{0} ** 2048,
        };
        cpu.update_next_interrupt();
        return cpu;
    }

    fn update_zero_and_negative_flags(self: *Self, result: u8) void {
        self.status.zero_flag = result == 0;
        // Check if bit 7 is set
        self.status.negative_flag = result & 0b1000_0000 != 0;
    }

    pub fn mem_read(self: *Self, addr: u16) u8 {
        if (addr >= 0x2000 and addr < PPU_REGISTERS_MIRRORS_END) {
            self.run_ppu();
        }

        if (addr >= RAM and addr < RAM_MIRRORS_END) {
            return self.ram[addr & 0x07FF];
        } else if (addr == 0x2000 or addr == 0x2001 or addr == 0x2003 or addr == 0x2005 or addr == 0x2006 or addr == 0x4014) {
            std.log.warn("Attempt to read from write-only PPU address {X:04}", .{addr});
            return 0;
        } else if (addr == 0x2002) {
            return @bitCast(self.ppu.status_read());
        } else if (addr == 0x2004) {
            return self.ppu.oam_data_read();
        } else if (addr == 0x2007) {
            return self.ppu.data_read();
        } else if (addr >= 0x2008 and addr < PPU_REGISTERS_MIRRORS_END) {
            const mirror_down_addr = addr & 0b00100000_00000111;
            return self.mem_read(mirror_down_addr);
        } else if (addr >= 0x4000 and addr <= 0x4015) {
            // TODO: implement APU
            return 0;
        } else if (addr == 0x4016) {
            return self.controllers.cntrl1_read();
        } else if (addr == 0x4017) {
            return self.controllers.cntrl2_read();
        } else if (addr >= 0x8000 and addr <= 0xFFFF) {
            return self.read_prg_rom(addr);
        } else {
            std.log.warn("Ignoring mem read at 0x{X:04}", .{addr});
            return 0;
        }
    }

    pub fn mem_write(self: *Self, addr: u16, data: u8) void {
        if (addr >= 0x2000 and addr < PPU_REGISTERS_MIRRORS_END) {
            self.run_ppu();
        }

        if (addr >= RAM and addr < RAM_MIRRORS_END) {
            self.ram[addr & 0x07FF] = data;
        } else if (addr == 0x2000) {
            self.ppu.ctrl_write(data);
        } else if (addr == 0x2001) {
            self.ppu.mask_write(data);
        } else if (addr == 0x2003) {
            self.ppu.oam_addr_write(data);
        } else if (addr == 0x2004) {
            self.ppu.oam_data_write(data);
        } else if (addr == 0x2005) {
            self.ppu.scroll_write(data);
        } else if (addr == 0x2006) {
            self.ppu.addr_write(data);
        } else if (addr == 0x2007) {
            self.ppu.data_write(data);
        } else if (addr >= 0x2008 and addr < PPU_REGISTERS_MIRRORS_END) {
            const mirror_down_addr = addr & 0b00100000_00000111;
            self.mem_write(mirror_down_addr, data);
        } else if (addr >= 0x4000 and addr <= 0x4013 or addr == 0x4015) {
            // TODO: implement APU
        } else if (addr == 0x4014) {
            self.run_ppu();
            self.dma_transfer(data);
        } else if (addr >= 0x4016 and addr <= 0x4017) {
            self.controllers.set_strobe(data);
        } else if (addr >= 0x8000 and addr <= 0xFFFF) {
            @panic("Attempt to write to cartridge ROM memory space");
        } else {
            std.log.warn("Ignoring mem write at 0x{X:04}", .{addr});
        }
    }

    pub fn run_ppu(self: *Self) void {
        self.ppu.run_to(self.cycles);
        self.update_next_interrupt();
        if (self.ppu.nmi_interrupt) {
            self.ppu.nmi_interrupt = false;
            // TODO: This will cause a small desync between the CPU and cycles (around 9 cycles) breaking the 3:1
            // ratio. Execute PPU after interrupt to mantain the ratio?
            self.interrupt(CPU.NMI);
        }
    }

    pub fn update_next_interrupt(self: *Self) void {
        self.next_interrupt = @min(
            self.ppu.requested_run_cycle(),
            std.math.minInt(u64), // TODO add APU
        );
    }

    pub fn mem_read_u16(self: *Self, addr: u16) u16 {
        const lo = self.mem_read(addr);
        const hi = @as(u16, self.mem_read(addr + 1));
        return (hi << 8) | lo;
    }

    fn mem_write_u16(self: *Self, addr: u16, data: u16) void {
        const hi: u8 = @truncate(data >> 8);
        const lo: u8 = @truncate(data);
        self.mem_write(addr, lo);
        self.mem_write(addr + 1, hi);
    }

    // PRG Rom Size might be 16 KiB or 32 KiB. Because `0x8000..0x10000` mapped region is 32 KiB of addressable space,
    // the upper 16 KiB needs to be mapped to the lower 16 KiB (if a game has only 16 KiB of PRG ROM)
    fn read_prg_rom(self: Self, addr: u16) u8 {
        var new_addr = addr - 0x8000;
        // Check if the PRG Rom Size is 16 KiB and we're trying to access memory pass the 16 KiB.
        if (self.rom.prg_rom.len == 0x4000 and new_addr >= 0x4000) {
            new_addr = new_addr % 0x4000;
        }
        return self.rom.prg_rom[new_addr];
    }

    // https://www.nesdev.org/wiki/PPU_programmer_reference#OAMDMA_-_Sprite_DMA_($4014_write)
    fn dma_transfer(self: *Self, page: u8) void {
        if (self.cycles % 2 == 1) {
            self.cycles +%= 1;
        }
        self.cycles +%= 1;
        self.cycles +%= 512;

        const page_u16 = @as(u16, page) << 8;
        for (0..256) |x| {
            const addr = page_u16 | @as(u16, @intCast(x));
            const byte = self.mem_read(addr);
            self.ppu.oam_data_write(byte);
        }
    }

    pub fn reset(self: *Self) void {
        self.register_a = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.sp = STACK_TOP;

        self.status = .{};
        self.status.interrupt_disable = true;
        self.status.break2 = true;

        self.pc = self.mem_read_u16(0xFFFC);
    }

    fn stack_push(self: *Self, data: u8) void {
        var addr = STACK_START + self.sp;
        self.mem_write(addr, data);
        addr -%= 1;

        if (addr < STACK_START) {
            std.log.warn("stack overflow while pushing!", .{});
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

    fn operand_address(self: *Self, mode: AdressingMode) u16 {
        return switch (mode) {
            AdressingMode.Immediate => self.pc,
            AdressingMode.ZeroPage => self.mem_read(self.pc),
            AdressingMode.ZeroPageX => self.mem_read(self.pc) +% self.register_x,
            AdressingMode.ZeroPageY => self.mem_read(self.pc) +% self.register_y,
            AdressingMode.Absolute => self.mem_read_u16(self.pc),
            AdressingMode.AbsoluteX => self.mem_read_u16(self.pc) +% self.register_x,
            AdressingMode.AbsoluteY => self.mem_read_u16(self.pc) +% self.register_y,
            AdressingMode.Relative => {
                const offset = @as(i8, @bitCast(self.mem_read(self.pc)));
                return self.pc +% 1 +% @as(u16, @bitCast(@as(i16, offset)));
            },
            AdressingMode.Indirect => {
                const ptr_addr = self.mem_read_u16(self.pc);

                // NOTE - 6502 bug mode with with page boundary:
                // if address $3000 contains $40, $30FF contains $80, and $3100 contains $50,
                // the result of JMP ($30FF) will be a transfer of control to $4080 rather than $5080 as you intended
                // i.e. the 6502 took the low byte of the address from $30FF and the high byte from $3000
                if (ptr_addr & 0xFF == 0xFF) {
                    const lo = self.mem_read(ptr_addr);
                    const hi = self.mem_read(ptr_addr & 0xFF00);

                    return @as(u16, hi) << 8 | lo;
                } else {
                    return self.mem_read_u16(ptr_addr);
                }
            },
            AdressingMode.IndirectX => {
                const base = self.mem_read(self.pc);
                const ptr = base +% self.register_x;
                const lo = self.mem_read(ptr);
                const hi = self.mem_read(ptr +% 1);

                return @as(u16, hi) << 8 | lo;
            },
            AdressingMode.IndirectY => {
                const base = self.mem_read(self.pc);
                const lo = self.mem_read(base);
                const hi = self.mem_read(base +% 1);
                const ptr_addr_base = @as(u16, hi) << 8 | lo;

                return ptr_addr_base +% self.register_y;
            },
            AdressingMode.Implicit => unreachable,
        };
    }

    fn branch(self: *Self, mode: AdressingMode, condition: bool) void {
        if (condition) {
            const jump_addr = self.operand_address(mode);
            self.pc = jump_addr;
        }
    }

    fn compare(self: *Self, mode: AdressingMode, register: u8) void {
        const addr = self.operand_address(mode);
        const data = self.mem_read(addr);
        const result = register -% data;

        self.status.carry_flag = register >= data;
        self.update_zero_and_negative_flags(result);
    }

    fn cmp(self: *Self, opcode: opcodes.OpCode) void {
        self.compare(opcode.addressing_mode(), self.register_a);
    }

    fn dec(self: *Self, opcode: opcodes.OpCode) void {
        const addr = self.operand_address(opcode.addressing_mode());
        const data = self.mem_read(addr);
        const result = data -% 1;

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
        const addr = self.operand_address(opcode.addressing_mode());
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
                const addr = self.operand_address(opcode.addressing_mode());
                const value = self.mem_read(addr);
                self.mem_write(addr, self.lsr_value(value));
            },
        }
    }

    fn eor(self: *Self, opcode: opcodes.OpCode) void {
        const data = self.mem_read(self.operand_address(opcode.addressing_mode()));
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
                const addr = self.operand_address(opcode.addressing_mode());
                const value = self.mem_read(addr);

                const result = @mulWithOverflow(value, 2);

                self.mem_write(addr, result[0]);
                self.status.carry_flag = result[1] == 1;
                self.update_zero_and_negative_flags(value);
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
                const addr = self.operand_address(opcode.addressing_mode());
                var value = self.mem_read(addr);
                const is_bit7_set = value & (1 << 7) != 0;

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
                const addr = self.operand_address(opcode.addressing_mode());
                var value = self.mem_read(addr);
                const is_bit0_set = value & 1 != 0;

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
        const value = self.mem_read(self.operand_address(opcode.addressing_mode()));
        self.register_a |= value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn inc(self: *Self, opcode: opcodes.OpCode) u8 {
        const addr = self.operand_address(opcode.addressing_mode());
        const value = self.mem_read(addr);
        const result = value +% 1;

        self.mem_write(addr, result);
        self.update_zero_and_negative_flags(result);

        return result;
    }

    fn lda(self: *Self, opcode: opcodes.OpCode) void {
        const addr = self.operand_address(opcode.addressing_mode());
        const value = self.mem_read(addr);
        self.register_a = value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn ldx(self: *Self, opcode: opcodes.OpCode) void {
        const addr = self.operand_address(opcode.addressing_mode());
        const value = self.mem_read(addr);
        self.register_x = value;
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn txa(self: *Self) void {
        self.register_a = self.register_x;
        self.update_zero_and_negative_flags(self.register_a);
    }

    pub fn run(self: *Self) void {
        const T = struct {
            fn callback(_: *Self) void {
                // do nothing
            }
        };
        self.run_with(T);
    }

    pub fn run_with(self: *Self, callback: anytype) void {
        var continue_exec = true;
        while (continue_exec) {
            callback.callback(self);
            continue_exec = self.tick();
        }
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
        const pc_state = self.pc;

        switch (opcode) {
            .ADC => {
                // NOTE: ignoring decimal mode
                const addr = self.operand_address(opcode.addressing_mode());
                const data = self.mem_read(addr);
                self.adc(data);
            },
            .SBC => {
                // NOTE: ignoring decimal mode
                const addr = self.operand_address(opcode.addressing_mode());
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
                const addr = self.operand_address(opcode.addressing_mode());
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
                const addr = self.operand_address(opcode.addressing_mode());
                self.mem_write(addr, self.register_a);
            },
            .STX => {
                const addr = self.operand_address(opcode.addressing_mode());
                self.mem_write(addr, self.register_x);
            },
            .STY => {
                const addr = self.operand_address(opcode.addressing_mode());
                self.mem_write(addr, self.register_y);
            },
            .CMP => self.cmp(opcode),
            .CPX => {
                self.compare(opcode.addressing_mode(), self.register_x);
            },
            .CPY => {
                self.compare(opcode.addressing_mode(), self.register_y);
            },
            .BNE => {
                self.branch(opcode.addressing_mode(), !self.status.zero_flag);
            },
            .BEQ => {
                self.branch(opcode.addressing_mode(), self.status.zero_flag);
            },
            .BCC => {
                self.branch(opcode.addressing_mode(), !self.status.carry_flag);
            },
            .BCS => {
                self.branch(opcode.addressing_mode(), self.status.carry_flag);
            },
            .BPL => {
                self.branch(opcode.addressing_mode(), !self.status.negative_flag);
            },
            .BMI => {
                self.branch(opcode.addressing_mode(), self.status.negative_flag);
            },
            .BVC => {
                self.branch(opcode.addressing_mode(), !self.status.overflow_flag);
            },
            .BVS => {
                self.branch(opcode.addressing_mode(), self.status.overflow_flag);
            },
            .BIT => {
                const data = self.mem_read(self.operand_address(opcode.addressing_mode()));

                self.status.zero_flag = self.register_a & data == 0;
                self.status.overflow_flag = data & 0b0100_0000 != 0;
                self.status.negative_flag = data & 0b1000_0000 != 0;
            },
            .CLC => {
                self.status.carry_flag = false;
            },
            .CLD => {
                self.status.decimal_mode = false;
            },
            .SED => {
                self.status.decimal_mode = true;
            },
            .CLI => {
                self.status.interrupt_disable = false;
            },
            .CLV => {
                self.status.overflow_flag = false;
            },
            .SEI => {
                self.status.interrupt_disable = true;
            },
            .SEC => {
                self.status.carry_flag = true;
            },
            .DEC => self.dec(opcode),
            .JMP => {
                const jmp_addr = self.operand_address(opcode.addressing_mode());
                self.pc = jmp_addr;
            },
            .JSR => {
                self.stack_push_u16(self.pc + 2 - 1);
                const jmp_addr = self.operand_address(opcode.addressing_mode());
                self.pc = jmp_addr;
            },
            .RTS => {
                self.pc = self.stack_pop_u16() + 1;
            },
            .RTI => {
                self.status = @bitCast(self.stack_pop());
                self.status.break_command = false;
                self.status.break2 = true;

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
            .PHA => {
                self.stack_push(self.register_a);
            },
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
                self.status = @bitCast(self.stack_pop());
                self.status.break_command = false;
                self.status.break2 = true;
            },
            .BRK => {
                // TODO: properly implement this
                self.status.break_command = true;
                // self.stack_push_u16(self.pc);
                // self.stack_push(@bitCast(self.status));
                // self.pc = self.mem_read_u16(0xFFFE);
                return;
            },

            // UNOFFICIAL OPCODES:
            .ANC => {
                self.register_a = self.@"and"(opcode, self.register_a);
                self.status.carry_flag = self.register_a & 0x80 != 0;
            },
            .AXS => {
                const value = self.mem_read(self.operand_address(opcode.addressing_mode()));
                self.register_x &= self.register_a;
                self.status.carry_flag = value > self.register_x;
                self.register_x -%= value;

                self.update_zero_and_negative_flags(self.register_x);
            },
            .ARR => {
                const addr = self.operand_address(opcode.addressing_mode());
                const data = self.mem_read(addr);

                self.register_a &= data;
                self.register_a >>= 1;

                self.status.carry_flag = self.register_a & 0b0100_0000 != 0;
                self.status.overflow_flag = (self.register_a >> 5) & 1 ^ (self.register_a >> 6) & 1 != 0;
                self.update_zero_and_negative_flags(self.register_a);
            },
            .ALR => {
                self.register_a = self.lsr_value(self.@"and"(opcode, self.register_a));
            },
            .ATX => {
                self.register_x = self.@"and"(opcode, self.register_a);
            },
            .AXA => {
                const addr = self.operand_address(opcode.addressing_mode());
                const result = self.register_x & self.register_a & 7;
                self.mem_write(addr, result);
            },
            .SAX => {
                const addr = self.operand_address(opcode.addressing_mode());
                const result = self.register_a & self.register_x;
                self.mem_write(addr, result);
            },
            .DCP => {
                self.dec(opcode);
                self.cmp(opcode);
            },
            .ISC => self.sbc(self.inc(opcode)),
            .LAS => {
                const value = self.mem_read(self.operand_address(opcode.addressing_mode()));
                const result = value & self.sp;

                self.register_a = result;
                self.register_x = result;
                self.sp = result;

                self.update_zero_and_negative_flags(result);
            },
            .LAX => {
                self.lda(opcode);
                self.ldx(opcode);
            },
            .RLA => {
                self.rol(opcode);
                self.register_a = self.@"and"(opcode, self.register_a);
            },
            .RRA => {
                self.ror(opcode);
                self.adc(self.mem_read(self.operand_address(opcode.addressing_mode())));
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
                const addr = self.operand_address(opcode.addressing_mode());
                const result = self.register_x & @as(u8, @truncate(addr >> 8)) + 1;
                self.mem_write(addr, result);
            },
            .SYA => {
                const addr = self.operand_address(opcode.addressing_mode());
                const result = self.register_y & @as(u8, @truncate(addr >> 8)) + 1;
                self.mem_write(addr, result);
            },
            .XAA => {
                self.txa();
                self.register_a = self.@"and"(opcode, self.register_a);
            },
            .XAS => {
                const addr = self.operand_address(opcode.addressing_mode());
                self.sp = self.register_x & self.register_a;
                const result = self.sp & @as(u8, @truncate(addr >> 8)) + 1;
                self.mem_write(addr, result);
            },
            .NOP, .TOP, .DOP, .KIL => {
                // Do nothing! Any arguments are ignored.
            },
        }

        var instruction_cycles = opcode.cycles();
        const branch_succeeded = self.pc != pc_state;
        // Add an extra cycle if the branching instruction succeeds
        if (opcode.is_branching() and branch_succeeded) {
            instruction_cycles += 1;
            // Check if the page was crossed by veryfying if the upper byte of the address has changed.
            // Accessing data on a different page costs two extra cycles on branching instructions.
            if (does_page_cross(self, opcode)) {
                instruction_cycles += 1;
            }
        }
        // Check if the page was crossed by veryfying if the upper byte of the address has changed.
        // Accessing data on a different page costs an extra cycle.
        else if (does_page_cross(self, opcode)) {
            instruction_cycles += 1;
        }

        if (self.pc == pc_state) {
            self.pc += opcode.size() - 1;
        }

        self.cycles += instruction_cycles;
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

        self.pc = self.mem_read_u16(int.vector_addr);

        self.cycles +%= int.cycles;
        self.next_interrupt = 0;
    }
};

fn does_page_cross(cpu: *CPU, opcode: opcodes.OpCode) bool {
    const mode = opcode.addressing_mode();
    switch (mode) {
        .AbsoluteX, .AbsoluteY => switch (opcode) {
            .ADC, .AND, .CMP, .EOR, .LDA, .LDX, .LDY, .ORA, .SBC, .LAS, .LAX, .TOP => {
                const base = cpu.mem_read_u16(cpu.pc);
                return @as(u8, @truncate(base >> 8)) != @as(u8, @truncate(cpu.operand_address(mode) >> 8));
            },
            else => return false,
        },
        .IndirectY => switch (opcode) {
            .ADC, .SBC, .AND, .EOR, .ORA, .LDA, .LAX => {
                const base = @as(u16, cpu.mem_read(cpu.pc +% 1)) << 8 | cpu.mem_read(cpu.pc);
                return @as(u8, @truncate(base >> 8)) != @as(u8, @truncate(cpu.operand_address(mode) >> 8));
            },
            else => return false,
        },
        else => return false,
    }
}

// test "0x00: BRK Force Interrupt" {
//     const allocator = std.testing.allocator;
//     const test_rom = try rom.TestRom.testRom(
//         allocator,
//         //      SEI   SEC   BRK   LDA
//         &[_]u8{ 0x78, 0x38, 0x00, 0xA9, 0x01 },
//     );
//     var bus = Bus.init(try Rom.load(test_rom));
//     defer allocator.free(test_rom);
//
//     var cpu = CPU.init(&bus);
//     cpu.mem_write_u16(0xFFFE, 0x1234);
//     cpu.run();
//
//     try std.testing.expect(cpu.status.break_command);
//     try std.testing.expectEqual(0x1234, cpu.pc);
//
//     const status: ProcessorStatus = @bitCast(cpu.stack_pop());
//
//     try std.testing.expect(status.interrupt_disable);
//     try std.testing.expect(status.carry_flag);
//     try std.testing.expectEqual(0x8003, cpu.stack_pop_u16());
// }

test "0xA9: LDA immediate load data" {
    //                          LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x05, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x05, cpu.register_a);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0xA9: LDA zero flag" {
    //                          LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x00, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x00, cpu.register_a);
    try std.testing.expect(cpu.status.zero_flag);
}

test "0xAA: TAX copies register A contents to X" {
    //                          LDA        TAX   BRK
    const instructions = [_]u8{ 0xA9, 0xA, 0xAA, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(10, cpu.register_x);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0xA8: TAY Transfer Accumulator to Y" {
    //                          LDA         TAY   BRK
    const instructions = [_]u8{ 0xA9, 0x69, 0xA8, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x69, cpu.register_y);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0xBA: TSX Transfer Stack Pointer to X" {
    //                          TSX   BRK
    const instructions = [_]u8{ 0xBA, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.sp = 0x05;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x05, cpu.register_x);
}

test "0x8A: TXA Transfer X to Accumulator" {
    //                          LDX         TXA   BRK
    const instructions = [_]u8{ 0xA2, 0x69, 0x8A, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x69, cpu.register_x);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);

    //                           LDX         TXA   BRK
    const instructions2 = [_]u8{ 0xA2, 0x00, 0x8A, 0x00 };
    var cpu2 = CPU.init(rom.DummyTestRom(&instructions2));
    cpu2.run_instructions(&instructions2);

    try std.testing.expectEqual(0x00, cpu2.register_x);
    try std.testing.expect(cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
}

test "0x98: TYA Transfer Y to Accumulator" {
    //                          LDY         TYA   BRK
    const instructions = [_]u8{ 0xA0, 0x69, 0x98, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x69, cpu.register_a);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0x9A: TXS Transfer X to Stack Pointer" {
    //                          LDX         TXS   BRK
    const instructions = [_]u8{ 0xA2, 0x69, 0x9A, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x69, cpu.sp);
}

test "4 ops (LDA, TAX, INX, BRK) working together" {
    //                          LDA        LDA         TAX   INX   BRK
    const instructions = [_]u8{ 0xA9, 0xA, 0xA9, 0xC0, 0xAA, 0xE8, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0xC0, cpu.register_a);
    try std.testing.expectEqual(0xC1, cpu.register_x);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(cpu.status.negative_flag);
}

test "INX overflow" {
    //                          LDA         TAX   INX   INX   BRK
    const instructions = [_]u8{ 0xA9, 0xFF, 0xAA, 0xE8, 0xE8, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_x);
}

test "0xA5: LDA from memory" {
    //                          LDA         BRK
    const instructions = [_]u8{ 0xA5, 0x10, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.mem_write(0x10, 0x55);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x55, cpu.register_a);
}

test "0xC9: CMP equal values" {
    //                          LDA         CMP         BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0xC9, 0x01, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.mem_write(0x10, 0x55);
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.zero_flag);
}

test "0xC9: CMP different values" {
    //                          LDA         CMP         BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0xC9, 0x03, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.mem_write(0x10, 0x55);
    cpu.run_instructions(&instructions);

    try std.testing.expect(!cpu.status.zero_flag);
}

test "0x69: ADC add with carry - no overflow" {
    //                          LDA         TAX         ADC         BRK
    const instructions = [_]u8{ 0xa9, 0xc0, 0xaa, 0xe8, 0x69, 0xc4, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x84, cpu.register_a);
    try std.testing.expectEqual(0xc1, cpu.register_x);
    try std.testing.expect(!cpu.status.overflow_flag);
    try std.testing.expect(cpu.status.carry_flag);
}

test "0x65: ADC add with carry - overflow" {
    //                          LDA         STA         ADC         BRK
    const instructions = [_]u8{ 0xa9, 0x80, 0x85, 0x01, 0x65, 0x01, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu.register_a);
    try std.testing.expectEqual(0, cpu.register_x);
    try std.testing.expect(cpu.status.overflow_flag);
    try std.testing.expect(cpu.status.carry_flag);
}

test "0xE9: SBC Subtract with Carry - no overflow" {
    //                          LDA         SBC   BRK
    const instructions = [_]u8{ 0xa9, 0xF0, 0xE9, 0x50, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x9F, cpu.register_a);
    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.overflow_flag);
    try std.testing.expect(!cpu.status.zero_flag);
}

test "0xE9: SBC Subtract with Carry - overflow" {
    //                          LDA         SBC   BRK
    const instructions = [_]u8{ 0xa9, 0xD0, 0xE9, 0x70, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x5F, cpu.register_a);
    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(cpu.status.overflow_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);
}

test "0x29: logical AND - true result" {
    //                          LDA         AND        BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x29, 0x01, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x01, cpu.register_a);
}

test "0x29: logical AND - false result" {
    //                          LDA         AND         BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x29, 0x00, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu.register_a);
    try std.testing.expect(cpu.status.zero_flag);
}

test "0x0A: ASL Arithmetic Shift Left - carry flag set" {
    //                          LDA         ASL   BRK
    const instructions = [_]u8{ 0xA9, 0x80, 0x0A, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu.register_a);
    try std.testing.expect(cpu.status.carry_flag);
}

test "0x0A: ASL Arithmetic Shift Left - carry flag not set" {
    //                          LDA         ASL   BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x0A, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
}

test "0x06: ASL Arithmetic Shift Left - read value from memory" {
    //                          LDA         STA         ASL   BRK
    const instructions = [_]u8{ 0xA9, 0x02, 0x85, 0xFF, 0x06, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(4, cpu.mem_read(0xFF));
    try std.testing.expect(!cpu.status.carry_flag);
}

test "0x4A: LSR Logical Shift Right - Accumulator" {
    //                          LDA         LSR   BRK
    const instructions = [_]u8{ 0xA9, 0x02, 0x4A, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);

    //                           LDA         LSR   BRK
    const instructions2 = [_]u8{ 0xA9, 0x03, 0x4A, 0x00 };
    var cpu2 = CPU.init(rom.DummyTestRom(&instructions2));
    cpu2.run_instructions(&instructions2);

    try std.testing.expectEqual(1, cpu2.register_a);
    try std.testing.expect(cpu2.status.carry_flag);
}

test "0x46: LSR Logical Shift Right - read value from memory" {
    //                          LDA         STA         LSR   BRK
    const instructions = [_]u8{ 0xA9, 0x02, 0x85, 0xFF, 0x46, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.mem_read(0xFF));
    try std.testing.expect(!cpu.status.carry_flag);

    //                           LDA         STA         LSR   BRK
    const instructions2 = [_]u8{ 0xA9, 0x03, 0x85, 0xFF, 0x46, 0xFF, 0x00 };
    var cpu2 = CPU.init(rom.DummyTestRom(&instructions2));
    cpu2.run_instructions(&instructions2);

    try std.testing.expectEqual(1, cpu2.mem_read(0xFF));
    try std.testing.expect(cpu2.status.carry_flag);
}

test "0x09: ORA Logical Inclusive OR" {
    //                          LDA         ORA   BRK
    const instructions = [_]u8{ 0xA9, 0x02, 0x09, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0xFF, cpu.register_a);
}

test "0x2A: ROL Rotate Left" {
    //                          LDA         SEC   ROL   BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x38, 0x2A, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(3, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);

    //                           LDA         ROL   BRK
    const instructions2 = [_]u8{ 0xA9, 0x80, 0x2A, 0x00 };
    var cpu2 = CPU.init(rom.DummyTestRom(&instructions2));
    cpu2.run_instructions(&instructions2);

    try std.testing.expectEqual(0, cpu2.register_a);
    try std.testing.expect(cpu2.status.carry_flag);
    try std.testing.expect(cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);

    //                           LDA         ROL   BRK
    const instructions3 = [_]u8{ 0xA9, 0x40, 0x2A, 0x00 };
    var cpu3 = CPU.init(rom.DummyTestRom(&instructions3));
    cpu3.run_instructions(&instructions3);

    try std.testing.expectEqual(0x80, cpu3.register_a);
    try std.testing.expect(!cpu3.status.carry_flag);
    try std.testing.expect(!cpu3.status.zero_flag);
    try std.testing.expect(cpu3.status.negative_flag);
}

test "0x26: ROL Rotate Left - read value from memory" {
    //                          LDA         STA         SEC   ROL   BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0xFF, 0x38, 0x26, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(3, cpu.mem_read(0xFF));
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);

    //                           LDA         STA         ROL   BRK
    const instructions2 = [_]u8{ 0xA9, 0x80, 0x85, 0xFF, 0x26, 0xFF, 0x00 };
    var cpu2 = CPU.init(rom.DummyTestRom(&instructions2));
    cpu2.run_instructions(&instructions2);

    try std.testing.expectEqual(0, cpu2.mem_read(0xFF));
    try std.testing.expect(cpu2.status.carry_flag);
    try std.testing.expect(cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
}

test "0x6A: ROR Rotate Right" {
    //                          LDA         SEC   ROR   BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x38, 0x6A, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x80, cpu.register_a);
    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);

    //                           LDA         ROR   BRK
    const instructions2 = [_]u8{ 0xA9, 0x1, 0x6A, 0x00 };
    var cpu2 = CPU.init(rom.DummyTestRom(&instructions2));
    cpu2.run_instructions(&instructions2);

    try std.testing.expectEqual(0, cpu2.register_a);
    try std.testing.expect(cpu2.status.carry_flag);
    try std.testing.expect(cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);

    //                           LDA         ROR   BRK
    const instructions3 = [_]u8{ 0xA9, 0x80, 0x6A, 0x00 };
    var cpu3 = CPU.init(rom.DummyTestRom(&instructions3));
    cpu3.run_instructions(&instructions3);

    try std.testing.expectEqual(0x40, cpu3.register_a);
    try std.testing.expect(!cpu3.status.carry_flag);
    try std.testing.expect(!cpu3.status.zero_flag);
    try std.testing.expect(!cpu3.status.negative_flag);
}

test "0x66: ROR Rotate Right - Read Value from Memory" {
    //                          LDA         STA         SEC   ROR   BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0xFF, 0x38, 0x66, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x80, cpu.mem_read(0xFF));
    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);

    //                           LDA        STA         ROR   BRK
    const instructions2 = [_]u8{ 0xA9, 0x1, 0x85, 0xFF, 0x66, 0xFF, 0x00 };
    var cpu2 = CPU.init(rom.DummyTestRom(&instructions2));
    cpu2.run_instructions(&instructions2);

    try std.testing.expectEqual(0, cpu2.mem_read(0xFF));
    try std.testing.expect(cpu2.status.carry_flag);
    try std.testing.expect(cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);

    //                           LDA         STA         ROR   BRK
    const instructions3 = [_]u8{ 0xA9, 0x80, 0x85, 0xFF, 0x66, 0xFF, 0x00 };
    var cpu3 = CPU.init(rom.DummyTestRom(&instructions3));
    cpu3.run_instructions(&instructions3);

    try std.testing.expectEqual(0x40, cpu3.mem_read(0xFF));
    try std.testing.expect(!cpu3.status.carry_flag);
    try std.testing.expect(!cpu3.status.zero_flag);
    try std.testing.expect(!cpu3.status.negative_flag);
}

test "0xD0: BNE jump if not equal - skip INX instruction" {
    //                          LDA         CMP         BNE         INX   BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0xC9, 0x03, 0xD0, 0x01, 0xE8, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.mem_write(0x10, 0x55);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu.register_x);
}

test "0xD0: BNE jump if not equal - execute INX instruction" {
    //                          LDA         CMP         BNE         INX   BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0xC9, 0x01, 0xD0, 0x01, 0xE8, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.mem_write(0x10, 0x55);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_x);
}

test "0x90: BCC Branch if Carry Clear - branch" {
    //                          LDA         ASL   BCC         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x0A, 0x90, 0x02, 0xA9, 0x03, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    // if the register A contains the value 2 that means we did branch, otherwise we would have the value 3.
    try std.testing.expectEqual(2, cpu.register_a);
}

test "0x90: BCC Branch if Carry Clear - no branch" {
    //                          LDA         ASL   BCC         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x80, 0x0A, 0x90, 0x02, 0xA9, 0x03, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    // if the register A contains the value 3 that means we didn't branch, otherwise we would have the value 0.
    try std.testing.expectEqual(3, cpu.register_a);
}

test "0xB0: BCS Branch if Carry Set - branch" {
    //                          LDA         ASL   BCS         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x80, 0x0A, 0xB0, 0x02, 0xA9, 0x03, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    // if the register A contains the value 0 that means we did branch, otherwise we would have the value 3.
    try std.testing.expectEqual(0, cpu.register_a);
}

test "0xB0: BCS Branch if Carry Set - no branch" {
    //                          LDA         ASL   BCS         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x0A, 0xB0, 0x02, 0xA9, 0x03, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    // if the register A contains the value 3 that means we did branch, otherwise we would have the value 1.
    try std.testing.expectEqual(3, cpu.register_a);
}

test "0xF0: BEQ Branch if Equal - branch" {
    //                          LDA         CMP         BEQ         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0xC9, 0x01, 0xF0, 0x02, 0xA9, 0x03, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    // if the register A contains the value 1 that means we did branch, otherwise we would have the value 3.
    try std.testing.expectEqual(1, cpu.register_a);
}

test "0xF0: BEQ Branch if Equal - no branch" {
    //                          LDA         CMP         BEQ         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0xC9, 0x02, 0xF0, 0x02, 0xA9, 0x03, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    // if the register A contains the value 3 that means we did branch, otherwise we would have the value 1.
    try std.testing.expectEqual(3, cpu.register_a);
}

test "0x30: BMI Branch if Minus - branch" {
    //                          LDA         STA         LDA         BIT         BMI         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x81, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x30, 0x02, 0xA9, 0x04, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    // if the register A contains the value 1 that means we did branch, otherwise we would have the value 4.
    try std.testing.expectEqual(1, cpu.register_a);
}

test "0x30: BMI Branch if Minus - no branch" {
    //                          LDA         STA         LDA         BIT         BMI         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x30, 0x02, 0xA9, 0x04, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    // if the register A contains the value 4 that means we didn't branch, otherwise we would have the value 1.
    try std.testing.expectEqual(4, cpu.register_a);
}

test "0x10: BPL Branch if Positive - no branch" {
    //                          LDA         STA         LDA         BIT         BPL         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x81, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x10, 0x02, 0xA9, 0x04, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    // if the register A contains the value 4 that means we didn't branch, otherwise we would have the value 1.
    try std.testing.expectEqual(4, cpu.register_a);
}

test "0x10: BPL Branch if Positive - branch" {
    //                          LDA         STA         LDA         BIT         BPL         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x10, 0x02, 0xA9, 0x04, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    // if the register A contains the value 1 that means we did branch, otherwise we would have the value 4.
    try std.testing.expectEqual(1, cpu.register_a);
}

test "0x50: BVC Branch if Overflow Clear - no branch" {
    //                          LDA         STA         LDA         BIT         BVC         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x40, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x50, 0x02, 0xA9, 0x04, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    // if the register A contains the value 4 that means we didn't branch, otherwise we would have the value 1.
    try std.testing.expectEqual(4, cpu.register_a);
}

test "0x50: BVC Branch if Overflow Clear  - branch" {
    //                          LDA         STA         LDA         BIT         BVC         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x50, 0x02, 0xA9, 0x04, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    // if the register A contains the value 1 that means we did branch, otherwise we would have the value 4.
    try std.testing.expectEqual(1, cpu.register_a);
}

test "0x70: BVS Branch if Overflow Set - branch" {
    //                          LDA         STA         LDA         BIT         BVS         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x40, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x70, 0x02, 0xA9, 0x04, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    // if the register A contains the value 1 that means we did branch, otherwise we would have the value 4.
    try std.testing.expectEqual(1, cpu.register_a);
}

test "0x70: BVS Branch if Overflow Set - no branch" {
    //                          LDA         STA         LDA         BIT         BVS         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x70, 0x02, 0xA9, 0x04, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    // if the register A contains the value 4 that means we didn't branch, otherwise we would have the value 1.
    try std.testing.expectEqual(4, cpu.register_a);
}

test "0x24: BIT test - set N,V,Z flags" {
    //                          LDA         STA         LDA         BIT         BRK
    const instructions = [_]u8{ 0xA9, 0xC0, 0x85, 0xFF, 0xA9, 0x02, 0x24, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu.register_a);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(cpu.status.overflow_flag);
    try std.testing.expect(cpu.status.zero_flag);
}

test "0x24: BIT test - set N,V flags" {
    //                          LDA         STA         LDA         BIT         BRK
    const instructions = [_]u8{ 0xA9, 0xC2, 0x85, 0xFF, 0xA9, 0x02, 0x24, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu.register_a);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(cpu.status.overflow_flag);
    try std.testing.expect(!cpu.status.zero_flag);
}

test "0x24: BIT test - set N flags" {
    //                          LDA         STA         LDA         BIT         BRK
    const instructions = [_]u8{ 0xA9, 0x81, 0x85, 0xFF, 0xA9, 0x01, 0x24, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_a);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.overflow_flag);
    try std.testing.expect(!cpu.status.zero_flag);
}

test "0x18: CLC Clear Carry Flag" {
    //                          LDA         ASL   CLC   BRK
    const instructions = [_]u8{ 0xA9, 0x80, 0x0A, 0x18, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expect(!cpu.status.carry_flag);
}

test "0xF8: SED Set Decimal Flag" {
    //                          SED   BRK
    const instructions = [_]u8{ 0xF8, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.decimal_mode);
}

test "0xD8: CLD Clear Decimal Flag" {
    //                          SED   CLD   BRK
    const instructions = [_]u8{ 0xF8, 0xD8, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expect(!cpu.status.decimal_mode);
}

test "0x78: SEI Set Interrupt Disable" {
    //                          SEI   BRK
    const instructions = [_]u8{ 0x78, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.interrupt_disable);
}

test "0x38: SEC Set Carry Flag" {
    //                          SEC   BRK
    const instructions = [_]u8{ 0x38, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.carry_flag);
}

test "0x58: CLI Clear Interrupt Disable" {
    //                          SEI   CLI   BRK
    const instructions = [_]u8{ 0x78, 0x58, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expect(!cpu.status.interrupt_disable);
}

test "0xB8: CLV Clear Overflow Flag" {
    //                          LDA         STA         BIT         CLV   BRK
    const instructions = [_]u8{ 0xA9, 0xC2, 0x85, 0xFF, 0x24, 0xFF, 0xB8, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expect(!cpu.status.overflow_flag);
}

test "0xA2: LDX Load X Register" {
    //                          LDX         BRK
    const instructions = [_]u8{ 0xA2, 0x01, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_x);
}

test "0xA0: LDY Load Y Register" {
    //                          LDY         BRK
    const instructions = [_]u8{ 0xA0, 0x01, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_y);
}

test "0xE0: CPX Compare X Register" {
    //                          LDX         CPX         BRK
    const instructions = [_]u8{ 0xA2, 0x01, 0xE0, 0x01, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.zero_flag);
    try std.testing.expect(cpu.status.carry_flag);

    //                           LDX         CPX         BRK
    const instructions2 = [_]u8{ 0xA2, 0x01, 0xE0, 0x02, 0x00 };
    var cpu2 = CPU.init(rom.DummyTestRom(&instructions2));
    cpu2.run_instructions(&instructions2);

    try std.testing.expect(cpu2.status.negative_flag);
}

test "0xC0: CPY Compare Y Register" {
    //                          LDY         CPY         BRK
    const instructions = [_]u8{ 0xA0, 0x01, 0xC0, 0x01, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.zero_flag);
    try std.testing.expect(cpu.status.carry_flag);

    //                           LDY         CPY         BRK
    const instructions2 = [_]u8{ 0xA0, 0x01, 0xC0, 0x02, 0x00 };
    var cpu2 = CPU.init(rom.DummyTestRom(&instructions2));
    cpu2.run_instructions(&instructions2);

    try std.testing.expect(cpu2.status.negative_flag);
}

test "0xC6: DEC Decrement Memory" {
    //                          LDA         STA         DEC         BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0xFF, 0xC6, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expectEqual(0, cpu.mem_read(0xff));

    //                           LDA         STA         DEC         BRK
    const instructions2 = [_]u8{ 0xA9, 0x02, 0x85, 0xFF, 0xC6, 0xFF, 0x00 };
    var cpu2 = CPU.init(rom.DummyTestRom(&instructions2));
    cpu2.run_instructions(&instructions2);

    try std.testing.expect(!cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
    try std.testing.expectEqual(1, cpu2.mem_read(0xff));

    //                           LDA         STA         DEC         BRK
    const instructions3 = [_]u8{ 0xA9, 0x81, 0x85, 0xFF, 0xC6, 0xFF, 0x00 };
    var cpu3 = CPU.init(rom.DummyTestRom(&instructions3));
    cpu3.run_instructions(&instructions3);

    try std.testing.expect(!cpu3.status.zero_flag);
    try std.testing.expect(cpu3.status.negative_flag);
    try std.testing.expectEqual(0x80, cpu3.mem_read(0xff));
}

test "0xCA: DEX Decrement X Register" {
    //                          LDX         DEX   BRK
    const instructions = [_]u8{ 0xA2, 0x02, 0xCA, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expectEqual(1, cpu.register_x);

    //                           LDX         DEX   BRK
    const instructions2 = [_]u8{ 0xA2, 0x01, 0xCA, 0x00 };
    var cpu2 = CPU.init(rom.DummyTestRom(&instructions2));
    cpu2.run_instructions(&instructions2);

    try std.testing.expect(cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
    try std.testing.expectEqual(0, cpu2.register_x);

    //                           LDX         DEX   BRK
    const instructions3 = [_]u8{ 0xA2, 0x81, 0xCA, 0x00 };
    var cpu3 = CPU.init(rom.DummyTestRom(&instructions3));
    cpu3.run_instructions(&instructions3);

    try std.testing.expect(!cpu3.status.zero_flag);
    try std.testing.expect(cpu3.status.negative_flag);
    try std.testing.expectEqual(0x80, cpu3.register_x);

    //                           LDX         DEX   BRK
    const instructions4 = [_]u8{ 0xA2, 0x00, 0xCA, 0x00 };
    var cpu4 = CPU.init(rom.DummyTestRom(&instructions4));
    cpu4.run_instructions(&instructions4);

    try std.testing.expect(!cpu4.status.zero_flag);
    try std.testing.expect(cpu4.status.negative_flag);
    try std.testing.expectEqual(0xFF, cpu4.register_x);
}

test "0x88: DEY Decrement Y Register" {
    //                          LDY         DEY   BRK
    const instructions = [_]u8{ 0xA0, 0x02, 0x88, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expectEqual(1, cpu.register_y);

    //                           LDY         DEY   BRK
    const instructions2 = [_]u8{ 0xA0, 0x01, 0x88, 0x00 };
    var cpu2 = CPU.init(rom.DummyTestRom(&instructions2));
    cpu2.run_instructions(&instructions2);

    try std.testing.expect(cpu2.status.zero_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
    try std.testing.expectEqual(0, cpu2.register_y);

    //                           LDY         DEY   BRK
    const instructions3 = [_]u8{ 0xA0, 0x81, 0x88, 0x00 };
    var cpu3 = CPU.init(rom.DummyTestRom(&instructions3));
    cpu3.run_instructions(&instructions3);

    try std.testing.expect(!cpu3.status.zero_flag);
    try std.testing.expect(cpu3.status.negative_flag);
    try std.testing.expectEqual(0x80, cpu3.register_y);
}

test "0x49: EOR Exclusive OR" {
    //                          LDA         EOR         BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x49, 0x01, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);

    //                           LDA         EOR         BRK
    const instructions2 = [_]u8{ 0xA9, 0x01, 0x49, 0x80, 0x00 };
    var cpu2 = CPU.init(rom.DummyTestRom(&instructions2));
    cpu2.run_instructions(&instructions2);

    try std.testing.expect(!cpu2.status.zero_flag);
    try std.testing.expect(cpu2.status.negative_flag);
}

test "0xE8: INX increments X register by 1" {
    //                          LDA         TAX   INX   BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0xAA, 0xE8, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu.register_x);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0xE6: INC Increment Memory" {
    //                          LDA         STA         INC         LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x85, 0x10, 0xE6, 0x10, 0xA5, 0x10, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu.register_a);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0xC8: INY Increment Y Register" {
    //                          LDY         INY   BRK
    const instructions = [_]u8{ 0xA0, 0x01, 0xC8, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu.register_y);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0x6C: JMP Jump to address" {
    //                          LDA         JMP               LDA         BRK
    const instructions = [_]u8{ 0xA9, 0x42, 0x6C, 0x11, 0x11, 0xA9, 0x69, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.mem_write_u16(0x1111, 0x8007);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x8008, cpu.pc);
    // if the register A contains the value 0x42 that means we jumped, otherwise we would have the value 0x69.
    try std.testing.expectEqual(0x42, cpu.register_a);
}

test "0x48: PHA Push Accumulator to Stack" {
    //                          LDA         PHA   LDA         PHA   LDA         PHA   BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x48, 0xA9, 0x02, 0x48, 0xA9, 0x03, 0x48, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x03, cpu.stack_pop());
    try std.testing.expectEqual(0x02, cpu.stack_pop());
    try std.testing.expectEqual(0x01, cpu.stack_pop());
}

test "0x68: PLA Pop Accumulator from Stack" {
    //                          LDA         PHA   LDA         PLA   BRK
    const instructions = [_]u8{ 0xA9, 0x01, 0x48, 0xA9, 0x02, 0x68, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x01, cpu.register_a);
}

test "0x20: JSR Jump to Subroutine" {
    //                          JSR               LDX         NOP   NOP   LDX         BRK
    const instructions = [_]u8{ 0x20, 0x05, 0x80, 0xA2, 0x01, 0x1A, 0x1A, 0xA2, 0x03, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.tick();
    try std.testing.expectEqual(0x8005, cpu.pc);

    cpu.tick();
    cpu.tick();
    cpu.tick();
    try std.testing.expectEqual(3, cpu.register_x);
}

test "0x60: RTS Return from Subroutine" {
    //                          JSR               JSR               JSR               LDX         RTS   INX   RTS   BRK
    const instructions = [_]u8{ 0x20, 0x09, 0x80, 0x20, 0x0C, 0x80, 0x20, 0x0E, 0x80, 0xA2, 0x00, 0x60, 0xE8, 0x60, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_x);
    try std.testing.expectEqual(0x800F, cpu.pc);
}

test "0x40: RTI Return from Interrupt" {
    //                          RTI   BRK
    const instructions = [_]u8{ 0x40, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.status.carry_flag = true;
    cpu.status.negative_flag = true;

    cpu.stack_push_u16(cpu.pc + 1);
    cpu.stack_push(@bitCast(cpu.status));

    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expectEqual(0x8002, cpu.pc);
}

test "0x08: PHP Push Processor Status" {
    //                          SED   SEI   PHP   BRK
    const instructions = [_]u8{ 0xF8, 0x78, 0x08, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    const status: ProcessorStatus = @bitCast(cpu.stack_pop());
    try std.testing.expect(status.decimal_mode);
    try std.testing.expect(status.interrupt_disable);
}

test "0x28: PLP Pull Processor Status" {
    //                          SED   SEI   PHP   PLP   BRK
    const instructions = [_]u8{ 0xF8, 0x78, 0x08, 0x28, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expect(cpu.status.decimal_mode);
    try std.testing.expect(cpu.status.interrupt_disable);
}

test "0x85: STA Store Accumulator" {
    //                          LDA         STA   BRK
    const instructions = [_]u8{ 0xA9, 0x69, 0x85, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x69, cpu.mem_read(0x00FF));
}

test "0x86: STX Store X Register" {
    //                          LDX         STX   BRK
    const instructions = [_]u8{ 0xA2, 0x69, 0x86, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x69, cpu.mem_read(0x00FF));
}

test "0x84: STY Store Y Register" {
    //                          LDY         STY         BRK
    const instructions = [_]u8{ 0xA0, 0x69, 0x84, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x69, cpu.mem_read(0x00FF));
}

test "0x0B: ANC" {
    //                          LDA         ANC         BRK
    const instructions = [_]u8{ 0xA9, 0x80, 0x0B, 0x80, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x80, cpu.register_a);
    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);
}

test "0x2B: ANC" {
    //                          LDA         ANC         BRK
    const instructions = [_]u8{ 0xA9, 0x00, 0x2B, 0x80, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x00, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(cpu.status.zero_flag);
}

test "0xCB: AXS" {
    //                          AXS         BRK
    const instructions = [_]u8{ 0xCB, 0x05, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.register_a = 1;
    cpu.register_x = 3;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(252, cpu.register_x);
    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(cpu.status.negative_flag);
}

test "0x6B: ARR" {
    //                          ARR         BRK
    const instructions = [_]u8{ 0x6B, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.register_a = 0b1100_0000;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(96, cpu.register_a);
    try std.testing.expect(cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.overflow_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);

    var cpu2 = CPU.init(rom.DummyTestRom(&instructions));
    cpu2.pc = 0x8000;
    cpu2.register_a = 0b0000_0001;
    cpu2.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu2.register_a);
    try std.testing.expect(!cpu2.status.carry_flag);
    try std.testing.expect(!cpu2.status.overflow_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
    try std.testing.expect(cpu2.status.zero_flag);

    var cpu3 = CPU.init(rom.DummyTestRom(&instructions));
    cpu3.pc = 0x8000;
    cpu3.register_a = 0b0100_0000;
    cpu3.run_instructions(&instructions);

    try std.testing.expectEqual(32, cpu3.register_a);
    try std.testing.expect(!cpu3.status.carry_flag);
    try std.testing.expect(cpu3.status.overflow_flag);
    try std.testing.expect(!cpu3.status.negative_flag);
    try std.testing.expect(!cpu3.status.zero_flag);

    var cpu4 = CPU.init(rom.DummyTestRom(&instructions));
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
    //                          ALR         BRK
    const instructions = [_]u8{ 0x4B, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.register_a = 0b0000_0010;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);

    var cpu2 = CPU.init(rom.DummyTestRom(&instructions));
    cpu2.pc = 0x8000;
    cpu2.register_a = 0b0000_0001;
    cpu2.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu2.register_a);
    try std.testing.expect(cpu2.status.carry_flag);
    try std.testing.expect(!cpu2.status.negative_flag);
    try std.testing.expect(cpu2.status.zero_flag);
}

test "0xAB: ATX" {
    //                          ATX         BRK
    const instructions = [_]u8{ 0xAB, 0x2, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.register_a = 2;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(2, cpu.register_x);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);

    var cpu2 = CPU.init(rom.DummyTestRom(&instructions));
    cpu2.pc = 0x8000;
    cpu2.register_a = 1;
    cpu2.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu2.register_x);
    try std.testing.expect(!cpu2.status.negative_flag);
    try std.testing.expect(cpu2.status.zero_flag);
}

test "0x9F: AXA" {
    //                          AXA               BRK
    const instructions = [_]u8{ 0x9F, 0x00, 0x10, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.register_a = 1;
    cpu.register_x = 3;
    cpu.register_y = 0x92;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.mem_read(0x1092));

    var cpu2 = CPU.init(rom.DummyTestRom(&instructions));
    cpu2.pc = 0x8000;
    cpu2.register_a = 4;
    cpu2.register_x = 1;
    cpu2.register_y = 0x92;
    cpu2.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu2.mem_read(0x1092));
}

test "0x87: SAX" {
    //                          SAX         BRK
    const instructions = [_]u8{ 0x87, 0x05, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.register_x = 0x80;
    cpu.register_a = 0x80;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0x80, cpu.mem_read(0x05));
}

test "0xC7: DCP" {
    //                          DCP         BRK
    const instructions = [_]u8{ 0xC7, 0x69, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.mem_write(0x69, 1);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu.mem_read(0x69));
    try std.testing.expect(cpu.status.carry_flag);

    var cpu2 = CPU.init(rom.DummyTestRom(&instructions));
    cpu2.pc = 0x8000;
    cpu2.mem_write(0x69, 0);
    cpu2.run_instructions(&instructions);

    try std.testing.expectEqual(255, cpu2.mem_read(0x69));
    try std.testing.expect(!cpu2.status.carry_flag);
}

test "0xE2: DOP" {
    //                          DOP         BRK
    const instructions = [_]u8{ 0xE2, 0x69, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.run_instructions(&instructions);
}

test "0xE7: ISC" {
    //                          ISC         BRK
    const instructions = [_]u8{ 0xE7, 0x69, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.register_a = 1;
    cpu.mem_write(0x69, 2);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(3, cpu.mem_read(0x69));
    try std.testing.expect(cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.overflow_flag);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);

    var cpu2 = CPU.init(rom.DummyTestRom(&instructions));
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
    //                          LAS               BRK
    const instructions = [_]u8{ 0xBB, 0x00, 0x10, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
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
    //                          LAX         BRK
    const instructions = [_]u8{ 0xA7, 0xDF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.mem_write(0xDF, 42);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(42, cpu.register_a);
    try std.testing.expectEqual(42, cpu.register_x);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.zero_flag);
}

test "0x27: RLA" {
    //                          RLA         BRK
    const instructions = [_]u8{ 0x27, 0xDF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.register_a = 5;
    cpu.mem_write(0xDF, 42);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(4, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0x67: RRA" {
    //                          RRA         BRK
    const instructions = [_]u8{ 0x67, 0xDF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.mem_write(0xDF, 42);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(21, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
    try std.testing.expect(!cpu.status.overflow_flag);
}

test "0x07: SLO" {
    //                          SLO         BRK
    const instructions = [_]u8{ 0x07, 0xDF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.register_a = 5;
    cpu.mem_write(0xDF, 42);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(85, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0x47: SRE" {
    //                          SRE         BRK
    const instructions = [_]u8{ 0x47, 0xDF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.register_a = 5;
    cpu.mem_write(0xDF, 42);
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(16, cpu.register_a);
    try std.testing.expect(!cpu.status.carry_flag);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0x9E: SXA" {
    //                          SXA               BRK
    const instructions = [_]u8{ 0x9E, 0x00, 0x10, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.register_x = 5;
    cpu.register_y = 0x05;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.mem_read(0x1005));
}

test "0x9C: SYA" {
    //                          SYA               BRK
    const instructions = [_]u8{ 0x9C, 0x00, 0x10, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.register_x = 0x05;
    cpu.register_y = 5;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(1, cpu.mem_read(0x1005));
}

test "0x8B: XAA" {
    //                          XAA         BRK
    const instructions = [_]u8{ 0x8B, 0xFF, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.register_x = 5;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(5, cpu.register_a);
    try std.testing.expect(!cpu.status.zero_flag);
    try std.testing.expect(!cpu.status.negative_flag);
}

test "0x9B: XAS" {
    //                          XAS               BRK
    const instructions = [_]u8{ 0x9B, 0x00, 0x10, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
    cpu.register_y = 5;
    cpu.register_x = 2;
    cpu.register_a = 2;
    cpu.run_instructions(&instructions);

    try std.testing.expectEqual(0, cpu.mem_read(0x1005));
    try std.testing.expectEqual(2, cpu.sp);
}

test "0xBD: LDA AbsoluteX - page cross" {
    // This test demonstrates the correct address calculation when an indexed
    // addressing mode crosses a page boundary.
    // On a real 6502, this operation
    // would incur an extra clock cycle penalty.
    //                          LDA   $02F0,X     BRK
    const instructions = [_]u8{ 0xBD, 0xF0, 0x02, 0x00 };
    var cpu = CPU.init(rom.DummyTestRom(&instructions));
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
