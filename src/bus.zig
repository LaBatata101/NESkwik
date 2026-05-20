const std = @import("std");
const CPU = @import("cpu.zig").CPU;
const PPU = @import("ppu.zig").PPU;
const APU = @import("apu/apu.zig").APU;
const Rom = @import("rom.zig").Rom;
const TestRom = @import("rom.zig").TestRom;
const Controllers = @import("controller.zig").Controllers;

pub const Bus = struct {
    const RAM_SIZE = 2048;
    ram: [RAM_SIZE]u8,
    /// This is incremented by the CPU,
    cycles: u64,
    /// Last value on the CPU data bus. Returned for undriven reads (write-only
    /// registers, unallocated I/O space).
    open_bus: u8,
    dma_start_delay: u8,
    dma_cycles: u16,

    apu: *APU,
    ppu: *PPU,
    rom: *Rom,
    controllers: Controllers,

    const Self = @This();

    pub const Snapshot = struct {
        ram: [RAM_SIZE]u8,
        cycles: u64,
        open_bus: u8,
        dma_start_delay: u8,
        dma_cycles: u16,
        controllers: Controllers,
        rom: Rom.Snapshot,

        pub fn deinit(self: *@This(), alloc: std.mem.Allocator) void {
            self.rom.deinit(alloc);
        }
    };

    pub fn init(rom: *Rom, ppu: *PPU, apu: *APU) Self {
        return .{
            .cycles = 0,
            .ram = [_]u8{0} ** RAM_SIZE,
            .open_bus = 0,
            .dma_start_delay = 0,
            .dma_cycles = 0,
            .rom = rom,
            .ppu = ppu,
            .apu = apu,
            .controllers = Controllers.init(),
        };
    }

    pub fn reset(self: *Self) void {
        self.cycles = 0;
        self.open_bus = 0;
        self.dma_start_delay = 0;
        self.dma_cycles = 0;
        self.ppu.reset();
        self.apu.reset();
        self.controllers.reset();
    }

    pub fn saveState(self: *const Self, alloc: std.mem.Allocator) !Snapshot {
        return .{
            .ram = self.ram,
            .cycles = self.cycles,
            .open_bus = self.open_bus,
            .dma_start_delay = self.dma_start_delay,
            .dma_cycles = self.dma_cycles,
            .controllers = self.controllers,
            .rom = try self.rom.saveState(alloc),
        };
    }

    pub fn loadState(self: *Self, snapshot: Snapshot) !void {
        self.ram = snapshot.ram;
        self.cycles = snapshot.cycles;
        self.open_bus = snapshot.open_bus;
        self.dma_start_delay = snapshot.dma_start_delay;
        self.dma_cycles = snapshot.dma_cycles;
        self.controllers = snapshot.controllers;
        try self.rom.loadState(snapshot.rom);
    }

    pub fn mem_read(self: *Self, addr: u16) u8 {
        const value: u8 = switch (addr) {
            0x0000...0x1FFF => self.ram[addr % RAM_SIZE],
            0x2000...0x3FFF => self.ppu.cpu_read(addr),
            // $4000-$4014 are write-only APU/OAM registers; reads return open bus.
            // $4018-$40FF is unallocated I/O space; also returns open bus.
            0x4000...0x4014, 0x4018...0x40FF => return self.open_bus,
            0x4015 => self.apu.read_status(),
            // Controllers only drive bits 0-4; bits 5-7 float as open bus.
            0x4016 => (self.open_bus & 0xE0) | self.controllers.cntrl1_read(),
            0x4017 => (self.open_bus & 0xE0) | self.controllers.cntrl2_read(),
            // $4100-$5FFF is cartridge expansion space. None of the implemented
            // mappers expose it yet, so treat it as unmapped.
            0x4100...0x5FFF => 0,
            0x6000...0x7FFF => self.rom.prg_ram_read(addr),
            0x8000...0xFFFF => self.rom.prg_rom_read(addr),
        };
        self.open_bus = value;
        return value;
    }

    pub fn mem_peek(self: *const Self, addr: u16) u8 {
        return switch (addr) {
            0x0000...0x1FFF => self.ram[addr % RAM_SIZE],
            0x2000...0x3FFF => self.ppu.cpu_peek(addr),
            0x4000...0x4014, 0x4018...0x40FF => self.open_bus,
            0x4015 => self.apu.peek_status(),
            0x4016 => (self.open_bus & 0xE0) | self.controllers.cntrl1_peek(),
            0x4017 => (self.open_bus & 0xE0) | self.controllers.cntrl2_peek(),
            0x6000...0x7FFF => self.rom.prg_ram_read(addr),
            0x8000...0xFFFF => self.rom.prg_rom_read(addr),
            else => return 0,
        };
    }

    pub fn mem_write(self: *Self, addr: u16, data: u8) void {
        switch (addr) {
            0...0x1FFF => self.ram[addr % RAM_SIZE] = data,
            0x2000...0x3FFF => self.ppu.cpu_write(addr, data),
            0x4014 => self.dma_transfer(data),
            0x4000...0x4013, 0x4015, 0x4017 => self.apu.write(addr, data),
            0x4016 => self.controllers.set_strobe(data),
            0x6000...0x7FFF => self.rom.prg_ram_write(addr, data),
            0x8000...0xFFFF => self.rom.prg_rom_write(addr, data),
            else => std.log.warn("BUS: Ignoring mem write at 0x{X:04}", .{addr}),
        }
    }

    pub fn mem_read_u16(self: *Self, addr: u16) u16 {
        const lo = self.mem_read(addr);
        const hi = @as(u16, self.mem_read(addr + 1));
        return (hi << 8) | lo;
    }

    pub fn mem_peek_u16(self: *Self, addr: u16) u16 {
        const lo = self.mem_peek(addr);
        const hi = @as(u16, self.mem_peek(addr + 1));
        return (hi << 8) | lo;
    }

    // https://www.nesdev.org/wiki/PPU_programmer_reference#OAMDMA_-_Sprite_DMA_($4014_write)
    fn dma_transfer(self: *Self, page: u8) void {
        const write_cycle = self.cycles + 3;
        self.dma_start_delay = 3;
        self.dma_cycles = 513 + @as(u16, @intFromBool(write_cycle % 2 == 1));

        const page_u16 = @as(u16, page) << 8;
        for (0..256) |x| {
            const addr = page_u16 | @as(u16, @intCast(x));
            const byte = self.mem_read(addr);
            self.ppu.oam_data_write(byte);
        }
    }
};

test "expansion space reads do not route into PRG ROM" {
    const alloc = std.testing.allocator;
    const program = [_]u8{0xEA};
    var test_rom = TestRom.init(alloc, &program);
    defer test_rom.deinit();

    // Fill PRG ROM with a non-zero pattern so an accidental PRG-ROM read is obvious.
    @memset(test_rom.prg_rom, 0xAA);

    var bus = Bus.init(&test_rom.rom, undefined, undefined);

    try std.testing.expectEqual(@as(u8, 0), bus.mem_read(0x4020));
    try std.testing.expectEqual(@as(u8, 0), bus.mem_read(0x5FFF));
}
