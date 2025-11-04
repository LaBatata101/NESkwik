const std = @import("std");
const CPU = @import("cpu.zig").CPU;
const PPU = @import("ppu.zig").PPU;
const APU = @import("apu/apu.zig").APU;
const Rom = @import("rom.zig").Rom;
const Controllers = @import("controller.zig").Controllers;

pub const Bus = struct {
    const RAM_SIZE = 2048;
    ram: [RAM_SIZE]u8,
    /// This is incremented by the CPU,
    cycles: u64,

    apu: *APU,
    ppu: *PPU,
    rom: *Rom,
    controllers: Controllers,

    const Self = @This();

    pub fn init(rom: *Rom, ppu: *PPU, apu: *APU) Self {
        return .{
            .cycles = 0,
            .ram = [_]u8{0} ** RAM_SIZE,
            .rom = rom,
            .ppu = ppu,
            .apu = apu,
            .controllers = Controllers.init(),
        };
    }

    pub fn mem_read(self: *Self, addr: u16) u8 {
        return switch (addr) {
            0...0x1FFF => self.ram[addr % RAM_SIZE],
            0x2000...0x3FFF => self.ppu.cpu_read(addr),
            0x4000...0x4015 => {
                if (addr == 0x4014) {
                    return self.ppu.dynamic_latch;
                }
                return self.apu.read_status(self.cycles);
            },
            0x4016 => self.controllers.cntrl1_read(),
            0x4017 => self.controllers.cntrl2_read(),
            0x6000...0x7FFF => self.rom.prg_ram_read(addr),
            0x4020...0x5FFF, 0x8000...0xFFFF => self.rom.prg_rom_read(addr),
            else => {
                std.log.warn("BUS: Ignoring mem read at 0x{X:04}", .{addr});
                return 0;
            },
        };
    }

    pub fn mem_peek(self: *const Self, addr: u16) u8 {
        return switch (addr) {
            0...0x1FFF => self.ram[addr % RAM_SIZE],
            0x2000...0x3FFF => self.ppu.cpu_peek(addr),
            0x4000...0x4015 => {
                if (addr == 0x4014) {
                    return self.ppu.dynamic_latch;
                }
                return self.apu.peek_status();
            },
            0x4016 => self.controllers.cntrl1_peek(),
            0x4017 => self.controllers.cntrl2_peek(),
            0x6000...0x7FFF => self.rom.prg_ram_read(addr),
            0x4020...0x5FFF, 0x8000...0xFFFF => self.rom.prg_rom_read(addr),
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
            0x4020...0x5FFF, 0x8000...0xFFFF => self.rom.prg_rom_write(addr, data),
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
};
