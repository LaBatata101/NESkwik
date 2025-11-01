const std = @import("std");
const Mapper = @import("mappers/mapper.zig").Mapper;

const NES_TAG = [_]u8{ 0x4e, 0x45, 0x53, 0x1a };
const PRG_ROM_PAGE_SIZE = 16384;
const CHR_ROM_PAGE_SIZE = 8192;

pub const Mirroring = enum {
    VERTICAL,
    HORIZONTAL,
    FOUR_SCREEN,
    SINGLE_SCREEN_LOWER,
    SINGLE_SCREEN_UPPER,
};

const Flag6 = packed struct(u8) {
    /// 0: vertical arrangement ("horizontal mirrored") (CIRAM A10 = PPU A11)
    /// 1: horizontal arrangement ("vertically mirrored") (CIRAM A10 = PPU A10)
    nametable_arrangement: u1,
    /// Cartridge contains battery-backed PRG RAM ($6000-7FFF) or other persistent memory
    has_battery: bool,
    /// 512-byte trainer at $7000-$71FF (stored before PRG data)
    has_trainer: bool,
    alternative_nametable_layout: bool,
    /// Lower nybble of mapper number
    mapper_lo: u4,

    fn mirroring_type(self: @This()) Mirroring {
        if (self.alternative_nametable_layout) {
            return Mirroring.FOUR_SCREEN;
        } else {
            switch (self.nametable_arrangement) {
                0 => return Mirroring.HORIZONTAL,
                1 => return Mirroring.VERTICAL,
            }
        }
    }
};

const Flag7 = packed struct(u8) {
    /// 0: Nintendo Entertainment System/Family Computer
    /// 1: Nintendo Vs. System
    /// 2: Nintendo Playchoice 10
    /// 3: Extended Console Type
    console_type: u2,
    /// NES 2.0 identifier
    nes_version: u2,
    /// Upper nybble of mapper number
    mapper_hi: u4,

    fn is_nes2(self: @This()) bool {
        return self.nes_version != 0;
    }
};

pub const Rom = struct {
    /// PRG ROM is mapped to address `0x8000..0x10000`.
    prg_rom: []u8,
    /// CHR ROM is mapped to address `0..0x1FFF`
    chr_rom: []u8,
    mapper: Mapper,

    const Self = @This();

    pub const InitError = error{InvalidNesFormat};

    pub fn init(allocator: std.mem.Allocator, raw: []u8) !Self {
        if (!std.mem.eql(u8, raw[0..4], &NES_TAG)) {
            return error.InvalidNesFormat;
        }

        const flag6: Flag6 = @bitCast(raw[6]);
        const flag7: Flag7 = @bitCast(raw[7]);

        const mapper_id = @as(u8, flag7.mapper_hi) | @as(u8, flag6.mapper_lo);

        // Byte 4 contains the number of 16KB PGR-ROM banks
        const prg_rom_banks = raw[4];
        // Byte 5 contains the number of 8KB CHR-ROM banks
        const chr_rom_banks = raw[5];

        const prg_rom_size = @as(usize, prg_rom_banks) * PRG_ROM_PAGE_SIZE;
        const chr_rom_size = @as(usize, chr_rom_banks) * CHR_ROM_PAGE_SIZE;

        const prg_rom_start: usize = 16 + @as(usize, if (flag6.has_trainer) 512 else 0);
        const chr_rom_start = prg_rom_start + prg_rom_size;

        const prg_ram_size: usize = blk: {
            const ram_size = raw[8];
            if (ram_size == 0) { // value 0 defaults to 8KB
                break :blk 8192;
            } else {
                break :blk @as(usize, ram_size) * 8192;
            }
        };

        const prg_rom = raw[prg_rom_start..(prg_rom_start + prg_rom_size)];
        const chr_rom = raw[chr_rom_start..(chr_rom_start + chr_rom_size)];

        std.log.info(
            \\{s}
            \\Mapper ID: {}
            \\Number of 16KB PRG-ROM banks: {}
            \\Number of 8KB CHR-ROM banks: {}
            \\PRG RAM size: {}
            \\Mirroring type: {s}
        , .{
            if (flag7.is_nes2()) "iNES 2.0" else "iNES 1.0",
            mapper_id,
            prg_rom_banks,
            chr_rom_banks,
            prg_ram_size,
            @tagName(flag6.mirroring_type()),
        });

        const mapper = try Mapper.init(allocator, mapper_id, .{
            .prg_rom = prg_rom,
            .chr_rom = chr_rom,
            .prg_rom_banks = prg_rom_banks,
            .prg_ram_size = prg_ram_size,
            .has_battery_backed_ram = flag6.has_battery,
            .mirroring_mode = flag6.mirroring_type(),
        });

        return .{
            .prg_rom = prg_rom,
            .chr_rom = chr_rom,
            .mapper = mapper,
        };
    }

    pub fn deinit(self: *Self) void {
        self.mapper.deinit();
    }

    /// Read from PRG ROM ($8000-$FFFF)
    pub fn prg_rom_read(self: *Self, addr: u16) u8 {
        return self.mapper.prg_rom_read(addr);
    }

    ///  Read from PRG RAM ($6000–$7FFF)
    pub fn prg_ram_read(self: *Self, addr: u16) u8 {
        return self.mapper.prg_ram_read(addr);
    }

    /// Write to PRG ROM ($8000-$FFFF) space
    pub fn prg_rom_write(self: *Self, addr: u16, value: u8) void {
        self.mapper.prg_rom_write(addr, value);
    }

    /// Write to PRG RAM ($6000–$7FFF) space
    pub fn prg_ram_write(self: *Self, addr: u16, value: u8) void {
        self.mapper.prg_ram_write(addr, value);
    }

    /// Read from CHR ROM/RAM space ($0000-$1FFF)
    pub fn chr_read(self: *Self, addr: u16) u8 {
        return self.mapper.chr_read(addr);
    }

    /// Write to CHR ROM/RAM space
    pub fn chr_write(self: *Self, addr: u16, value: u8) void {
        self.mapper.chr_write(addr, value);
    }

    /// Get current mirroring mode (can change with some mappers)
    pub fn get_mirroring(self: *const Self) Mirroring {
        return self.mapper.mirroring();
    }

    /// Check if mapper has an active IRQ
    pub fn mapper_irq_active(self: *Self) bool {
        return self.mapper.irq_active();
    }

    /// Clear mapper IRQ
    pub fn mapper_irq_clear(self: *Self) void {
        self.mapper.irq_clear();
    }

    /// Clock the mapper (for scanline counters, etc.)
    pub fn mapper_ppu_clock(self: *Self, addr: u16) void {
        self.mapper.ppu_clock(addr);
    }

    pub fn mapper_cpu_clock(self: *Self) void {
        self.mapper.cpu_clock();
    }
};

// Only for testing
pub fn DummyTestRom(allocator: std.mem.Allocator, opcodes: []const u8) Rom {
    const prg_rom = opcodes;
    const chr_rom = &[_]u8{0};
    return .{
        .prg_rom = @constCast(prg_rom),
        .chr_rom = @constCast(chr_rom),
        .mapper = Mapper.init(allocator, 0, .{
            .rom_path = "test.rom",
            .prg_rom = prg_rom,
            .chr_rom = chr_rom,
            .prg_rom_banks = 0,
            .prg_ram_size = 0,
            .has_battery_backed_ram = false,
            .mirroring_mode = Mirroring.HORIZONTAL,
        }) catch @panic("Failed to init mapper\n"),
    };
}

pub const TestRom = struct {
    header: []const u8,
    trainer: ?[]const u8,
    prg_rom: []const u8,
    chr_rom: []const u8,

    fn createRawRom(self: @This(), allocator: std.mem.Allocator) ![]u8 {
        var array = try std.ArrayList(u8).initCapacity(
            allocator,
            self.header.len + self.prg_rom.len + self.chr_rom.len + if (self.trainer) |trainer| trainer.len else 0,
        );
        try array.appendSlice(allocator, self.header);
        if (self.trainer) |trainer| {
            try array.appendSlice(allocator, trainer);
        }

        try array.appendSlice(allocator, self.prg_rom);
        try array.appendSlice(allocator, self.chr_rom);
        return try array.toOwnedSlice(allocator);
    }

    pub fn testRom(allocator: std.mem.Allocator, program: []const u8) ![]u8 {
        var prg_rom_contents = try std.ArrayList(u8).initCapacity(allocator, program.len);
        defer prg_rom_contents.deinit(allocator);

        try prg_rom_contents.appendSlice(allocator, program);
        try prg_rom_contents.resize(allocator, 2 * PRG_ROM_PAGE_SIZE);

        const test_rom = TestRom{
            .header = &[_]u8{ 0x4E, 0x45, 0x53, 0x1A, 0x02, 0x01, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
            .trainer = null,
            .prg_rom = prg_rom_contents.items,
            .chr_rom = &[_]u8{2} ** CHR_ROM_PAGE_SIZE,
        };
        return try test_rom.createRawRom(allocator);
    }
};

// TODO: fix tests
test "ROM creation" {
    const allocator = std.testing.allocator;
    var header = [_]u8{
        0x4E, 0x45, 0x53, 0x1A, 0x02, 0x01, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    var prg_rom = [_]u8{1} ** (2 * PRG_ROM_PAGE_SIZE);
    var chr_rom = [_]u8{2} ** CHR_ROM_PAGE_SIZE;
    const test_rom = TestRom{
        .header = header[0..],
        .trainer = null,
        .prg_rom = &prg_rom,
        .chr_rom = &chr_rom,
    };

    const raw_rom = try test_rom.createRawRom(allocator);
    defer allocator.free(raw_rom);
    const rom = try Rom.init(raw_rom);

    try std.testing.expect(std.mem.eql(u8, &prg_rom, rom.prg_rom));
    try std.testing.expect(std.mem.eql(u8, &chr_rom, rom.chr_rom));
    // try std.testing.expectEqual(3, rom.mapper_id);
    // try std.testing.expectEqual(Mirroring.VERTICAL, rom.mirroring);
}

test "ROM with trainer section" {
    const allocator = std.testing.allocator;
    var header = [_]u8{
        0x4E, 0x45, 0x53, 0x1A, 0x02, 0x01, 0x31 | 0b100, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    var prg_rom = [_]u8{1} ** (2 * PRG_ROM_PAGE_SIZE);
    var chr_rom = [_]u8{2} ** CHR_ROM_PAGE_SIZE;
    var trainer = [_]u8{0} ** 512;
    const test_rom = TestRom{
        .header = header[0..],
        .trainer = &trainer,
        .prg_rom = &prg_rom,
        .chr_rom = &chr_rom,
    };

    const raw_rom = try test_rom.createRawRom(allocator);
    defer allocator.free(raw_rom);
    const rom = try Rom.init(raw_rom);

    try std.testing.expect(std.mem.eql(u8, &prg_rom, rom.prg_rom));
    try std.testing.expect(std.mem.eql(u8, &chr_rom, rom.chr_rom));
    // try std.testing.expectEqual(3, rom.mapper_id);
    // try std.testing.expectEqual(Mirroring.VERTICAL, rom.mirroring);
}

test "ROM NES2.0 format not supported" {
    const allocator = std.testing.allocator;
    var header = [_]u8{
        0x4E, 0x45, 0x53, 0x1A, 0x02, 0x01, 0x31, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    var prg_rom = [_]u8{1} ** PRG_ROM_PAGE_SIZE;
    var chr_rom = [_]u8{2} ** CHR_ROM_PAGE_SIZE;
    const test_rom = TestRom{
        .header = header[0..],
        .trainer = null,
        .prg_rom = &prg_rom,
        .chr_rom = &chr_rom,
    };

    const raw_rom = try test_rom.createRawRom(allocator);
    defer allocator.free(raw_rom);

    // try std.testing.expectError(error.InvalidNesFormatVersion, Rom.load(raw_rom));
}
