const std = @import("std");
const Mapper = @import("mappers/mapper.zig").Mapper;

const NES_TAG = [_]u8{ 0x4e, 0x45, 0x53, 0x1a };
const PRG_ROM_PAGE_SIZE = 16384;
const CHR_ROM_PAGE_SIZE = 8192;

pub const Mirroring = enum {
    VERTICAL,
    HORIZONTAL,
    FOUR_SCREEN,
};

pub const Rom = struct {
    /// PRG ROM is mapped to address `0x8000..0x10000`.
    prg_rom: []u8,
    /// CHR ROM is mapped to address `0..0x1FFF`
    chr_rom: []u8,
    mapper_id: u8,
    mirroring: Mirroring,
    mapper: Mapper,

    const Self = @This();

    pub const InitError = error{InvalidNesFormat};

    pub fn init(allocator: std.mem.Allocator, raw: []u8) !Self {
        if (!std.mem.eql(u8, raw[0..4], &NES_TAG)) {
            return error.InvalidNesFormat;
        }

        // Byte 6 and 7 of the NES file format contains information about the data in the file.
        // The uppper 4 bits of byte 6 contains the 4 lower bits of the ROM Mapper type and
        // the upper 4 bits of byte 7 contains the 4 upper bits of the ROM Mapper type.
        const mapper_id = raw[7] & 0b1111_0000 | raw[6] >> 4;

        const is_four_screen = raw[6] & 0b1000 != 0;
        const is_vertical_mirroring = raw[6] & 0b1 != 0;
        var screen_mirroring: Mirroring = undefined;
        if (is_four_screen) {
            screen_mirroring = Mirroring.FOUR_SCREEN;
        } else if (is_vertical_mirroring) {
            screen_mirroring = Mirroring.VERTICAL;
        } else {
            screen_mirroring = Mirroring.HORIZONTAL;
        }

        // Byte 4 contains the number of 16KB PGR-ROM banks
        const prg_rom_size = @as(usize, raw[4]) * PRG_ROM_PAGE_SIZE;
        // Byte 5 contains the number of 8KB CHR-ROM banks
        const chr_rom_size = @as(usize, raw[5]) * CHR_ROM_PAGE_SIZE;

        // If bit 2 of byte 6 is set, there's a 512-byte trainer section in the file to skip
        const skip_trainer = raw[6] & 0b100 != 0;

        const prg_rom_start: usize = 16 + @as(usize, if (skip_trainer) 512 else 0);
        const chr_rom_start = prg_rom_start + prg_rom_size;

        std.log.info(
            "Number of 16KB PRG-ROM banks: {}\nNumber of 8KB CHR-ROM banks: {}\niNes version: {s}\nMirroring type: {s}",
            .{ raw[4], raw[5], if ((raw[7] >> 2) & 0b11 != 0) "2.0" else "1.0", @tagName(screen_mirroring) },
        );

        const prg_rom = raw[prg_rom_start..(prg_rom_start + prg_rom_size)];
        const chr_rom = raw[chr_rom_start..(chr_rom_start + chr_rom_size)];
        const mapper = try Mapper.init(allocator, mapper_id, prg_rom, chr_rom, screen_mirroring);

        return .{
            .mapper_id = mapper_id,
            .mirroring = screen_mirroring,
            .prg_rom = prg_rom,
            .chr_rom = chr_rom,
            .mapper = mapper,
        };
    }

    pub fn deinit(self: *Self) void {
        self.mapper.deinit();
    }

    /// Read from PRG ROM space ($8000-$FFFF)
    pub fn prg_read(self: *Self, addr: u16) u8 {
        return self.mapper.prg_read(addr);
    }

    /// Write to PRG ROM space (for mappers that support banking)
    pub fn prg_write(self: *Self, addr: u16, value: u8) void {
        self.mapper.prg_write(addr, value);
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
    pub fn get_mirroring(self: *Self) Mirroring {
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
    pub fn mapper_ppu_clock(self: *Self) void {
        self.mapper.ppu_clock();
    }

    pub fn mapper_cpu_clock(self: *Self) void {
        self.mapper.cpu_clock();
    }
};

pub fn DummyTestRom(opcodes: []const u8) Rom {
    return .{
        .mapper_id = 0,
        .mirroring = .HORIZONTAL,
        .prg_rom = @constCast(opcodes),
        .chr_rom = @constCast(&[_]u8{0}),
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
    try std.testing.expectEqual(3, rom.mapper_id);
    try std.testing.expectEqual(Mirroring.VERTICAL, rom.mirroring);
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
    try std.testing.expectEqual(3, rom.mapper_id);
    try std.testing.expectEqual(Mirroring.VERTICAL, rom.mirroring);
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
