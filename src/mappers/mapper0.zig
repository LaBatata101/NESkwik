const std = @import("std");
const Mirroring = @import("../rom.zig").Mirroring;
const Mapper = @import("mapper.zig").Mapper;

/// See: https://www.nesdev.org/wiki/INES_Mapper_000
/// Mapper 0 (NROM)
/// Features:
/// - 16 KB or 32 KB PRG ROM
/// - 8 KB CHR ROM or CHR RAM
/// - Fixed mirroring (horizontal or vertical)
pub const Mapper0 = struct {
    prg_rom: []const u8,
    prg_ram: []u8,
    chr_rom: []const u8,
    chr_ram: []u8,
    chr_is_ram: bool,
    mirroring_mode: Mirroring,
    allocator: std.mem.Allocator,

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator, prg_rom: []const u8, chr_rom: []const u8, prg_ram_size: usize, mirroring_mode: Mirroring) !*Self {
        const self = try allocator.create(Self);
        const chr_is_ram = chr_rom.len == 0;

        var chr_ram: []u8 = undefined;
        if (chr_is_ram) {
            chr_ram = try allocator.alloc(u8, 8192);
            @memset(chr_ram, 0);
        } else {
            chr_ram = &.{};
        }
        const prg_ram: []u8 = try allocator.alloc(u8, prg_ram_size);

        self.* = .{
            .prg_rom = prg_rom,
            .prg_ram = prg_ram,
            .chr_ram = chr_ram,
            .chr_rom = chr_rom,
            .chr_is_ram = chr_is_ram,
            .mirroring_mode = mirroring_mode,
            .allocator = allocator,
        };

        return self;
    }

    const VTableImpl = Mapper.VTable{
        .deinit = @ptrCast(&Self.deinit),
        .chr_read = @ptrCast(&Self.chr_read),
        .chr_write = @ptrCast(&Self.chr_write),
        .prg_rom_read = @ptrCast(&Self.prg_rom_read),
        .prg_rom_write = @ptrCast(&Self.prg_rom_write),
        .prg_ram_read = @ptrCast(&Self.prg_ram_read),
        .prg_ram_write = @ptrCast(&Self.prg_ram_write),
        .cpu_clock = @ptrCast(&Self.cpu_clock),
        .irq_active = @ptrCast(&Self.irq_active),
        .irq_clear = @ptrCast(&Self.irq_clear),
        .mirroring = @ptrCast(&Self.mirroring),
    };

    pub fn as_mapper(self: *Self) Mapper {
        return .{ .ptr = self, .vtable = &VTableImpl };
    }

    pub fn deinit(self: *Self) void {
        self.allocator.free(self.chr_ram);
        self.allocator.free(self.prg_ram);
        self.allocator.destroy(self);
    }

    pub fn prg_rom_read(self: *Self, addr: u16) u8 {
        return self.prg_rom[addr % self.prg_rom.len];
    }

    pub fn prg_rom_write(self: *Self, addr: u16, value: u8) void {
        _ = self;
        std.log.debug("Mapper 0: Attempted write to PRG ROM at ${X:04} = ${X:02}", .{ addr, value });
    }

    pub fn prg_ram_read(self: *Self, addr: u16) u8 {
        return self.prg_ram[addr % self.prg_ram.len];
    }

    pub fn prg_ram_write(self: *Self, addr: u16, value: u8) void {
        self.prg_ram[addr % self.prg_ram.len] = value;
    }

    pub fn chr_read(self: *Self, addr: u16) u8 {
        return if (self.chr_is_ram) self.chr_ram[addr % self.chr_ram.len] else self.chr_rom[addr % self.chr_rom.len];
    }

    pub fn chr_write(self: *Self, addr: u16, value: u8) void {
        // Only allow writes if CHR RAM is present
        if (!self.chr_is_ram) {
            return;
        }

        self.chr_ram[addr % self.chr_ram.len] = value;
    }

    pub fn mirroring(self: *const Self) Mirroring {
        return self.mirroring_mode;
    }

    pub fn cpu_clock(self: *Self) void {
        _ = self;
    }

    pub fn irq_active(self: *Self) bool {
        _ = self;
        return false;
    }

    pub fn irq_clear(self: *Self) void {
        _ = self;
    }
};

test "Mapper0 16KB PRG ROM mirroring" {
    const allocator = std.testing.allocator;

    const prg_rom = try allocator.alloc(u8, 0x4000); // 16KB
    defer allocator.free(prg_rom);

    // Fill with test pattern
    for (prg_rom, 0..) |*byte, i| {
        byte.* = @truncate(i);
    }

    var mapper = try Mapper0.init(allocator, prg_rom, &[_]u8{}, .HORIZONTAL);
    defer mapper.deinit();

    // Test that the upper 16KB mirrors the lower 16KB
    try std.testing.expectEqual(prg_rom[0], mapper.prg_rom_read(0x8000));
    try std.testing.expectEqual(prg_rom[0], mapper.prg_rom_read(0xC000));
    try std.testing.expectEqual(prg_rom[0x100], mapper.prg_rom_read(0x8100));
    try std.testing.expectEqual(prg_rom[0x100], mapper.prg_rom_read(0xC100));
}

test "Mapper0 32KB PRG ROM no mirroring" {
    const allocator = std.testing.allocator;

    const prg_rom = try allocator.alloc(u8, 0x8000); // 32KB
    defer allocator.free(prg_rom);

    // Fill with test pattern
    for (prg_rom, 0..) |*byte, i| {
        byte.* = @truncate(i);
    }

    var mapper = try Mapper0.init(allocator, prg_rom, &[_]u8{}, .HORIZONTAL);
    defer mapper.deinit();

    // Test that different addresses return different values
    try std.testing.expectEqual(prg_rom[0], mapper.prg_rom_read(0x8000));
    try std.testing.expectEqual(prg_rom[0x4000], mapper.prg_rom_read(0xC000));
    try std.testing.expect(mapper.prg_rom_read(0x8000) != mapper.prg_rom_read(0xC000));
}

test "Mapper0 CHR RAM" {
    const allocator = std.testing.allocator;

    const prg_rom = try allocator.alloc(u8, 0x4000);
    defer allocator.free(prg_rom);

    var mapper = try Mapper0.init(allocator, prg_rom, &[_]u8{}, .HORIZONTAL);
    defer mapper.deinit();

    // Test CHR RAM writes and reads
    mapper.chr_write(0x0000, 0x42);
    mapper.chr_write(0x1000, 0x69);

    try std.testing.expectEqual(@as(u8, 0x42), mapper.chr_read(0x0000));
    try std.testing.expectEqual(@as(u8, 0x69), mapper.chr_read(0x1000));
}

test "Mapper0 CHR ROM read-only" {
    const allocator = std.testing.allocator;

    const prg_rom = try allocator.alloc(u8, 0x4000);
    defer allocator.free(prg_rom);

    const chr_rom = try allocator.alloc(u8, 0x2000);
    defer allocator.free(chr_rom);
    @memset(chr_rom, 0xFF);

    var mapper = try Mapper0.init(allocator, prg_rom, chr_rom, .HORIZONTAL);
    defer mapper.deinit();

    // Attempt to write to CHR ROM (should be ignored)
    mapper.chr_write(0x0000, 0x42);

    // Should still read the original value
    try std.testing.expectEqual(@as(u8, 0xFF), mapper.chr_read(0x0000));
}
