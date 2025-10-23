const std = @import("std");
const Mirroring = @import("../rom.zig").Mirroring;
const Mapper = @import("mapper.zig").Mapper;

/// Mapper 0 (NROM)
/// The simplest mapper with no bank switching
/// - 16 KB or 32 KB PRG ROM
/// - 8 KB CHR ROM or CHR RAM
/// - Fixed mirroring (horizontal or vertical)
pub const Mapper0 = struct {
    prg_rom: []const u8,
    chr_memory: []u8,
    chr_is_ram: bool,
    mirroring_mode: Mirroring,
    allocator: std.mem.Allocator,

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator, prg_rom: []const u8, chr_rom: []const u8, mirroring_mode: Mirroring) !*Self {
        const self = try allocator.create(Self);
        // Determine if we need CHR RAM (when CHR ROM is empty or very small)
        const chr_is_ram = chr_rom.len == 0;

        var chr_memory: []u8 = undefined;

        if (chr_is_ram) {
            // Allocate 8KB CHR RAM
            chr_memory = try allocator.alloc(u8, 8192);
            @memset(chr_memory, 0);
        } else {
            // Copy CHR ROM to mutable memory (some games expect to write to CHR)
            chr_memory = try allocator.alloc(u8, chr_rom.len);
            @memcpy(chr_memory, chr_rom);
        }

        self.* = .{
            .prg_rom = prg_rom,
            .chr_memory = chr_memory,
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
        .prg_read = @ptrCast(&Self.prg_read),
        .prg_write = @ptrCast(&Self.prg_write),
        .ppu_clock = @ptrCast(&Self.ppu_clock),
        .cpu_clock = @ptrCast(&Self.cpu_clock),
        .irq_active = @ptrCast(&Self.irq_active),
        .irq_clear = @ptrCast(&Self.irq_clear),
        .mirroring = @ptrCast(&Self.mirroring),
    };

    pub fn as_mapper(self: *Self) Mapper {
        return .{ .ptr = self, .vtable = &VTableImpl };
    }

    pub fn deinit(self: *Self) void {
        self.allocator.free(self.chr_memory);
        self.allocator.destroy(self);
    }

    pub fn prg_read(self: *Self, addr: u16) u8 {
        // PRG ROM is mapped to $8000-$FFFF
        const mapped_addr = addr - 0x8000;

        // If PRG ROM is 16KB, mirror it in the upper 16KB
        if (self.prg_rom.len <= 0x4000 and mapped_addr >= 0x4000) {
            return self.prg_rom[mapped_addr % 0x4000];
        }

        return self.prg_rom[mapped_addr];
    }

    pub fn prg_write(self: *Self, addr: u16, value: u8) void {
        // Mapper 0 doesn't support PRG ROM writes
        // Some games might write here anyway, so we just ignore it
        _ = self;

        std.log.debug("Mapper 0: Attempted write to PRG ROM at ${X:04} = ${X:02}", .{ addr, value });
    }

    pub fn chr_read(self: *Self, addr: u16) u8 {
        // CHR memory is mapped to $0000-$1FFF
        const mapped_addr = addr & 0x1FFF;

        if (mapped_addr < self.chr_memory.len) {
            return self.chr_memory[mapped_addr];
        }

        return 0;
    }

    pub fn chr_write(self: *Self, addr: u16, value: u8) void {
        // Only allow writes if CHR RAM is present
        if (!self.chr_is_ram) {
            return;
        }

        const mapped_addr = addr & 0x1FFF;

        if (mapped_addr < self.chr_memory.len) {
            self.chr_memory[mapped_addr] = value;
        }
    }

    pub fn mirroring(self: *const Self) Mirroring {
        return self.mirroring_mode;
    }

    pub fn ppu_clock(self: *Self) void {
        _ = self;
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
    try std.testing.expectEqual(prg_rom[0], mapper.prg_read(0x8000));
    try std.testing.expectEqual(prg_rom[0], mapper.prg_read(0xC000));
    try std.testing.expectEqual(prg_rom[0x100], mapper.prg_read(0x8100));
    try std.testing.expectEqual(prg_rom[0x100], mapper.prg_read(0xC100));
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
    try std.testing.expectEqual(prg_rom[0], mapper.prg_read(0x8000));
    try std.testing.expectEqual(prg_rom[0x4000], mapper.prg_read(0xC000));
    try std.testing.expect(mapper.prg_read(0x8000) != mapper.prg_read(0xC000));
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
