const std = @import("std");
const Mapper = @import("mapper.zig").Mapper;
const Mirroring = @import("../rom.zig").Mirroring;

/// See: https://www.nesdev.org/wiki/INES_Mapper_002
/// Mapper 2 (UxROM)
/// Features:
/// - 16KB PRG ROM banking (switchable at $8000-$BFFF)
/// - 16KB PRG ROM fixed at $C000-$FFFF (last bank)
/// - 8KB CHR RAM (no CHR ROM banking)
/// - Fixed mirroring (set by ROM header)
/// - No IRQ support
pub const Mapper2 = struct {
    prg_rom: []const u8,
    chr_rom: []const u8,
    chr_ram: []u8,
    chr_is_ram: bool,

    selected_bank: u8,
    num_banks: u8,
    mirroring_mode: Mirroring,

    allocator: std.mem.Allocator,

    const Self = @This();

    pub fn init(
        allocator: std.mem.Allocator,
        prg_rom: []const u8,
        chr_rom: []const u8,
        prg_rom_banks: u8,
        mirroring_mode: Mirroring,
    ) !*Self {
        const self = try allocator.create(Self);

        const chr_is_ram = chr_rom.len == 0;
        var chr_ram: []u8 = undefined;
        if (chr_is_ram) {
            chr_ram = try allocator.alloc(u8, 8192);
            @memset(chr_ram, 0);
        } else {
            chr_ram = &.{};
        }

        self.* = .{
            .prg_rom = prg_rom,
            .chr_ram = chr_ram,
            .chr_rom = chr_rom,
            .selected_bank = 0,
            .chr_is_ram = chr_is_ram,
            .num_banks = prg_rom_banks,
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
        self.allocator.destroy(self);
    }

    pub fn prg_rom_read(self: *const Self, addr: u16) u8 {
        return switch (addr) {
            // Switchable bank
            0x8000...0xBFFF => blk: {
                const bank_offset = @as(u32, self.selected_bank) * 0x4000;
                const offset = addr - 0x8000;
                break :blk self.prg_rom[bank_offset + offset];
            },
            // Fixed to last bank
            0xC000...0xFFFF => blk: {
                const last_bank_offset = (@as(u32, self.num_banks) - 1) * 0x4000;
                const offset = addr - 0xC000;
                break :blk self.prg_rom[last_bank_offset + offset];
            },
            else => 0,
        };
    }

    pub fn prg_rom_write(self: *Self, addr: u16, value: u8) void {
        _ = addr;
        self.selected_bank = value & (self.num_banks - 1);
    }

    pub fn prg_ram_read(self: *Self, addr: u16) u8 {
        _ = self;
        _ = addr;
        return 0;
    }

    pub fn prg_ram_write(self: *Self, addr: u16, value: u8) void {
        _ = self;
        _ = addr;
        _ = value;
    }

    pub fn chr_read(self: *const Self, addr: u16) u8 {
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

// ============================================================================
// Tests
// ============================================================================

test "Mapper2 initialization" {
    const allocator = std.testing.allocator;

    const prg_rom = try allocator.alloc(u8, 0x20000); // 128KB (8 banks)
    defer allocator.free(prg_rom);
    @memset(prg_rom, 0);

    var mapper = try Mapper2.init(allocator, prg_rom, &[_]u8{}, .HORIZONTAL);
    defer mapper.deinit();

    try std.testing.expectEqual(@as(u8, 8), mapper.num_banks);
    try std.testing.expectEqual(@as(u8, 0), mapper.selected_bank);
    try std.testing.expectEqual(Mirroring.HORIZONTAL, mapper.mirroring());
}

test "Mapper2 PRG ROM banking" {
    const allocator = std.testing.allocator;

    // Create 8 banks of 16KB each
    var prg_rom = try allocator.alloc(u8, 0x20000); // 128KB
    defer allocator.free(prg_rom);

    // Fill each bank with its bank number for easy identification
    for (0..8) |bank| {
        const bank_start = bank * 0x4000;
        const bank_end = bank_start + 0x4000;
        @memset(prg_rom[bank_start..bank_end], @as(u8, @truncate(bank)));
    }

    var mapper = try Mapper2.init(allocator, prg_rom, &[_]u8{}, .HORIZONTAL);
    defer mapper.deinit();

    // Initially, bank 0 should be at $8000
    try std.testing.expectEqual(@as(u8, 0), mapper.prg_read(0x8000));
    try std.testing.expectEqual(@as(u8, 0), mapper.prg_read(0xBFFF));

    // Last bank (bank 7) should always be at $C000
    try std.testing.expectEqual(@as(u8, 7), mapper.prg_read(0xC000));
    try std.testing.expectEqual(@as(u8, 7), mapper.prg_read(0xFFFF));

    // Switch to bank 3
    mapper.prg_rom_write(0x8000, 3);
    try std.testing.expectEqual(@as(u8, 3), mapper.prg_read(0x8000));
    try std.testing.expectEqual(@as(u8, 3), mapper.prg_read(0xBFFF));

    // Last bank should still be fixed at $C000
    try std.testing.expectEqual(@as(u8, 7), mapper.prg_read(0xC000));

    // Switch to bank 5
    mapper.prg_rom_write(0xC000, 5); // Write to any address in ROM space
    try std.testing.expectEqual(@as(u8, 5), mapper.prg_read(0x8000));

    // Test bank wrapping (bank 15 % 8 = 7)
    mapper.prg_rom_write(0x8000, 15);
    try std.testing.expectEqual(@as(u8, 7), mapper.prg_read(0x8000));
}

test "Mapper2 CHR RAM read/write" {
    const allocator = std.testing.allocator;

    const prg_rom = try allocator.alloc(u8, 0x8000); // 32KB (2 banks)
    defer allocator.free(prg_rom);
    @memset(prg_rom, 0);

    var mapper = try Mapper2.init(allocator, prg_rom, &[_]u8{}, .VERTICAL);
    defer mapper.deinit();

    // CHR RAM should be initialized to 0
    try std.testing.expectEqual(@as(u8, 0), mapper.chr_read(0x0000));
    try std.testing.expectEqual(@as(u8, 0), mapper.chr_read(0x1FFF));

    // Write to CHR RAM
    mapper.chr_write(0x0000, 0x42);
    mapper.chr_write(0x1000, 0x69);
    mapper.chr_write(0x1FFF, 0xFF);

    // Read back
    try std.testing.expectEqual(@as(u8, 0x42), mapper.chr_read(0x0000));
    try std.testing.expectEqual(@as(u8, 0x69), mapper.chr_read(0x1000));
    try std.testing.expectEqual(@as(u8, 0xFF), mapper.chr_read(0x1FFF));

    // Test address wrapping (0x2000 wraps to 0x0000)
    mapper.chr_write(0x2000, 0xAA);
    try std.testing.expectEqual(@as(u8, 0xAA), mapper.chr_read(0x0000));
}

test "Mapper2 minimum ROM size (32KB)" {
    const allocator = std.testing.allocator;

    // Minimum UxROM size is 32KB (2 banks)
    var prg_rom = try allocator.alloc(u8, 0x8000); // 32KB
    defer allocator.free(prg_rom);

    // Bank 0
    @memset(prg_rom[0x0000..0x4000], 0xAA);
    // Bank 1
    @memset(prg_rom[0x4000..0x8000], 0xBB);

    var mapper = try Mapper2.init(allocator, prg_rom, &[_]u8{}, .HORIZONTAL);
    defer mapper.deinit();

    try std.testing.expectEqual(@as(u8, 2), mapper.num_banks);

    // Bank 0 at $8000
    try std.testing.expectEqual(@as(u8, 0xAA), mapper.prg_read(0x8000));

    // Bank 1 (last bank) at $C000
    try std.testing.expectEqual(@as(u8, 0xBB), mapper.prg_read(0xC000));

    // Switch to bank 1 at $8000
    mapper.prg_rom_write(0x8000, 1);
    try std.testing.expectEqual(@as(u8, 0xBB), mapper.prg_read(0x8000));
}

test "Mapper2 maximum practical ROM size (2MB)" {
    const allocator = std.testing.allocator;

    // Most UxROM games are much smaller, but test a large ROM
    const rom_size = 0x20000; // 128KB for testing (2MB would be too large for test)
    var prg_rom = try allocator.alloc(u8, rom_size);
    defer allocator.free(prg_rom);

    const num_banks = rom_size / 0x4000;
    for (0..num_banks) |bank| {
        const start = bank * 0x4000;
        const end = start + 0x4000;
        @memset(prg_rom[start..end], @as(u8, @truncate(bank)));
    }

    var mapper = try Mapper2.init(allocator, prg_rom, &[_]u8{}, .HORIZONTAL);
    defer mapper.deinit();

    // Test all banks are accessible
    for (0..num_banks) |bank| {
        mapper.prg_rom_write(0x8000, @truncate(bank));
        try std.testing.expectEqual(@as(u8, @truncate(bank)), mapper.prg_read(0x8000));
    }
}

test "Mapper2 invalid ROM size" {
    const allocator = std.testing.allocator;

    // ROM size not a multiple of 16KB should still work but warn
    const prg_rom = try allocator.alloc(u8, 0x5000); // 20KB (not multiple of 16KB)
    defer allocator.free(prg_rom);

    var mapper = try Mapper2.init(allocator, prg_rom, &[_]u8{}, .HORIZONTAL);
    defer mapper.deinit();

    // Should have calculated 1 bank (truncated)
    try std.testing.expectEqual(@as(u8, 1), mapper.num_banks);
}

test "Mapper2 bank selection with high bits" {
    const allocator = std.testing.allocator;

    var prg_rom = try allocator.alloc(u8, 0x10000); // 64KB (4 banks)
    defer allocator.free(prg_rom);

    for (0..4) |bank| {
        const start = bank * 0x4000;
        const end = start + 0x4000;
        @memset(prg_rom[start..end], @as(u8, @truncate(bank)));
    }

    var mapper = try Mapper2.init(allocator, prg_rom, &[_]u8{}, .HORIZONTAL);
    defer mapper.deinit();

    // Write with high bits set (should be masked)
    mapper.prg_rom_write(0x8000, 0xF2); // Binary: 11110010, should select bank 2
    try std.testing.expectEqual(@as(u8, 2), mapper.prg_read(0x8000));

    // Upper bits should be ignored
    mapper.prg_rom_write(0x8000, 0x83); // Binary: 10000011, should select bank 3
    try std.testing.expectEqual(@as(u8, 3), mapper.prg_read(0x8000));
}
