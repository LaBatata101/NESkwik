const std = @import("std");
const Mapper = @import("mapper.zig").Mapper;
const Mirroring = @import("../rom.zig").Mirroring;
const MapperParams = @import("mapper.zig").MapperParams;

/// See: https://www.nesdev.org/wiki/CNROM
/// Mapper 3 (CNROM)
/// Features:
/// - 16KB or 32KB PRG ROM (no banking)
/// - 8KB CHR ROM banking (up to 2048KB / 256 banks)
/// - Fixed mirroring (set by ROM header)
/// - No PRG RAM
/// - No IRQ support
///
/// This is one of the simplest mappers - it only allows switching CHR ROM banks.
/// PRG ROM is fixed and cannot be banked.
pub const Mapper3 = struct {
    prg_rom: []const u8,
    chr_rom: []const u8,
    prg_ram: []u8,

    selected_chr_bank: u8,
    num_chr_banks: u8,

    mirroring_mode: Mirroring,

    allocator: std.mem.Allocator,

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator, params: MapperParams) !*Self {
        const self = try allocator.create(Self);

        // CNROM always has CHR ROM (never CHR RAM)
        const num_chr_banks: u8 = @intCast(params.chr_rom.len / 0x2000);

        const prg_ram = try allocator.alloc(u8, params.prg_ram_size);
        @memset(prg_ram, 0);

        self.* = .{
            .prg_rom = params.prg_rom,
            .chr_rom = params.chr_rom,
            .prg_ram = prg_ram,
            .selected_chr_bank = 0,
            .num_chr_banks = num_chr_banks,
            .mirroring_mode = params.mirroring_mode,
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
        .irq_active = @ptrCast(&Self.irq_active),
        .mirroring = @ptrCast(&Self.mirroring),
    };

    pub fn as_mapper(self: *Self) Mapper {
        return .{ .ptr = self, .vtable = &VTableImpl };
    }

    pub fn deinit(self: *Self) void {
        self.allocator.free(self.prg_ram);
        self.allocator.destroy(self);
    }

    pub fn prg_rom_read(self: *const Self, addr: u16) u8 {
        // PRG ROM is fixed, no banking
        // For 16KB ROMs, mirror the upper 16KB
        return self.prg_rom[addr % self.prg_rom.len];
    }

    pub fn prg_rom_write(self: *Self, addr: u16, value: u8) void {
        _ = addr;
        // Any write to PRG ROM space selects CHR bank
        // Use only the bits needed based on number of banks
        self.selected_chr_bank = value & (self.num_chr_banks - 1);
    }

    pub fn prg_ram_read(self: *Self, addr: u16) u8 {
        // CNROM has no PRG RAM, but we implement it for tests that use it.
        if (self.prg_ram.len == 0) {
            return 0;
        }
        return self.prg_ram[addr % self.prg_ram.len];
    }

    pub fn prg_ram_write(self: *Self, addr: u16, value: u8) void {
        // CNROM has no PRG RAM, but we implement it for tests that use it.
        if (self.prg_ram.len > 0) {
            self.prg_ram[addr % self.prg_ram.len] = value;
        }
    }

    pub fn chr_read(self: *const Self, addr: u16) u8 {
        // Calculate offset into CHR ROM based on selected bank
        const bank_offset = @as(u32, self.selected_chr_bank) * 0x2000;
        const final_addr = bank_offset + addr;

        return self.chr_rom[final_addr % self.chr_rom.len];
    }

    pub fn chr_write(self: *Self, addr: u16, value: u8) void {
        _ = self;
        _ = addr;
        _ = value;
        // CNROM has CHR ROM, not CHR RAM, so writes are ignored
    }

    pub fn mirroring(self: *const Self) Mirroring {
        return self.mirroring_mode;
    }

    pub fn irq_active(self: *Self) bool {
        _ = self;
        return false;
    }
};

// ============================================================================
// Tests
// ============================================================================

test "Mapper3 initialization" {
    const allocator = std.testing.allocator;

    const prg_rom = try allocator.alloc(u8, 0x8000); // 32KB
    defer allocator.free(prg_rom);
    @memset(prg_rom, 0);

    const chr_rom = try allocator.alloc(u8, 0x8000); // 32KB (4 banks)
    defer allocator.free(chr_rom);
    @memset(chr_rom, 0);

    var mapper = try Mapper3.init(allocator, .{
        .prg_rom = prg_rom,
        .chr_rom = chr_rom,
        .mirroring_mode = .HORIZONTAL,
        .has_battery_backed_ram = false,
        .prg_ram_size = 0,
        .prg_rom_banks = 0,
        .rom_path = "test.rom",
    });
    defer mapper.deinit();

    try std.testing.expectEqual(@as(u8, 4), mapper.num_chr_banks);
    try std.testing.expectEqual(@as(u8, 0), mapper.selected_chr_bank);
    try std.testing.expectEqual(Mirroring.HORIZONTAL, mapper.mirroring());
}

test "Mapper3 CHR banking" {
    const allocator = std.testing.allocator;

    const prg_rom = try allocator.alloc(u8, 0x4000); // 16KB
    defer allocator.free(prg_rom);
    @memset(prg_rom, 0);

    // Create 4 banks of 8KB CHR ROM
    var chr_rom = try allocator.alloc(u8, 0x8000); // 32KB
    defer allocator.free(chr_rom);

    // Fill each bank with its bank number for easy identification
    for (0..4) |bank| {
        const bank_start = bank * 0x2000;
        const bank_end = bank_start + 0x2000;
        @memset(chr_rom[bank_start..bank_end], @as(u8, @truncate(bank)));
    }

    var mapper = try Mapper3.init(allocator, .{
        .prg_rom = prg_rom,
        .chr_rom = chr_rom,
        .mirroring_mode = .HORIZONTAL,
        .has_battery_backed_ram = false,
        .prg_ram_size = 0,
        .prg_rom_banks = 0,
        .rom_path = "test.rom",
    });
    defer mapper.deinit();

    // Initially, bank 0 should be selected
    try std.testing.expectEqual(@as(u8, 0), mapper.chr_read(0x0000));
    try std.testing.expectEqual(@as(u8, 0), mapper.chr_read(0x1FFF));

    // Switch to bank 1
    mapper.prg_rom_write(0x8000, 1);
    try std.testing.expectEqual(@as(u8, 1), mapper.chr_read(0x0000));
    try std.testing.expectEqual(@as(u8, 1), mapper.chr_read(0x1FFF));

    // Switch to bank 2
    mapper.prg_rom_write(0x8000, 2);
    try std.testing.expectEqual(@as(u8, 2), mapper.chr_read(0x0000));
    try std.testing.expectEqual(@as(u8, 2), mapper.chr_read(0x1FFF));

    // Switch to bank 3
    mapper.prg_rom_write(0x8000, 3);
    try std.testing.expectEqual(@as(u8, 3), mapper.chr_read(0x0000));
    try std.testing.expectEqual(@as(u8, 3), mapper.chr_read(0x1FFF));
}

test "Mapper3 PRG ROM mirroring (16KB)" {
    const allocator = std.testing.allocator;

    // 16KB PRG ROM
    const prg_rom = try allocator.alloc(u8, 0x4000);
    defer allocator.free(prg_rom);

    for (prg_rom, 0..) |*byte, i| {
        byte.* = @truncate(i);
    }

    const chr_rom = try allocator.alloc(u8, 0x2000);
    defer allocator.free(chr_rom);
    @memset(chr_rom, 0);

    var mapper = try Mapper3.init(allocator, .{
        .prg_rom = prg_rom,
        .chr_rom = chr_rom,
        .mirroring_mode = .VERTICAL,
        .has_battery_backed_ram = false,
        .prg_ram_size = 0,
        .prg_rom_banks = 0,
        .rom_path = "test.rom",
    });
    defer mapper.deinit();

    // Test that the upper 16KB mirrors the lower 16KB
    try std.testing.expectEqual(prg_rom[0], mapper.prg_rom_read(0x8000));
    try std.testing.expectEqual(prg_rom[0], mapper.prg_rom_read(0xC000));
    try std.testing.expectEqual(prg_rom[0x100], mapper.prg_rom_read(0x8100));
    try std.testing.expectEqual(prg_rom[0x100], mapper.prg_rom_read(0xC100));
}

test "Mapper3 PRG ROM no mirroring (32KB)" {
    const allocator = std.testing.allocator;

    // 32KB PRG ROM
    const prg_rom = try allocator.alloc(u8, 0x8000);
    defer allocator.free(prg_rom);

    for (prg_rom, 0..) |*byte, i| {
        byte.* = @truncate(i);
    }

    const chr_rom = try allocator.alloc(u8, 0x2000);
    defer allocator.free(chr_rom);
    @memset(chr_rom, 0);

    var mapper = try Mapper3.init(allocator, .{
        .prg_rom = prg_rom,
        .chr_rom = chr_rom,
        .mirroring_mode = .VERTICAL,
        .has_battery_backed_ram = false,
        .prg_ram_size = 0,
        .prg_rom_banks = 0,
        .rom_path = "test.rom",
    });
    defer mapper.deinit();

    // Test that different addresses return different values
    try std.testing.expectEqual(prg_rom[0], mapper.prg_rom_read(0x8000));
    try std.testing.expectEqual(prg_rom[0x4000], mapper.prg_rom_read(0xC000));
}

test "Mapper3 bank masking" {
    const allocator = std.testing.allocator;

    const prg_rom = try allocator.alloc(u8, 0x4000);
    defer allocator.free(prg_rom);
    @memset(prg_rom, 0);

    // Create 4 banks of CHR ROM
    var chr_rom = try allocator.alloc(u8, 0x8000);
    defer allocator.free(chr_rom);

    for (0..4) |bank| {
        const bank_start = bank * 0x2000;
        const bank_end = bank_start + 0x2000;
        @memset(chr_rom[bank_start..bank_end], @as(u8, @truncate(bank)));
    }

    var mapper = try Mapper3.init(allocator, .{
        .prg_rom = prg_rom,
        .chr_rom = chr_rom,
        .mirroring_mode = .HORIZONTAL,
        .has_battery_backed_ram = false,
        .prg_ram_size = 0,
        .prg_rom_banks = 0,
        .rom_path = "test.rom",
    });
    defer mapper.deinit();

    // Write with high bits set (should be masked)
    mapper.prg_rom_write(0x8000, 0xFF); // Should select bank 3 (4 banks = 2 bits)
    try std.testing.expectEqual(@as(u8, 3), mapper.chr_read(0x0000));

    // Write bank 5 (should wrap to bank 1 with 4 banks)
    mapper.prg_rom_write(0x8000, 5);
    try std.testing.expectEqual(@as(u8, 1), mapper.chr_read(0x0000));
}
