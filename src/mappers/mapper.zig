const std = @import("std");
const Mirroring = @import("../rom.zig").Mirroring;
const Mapper0 = @import("mapper0.zig").Mapper0;

pub const Mapper = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        deinit: *const fn (ptr: *anyopaque) void,
        prg_read: *const fn (ptr: *anyopaque, addr: u16) u8,
        prg_write: *const fn (ptr: *anyopaque, addr: u16, value: u8) void,
        chr_read: *const fn (ptr: *anyopaque, addr: u16) u8,
        chr_write: *const fn (ptr: *anyopaque, addr: u16, value: u8) void,
        mirroring: *const fn (ptr: *const anyopaque) Mirroring,
        irq_active: *const fn (ptr: *anyopaque) bool,
        irq_clear: *const fn (ptr: *anyopaque) void,
        ppu_clock: *const fn (ptr: *anyopaque) void,
        cpu_clock: *const fn (ptr: *anyopaque) void,
    };

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator, mapper_id: u8, prg_rom: []const u8, chr_rom: []const u8, mirroring_mode: Mirroring) !Self {
        switch (mapper_id) {
            0 => {
                const mapper = try Mapper0.init(allocator, prg_rom, chr_rom, mirroring_mode);
                return mapper.as_mapper();
            },
            else => std.debug.panic("Unsupported mapper: {}\n", .{mapper_id}),
        }
    }

    pub fn deinit(self: *Self) void {
        self.vtable.deinit(self.ptr);
    }

    /// Read from PRG ROM space ($8000-$FFFF)
    pub fn prg_read(self: *Self, addr: u16) u8 {
        return self.vtable.prg_read(self.ptr, addr);
    }

    /// Write to PRG ROM space (for mappers that support it)
    pub fn prg_write(self: *Self, addr: u16, value: u8) void {
        self.vtable.prg_write(self.ptr, addr, value);
    }

    /// Read from CHR ROM/RAM space ($0000-$1FFF)
    pub fn chr_read(self: *Self, addr: u16) u8 {
        return self.vtable.chr_read(self.ptr, addr);
    }

    /// Write to CHR ROM/RAM space
    pub fn chr_write(self: *Self, addr: u16, value: u8) void {
        self.vtable.chr_write(self.ptr, addr, value);
    }

    /// Get current mirroring mode
    pub fn mirroring(self: *const Self) Mirroring {
        return self.vtable.mirroring(self.ptr);
    }

    /// Check if mapper requests IRQ
    pub fn irq_active(self: *Self) bool {
        return self.vtable.irq_active(self.ptr);
    }

    /// Clear mapper IRQ flag
    pub fn irq_clear(self: *Self) void {
        self.vtable.irq_clear(self.ptr);
    }

    /// Called on each PPU clock for mappers that need it
    pub fn ppu_clock(self: *Self) void {
        self.vtable.ppu_clock(self.ptr);
    }

    /// Called on each CPU clock for mappers that need it
    pub fn cpu_clock(self: *Self) void {
        self.vtable.cpu_clock(self.ptr);
    }
};
