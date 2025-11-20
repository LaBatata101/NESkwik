const std = @import("std");
const Mirroring = @import("../rom.zig").Mirroring;
const Mapper0 = @import("mapper0.zig").Mapper0;
const Mapper1 = @import("mapper1.zig").Mapper1;
const Mapper2 = @import("mapper2.zig").Mapper2;
const Mapper3 = @import("mapper3.zig").Mapper3;
const Mapper4 = @import("mapper4.zig").Mapper4;

pub const MapperParams = struct {
    rom_path: []const u8,
    prg_rom: []const u8,
    chr_rom: []const u8,
    prg_rom_banks: u8,
    prg_ram_size: usize,
    has_battery_backed_ram: bool,
    mirroring_mode: Mirroring,
};

pub const Mapper = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    const DefaultImpl = struct {
        fn ppu_address_updated(ptr: *anyopaque, addr: u16) void {
            _ = ptr;
            _ = addr;
        }
    };
    pub const VTable = struct {
        deinit: *const fn (ptr: *anyopaque) void,
        prg_rom_read: *const fn (ptr: *anyopaque, addr: u16) u8,
        prg_rom_write: *const fn (ptr: *anyopaque, addr: u16, value: u8) void,
        prg_ram_read: *const fn (ptr: *anyopaque, addr: u16) u8,
        prg_ram_write: *const fn (ptr: *anyopaque, addr: u16, value: u8) void,
        chr_read: *const fn (ptr: *anyopaque, addr: u16) u8,
        chr_write: *const fn (ptr: *anyopaque, addr: u16, value: u8) void,
        mirroring: *const fn (ptr: *const anyopaque) Mirroring,
        irq_active: *const fn (ptr: *anyopaque) bool,
        ppu_address_updated: *const fn (ptr: *anyopaque, addr: u16) void = @ptrCast(&DefaultImpl.ppu_address_updated),
    };

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator, mapper_id: u8, params: MapperParams) !Self {
        switch (mapper_id) {
            0 => {
                const mapper = try Mapper0.init(allocator, params);
                return mapper.as_mapper();
            },
            1 => {
                const mapper = try Mapper1.init(allocator, params);
                return mapper.as_mapper();
            },
            2 => {
                const mapper = try Mapper2.init(allocator, params);
                return mapper.as_mapper();
            },
            3 => {
                const mapper = try Mapper3.init(allocator, params);
                return mapper.as_mapper();
            },
            4 => {
                const mapper = try Mapper4.init(allocator, params);
                return mapper.as_mapper();
            },
            else => std.debug.panic("Unsupported mapper: {}\n", .{mapper_id}),
        }
    }

    pub fn deinit(self: *Self) void {
        self.vtable.deinit(self.ptr);
    }

    /// Read from PRG ROM space ($8000-$FFFF)
    pub fn prg_rom_read(self: *Self, addr: u16) u8 {
        return self.vtable.prg_rom_read(self.ptr, addr);
    }

    /// Write to PRG ROM space ($8000-$FFFF)
    pub fn prg_rom_write(self: *Self, addr: u16, value: u8) void {
        self.vtable.prg_rom_write(self.ptr, addr, value);
    }

    /// Read from PRG RAM space ($6000–$7FFF)
    pub fn prg_ram_read(self: *Self, addr: u16) u8 {
        return self.vtable.prg_ram_read(self.ptr, addr);
    }

    /// Write to PRG RAM space ($6000–$7FFF)
    pub fn prg_ram_write(self: *Self, addr: u16, value: u8) void {
        self.vtable.prg_ram_write(self.ptr, addr, value);
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

    pub fn ppu_address_updated(self: *Self, addr: u16) void {
        self.vtable.ppu_address_updated(self.ptr, addr);
    }
};
