const std = @import("std");

pub const Ram = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    const Self = @This();
    pub const VTable = struct {
        deinit: *const fn (ptr: *anyopaque) void,
        read: *const fn (ptr: *const anyopaque, addr: u16) u8,
        write: *const fn (ptr: *const anyopaque, addr: u16, value: u8) void,
    };

    pub fn deinit(self: *Self) void {
        self.vtable.deinit(self.ptr);
    }

    pub fn write(self: *Self, addr: u16, value: u8) void {
        self.vtable.write(self.ptr, addr, value);
    }

    pub fn read(self: *const Self, addr: u16) u8 {
        return self.vtable.read(self.ptr, addr);
    }
};

pub const VolatileRam = struct {
    alloc: std.mem.Allocator,
    buffer: []u8,

    const Self = @This();
    const VTable = Ram.VTable{
        .deinit = @ptrCast(&Self.deinit),
        .read = @ptrCast(&Self.read),
        .write = @ptrCast(&Self.write),
    };

    pub fn init(alloc: std.mem.Allocator, size: usize) !*Self {
        const self = try alloc.create(Self);
        const buffer = try alloc.alloc(u8, size);
        @memset(buffer, 0);

        self.* = .{
            .alloc = alloc,
            .buffer = buffer,
        };
        return self;
    }

    fn deinit(self: *Self) void {
        self.alloc.free(self.buffer);
        self.alloc.destroy(self);
    }

    fn write(self: *Self, addr: u16, value: u8) void {
        self.buffer[addr % self.buffer.len] = value;
    }

    fn read(self: *const Self, addr: u16) u8 {
        return self.buffer[addr % self.buffer.len];
    }

    pub fn as_ram(self: *Self) Ram {
        return .{ .ptr = self, .vtable = &VTable };
    }
};

pub const BatteryBackedRam = struct {
    alloc: std.mem.Allocator,
    file: std.fs.File,
    buffer: []u8,
    save_file_path: []u8,

    const Self = @This();
    const VTable = Ram.VTable{
        .deinit = @ptrCast(&Self.deinit),
        .read = @ptrCast(&Self.read),
        .write = @ptrCast(&Self.write),
    };

    pub fn init(alloc: std.mem.Allocator, path: []const u8, size: usize) !*Self {
        const self = try alloc.create(Self);

        const dir = std.fs.path.dirname(path) orelse @panic("Path is root directory");
        const base = std.fs.path.stem(path);

        // TODO: support windows
        const save_path = try std.fmt.allocPrint(alloc, "{s}/{s}.sav", .{ dir, base });
        const file = try std.fs.createFileAbsolute(save_path, .{
            .read = true,
            .truncate = false,
        });

        const stat = try file.stat();
        if (stat.size < size) {
            try file.setEndPos(size);
        }

        // TODO: support windows
        const buffer = try std.posix.mmap(
            null,
            size,
            std.posix.PROT.READ | std.posix.PROT.WRITE,
            .{ .TYPE = .SHARED },
            file.handle,
            0,
        );

        self.* = .{
            .alloc = alloc,
            .file = file,
            .buffer = buffer,
            .save_file_path = save_path,
        };
        return self;
    }

    fn deinit(self: *Self) void {
        std.posix.munmap(@alignCast(self.buffer));
        self.file.close();
        self.alloc.free(self.save_file_path);
        self.alloc.destroy(self);
    }

    fn write(self: *Self, addr: u16, value: u8) void {
        self.buffer[addr % self.buffer.len] = value;
    }

    fn read(self: *const Self, addr: u16) u8 {
        return self.buffer[addr % self.buffer.len];
    }

    pub fn as_ram(self: *Self) Ram {
        return .{ .ptr = self, .vtable = &VTable };
    }
};
