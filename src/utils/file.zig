const std = @import("std");
const builtin = @import("builtin");
const sdlError = @import("sdl.zig").sdlError;
const c = @import("../root.zig").c;

pub fn readFile(alloc: std.mem.Allocator, uri: []const u8) ![]u8 {
    if (builtin.abi.isAndroid()) {
        const uriz = try alloc.dupeZ(u8, uri);
        defer alloc.free(uriz);
        var size: usize = 0;

        const data = sdlError(c.SDL_LoadFile(uriz.ptr, &size));
        defer c.SDL_free(data);

        return try alloc.dupe(u8, @as([*]const u8, @ptrCast(data))[0..size]);
    } else {
        const file = std.fs.openFileAbsolute(uri, .{}) catch |err| switch (err) {
            else => {
                std.debug.panic("Error while opening file: {any}\n", .{err});
            },
        };
        defer file.close();

        const file_size = try file.getEndPos();
        try file.seekTo(0);

        const buffer = try alloc.alloc(u8, file_size);
        _ = try file.read(buffer);

        return buffer;
    }
}
