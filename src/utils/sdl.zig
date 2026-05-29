const std = @import("std");
const c = @import("../root.zig").c;

pub fn sdlError(result: anytype) if (@typeInfo(@TypeOf(result)) == .bool) void else @TypeOf(result) {
    const error_text = c.SDL_GetError();
    const _type = @typeInfo(@TypeOf(result));
    switch (_type) {
        .bool => if (!result) {
            std.debug.panic("SDL Error: {s}\n", .{error_text});
        },
        .pointer => if (result == null) {
            std.debug.panic("SDL Error: {s}\n", .{error_text});
        },
        .optional => if (result) |value| {
            return value;
        } else {
            std.debug.panic("SDL Error: {s}\n", .{error_text});
        },
        else => std.debug.panic("Not implemented: {}\n", .{_type}),
    }

    if (_type != .bool) {
        return result;
    }
}
