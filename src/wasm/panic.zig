const c = @import("../root.zig").c;

pub fn panic(msg: []const u8, _: ?usize) noreturn {
    c.SDL_Log("Fatal error: %.*s", @as(c_int, @intCast(msg.len)), msg.ptr);
    @trap();
}
