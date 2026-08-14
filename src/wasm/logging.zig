const std = @import("std");
const c = @import("../root.zig").c;

pub fn init(_: std.mem.Allocator, _: std.Io) !void {}
pub fn deinit(_: std.mem.Allocator) void {}

pub fn logFn(
    comptime message_level: std.log.Level,
    comptime scope: @EnumLiteral(),
    comptime format: []const u8,
    args: anytype,
) void {
    _ = scope;
    const priority, const level_text = logLevel(message_level);

    // Longer messages are truncated.
    var buffer: [2048]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    writer.print("[{s}] " ++ format, .{level_text} ++ args) catch {};
    const message = writer.buffered();

    c.SDL_LogMessage(c.SDL_LOG_CATEGORY_APPLICATION, priority, "%.*s", @as(c_int, @intCast(message.len)), message.ptr);
}

fn logLevel(comptime level: std.log.Level) struct { c.SDL_LogPriority, []const u8 } {
    return switch (level) {
        .err => .{ c.SDL_LOG_PRIORITY_ERROR, "ERROR" },
        .warn => .{ c.SDL_LOG_PRIORITY_WARN, "WARN" },
        .info => .{ c.SDL_LOG_PRIORITY_INFO, "INFO" },
        .debug => .{ c.SDL_LOG_PRIORITY_DEBUG, "DEBUG" },
    };
}
