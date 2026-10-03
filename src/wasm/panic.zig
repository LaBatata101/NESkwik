const c = @import("../root.zig").c;
const browser = @import("browser.zig");

/// Logs the panic and shows it in a dialog on the page, like the message box
/// of native builds, then traps the panicking thread.
pub fn panic(msg: []const u8, _: ?usize) noreturn {
    const stack = browser.panicStack();
    c.SDL_Log("Fatal error: %.*s\nStack trace:\n%s", @as(c_int, @intCast(msg.len)), msg.ptr, stack);
    browser.showPanic(msg, stack);
    @trap();
}
