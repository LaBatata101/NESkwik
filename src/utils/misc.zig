const std = @import("std");


pub fn formatPlayTime(secs: u64, alloc: std.mem.Allocator) []const u8 {
    if (secs < 60) return "< 1m";
    const minutes = secs / 60;
    const hours = minutes / 60;
    const mins_rem = minutes % 60;
    if (hours == 0) return std.fmt.allocPrint(alloc, "{d}m", .{minutes}) catch "?";
    if (mins_rem == 0) return std.fmt.allocPrint(alloc, "{d}h", .{hours}) catch "?";
    return std.fmt.allocPrint(alloc, "{d}h {d}m", .{ hours, mins_rem }) catch "?";
}
