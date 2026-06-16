const std = @import("std");

pub fn formatByteCount(alloc: std.mem.Allocator, bytes: u64) []const u8 {
    const kb: u64 = 1024;
    const mb: u64 = kb * 1024;
    if (bytes >= mb) {
        const value = @as(f64, @floatFromInt(bytes)) / @as(f64, @floatFromInt(mb));
        return std.fmt.allocPrint(alloc, "{d:.1} MB", .{value}) catch "? MB";
    }
    if (bytes >= kb) {
        const value = @as(f64, @floatFromInt(bytes)) / @as(f64, @floatFromInt(kb));
        return std.fmt.allocPrint(alloc, "{d:.1} KB", .{value}) catch "? KB";
    }
    return std.fmt.allocPrint(alloc, "{d} B", .{bytes}) catch "? B";
}

pub fn formatPlayTime(secs: u64, alloc: std.mem.Allocator) []const u8 {
    if (secs < 60) return "< 1m";
    const minutes = secs / 60;
    const hours = minutes / 60;
    const mins_rem = minutes % 60;
    if (hours == 0) return std.fmt.allocPrint(alloc, "{d}m", .{minutes}) catch "?";
    if (mins_rem == 0) return std.fmt.allocPrint(alloc, "{d}h", .{hours}) catch "?";
    return std.fmt.allocPrint(alloc, "{d}h {d}m", .{ hours, mins_rem }) catch "?";
}
