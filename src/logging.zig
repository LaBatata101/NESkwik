const std = @import("std");
const zeit = @import("zeit");

const paths = @import("utils/paths.zig");

const LOG_FILENAME = "NESkwik.log";

var mutex: std.Thread.Mutex = .{};
var log_file: ?std.fs.File = null;
var log_path: ?[]u8 = null;
var log_timezone: zeit.TimeZone = zeit.utc;

const FILE_WRITER_BUFFER_SIZE = 1024;

pub fn init(alloc: std.mem.Allocator) !void {
    if (log_file != null) return;

    var local_timezone = zeit.local(alloc, null) catch zeit.utc;
    errdefer local_timezone.deinit();

    const log_dir = try paths.getLogDir(alloc);
    defer alloc.free(log_dir);

    try std.fs.cwd().makePath(log_dir);

    const resolved_log_path = try std.fs.path.join(alloc, &.{ log_dir, LOG_FILENAME });
    errdefer alloc.free(resolved_log_path);

    const file = try std.fs.createFileAbsolute(resolved_log_path, .{ .truncate = true });
    errdefer file.close();

    log_file = file;
    log_path = resolved_log_path;
    log_timezone = local_timezone;
}

pub fn deinit(alloc: std.mem.Allocator) void {
    if (log_file) |file| {
        file.sync() catch {};
        file.close();
        log_file = null;
    }

    if (log_path) |resolved_log_path| {
        alloc.free(resolved_log_path);
        log_path = null;
    }

    log_timezone.deinit();
    log_timezone = zeit.utc;
}

pub fn path() ?[]const u8 {
    return log_path;
}

pub fn logFn(
    comptime message_level: std.log.Level,
    comptime scope: @Type(.enum_literal),
    comptime format: []const u8,
    args: anytype,
) void {
    _ = scope;

    mutex.lock();
    defer mutex.unlock();

    if (log_file) |file| {
        var buffer: [FILE_WRITER_BUFFER_SIZE]u8 = undefined;
        var file_writer = file.writerStreaming(&buffer);
        const writer = &file_writer.interface;
        writeLog(writer, message_level, format, args) catch {};
        writer.flush() catch {};
    }

    std.debug.lockStdErr();
    defer std.debug.unlockStdErr();
    var stderr_buffer: [FILE_WRITER_BUFFER_SIZE]u8 = undefined;
    var stderr_writer = std.fs.File.stderr().writer(&stderr_buffer);
    const writer = &stderr_writer.interface;
    writeLog(writer, message_level, format, args) catch {};
    writer.flush() catch {};
}

pub fn writePanic(message: []const u8) void {
    mutex.lock();
    defer mutex.unlock();

    const file = log_file orelse return;
    var buffer: [FILE_WRITER_BUFFER_SIZE]u8 = undefined;
    var file_writer = file.writerStreaming(&buffer);
    const writer = &file_writer.interface;

    writeTimestamp(writer) catch {};
    writer.print(" [FATAL] {s}\n", .{message}) catch {};
    writer.flush() catch {};
    file.sync() catch {};
}

fn writeLog(
    writer: anytype,
    comptime message_level: std.log.Level,
    comptime format: []const u8,
    args: anytype,
) !void {
    try writeTimestamp(writer);
    try writer.print(" [{s}] ", .{levelText(message_level)});
    try writer.print(format, args);
    try writer.writeByte('\n');
}

fn writeTimestamp(writer: anytype) !void {
    const now = try zeit.instant(.{ .timezone = &log_timezone });
    try now.time().strftime(writer, "%Y-%m-%d %H:%M:%S");
}

fn levelText(comptime level: std.log.Level) []const u8 {
    return switch (level) {
        .err => "ERROR",
        .warn => "WARN",
        .info => "INFO",
        .debug => "DEBUG",
    };
}
