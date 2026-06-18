const std = @import("std");
const zeit = @import("zeit");

const paths = @import("utils/paths.zig");

const LOG_FILENAME = "NESkwik.log";
const MAX_LOG_FILE_SIZE_BYTES = 10 * 1024 * 1024;

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

    const file = try std.fs.createFileAbsolute(resolved_log_path, .{ .truncate = false });
    errdefer file.close();

    const file_size = try file.getEndPos();
    if (file_size > MAX_LOG_FILE_SIZE_BYTES) {
        try file.setEndPos(0);
        try file.seekTo(0);
    } else {
        try file.seekFromEnd(0);
    }

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
    writer: *std.Io.Writer,
    comptime message_level: std.log.Level,
    comptime format: []const u8,
    args: anytype,
) !void {
    var prefixing_writer = LinePrefixingWriter.init(writer, levelText(message_level));
    try prefixing_writer.writer.print(format, args);
    try prefixing_writer.finish();
}

fn writeTimestamp(writer: *std.Io.Writer) !void {
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

const LinePrefixingWriter = struct {
    out: *std.Io.Writer,
    level_text: []const u8,
    at_line_start: bool = true,
    wrote_anything: bool = false,
    writer: std.Io.Writer = .{
        .buffer = &.{},
        .vtable = &.{ .drain = drain },
    },

    fn init(out: *std.Io.Writer, level_text: []const u8) LinePrefixingWriter {
        return .{
            .out = out,
            .level_text = level_text,
        };
    }

    fn drain(writer: *std.Io.Writer, data: []const []const u8, splat: usize) std.Io.Writer.Error!usize {
        const self: *LinePrefixingWriter = @alignCast(@fieldParentPtr("writer", writer));

        try self.writeBytes(writer.buffered());
        writer.end = 0;

        var consumed: usize = 0;
        for (data[0 .. data.len - 1]) |bytes| {
            try self.writeBytes(bytes);
            consumed += bytes.len;
        }

        const splat_bytes = data[data.len - 1];
        for (0..splat) |_| {
            try self.writeBytes(splat_bytes);
            consumed += splat_bytes.len;
        }

        return consumed;
    }

    fn writeBytes(self: *LinePrefixingWriter, bytes: []const u8) std.Io.Writer.Error!void {
        var start: usize = 0;
        while (start < bytes.len) {
            if (self.at_line_start) try self.writePrefix();

            const newline_index = std.mem.indexOfScalarPos(u8, bytes, start, '\n') orelse {
                try self.out.writeAll(bytes[start..]);
                self.at_line_start = false;
                return;
            };

            try self.out.writeAll(bytes[start..newline_index]);
            try self.out.writeByte('\n');
            self.at_line_start = true;
            start = newline_index + 1;
        }
    }

    fn finish(self: *LinePrefixingWriter) std.Io.Writer.Error!void {
        if (!self.wrote_anything) try self.writePrefix();
        if (!self.at_line_start) try self.out.writeByte('\n');
    }

    fn writePrefix(self: *LinePrefixingWriter) std.Io.Writer.Error!void {
        writeTimestamp(self.out) catch return error.WriteFailed;
        self.out.print(" [{s}] ", .{self.level_text}) catch return error.WriteFailed;
        self.at_line_start = false;
        self.wrote_anything = true;
    }
};
