const std = @import("std");
const compress = @import("../utils/compress.zig");

pub const alpn = "neskwik/netplay";
pub const session_code_prefix = "neskwik:";

pub const max_rom_size: usize = 1 * 1024 * 1024;
pub const max_snapshot_size: usize = 1 * 1024 * 1024;
pub const max_display_name: usize = 255;
pub const framebuffer_size: usize = 256 * 240 * 4;
pub const max_message_size: usize = max_snapshot_size + max_rom_size;

pub const Digest = [32]u8;
pub const AcknowledgementDisposition = enum { current, stale };

pub const Preview = struct {
    /// Display filename shown to the client before it accepts the session.
    name: []const u8,
    /// Uncompressed ROM size in bytes.
    rom_size: u32,
    /// BLAKE3 digest of the complete ROM bytes.
    rom_hash: Digest,
    /// RGBA preview of the host's current 256x240 frame.
    framebuffer: []const u8,
};

pub const JoinData = struct {
    /// Display filename associated with the transferred ROM.
    name: []const u8,
    /// Complete, uncompressed ROM bytes. The client verifies them against the
    /// approved `Preview.rom_hash`.
    rom: []const u8,
    /// Encoded emulator snapshot from which both peers begin.
    snapshot: []const u8,
    /// Wire value of the host's emulation-speed setting.
    speed: u8,
    /// State generation containing the initial snapshot.
    epoch: u32,
    /// Authoritative frame represented by the initial snapshot.
    frame: u64,
};

pub const Frame = struct {
    /// State generation in which this frame must be applied.
    epoch: u32,
    /// Monotonically increasing authoritative frame number.
    frame: u64,
    /// Host controller state used to execute this frame.
    player1: u8,
    /// Client controller state used by the host to execute this frame.
    player2: u8,
    /// Optional host checkpoint digest of the state after this frame.
    digest: ?Digest = null,
};

pub const Ack = struct {
    /// State generation being acknowledged.
    epoch: u32,
    /// Latest authoritative frame applied by the client.
    frame: u64,
    /// Client controller state for a subsequent authoritative frame.
    player2: u8,
    /// Client checkpoint digest when the acknowledged frame requested one.
    digest: ?Digest = null,
};

pub const Rebase = struct {
    /// New state generation established by this rebase.
    epoch: u32,
    /// Authoritative frame represented by `snapshot`.
    frame: u64,
    /// Encoded replacement state from the host.
    snapshot: []const u8,
};

pub const Control = union(enum) {
    paused: bool,
    speed: u8,
};

pub const Message = union(enum(u8)) {
    preview: Preview,
    join: void,
    join_data: JoinData,
    ready: Ack,
    frame: Frame,
    ack: Ack,
    control: Control,
    rebase: Rebase,
    disconnect: []const u8,

    pub fn deinit(self: *Message, alloc: std.mem.Allocator) void {
        switch (self.*) {
            .preview => |*value| {
                alloc.free(value.name);
                alloc.free(value.framebuffer);
            },
            .join_data => |*value| {
                alloc.free(value.name);
                alloc.free(value.rom);
                alloc.free(value.snapshot);
            },
            .rebase => |*value| alloc.free(value.snapshot),
            .disconnect => |value| alloc.free(value),
            else => {},
        }
    }
};

pub const MessageTag = std.meta.Tag(Message);

pub fn validateReady(expected_epoch: u32, expected_frame: u64, accepting_ready: bool, ready: Ack) !void {
    if (!accepting_ready) return error.DuplicateReady;
    if (ready.epoch != expected_epoch) return error.UnexpectedEpoch;
    if (ready.frame != expected_frame) return error.UnexpectedReadyFrame;
}

pub fn parseSessionCode(code: []const u8) ![]const u8 {
    const trimmed = std.mem.trim(u8, code, " \t\r\n");
    if (!std.mem.startsWith(u8, trimmed, session_code_prefix)) return error.InvalidSessionCode;

    const ticket = trimmed[session_code_prefix.len..];
    if (!isValidTicket(ticket)) return error.InvalidSessionCode;
    return ticket;
}

/// `ticket` comes from the local iroh endpoint.
pub fn makeSessionCode(alloc: std.mem.Allocator, ticket: []const u8) std.mem.Allocator.Error![]u8 {
    std.debug.assert(isValidTicket(ticket));
    return std.fmt.allocPrint(alloc, session_code_prefix ++ "{s}", .{ticket});
}

fn isValidTicket(ticket: []const u8) bool {
    if (ticket.len == 0 or ticket.len > 4096) return false;
    for (ticket) |byte| {
        if (!std.ascii.isPrint(byte)) return false;
    }
    return true;
}

/// Wire framing is a little-endian u32 payload length followed by a tagged payload.
/// Outgoing messages are built locally, so their limits are asserted rather than checked.
pub fn encode(alloc: std.mem.Allocator, message: Message) ![]u8 {
    var framed: std.Io.Writer.Allocating = .init(alloc);
    errdefer framed.deinit();
    // An allocating writer only fails when out of memory.
    writePayload(alloc, &framed.writer, message) catch return error.OutOfMemory;

    const payload_len = framed.written().len - 4;
    std.debug.assert(payload_len <= max_message_size);
    std.mem.writeInt(u32, framed.written()[0..4], @intCast(payload_len), .little);
    return try framed.toOwnedSlice();
}

fn writePayload(alloc: std.mem.Allocator, writer: *std.Io.Writer, message: Message) !void {
    try writer.writeAll(&[_]u8{0} ** 4);
    try writeInt(writer, u8, @intFromEnum(message));
    switch (message) {
        .preview => |value| {
            try writeString(writer, value.name, max_display_name);
            try writeInt(writer, u32, value.rom_size);
            try writer.writeAll(&value.rom_hash);
            try writeCompressedBytes(alloc, writer, value.framebuffer, framebuffer_size);
        },
        .join => {},
        .join_data => |value| {
            try writeString(writer, value.name, max_display_name);
            try writeCompressedBytes(alloc, writer, value.rom, max_rom_size);
            try writeBytes(writer, value.snapshot, max_snapshot_size);
            try writeInt(writer, u8, value.speed);
            try writeInt(writer, u32, value.epoch);
            try writeInt(writer, u64, value.frame);
        },
        .ready, .ack => |value| try writeAck(writer, value),
        .frame => |value| {
            try writeInt(writer, u32, value.epoch);
            try writeInt(writer, u64, value.frame);
            try writeInt(writer, u8, value.player1);
            try writeInt(writer, u8, value.player2);
            try writeDigest(writer, value.digest);
        },
        .control => |value| switch (value) {
            .paused => |paused| {
                try writeInt(writer, u8, 0);
                try writeInt(writer, u8, @intFromBool(paused));
            },
            .speed => |speed| {
                try writeInt(writer, u8, 1);
                try writeInt(writer, u8, speed);
            },
        },
        .rebase => |value| {
            try writeInt(writer, u32, value.epoch);
            try writeInt(writer, u64, value.frame);
            try writeBytes(writer, value.snapshot, max_snapshot_size);
        },
        .disconnect => |reason| try writeString(writer, reason, 1024),
    }
}

/// Decodes one complete framed message and rejects trailing or truncated data.
pub fn decode(alloc: std.mem.Allocator, bytes: []const u8) !Message {
    if (bytes.len < 5) return error.TruncatedMessage;
    const len = std.mem.readInt(u32, bytes[0..4], .little);
    if (len > max_message_size) return error.MessageTooLarge;
    if (bytes.len != @as(usize, len) + 4) return error.InvalidMessageLength;
    return decodePayload(alloc, bytes[4..]);
}

/// Decodes a payload after its framing header has already been consumed and
/// its length checked against `max_message_size`.
pub fn decodePayload(alloc: std.mem.Allocator, payload: []const u8) !Message {
    if (payload.len == 0) return error.TruncatedMessage;

    var reader: std.Io.Reader = .fixed(payload);
    const tag = std.enums.fromInt(std.meta.Tag(Message), try readInt(&reader, u8)) orelse
        return error.UnknownMessageTag;
    var result: Message = switch (tag) {
        .preview => blk: {
            const name = try readString(alloc, &reader, max_display_name);
            errdefer alloc.free(name);
            const rom_size = try readInt(&reader, u32);
            if (rom_size > max_rom_size) return error.RomTooLarge;
            var hash: Digest = undefined;
            try reader.readSliceAll(&hash);
            const framebuffer_bytes = try readCompressedBytes(alloc, &reader, framebuffer_size);
            errdefer alloc.free(framebuffer_bytes);
            if (framebuffer_bytes.len != framebuffer_size) return error.InvalidFramebufferSize;
            break :blk .{ .preview = .{ .name = name, .rom_size = rom_size, .rom_hash = hash, .framebuffer = framebuffer_bytes } };
        },
        .join => .{ .join = {} },
        .join_data => blk: {
            const name = try readString(alloc, &reader, max_display_name);
            errdefer alloc.free(name);
            const rom = try readCompressedBytes(alloc, &reader, max_rom_size);
            errdefer alloc.free(rom);
            const snapshot = try readBytes(alloc, &reader, max_snapshot_size);
            errdefer alloc.free(snapshot);
            break :blk .{ .join_data = .{
                .name = name,
                .rom = rom,
                .snapshot = snapshot,
                .speed = try readInt(&reader, u8),
                .epoch = try readInt(&reader, u32),
                .frame = try readInt(&reader, u64),
            } };
        },
        .ready => .{ .ready = try readAck(&reader) },
        .frame => .{ .frame = .{
            .epoch = try readInt(&reader, u32),
            .frame = try readInt(&reader, u64),
            .player1 = try readInt(&reader, u8),
            .player2 = try readInt(&reader, u8),
            .digest = try readDigest(&reader),
        } },
        .ack => .{ .ack = try readAck(&reader) },
        .control => .{ .control = switch (try readInt(&reader, u8)) {
            0 => .{ .paused = switch (try readInt(&reader, u8)) {
                0 => false,
                1 => true,
                else => return error.InvalidBoolean,
            } },
            1 => .{ .speed = try readInt(&reader, u8) },
            else => return error.InvalidControlTag,
        } },
        .rebase => .{ .rebase = .{
            .epoch = try readInt(&reader, u32),
            .frame = try readInt(&reader, u64),
            .snapshot = try readBytes(alloc, &reader, max_snapshot_size),
        } },
        .disconnect => .{ .disconnect = try readString(alloc, &reader, 1024) },
    };
    errdefer result.deinit(alloc);
    if (reader.seek != reader.end) return error.TrailingMessageData;
    return result;
}

pub fn validateNext(expected_epoch: u32, expected_frame: u64, actual_epoch: u32, actual_frame: u64) !void {
    if (actual_epoch != expected_epoch) return error.UnexpectedEpoch;
    if (actual_frame != expected_frame) return error.OutOfOrderFrame;
}

/// Acknowledgements from an older epoch can still be in flight after a rebase.
/// They are harmless and must be discarded rather than terminating the session.
pub fn validateAcknowledgement(current_epoch: u32, current_frame: u64, ack_epoch: u32, ack_frame: u64) !AcknowledgementDisposition {
    if (ack_epoch < current_epoch) return .stale;
    if (ack_epoch > current_epoch or ack_frame > current_frame) return error.InvalidAcknowledgement;
    return .current;
}

fn writeAck(writer: *std.Io.Writer, value: Ack) !void {
    try writeInt(writer, u32, value.epoch);
    try writeInt(writer, u64, value.frame);
    try writeInt(writer, u8, value.player2);
    try writeDigest(writer, value.digest);
}

fn readAck(reader: *std.Io.Reader) !Ack {
    return .{
        .epoch = try readInt(reader, u32),
        .frame = try readInt(reader, u64),
        .player2 = try readInt(reader, u8),
        .digest = try readDigest(reader),
    };
}

fn writeDigest(writer: *std.Io.Writer, digest: ?Digest) !void {
    try writeInt(writer, u8, @intFromBool(digest != null));
    if (digest) |value| try writer.writeAll(&value);
}

fn readDigest(reader: *std.Io.Reader) !?Digest {
    return switch (try readInt(reader, u8)) {
        0 => null,
        1 => blk: {
            var value: Digest = undefined;
            try reader.readSliceAll(&value);
            break :blk value;
        },
        else => error.InvalidBoolean,
    };
}

fn writeString(writer: *std.Io.Writer, value: []const u8, max: usize) !void {
    std.debug.assert(std.unicode.utf8ValidateSlice(value));
    try writeBytes(writer, value, max);
}

fn readString(alloc: std.mem.Allocator, reader: *std.Io.Reader, max: usize) ![]u8 {
    const result = try readBytes(alloc, reader, max);
    errdefer alloc.free(result);
    if (!std.unicode.utf8ValidateSlice(result)) return error.InvalidUtf8;
    return result;
}

fn writeBytes(writer: *std.Io.Writer, value: []const u8, max: usize) !void {
    std.debug.assert(value.len <= max);
    try writeInt(writer, u32, @intCast(value.len));
    try writer.writeAll(value);
}

fn readBytes(alloc: std.mem.Allocator, reader: *std.Io.Reader, max: usize) ![]u8 {
    const len = try readInt(reader, u32);
    if (len > max) return error.PayloadTooLarge;
    const result = try alloc.alloc(u8, len);
    errdefer alloc.free(result);
    try reader.readSliceAll(result);
    return result;
}

fn writeCompressedBytes(alloc: std.mem.Allocator, writer: *std.Io.Writer, value: []const u8, size_limit: usize) !void {
    std.debug.assert(value.len <= size_limit);

    const compressed = try compress.compressBytes(alloc, value, .{ .level = .fast });
    defer alloc.free(compressed);

    try writeInt(writer, u32, @intCast(value.len));
    try writeInt(writer, u32, @intCast(compressed.len));
    try writer.writeAll(compressed);
}

fn readCompressedBytes(alloc: std.mem.Allocator, reader: *std.Io.Reader, size_limit: usize) ![]u8 {
    const uncompressed_len = try readInt(reader, u32);
    if (uncompressed_len > size_limit) return error.PayloadTooLarge;

    const compressed_len = try readInt(reader, u32);
    const compressed_bytes = reader.take(compressed_len) catch return error.TruncatedMessage;

    return compress.decompressBytes(alloc, compressed_bytes, uncompressed_len) catch |err| switch (err) {
        error.OutOfMemory => return error.OutOfMemory,
        error.InvalidCompressedData => return error.InvalidCompressedPayload,
    };
}

fn writeInt(writer: *std.Io.Writer, comptime T: type, value: T) !void {
    var buffer: [@sizeOf(T)]u8 = undefined;
    std.mem.writeInt(T, &buffer, value, .little);
    try writer.writeAll(&buffer);
}

fn readInt(reader: *std.Io.Reader, comptime T: type) !T {
    var buffer: [@sizeOf(T)]u8 = undefined;
    reader.readSliceAll(&buffer) catch return error.TruncatedMessage;
    return std.mem.readInt(T, &buffer, .little);
}

test "session codes are prefixed and trimmed" {
    const alloc = std.testing.allocator;
    try std.testing.expectEqualStrings("ticket", try parseSessionCode("  neskwik:ticket\n"));
    try std.testing.expectError(error.InvalidSessionCode, parseSessionCode("ticket"));
    try std.testing.expectError(error.InvalidSessionCode, parseSessionCode("neskwik:"));
    try std.testing.expectError(error.InvalidSessionCode, parseSessionCode("neskwik:bad\nvalue"));
    const code = try makeSessionCode(alloc, "ticket");
    defer alloc.free(code);
    try std.testing.expectEqualStrings("neskwik:ticket", code);
}

test "every protocol message round trips" {
    const alloc = std.testing.allocator;
    const framebuffer: [framebuffer_size]u8 = [_]u8{0x5a} ** framebuffer_size;
    const hash: Digest = [_]u8{0xa5} ** 32;
    const messages = [_]Message{
        .{ .preview = .{ .name = "game.nes", .rom_size = 123, .rom_hash = hash, .framebuffer = &framebuffer } },
        .{ .join = {} },
        .{ .join_data = .{ .name = "game.nes", .rom = "rom", .snapshot = "state", .speed = 100, .epoch = 2, .frame = 9 } },
        .{ .ready = .{ .epoch = 2, .frame = 9, .player2 = 3 } },
        .{ .frame = .{ .epoch = 2, .frame = 10, .player1 = 1, .player2 = 2, .digest = hash } },
        .{ .ack = .{ .epoch = 2, .frame = 10, .player2 = 4, .digest = hash } },
        .{ .control = .{ .paused = true } },
        .{ .control = .{ .speed = 200 } },
        .{ .rebase = .{ .epoch = 3, .frame = 0, .snapshot = "state" } },
        .{ .disconnect = "bye" },
    };
    for (messages) |message| {
        const encoded = try encode(alloc, message);
        defer alloc.free(encoded);
        var decoded = try decode(alloc, encoded);
        decoded.deinit(alloc);
    }
}

test "preview framebuffer and ROM are compressed on the wire" {
    const alloc = std.testing.allocator;
    const hash: Digest = [_]u8{0xa5} ** 32;
    const framebuffer: [framebuffer_size]u8 = [_]u8{0x5a} ** framebuffer_size;

    const encoded_preview = try encode(alloc, .{ .preview = .{
        .name = "game.nes",
        .rom_size = 64 * 1024,
        .rom_hash = hash,
        .framebuffer = &framebuffer,
    } });
    defer alloc.free(encoded_preview);
    try std.testing.expect(encoded_preview.len < framebuffer_size / 8);
    var decoded_preview = try decode(alloc, encoded_preview);
    defer decoded_preview.deinit(alloc);
    try std.testing.expectEqualSlices(u8, &framebuffer, decoded_preview.preview.framebuffer);

    const rom = try alloc.alloc(u8, 64 * 1024);
    defer alloc.free(rom);
    @memset(rom, 0x3c);
    const encoded_join = try encode(alloc, .{ .join_data = .{
        .name = "game.nes",
        .rom = rom,
        .snapshot = "state",
        .speed = 1,
        .epoch = 2,
        .frame = 9,
    } });
    defer alloc.free(encoded_join);
    try std.testing.expect(encoded_join.len < rom.len / 8);
    var decoded_join = try decode(alloc, encoded_join);
    defer decoded_join.deinit(alloc);
    try std.testing.expectEqualSlices(u8, rom, decoded_join.join_data.rom);
}

test "maximum ROM and snapshot fit in one join message" {
    const alloc = std.testing.allocator;
    const rom = try alloc.alloc(u8, @divTrunc(max_rom_size, 2));
    defer alloc.free(rom);
    var random_state: u32 = 0x12345678;
    for (rom) |*byte| {
        random_state = random_state *% 1664525 +% 1013904223;
        byte.* = @truncate(random_state >> 24);
    }
    const snapshot = try alloc.alloc(u8, max_snapshot_size);
    defer alloc.free(snapshot);
    @memset(snapshot, 0);

    const encoded = try encode(alloc, .{ .join_data = .{
        .name = "maximum.nes",
        .rom = rom,
        .snapshot = snapshot,
        .speed = 1,
        .epoch = 1,
        .frame = 0,
    } });
    defer alloc.free(encoded);
    try std.testing.expect(encoded.len - 4 <= max_message_size);
}

test "compressed payloads reject length mismatches and oversized output" {
    const alloc = std.testing.allocator;
    const input = [_]u8{0x42} ** 4096;
    var encoded: std.Io.Writer.Allocating = .init(alloc);
    defer encoded.deinit();
    try writeCompressedBytes(alloc, &encoded.writer, &input, input.len);

    // The payload must expand to exactly the declared length.
    const bytes = encoded.written();
    std.mem.writeInt(u32, bytes[0..4], input.len - 1, .little);
    var mismatched_reader: std.Io.Reader = .fixed(bytes);
    try std.testing.expectError(
        error.InvalidCompressedPayload,
        readCompressedBytes(alloc, &mismatched_reader, input.len),
    );

    var oversized_header: [8]u8 = undefined;
    std.mem.writeInt(u32, oversized_header[0..4], 4097, .little);
    std.mem.writeInt(u32, oversized_header[4..8], 0, .little);
    var oversized_reader: std.Io.Reader = .fixed(&oversized_header);
    try std.testing.expectError(
        error.PayloadTooLarge,
        readCompressedBytes(alloc, &oversized_reader, input.len),
    );

    var impossible_compressed_len: [8]u8 = undefined;
    std.mem.writeInt(u32, impossible_compressed_len[0..4], 1, .little);
    std.mem.writeInt(u32, impossible_compressed_len[4..8], std.math.maxInt(u32), .little);
    var impossible_reader: std.Io.Reader = .fixed(&impossible_compressed_len);
    try std.testing.expectError(
        error.TruncatedMessage,
        readCompressedBytes(alloc, &impossible_reader, input.len),
    );
}

test "protocol rejects malformed framing and ordering" {
    const alloc = std.testing.allocator;
    const encoded = try encode(alloc, .{ .join = {} });
    defer alloc.free(encoded);
    try std.testing.expectError(error.TruncatedMessage, decode(alloc, encoded[0 .. encoded.len - 1]));
    var unknown = [_]u8{ 1, 0, 0, 0, 0xff };
    try std.testing.expectError(error.UnknownMessageTag, decode(alloc, &unknown));
    try std.testing.expectError(error.OutOfOrderFrame, validateNext(1, 4, 1, 5));
    try std.testing.expectError(error.UnexpectedEpoch, validateNext(1, 4, 2, 4));

    var oversized = [_]u8{0} ** 5;
    std.mem.writeInt(u32, oversized[0..4], max_message_size + 1, .little);
    try std.testing.expectError(error.MessageTooLarge, decode(alloc, &oversized));
}

test "acknowledgements discard stale epochs and reject future positions" {
    try std.testing.expectEqual(
        AcknowledgementDisposition.current,
        try validateAcknowledgement(2, 10, 2, 9),
    );
    try std.testing.expectEqual(
        AcknowledgementDisposition.stale,
        try validateAcknowledgement(2, 0, 1, 71),
    );
    try std.testing.expectError(error.InvalidAcknowledgement, validateAcknowledgement(2, 10, 3, 0));
    try std.testing.expectError(error.InvalidAcknowledgement, validateAcknowledgement(2, 10, 2, 11));
}

test "ready must match the active synchronization point and cannot repeat" {
    const ready = Ack{ .epoch = 4, .frame = 12, .player2 = 3 };
    try validateReady(4, 12, true, ready);
    try std.testing.expectError(error.DuplicateReady, validateReady(4, 12, false, ready));
    try std.testing.expectError(error.UnexpectedEpoch, validateReady(3, 12, true, ready));
    try std.testing.expectError(error.UnexpectedReadyFrame, validateReady(4, 11, true, ready));
    try std.testing.expectError(
        error.UnexpectedReadyFrame,
        validateReady(4, 12, true, .{ .epoch = 4, .frame = std.math.maxInt(u64), .player2 = 3 }),
    );
}
