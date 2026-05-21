const std = @import("std");
const zeit = @import("zeit");

const APU = @import("apu/apu.zig").APU;
const Bus = @import("bus.zig").Bus;
const CPU = @import("cpu.zig").CPU;
const Mapper = @import("mappers/mapper.zig").Mapper;
const PPU = @import("ppu.zig").PPU;
const System = @import("system.zig").System;
const paths = @import("paths.zig");
const compress = @import("utils/compress.zig");
const Mirroring = @import("rom.zig").Mirroring;
const Controllers = @import("controller.zig").Controllers;

pub const SLOT_COUNT = 10;

const MAGIC: [4]u8 = "NSST".*;
const VERSION: u32 = 1;
const STATE_DIR_NAME = "save-states";
const EXT = ".nsst";

pub const SlotInfo = struct {
    exists: bool = false,
    saved_at: i64 = 0,
    display_time: [19]u8 = [_]u8{' '} ** 19,
};

const Snapshot = struct {
    saved_at: i64,
    cpu: CPU.Snapshot,
    bus: Bus.Snapshot,
    ppu: PPU.Snapshot,
    apu: APU.Snapshot,

    fn deinit(self: *@This(), alloc: std.mem.Allocator) void {
        self.bus.deinit(alloc);
    }
};

pub fn saveSlot(alloc: std.mem.Allocator, rom_path: []const u8, system: *System, slot: usize) !void {
    if (slot >= SLOT_COUNT) return error.InvalidSaveStateSlot;

    const dir = try romDir(alloc, rom_path);
    defer alloc.free(dir);
    try makeDirAll(dir);

    const path = try slotPathInDir(alloc, dir, slot);
    defer alloc.free(path);

    var snapshot = Snapshot{
        .saved_at = (try zeit.instant(.{})).unixTimestamp(),
        .cpu = system.cpu.saveState(),
        .bus = try system.bus.saveState(alloc),
        .ppu = system.ppu.saveState(),
        .apu = system.apu.saveState(),
    };
    defer snapshot.deinit(alloc);

    const file = try std.fs.createFileAbsolute(path, .{ .truncate = true });
    defer file.close();
    try writeSnapshot(alloc, file, snapshot);
}

pub fn loadSlot(alloc: std.mem.Allocator, rom_path: []const u8, system: *System, slot: usize) !void {
    if (slot >= SLOT_COUNT) return error.InvalidSaveStateSlot;

    const path = try slotPath(alloc, rom_path, slot);
    defer alloc.free(path);

    const file = try std.fs.openFileAbsolute(path, .{});
    defer file.close();

    var snapshot = try readSnapshot(alloc, file);
    defer snapshot.deinit(alloc);

    try system.bus.loadState(snapshot.bus);
    system.ppu.loadState(snapshot.ppu);
    system.apu.loadState(snapshot.apu);
    system.cpu.loadState(snapshot.cpu);
}

pub fn slotInfo(alloc: std.mem.Allocator, rom_path: []const u8, slot: usize) SlotInfo {
    if (slot >= SLOT_COUNT) return .{};
    const path = slotPath(alloc, rom_path, slot) catch return .{};
    defer alloc.free(path);

    const file = std.fs.openFileAbsolute(path, .{}) catch return .{};
    defer file.close();

    var info: SlotInfo = .{ .exists = true };
    info.saved_at = readHeaderTimestamp(file) catch return .{};
    info.display_time = formatTimestamp(alloc, info.saved_at);
    return info;
}

fn writeSnapshot(alloc: std.mem.Allocator, file: std.fs.File, snapshot: Snapshot) !void {
    try file.writeAll(&MAGIC);
    try writeInt(file, u32, VERSION);
    try writeInt(file, i64, snapshot.saved_at);

    var body = ByteWriter.init(alloc);
    defer body.deinit();

    try writeStruct(&body, snapshot.cpu);
    try writeBusSnapshot(&body, snapshot.bus);
    try writeStruct(&body, snapshot.ppu);
    try writeStruct(&body, snapshot.apu);

    const compressed = try compressBytes(alloc, body.bytes());
    defer alloc.free(compressed);

    try writeInt(file, u32, @intCast(body.bytes().len));
    try writeInt(file, u32, @intCast(compressed.len));
    try file.writeAll(compressed);
}

fn readSnapshot(alloc: std.mem.Allocator, file: std.fs.File) !Snapshot {
    const header = try readHeader(file);
    const body = try readSnapshotBody(alloc, file, header.version);
    defer alloc.free(body);

    var reader = ByteReader.init(body);

    const cpu = try readStruct(CPU.Snapshot, &reader);
    var bus = try readBusSnapshot(alloc, &reader);
    errdefer bus.deinit(alloc);
    const ppu = try readStruct(PPU.Snapshot, &reader);
    const apu = try readStruct(APU.Snapshot, &reader);
    return .{
        .saved_at = header.saved_at,
        .cpu = cpu,
        .bus = bus,
        .ppu = ppu,
        .apu = apu,
    };
}

const Header = struct {
    version: u32,
    saved_at: i64,
};

fn readHeader(file: std.fs.File) !Header {
    var magic: [4]u8 = undefined;
    try readNoEof(file, &magic);
    if (!std.mem.eql(u8, &magic, &MAGIC)) return error.InvalidSaveState;

    const version = try readInt(file, u32);
    if (version != VERSION) return error.UnsupportedSaveStateVersion;

    return .{
        .version = version,
        .saved_at = try readInt(file, i64),
    };
}

fn readHeaderTimestamp(file: std.fs.File) !i64 {
    return (try readHeader(file)).saved_at;
}

fn readSnapshotBody(alloc: std.mem.Allocator, file: std.fs.File, version: u32) ![]u8 {
    if (version != VERSION) return error.UnsupportedSaveStateVersion;

    const uncompressed_len = try readInt(file, u32);
    const compressed_len = try readInt(file, u32);
    const compressed = try alloc.alloc(u8, compressed_len);
    defer alloc.free(compressed);
    try readNoEof(file, compressed);

    return try decompressBytes(alloc, compressed, uncompressed_len);
}

fn writeBusSnapshot(writer: anytype, snapshot: Bus.Snapshot) !void {
    try writer.writeAll(&snapshot.ram);
    try writeInt(writer, u64, snapshot.cycles);
    try writeInt(writer, u8, snapshot.open_bus);
    try writeInt(writer, u8, snapshot.dma_start_delay);
    try writeInt(writer, u16, snapshot.dma_cycles);
    try writeStruct(writer, snapshot.controllers);
    try writeMapperSnapshot(writer, snapshot.rom.mapper);
}

fn readBusSnapshot(alloc: std.mem.Allocator, reader: anytype) !Bus.Snapshot {
    var ram: [2048]u8 = undefined;
    try readNoEof(reader, &ram);
    return .{
        .ram = ram,
        .cycles = try readInt(reader, u64),
        .open_bus = try readInt(reader, u8),
        .dma_start_delay = try readInt(reader, u8),
        .dma_cycles = try readInt(reader, u16),
        .controllers = try readStruct(Controllers, reader),
        .rom = .{ .mapper = try readMapperSnapshot(alloc, reader) },
    };
}

fn writeMapperSnapshot(writer: anytype, snapshot: Mapper.Snapshot) !void {
    switch (snapshot) {
        .mapper0 => |value| {
            try writeInt(writer, u8, 0);
            try writeBytes(writer, value.prg_ram);
            try writeBytes(writer, value.chr_ram);
        },
        .mapper1 => |value| {
            try writeInt(writer, u8, 1);
            try writeStruct(writer, value.load_register);
            try writeStruct(writer, value.write_index);
            try writeStruct(writer, value.control);
            try writeStruct(writer, value.prg_bank);
            try writeStruct(writer, value.chr_bank_1);
            try writeStruct(writer, value.chr_bank_2);
            try writeBytes(writer, value.prg_ram);
            try writeBytes(writer, value.chr_ram);
        },
        .mapper2 => |value| {
            try writeInt(writer, u8, 2);
            try writeStruct(writer, value.selected_bank);
            try writeBytes(writer, value.chr_ram);
        },
        .mapper3 => |value| {
            try writeInt(writer, u8, 3);
            try writeStruct(writer, value.selected_chr_bank);
            try writeBytes(writer, value.prg_ram);
        },
        .mapper4 => |value| {
            try writeInt(writer, u8, 4);
            try writeStruct(writer, value.bank_registers);
            try writeStruct(writer, value.bank_select);
            try writeStruct(writer, value.prg_inversion);
            try writeStruct(writer, value.chr_inversion);
            try writeStruct(writer, value.irq_flag);
            try writeStruct(writer, value.irq_counter);
            try writeStruct(writer, value.irq_reload_flag);
            try writeStruct(writer, value.irq_counter_reload);
            try writeStruct(writer, value.irq_enabled);
            try writeStruct(writer, value.ppu_a12);
            try writeStruct(writer, value.ppu_a12_low_cycle);
            try writeStruct(writer, value.mirroring_mode);
            try writeBytes(writer, value.prg_ram);
            try writeBytes(writer, value.chr_ram);
        },
    }
}

fn readMapperSnapshot(alloc: std.mem.Allocator, reader: anytype) !Mapper.Snapshot {
    return switch (try readInt(reader, u8)) {
        0 => blk: {
            const prg_ram = try readBytes(alloc, reader);
            errdefer alloc.free(prg_ram);
            break :blk .{ .mapper0 = .{
                .prg_ram = prg_ram,
                .chr_ram = try readBytes(alloc, reader),
            } };
        },
        1 => blk: {
            const load_register = try readStruct(u8, reader);
            const write_index = try readStruct(u8, reader);
            const control = try readStruct(u8, reader);
            const prg_bank = try readStruct(u8, reader);
            const chr_bank_1 = try readStruct(u8, reader);
            const chr_bank_2 = try readStruct(u8, reader);
            const prg_ram = try readBytes(alloc, reader);
            errdefer alloc.free(prg_ram);
            break :blk .{ .mapper1 = .{
                .load_register = load_register,
                .write_index = write_index,
                .control = control,
                .prg_bank = prg_bank,
                .chr_bank_1 = chr_bank_1,
                .chr_bank_2 = chr_bank_2,
                .prg_ram = prg_ram,
                .chr_ram = try readBytes(alloc, reader),
            } };
        },
        2 => .{ .mapper2 = .{
            .selected_bank = try readStruct(u8, reader),
            .chr_ram = try readBytes(alloc, reader),
        } },
        3 => .{ .mapper3 = .{
            .selected_chr_bank = try readStruct(u8, reader),
            .prg_ram = try readBytes(alloc, reader),
        } },
        4 => blk: {
            const bank_registers = try readStruct([10]usize, reader);
            const bank_select = try readStruct(u8, reader);
            const prg_inversion = try readStruct(bool, reader);
            const chr_inversion = try readStruct(bool, reader);
            const irq_flag = try readStruct(bool, reader);
            const irq_counter = try readStruct(u8, reader);
            const irq_reload_flag = try readStruct(bool, reader);
            const irq_counter_reload = try readStruct(u8, reader);
            const irq_enabled = try readStruct(bool, reader);
            const ppu_a12 = try readStruct(bool, reader);
            const ppu_a12_low_cycle = try readStruct(u64, reader);
            const mirroring_mode = try readStruct(Mirroring, reader);
            const prg_ram = try readBytes(alloc, reader);
            errdefer alloc.free(prg_ram);
            break :blk .{ .mapper4 = .{
                .bank_registers = bank_registers,
                .bank_select = bank_select,
                .prg_inversion = prg_inversion,
                .chr_inversion = chr_inversion,
                .irq_flag = irq_flag,
                .irq_counter = irq_counter,
                .irq_reload_flag = irq_reload_flag,
                .irq_counter_reload = irq_counter_reload,
                .irq_enabled = irq_enabled,
                .ppu_a12 = ppu_a12,
                .ppu_a12_low_cycle = ppu_a12_low_cycle,
                .mirroring_mode = mirroring_mode,
                .prg_ram = prg_ram,
                .chr_ram = try readBytes(alloc, reader),
            } };
        },
        else => error.UnsupportedMapperSnapshot,
    };
}

fn writeStruct(writer: anytype, value: anytype) !void {
    try writer.writeAll(std.mem.asBytes(&value));
}

fn readStruct(comptime T: type, reader: anytype) !T {
    var value: T = undefined;
    try readNoEof(reader, std.mem.asBytes(&value));
    return value;
}

fn writeBytes(writer: anytype, bytes: []const u8) !void {
    try writeInt(writer, u32, @intCast(bytes.len));
    try writer.writeAll(bytes);
}

fn readBytes(alloc: std.mem.Allocator, reader: anytype) ![]u8 {
    const len = try readInt(reader, u32);
    const bytes = try alloc.alloc(u8, len);
    errdefer alloc.free(bytes);
    try readNoEof(reader, bytes);
    return bytes;
}

fn writeInt(writer: anytype, comptime T: type, value: T) !void {
    var buf: [@sizeOf(T)]u8 = undefined;
    std.mem.writeInt(T, &buf, value, .little);
    try writer.writeAll(&buf);
}

fn readInt(reader: anytype, comptime T: type) !T {
    var buf: [@sizeOf(T)]u8 = undefined;
    try readNoEof(reader, &buf);
    return std.mem.readInt(T, &buf, .little);
}

fn readNoEof(reader: anytype, buffer: []u8) !void {
    if (try reader.readAll(buffer) != buffer.len) return error.UnexpectedEof;
}

const ByteWriter = struct {
    alloc: std.mem.Allocator,
    list: std.ArrayList(u8) = .empty,

    fn init(alloc: std.mem.Allocator) @This() {
        return .{ .alloc = alloc };
    }

    fn deinit(self: *@This()) void {
        self.list.deinit(self.alloc);
    }

    fn writeAll(self: *@This(), data: []const u8) !void {
        try self.list.appendSlice(self.alloc, data);
    }

    fn bytes(self: *const @This()) []const u8 {
        return self.list.items;
    }
};

const ByteReader = struct {
    bytes: []const u8,
    pos: usize = 0,

    fn init(bytes: []const u8) @This() {
        return .{ .bytes = bytes };
    }

    fn readAll(self: *@This(), out: []u8) !usize {
        const remaining = self.bytes.len - self.pos;
        const n = @min(out.len, remaining);
        @memcpy(out[0..n], self.bytes[self.pos..][0..n]);
        self.pos += n;
        return n;
    }
};

fn compressBytes(alloc: std.mem.Allocator, bytes: []const u8) ![]u8 {
    var reader: std.Io.Reader = .fixed(bytes);
    var writer: std.Io.Writer.Allocating = .init(alloc);
    errdefer writer.deinit();

    try compress.compress(&reader, &writer.writer, .{});
    return try writer.toOwnedSlice();
}

fn decompressBytes(alloc: std.mem.Allocator, compressed: []const u8, expected_len: usize) ![]u8 {
    var reader: std.Io.Reader = .fixed(compressed);
    var writer: std.Io.Writer.Allocating = .init(alloc);
    errdefer writer.deinit();

    var decompressor: std.compress.flate.Decompress = .init(&reader, .gzip, &.{});
    _ = decompressor.reader.streamRemaining(&writer.writer) catch |err| switch (err) {
        error.ReadFailed, error.WriteFailed => return error.InvalidCompressedSaveState,
    };
    if (writer.written().len != expected_len) return error.InvalidCompressedSaveState;

    return try writer.toOwnedSlice();
}

fn slotPath(alloc: std.mem.Allocator, rom_path: []const u8, slot: usize) ![]u8 {
    const dir = try romDir(alloc, rom_path);
    defer alloc.free(dir);
    return slotPathInDir(alloc, dir, slot);
}

fn slotPathInDir(alloc: std.mem.Allocator, dir: []const u8, slot: usize) ![]u8 {
    return std.fmt.allocPrint(
        alloc,
        "{s}" ++ std.fs.path.sep_str ++ "slot-{d}" ++ EXT,
        .{ dir, slot + 1 },
    );
}

fn romDir(alloc: std.mem.Allocator, rom_path: []const u8) ![]u8 {
    const data_dir = try paths.getDataDir(alloc);
    defer alloc.free(data_dir);

    return std.fmt.allocPrint(
        alloc,
        "{s}" ++ std.fs.path.sep_str ++ "{s}" ++ std.fs.path.sep_str ++ "{s}",
        .{ data_dir, STATE_DIR_NAME, std.fs.path.stem(rom_path) },
    );
}

fn makeDirAll(path: []const u8) !void {
    std.fs.makeDirAbsolute(path) catch |err| switch (err) {
        error.PathAlreadyExists => return,
        error.FileNotFound => {
            const parent = std.fs.path.dirname(path) orelse return error.FileNotFound;
            try makeDirAll(parent);
            try std.fs.makeDirAbsolute(path);
        },
        else => return err,
    };
}

fn formatTimestamp(alloc: std.mem.Allocator, timestamp: i64) [19]u8 {
    if (timestamp <= 0) return [_]u8{' '} ** 19;

    var timezone = zeit.local(alloc, null) catch zeit.utc;
    defer timezone.deinit();

    const instant = zeit.instant(.{
        .source = .{ .unix_timestamp = timestamp },
        .timezone = &timezone,
    }) catch return [_]u8{' '} ** 19;

    return formatDateTime(instant.time());
}

fn formatDateTime(time: zeit.Time) [19]u8 {
    var buf: [19]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buf);
    time.strftime(&writer, "%Y/%m/%d %H:%M:%S") catch unreachable;
    return buf;
}
