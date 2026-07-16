const std = @import("std");
const zeit = @import("zeit");

const utils = @import("utils/misc.zig");
const APU = @import("apu/apu.zig").APU;
const Bus = @import("bus.zig").Bus;
const CPU = @import("cpu.zig").CPU;
const Mapper = @import("mappers/mapper.zig").Mapper;
const PPU = @import("ppu.zig").PPU;
const System = @import("system.zig").System;
const paths = @import("utils/paths.zig");
const compress = @import("utils/compress.zig");
const Mirroring = @import("rom.zig").Mirroring;
const Controllers = @import("controller.zig").Controllers;

pub const SLOT_COUNT = 10;
pub const MAX_NETWORK_SNAPSHOT_SIZE: usize = 16 * 1024 * 1024;

const MAGIC: [4]u8 = "NSST".*;
const VERSION: u32 = 1;
const STATE_DIR_NAME = "save-states";
const EXT = "nsst";

pub const SlotInfo = struct {
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

/// An owned, complete emulation snapshot used by netplay. This is intentionally
/// separate from the on-disk slot container so the existing slot format stays
/// byte-for-byte compatible.
pub const NetworkSnapshot = struct {
    cpu: CPU.Snapshot,
    bus: Bus.Snapshot,
    ppu: PPU.Snapshot,
    apu: APU.Snapshot,

    pub fn deinit(self: *@This(), alloc: std.mem.Allocator) void {
        self.bus.deinit(alloc);
        self.* = undefined;
    }
};

pub const ComponentDigests = struct {
    cpu: [32]u8,
    bus: [32]u8,
    ppu: [32]u8,
    apu: [32]u8,
};

pub fn capture(alloc: std.mem.Allocator, system: *const System) !NetworkSnapshot {
    return .{
        .cpu = system.cpu.saveState(),
        .bus = try system.bus.saveState(alloc),
        .ppu = system.ppu.saveState(),
        .apu = system.apu.saveState(),
    };
}

pub fn apply(system: *System, snapshot: *const NetworkSnapshot) !void {
    try system.bus.loadState(&snapshot.bus);
    system.ppu.loadState(&snapshot.ppu);
    system.apu.loadState(&snapshot.apu);
    system.cpu.loadState(snapshot.cpu);
}

/// Encodes field-by-field in a fixed little-endian representation, then gzip
/// compresses it. No struct padding, pointers, or native enum layout enter the
/// network representation.
pub fn encodeNetwork(alloc: std.mem.Allocator, snapshot: *const NetworkSnapshot) ![]u8 {
    var body: std.Io.Writer.Allocating = .init(alloc);
    defer body.deinit();
    try writeCanonical(&body.writer, CPU.Snapshot, snapshot.cpu);
    try writeCanonicalBus(&body.writer, snapshot.bus);
    try writeCanonical(&body.writer, PPU.Snapshot, snapshot.ppu);
    try writeCanonical(&body.writer, APU.Snapshot, snapshot.apu);
    if (body.written().len > MAX_NETWORK_SNAPSHOT_SIZE) return error.SnapshotTooLarge;

    const compressed = try compressBytes(alloc, body.written());
    defer alloc.free(compressed);
    if (compressed.len > MAX_NETWORK_SNAPSHOT_SIZE) return error.SnapshotTooLarge;

    const header_len = 4 + @sizeOf(u32) * 2;
    const result = try alloc.alloc(u8, header_len + compressed.len);
    @memcpy(result[0..4], "NSNW");
    std.mem.writeInt(u32, result[4..8], @intCast(body.written().len), .little);
    std.mem.writeInt(u32, result[8..12], @intCast(compressed.len), .little);
    @memcpy(result[12..], compressed);
    return result;
}

pub fn decodeNetwork(alloc: std.mem.Allocator, encoded: []const u8) !NetworkSnapshot {
    if (encoded.len < 12 or !std.mem.eql(u8, encoded[0..4], "NSNW")) return error.InvalidNetworkSnapshot;
    const uncompressed_len = std.mem.readInt(u32, encoded[4..8], .little);
    const compressed_len = std.mem.readInt(u32, encoded[8..12], .little);
    if (uncompressed_len > MAX_NETWORK_SNAPSHOT_SIZE or compressed_len > MAX_NETWORK_SNAPSHOT_SIZE)
        return error.SnapshotTooLarge;
    if (encoded.len != 12 + @as(usize, compressed_len)) return error.InvalidNetworkSnapshot;

    const body = try decompressBytes(alloc, encoded[12..], uncompressed_len);
    defer alloc.free(body);
    var reader: std.Io.Reader = .fixed(body);

    var snapshot: NetworkSnapshot = undefined;
    snapshot.cpu = try readCanonical(CPU.Snapshot, &reader);
    snapshot.bus = try readCanonicalBus(alloc, &reader);
    errdefer snapshot.bus.deinit(alloc);
    snapshot.ppu = try readCanonical(PPU.Snapshot, &reader);
    snapshot.apu = try readCanonical(APU.Snapshot, &reader);
    if (reader.seek != reader.end) return error.InvalidNetworkSnapshot;
    return snapshot;
}

pub fn digest(snapshot: *const NetworkSnapshot) ![32]u8 {
    var buffer: std.Io.Writer.Allocating = .init(std.heap.page_allocator);
    defer buffer.deinit();
    try writeCanonical(&buffer.writer, CPU.Snapshot, snapshot.cpu);
    try writeCanonicalBus(&buffer.writer, snapshot.bus);
    try writeCanonical(&buffer.writer, PPU.Snapshot, snapshot.ppu);
    try writeCanonicalApuDigest(&buffer.writer, snapshot.apu);
    if (buffer.written().len > MAX_NETWORK_SNAPSHOT_SIZE) return error.SnapshotTooLarge;
    var result: [32]u8 = undefined;
    std.crypto.hash.Blake3.hash(buffer.written(), &result, .{});
    return result;
}

/// Diagnostic hashes using the same canonical encoders as the full netplay
/// digest. They are logged only at checkpoints to identify a divergent core
/// subsystem without transferring additional protocol data.
pub fn componentDigests(snapshot: *const NetworkSnapshot) !ComponentDigests {
    return .{
        .cpu = try digestCanonical(CPU.Snapshot, snapshot.cpu),
        .bus = try digestCanonicalBus(snapshot.bus),
        .ppu = try digestCanonical(PPU.Snapshot, snapshot.ppu),
        .apu = try digestCanonicalApu(snapshot.apu),
    };
}

fn digestCanonical(comptime T: type, value: T) ![32]u8 {
    var buffer: std.Io.Writer.Allocating = .init(std.heap.page_allocator);
    defer buffer.deinit();
    try writeCanonical(&buffer.writer, T, value);
    var result: [32]u8 = undefined;
    std.crypto.hash.Blake3.hash(buffer.written(), &result, .{});
    return result;
}

fn digestCanonicalBus(value: Bus.Snapshot) ![32]u8 {
    var buffer: std.Io.Writer.Allocating = .init(std.heap.page_allocator);
    defer buffer.deinit();
    try writeCanonicalBus(&buffer.writer, value);
    var result: [32]u8 = undefined;
    std.crypto.hash.Blake3.hash(buffer.written(), &result, .{});
    return result;
}

fn digestCanonicalApu(value: APU.Snapshot) ![32]u8 {
    var buffer: std.Io.Writer.Allocating = .init(std.heap.page_allocator);
    defer buffer.deinit();
    try writeCanonicalApuDigest(&buffer.writer, value);
    var result: [32]u8 = undefined;
    std.crypto.hash.Blake3.hash(buffer.written(), &result, .{});
    return result;
}

/// Hashes NES-visible APU state while excluding audio-renderer bookkeeping.
/// These fields depend on the local blip/output-buffer phase and do not affect
/// CPU-visible APU behavior.
fn writeCanonicalApuDigest(writer: *std.Io.Writer, value: APU.Snapshot) !void {
    var core = value;
    core.pulse1.waveform_last_amp = 0;
    core.pulse2.waveform_last_amp = 0;
    core.triangle.waveform_last_amp = 0;
    core.noise.waveform_last_amp = 0;
    core.dmc.waveform_last_amp = 0;
    core.next_transfer_cyc = 0;
    core.last_frame_cyc = 0;
    try writeCanonical(writer, APU.Snapshot, core);
}

pub fn saveSlot(alloc: std.mem.Allocator, rom_name: []const u8, system: *System, slot: usize) !SlotInfo {
    std.debug.assert(slot <= SLOT_COUNT);

    var state_dir = try openStateDir(alloc, rom_name);
    defer state_dir.close();

    const slot_filename = try std.fmt.allocPrint(alloc, "slot-{}.{s}", .{ slot + 1, EXT });
    defer alloc.free(slot_filename);

    const file = try state_dir.createFile(slot_filename, .{});
    defer file.close();

    const snapshot = try alloc.create(Snapshot);
    errdefer alloc.destroy(snapshot);

    snapshot.* = .{
        .saved_at = (try zeit.instant(.{})).unixTimestamp(),
        .cpu = system.cpu.saveState(),
        .bus = try system.bus.saveState(alloc),
        .ppu = system.ppu.saveState(),
        .apu = system.apu.saveState(),
    };
    defer {
        snapshot.deinit(alloc);
        alloc.destroy(snapshot);
    }

    try writeSnapshot(alloc, file, snapshot);
    return .{ .display_time = utils.formatTimestamp(alloc, snapshot.saved_at) };
}

pub fn loadSlot(alloc: std.mem.Allocator, rom_name: []const u8, system: *System, slot: usize) !void {
    std.debug.assert(slot < SLOT_COUNT);

    const file = try openStateFile(alloc, rom_name, slot);
    defer file.close();

    const snapshot = try readSnapshot(alloc, file);
    defer {
        snapshot.deinit(alloc);
        alloc.destroy(snapshot);
    }

    try system.bus.loadState(&snapshot.bus);
    system.ppu.loadState(&snapshot.ppu);
    system.apu.loadState(&snapshot.apu);
    system.cpu.loadState(snapshot.cpu);
}

pub fn slotInfo(alloc: std.mem.Allocator, rom_name: []const u8, slot: usize) !SlotInfo {
    std.debug.assert(slot < SLOT_COUNT);

    const file = try openStateFile(alloc, rom_name, slot);
    defer file.close();

    return .{ .display_time = utils.formatTimestamp(alloc, try readHeaderTimestamp(file)) };
}

fn writeSnapshot(alloc: std.mem.Allocator, file: std.fs.File, snapshot: *const Snapshot) !void {
    var file_buffer: [4096]u8 = undefined;
    var file_writer = file.writer(&file_buffer);
    const writer = &file_writer.interface;

    try writer.writeAll(&MAGIC);
    try writeInt(writer, u32, VERSION);
    try writeInt(writer, i64, snapshot.saved_at);

    var body: std.Io.Writer.Allocating = .init(alloc);
    defer body.deinit();

    try writeValue(&body.writer, CPU.Snapshot, snapshot.cpu);
    try writeBusSnapshot(&body.writer, snapshot.bus);
    try writeValue(&body.writer, PPU.Snapshot, snapshot.ppu);
    try writeValue(&body.writer, APU.Snapshot, snapshot.apu);

    const compressed = try compressBytes(alloc, body.written());
    defer alloc.free(compressed);

    try writeInt(writer, u32, @intCast(body.written().len));
    try writeInt(writer, u32, @intCast(compressed.len));
    try writer.writeAll(compressed);
    try writer.flush();
}

fn readSnapshot(alloc: std.mem.Allocator, file: std.fs.File) !*Snapshot {
    var file_buffer: [4096]u8 = undefined;
    var file_reader = file.reader(&file_buffer);
    const reader = &file_reader.interface;

    const header = try readHeader(reader);
    const body = try readSnapshotBody(alloc, reader, header.version);
    defer alloc.free(body);

    var body_reader: std.Io.Reader = .fixed(body);

    const snapshot = try alloc.create(Snapshot);
    errdefer alloc.destroy(snapshot);

    snapshot.saved_at = header.saved_at;
    snapshot.cpu = try readValue(CPU.Snapshot, &body_reader);
    snapshot.bus = try readBusSnapshot(alloc, &body_reader);
    snapshot.ppu = try readValue(PPU.Snapshot, &body_reader);
    snapshot.apu = try readValue(APU.Snapshot, &body_reader);

    return snapshot;
}

const Header = struct {
    version: u32,
    saved_at: i64,
};

fn readHeader(reader: *std.Io.Reader) !Header {
    var magic: [4]u8 = undefined;
    try reader.readSliceAll(&magic);
    if (!std.mem.eql(u8, &magic, &MAGIC)) return error.InvalidSaveState;

    const version = try readInt(reader, u32);
    if (version != VERSION) return error.UnsupportedSaveStateVersion;

    return .{
        .version = version,
        .saved_at = try readInt(reader, i64),
    };
}

fn readHeaderTimestamp(file: std.fs.File) !i64 {
    var file_buffer: [32]u8 = undefined;
    var file_reader = file.reader(&file_buffer);
    return (try readHeader(&file_reader.interface)).saved_at;
}

fn readSnapshotBody(alloc: std.mem.Allocator, reader: *std.Io.Reader, version: u32) ![]u8 {
    if (version != VERSION) return error.UnsupportedSaveStateVersion;

    const uncompressed_len = try readInt(reader, u32);
    const compressed_len = try readInt(reader, u32);
    const compressed = try alloc.alloc(u8, compressed_len);
    defer alloc.free(compressed);
    try reader.readSliceAll(compressed);

    return try decompressBytes(alloc, compressed, uncompressed_len);
}

fn writeBusSnapshot(writer: *std.Io.Writer, snapshot: Bus.Snapshot) !void {
    try writer.writeAll(&snapshot.ram);
    try writeInt(writer, u64, snapshot.cycles);
    try writeInt(writer, u8, snapshot.open_bus);
    try writeInt(writer, u8, snapshot.dma_start_delay);
    try writeInt(writer, u16, snapshot.dma_cycles);
    try writeValue(writer, Controllers, snapshot.controllers);
    try writeMapperSnapshot(writer, snapshot.rom.mapper);
}

fn readBusSnapshot(alloc: std.mem.Allocator, reader: *std.Io.Reader) !Bus.Snapshot {
    var ram: [2048]u8 = undefined;
    try reader.readSliceAll(&ram);
    return .{
        .ram = ram,
        .cycles = try readInt(reader, u64),
        .open_bus = try readInt(reader, u8),
        .dma_start_delay = try readInt(reader, u8),
        .dma_cycles = try readInt(reader, u16),
        .controllers = try readValue(Controllers, reader),
        .rom = .{ .mapper = try readMapperSnapshot(alloc, reader) },
    };
}

fn writeMapperSnapshot(writer: *std.Io.Writer, snapshot: Mapper.Snapshot) !void {
    switch (snapshot) {
        .mapper0 => |value| {
            try writeInt(writer, u8, 0);
            try writeSlice(writer, value.prg_ram);
            try writeSlice(writer, value.chr_ram);
        },
        .mapper1 => |value| {
            try writeInt(writer, u8, 1);
            try writeValue(writer, u8, value.load_register);
            try writeValue(writer, u8, value.write_index);
            try writeValue(writer, u8, value.control);
            try writeValue(writer, u8, value.prg_bank);
            try writeValue(writer, u8, value.chr_bank_1);
            try writeValue(writer, u8, value.chr_bank_2);
            try writeSlice(writer, value.prg_ram);
            try writeSlice(writer, value.chr_ram);
        },
        .mapper2 => |value| {
            try writeInt(writer, u8, 2);
            try writeValue(writer, u8, value.selected_bank);
            try writeSlice(writer, value.chr_ram);
        },
        .mapper3 => |value| {
            try writeInt(writer, u8, 3);
            try writeValue(writer, u8, value.selected_chr_bank);
            try writeSlice(writer, value.prg_ram);
        },
        .mapper4 => |value| {
            try writeInt(writer, u8, 4);
            try writeValue(writer, [10]usize, value.bank_registers);
            try writeValue(writer, u8, value.bank_select);
            try writeValue(writer, bool, value.prg_inversion);
            try writeValue(writer, bool, value.chr_inversion);
            try writeValue(writer, bool, value.irq_flag);
            try writeValue(writer, u8, value.irq_counter);
            try writeValue(writer, bool, value.irq_reload_flag);
            try writeValue(writer, u8, value.irq_counter_reload);
            try writeValue(writer, bool, value.irq_enabled);
            try writeValue(writer, bool, value.ppu_a12);
            try writeValue(writer, u64, value.ppu_a12_low_cycle);
            try writeValue(writer, Mirroring, value.mirroring_mode);
            try writeSlice(writer, value.prg_ram);
            try writeSlice(writer, value.chr_ram);
        },
    }
}

fn readMapperSnapshot(alloc: std.mem.Allocator, reader: *std.Io.Reader) !Mapper.Snapshot {
    return switch (try readInt(reader, u8)) {
        0 => .{
            .mapper0 = .{
                .prg_ram = try readSlice(alloc, reader),
                .chr_ram = try readSlice(alloc, reader),
            },
        },
        1 => .{
            .mapper1 = .{
                .load_register = try readValue(u8, reader),
                .write_index = try readValue(u8, reader),
                .control = try readValue(u8, reader),
                .prg_bank = try readValue(u8, reader),
                .chr_bank_1 = try readValue(u8, reader),
                .chr_bank_2 = try readValue(u8, reader),
                .prg_ram = try readSlice(alloc, reader),
                .chr_ram = try readSlice(alloc, reader),
            },
        },
        2 => .{
            .mapper2 = .{
                .selected_bank = try readValue(u8, reader),
                .chr_ram = try readSlice(alloc, reader),
            },
        },
        3 => .{
            .mapper3 = .{
                .selected_chr_bank = try readValue(u8, reader),
                .prg_ram = try readSlice(alloc, reader),
            },
        },
        4 => .{
            .mapper4 = .{
                .bank_registers = try readValue([10]usize, reader),
                .bank_select = try readValue(u8, reader),
                .prg_inversion = try readValue(bool, reader),
                .chr_inversion = try readValue(bool, reader),
                .irq_flag = try readValue(bool, reader),
                .irq_counter = try readValue(u8, reader),
                .irq_reload_flag = try readValue(bool, reader),
                .irq_counter_reload = try readValue(u8, reader),
                .irq_enabled = try readValue(bool, reader),
                .ppu_a12 = try readValue(bool, reader),
                .ppu_a12_low_cycle = try readValue(u64, reader),
                .mirroring_mode = try readValue(Mirroring, reader),
                .prg_ram = try readSlice(alloc, reader),
                .chr_ram = try readSlice(alloc, reader),
            },
        },
        else => error.UnsupportedMapperSnapshot,
    };
}

fn writeValue(writer: *std.Io.Writer, comptime T: type, value: T) !void {
    try writer.writeAll(std.mem.asBytes(&value));
}

fn readValue(comptime T: type, reader: *std.Io.Reader) !T {
    var value: T = undefined;
    try reader.readSliceAll(std.mem.asBytes(&value));
    return value;
}

fn writeSlice(writer: *std.Io.Writer, bytes: []const u8) !void {
    try writeInt(writer, u32, @intCast(bytes.len));
    try writer.writeAll(bytes);
}

fn readSlice(alloc: std.mem.Allocator, reader: *std.Io.Reader) ![]u8 {
    const len = try readInt(reader, u32);
    const bytes = try alloc.alloc(u8, len);
    try reader.readSliceAll(bytes);
    return bytes;
}

fn writeInt(writer: *std.Io.Writer, comptime T: type, value: T) !void {
    var buf: [@sizeOf(T)]u8 = undefined;
    std.mem.writeInt(T, &buf, value, .little);
    try writer.writeAll(&buf);
}

fn readInt(reader: *std.Io.Reader, comptime T: type) !T {
    var buf: [@sizeOf(T)]u8 = undefined;
    try reader.readSliceAll(&buf);
    return std.mem.readInt(T, &buf, .little);
}

fn writeCanonicalBus(writer: *std.Io.Writer, snapshot: Bus.Snapshot) !void {
    try writer.writeAll(&snapshot.ram);
    try writeCanonical(writer, u64, snapshot.cycles);
    try writeCanonical(writer, u8, snapshot.open_bus);
    try writeCanonical(writer, u8, snapshot.dma_start_delay);
    try writeCanonical(writer, u16, snapshot.dma_cycles);
    try writeCanonical(writer, Controllers, snapshot.controllers);
    try writeCanonicalMapper(writer, snapshot.rom.mapper);
}

fn readCanonicalBus(alloc: std.mem.Allocator, reader: *std.Io.Reader) !Bus.Snapshot {
    var ram: [2048]u8 = undefined;
    try reader.readSliceAll(&ram);
    return .{
        .ram = ram,
        .cycles = try readCanonical(u64, reader),
        .open_bus = try readCanonical(u8, reader),
        .dma_start_delay = try readCanonical(u8, reader),
        .dma_cycles = try readCanonical(u16, reader),
        .controllers = try readCanonical(Controllers, reader),
        .rom = .{ .mapper = try readCanonicalMapper(alloc, reader) },
    };
}

fn writeCanonicalMapper(writer: *std.Io.Writer, snapshot: Mapper.Snapshot) !void {
    switch (snapshot) {
        .mapper0 => |value| {
            try writeCanonical(writer, u8, 0);
            try writeCanonicalSlice(writer, value.prg_ram);
            try writeCanonicalSlice(writer, value.chr_ram);
        },
        .mapper1 => |value| {
            try writeCanonical(writer, u8, 1);
            try writeCanonical(writer, u8, value.load_register);
            try writeCanonical(writer, u8, value.write_index);
            try writeCanonical(writer, u8, value.control);
            try writeCanonical(writer, u8, value.prg_bank);
            try writeCanonical(writer, u8, value.chr_bank_1);
            try writeCanonical(writer, u8, value.chr_bank_2);
            try writeCanonicalSlice(writer, value.prg_ram);
            try writeCanonicalSlice(writer, value.chr_ram);
        },
        .mapper2 => |value| {
            try writeCanonical(writer, u8, 2);
            try writeCanonical(writer, u8, value.selected_bank);
            try writeCanonicalSlice(writer, value.chr_ram);
        },
        .mapper3 => |value| {
            try writeCanonical(writer, u8, 3);
            try writeCanonical(writer, u8, value.selected_chr_bank);
            try writeCanonicalSlice(writer, value.prg_ram);
        },
        .mapper4 => |value| {
            try writeCanonical(writer, u8, 4);
            for (value.bank_registers) |bank| try writeCanonical(writer, u64, bank);
            try writeCanonical(writer, u8, value.bank_select);
            try writeCanonical(writer, bool, value.prg_inversion);
            try writeCanonical(writer, bool, value.chr_inversion);
            try writeCanonical(writer, bool, value.irq_flag);
            try writeCanonical(writer, u8, value.irq_counter);
            try writeCanonical(writer, bool, value.irq_reload_flag);
            try writeCanonical(writer, u8, value.irq_counter_reload);
            try writeCanonical(writer, bool, value.irq_enabled);
            try writeCanonical(writer, bool, value.ppu_a12);
            try writeCanonical(writer, u64, value.ppu_a12_low_cycle);
            try writeCanonical(writer, Mirroring, value.mirroring_mode);
            try writeCanonicalSlice(writer, value.prg_ram);
            try writeCanonicalSlice(writer, value.chr_ram);
        },
    }
}

fn readCanonicalMapper(alloc: std.mem.Allocator, reader: *std.Io.Reader) !Mapper.Snapshot {
    return switch (try readCanonical(u8, reader)) {
        0 => blk: {
            const prg = try readCanonicalSlice(alloc, reader);
            errdefer alloc.free(prg);
            const chr = try readCanonicalSlice(alloc, reader);
            break :blk .{ .mapper0 = .{ .prg_ram = prg, .chr_ram = chr } };
        },
        1 => blk: {
            const load_register = try readCanonical(u8, reader);
            const write_index = try readCanonical(u8, reader);
            const control = try readCanonical(u8, reader);
            const prg_bank = try readCanonical(u8, reader);
            const chr_bank_1 = try readCanonical(u8, reader);
            const chr_bank_2 = try readCanonical(u8, reader);
            const prg = try readCanonicalSlice(alloc, reader);
            errdefer alloc.free(prg);
            const chr = try readCanonicalSlice(alloc, reader);
            break :blk .{ .mapper1 = .{
                .load_register = load_register,
                .write_index = write_index,
                .control = control,
                .prg_bank = prg_bank,
                .chr_bank_1 = chr_bank_1,
                .chr_bank_2 = chr_bank_2,
                .prg_ram = prg,
                .chr_ram = chr,
            } };
        },
        2 => .{ .mapper2 = .{
            .selected_bank = try readCanonical(u8, reader),
            .chr_ram = try readCanonicalSlice(alloc, reader),
        } },
        3 => .{ .mapper3 = .{
            .selected_chr_bank = try readCanonical(u8, reader),
            .prg_ram = try readCanonicalSlice(alloc, reader),
        } },
        4 => blk: {
            var banks: [10]usize = undefined;
            for (&banks) |*bank| bank.* = @intCast(try readCanonical(u64, reader));
            const bank_select = try readCanonical(u8, reader);
            const prg_inversion = try readCanonical(bool, reader);
            const chr_inversion = try readCanonical(bool, reader);
            const irq_flag = try readCanonical(bool, reader);
            const irq_counter = try readCanonical(u8, reader);
            const irq_reload_flag = try readCanonical(bool, reader);
            const irq_counter_reload = try readCanonical(u8, reader);
            const irq_enabled = try readCanonical(bool, reader);
            const ppu_a12 = try readCanonical(bool, reader);
            const ppu_a12_low_cycle = try readCanonical(u64, reader);
            const mirroring_mode = try readCanonical(Mirroring, reader);
            const prg = try readCanonicalSlice(alloc, reader);
            errdefer alloc.free(prg);
            const chr = try readCanonicalSlice(alloc, reader);
            break :blk .{ .mapper4 = .{
                .bank_registers = banks,
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
                .prg_ram = prg,
                .chr_ram = chr,
            } };
        },
        else => error.UnsupportedMapperSnapshot,
    };
}

fn writeCanonicalSlice(writer: *std.Io.Writer, value: []const u8) !void {
    if (value.len > MAX_NETWORK_SNAPSHOT_SIZE) return error.SnapshotTooLarge;
    try writeCanonical(writer, u32, @intCast(value.len));
    try writer.writeAll(value);
}

fn readCanonicalSlice(alloc: std.mem.Allocator, reader: *std.Io.Reader) ![]u8 {
    const len = try readCanonical(u32, reader);
    if (len > MAX_NETWORK_SNAPSHOT_SIZE) return error.SnapshotTooLarge;
    const value = try alloc.alloc(u8, len);
    errdefer alloc.free(value);
    try reader.readSliceAll(value);
    return value;
}

fn writeCanonical(writer: *std.Io.Writer, comptime T: type, value: T) !void {
    switch (@typeInfo(T)) {
        .void => {},
        .bool => try writer.writeByte(@intFromBool(value)),
        .int => |info| {
            const ValueBits = std.meta.Int(.unsigned, info.bits);
            const byte_len = (info.bits + 7) / 8;
            const Storage = std.meta.Int(.unsigned, byte_len * 8);
            const value_bits: ValueBits = @bitCast(value);
            var buffer: [byte_len]u8 = undefined;
            std.mem.writeInt(Storage, &buffer, @intCast(value_bits), .little);
            try writer.writeAll(&buffer);
        },
        .float => |info| {
            const U = std.meta.Int(.unsigned, info.bits);
            try writeCanonical(writer, U, @bitCast(value));
        },
        .@"enum" => |info| try writeCanonical(writer, info.tag_type, @intFromEnum(value)),
        .optional => |info| {
            if (value) |child| {
                try writer.writeByte(1);
                try writeCanonical(writer, info.child, child);
            } else try writer.writeByte(0);
        },
        .array => |info| for (value) |element| try writeCanonical(writer, info.child, element),
        .@"struct" => |info| inline for (info.fields) |field| {
            try writeCanonical(writer, field.type, @field(value, field.name));
        },
        .@"union" => |info| {
            const Tag = info.tag_type orelse @compileError("canonical snapshot unions must be tagged");
            const tag = std.meta.activeTag(value);
            try writeCanonical(writer, Tag, tag);
            inline for (info.fields) |field| {
                if (tag == @field(Tag, field.name)) try writeCanonical(writer, field.type, @field(value, field.name));
            }
        },
        else => @compileError("unsupported canonical snapshot type: " ++ @typeName(T)),
    }
}

fn readCanonical(comptime T: type, reader: *std.Io.Reader) !T {
    return switch (@typeInfo(T)) {
        .void => {},
        .bool => switch (try reader.takeByte()) {
            0 => false,
            1 => true,
            else => error.InvalidCanonicalBoolean,
        },
        .int => |info| blk: {
            const ValueBits = std.meta.Int(.unsigned, info.bits);
            const byte_len = (info.bits + 7) / 8;
            const Storage = std.meta.Int(.unsigned, byte_len * 8);
            var buffer: [byte_len]u8 = undefined;
            try reader.readSliceAll(&buffer);
            const stored = std.mem.readInt(Storage, &buffer, .little);
            const bits: ValueBits = @truncate(stored);
            break :blk @bitCast(bits);
        },
        .float => |info| @bitCast(try readCanonical(std.meta.Int(.unsigned, info.bits), reader)),
        .@"enum" => |info| std.enums.fromInt(T, try readCanonical(info.tag_type, reader)) orelse
            return error.InvalidCanonicalEnum,
        .optional => |info| switch (try reader.takeByte()) {
            0 => null,
            1 => try readCanonical(info.child, reader),
            else => error.InvalidCanonicalBoolean,
        },
        .array => |info| blk: {
            var result: T = undefined;
            for (&result) |*element| element.* = try readCanonical(info.child, reader);
            break :blk result;
        },
        .@"struct" => |info| blk: {
            var result: T = undefined;
            inline for (info.fields) |field| @field(result, field.name) = try readCanonical(field.type, reader);
            break :blk result;
        },
        .@"union" => |info| blk: {
            const Tag = info.tag_type orelse @compileError("canonical snapshot unions must be tagged");
            const tag = try readCanonical(Tag, reader);
            inline for (info.fields) |field| {
                if (tag == @field(Tag, field.name)) {
                    break :blk @unionInit(T, field.name, try readCanonical(field.type, reader));
                }
            }
            return error.InvalidCanonicalUnion;
        },
        else => @compileError("unsupported canonical snapshot type: " ++ @typeName(T)),
    };
}

fn compressBytes(alloc: std.mem.Allocator, bytes: []const u8) ![]u8 {
    var reader: std.Io.Reader = .fixed(bytes);
    var writer: std.Io.Writer.Allocating = .init(alloc);
    errdefer writer.deinit();

    try compress.compressAlloc(alloc, &reader, &writer.writer, .{});
    return try writer.toOwnedSlice();
}

fn decompressBytes(alloc: std.mem.Allocator, compressed: []const u8, expected_len: usize) ![]u8 {
    var reader: std.Io.Reader = .fixed(compressed);
    const output = try alloc.alloc(u8, expected_len);
    errdefer alloc.free(output);
    var writer: std.Io.Writer = .fixed(output);

    var decompressor: std.compress.flate.Decompress = .init(&reader, .gzip, &.{});
    _ = decompressor.reader.streamRemaining(&writer) catch |err| switch (err) {
        error.ReadFailed, error.WriteFailed => return error.InvalidCompressedSaveState,
    };
    if (writer.end != expected_len) return error.InvalidCompressedSaveState;

    return output;
}

test "canonical network snapshot encode decode" {
    const alloc = std.testing.allocator;
    var snapshot: NetworkSnapshot = undefined;
    @memset(std.mem.asBytes(&snapshot), 0);
    snapshot.bus.rom.mapper = .{ .mapper0 = .{ .prg_ram = &.{}, .chr_ram = &.{} } };

    const encoded = try encodeNetwork(alloc, &snapshot);
    defer alloc.free(encoded);
    var decoded = try decodeNetwork(alloc, encoded);
    defer decoded.deinit(alloc);
    try std.testing.expectEqual(snapshot.cpu.pc, decoded.cpu.pc);
    try std.testing.expectEqual(snapshot.bus.cycles, decoded.bus.cycles);
    try std.testing.expectEqual(try digest(&snapshot), try digest(&decoded));
    try std.testing.expectEqual(try componentDigests(&snapshot), try componentDigests(&decoded));

    var bad = try alloc.dupe(u8, encoded);
    defer alloc.free(bad);
    bad[0] = 'X';
    try std.testing.expectError(error.InvalidNetworkSnapshot, decodeNetwork(alloc, bad));
}

test "canonical network snapshot supports every mapper variant" {
    const alloc = std.testing.allocator;
    const variants = [_]Mapper.Snapshot{
        .{ .mapper0 = .{ .prg_ram = @constCast("prg"), .chr_ram = @constCast("chr") } },
        .{ .mapper1 = .{
            .load_register = 1,
            .write_index = 2,
            .control = 3,
            .prg_bank = 4,
            .chr_bank_1 = 5,
            .chr_bank_2 = 6,
            .prg_ram = @constCast("prg"),
            .chr_ram = @constCast("chr"),
        } },
        .{ .mapper2 = .{ .selected_bank = 2, .chr_ram = @constCast("chr") } },
        .{ .mapper3 = .{ .selected_chr_bank = 3, .prg_ram = @constCast("prg") } },
        .{ .mapper4 = .{
            .bank_registers = .{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 },
            .bank_select = 4,
            .prg_inversion = true,
            .chr_inversion = false,
            .irq_flag = true,
            .irq_counter = 7,
            .irq_reload_flag = false,
            .irq_counter_reload = 8,
            .irq_enabled = true,
            .ppu_a12 = false,
            .ppu_a12_low_cycle = 99,
            .mirroring_mode = .HORIZONTAL,
            .prg_ram = @constCast("prg"),
            .chr_ram = @constCast("chr"),
        } },
    };
    for (variants) |mapper| {
        var snapshot: NetworkSnapshot = undefined;
        @memset(std.mem.asBytes(&snapshot), 0);
        snapshot.bus.rom.mapper = mapper;
        const encoded = try encodeNetwork(alloc, &snapshot);
        defer alloc.free(encoded);
        var decoded = try decodeNetwork(alloc, encoded);
        defer decoded.deinit(alloc);
        try std.testing.expectEqual(std.meta.activeTag(mapper), std.meta.activeTag(decoded.bus.rom.mapper));
        try std.testing.expectEqual(try digest(&snapshot), try digest(&decoded));
    }
}

test "netplay digest excludes presentation-only APU bookkeeping" {
    var snapshot: NetworkSnapshot = undefined;
    @memset(std.mem.asBytes(&snapshot), 0);
    snapshot.bus.rom.mapper = .{ .mapper0 = .{ .prg_ram = &.{}, .chr_ram = &.{} } };
    const baseline = try digest(&snapshot);

    snapshot.apu.next_transfer_cyc = 1234;
    snapshot.apu.last_frame_cyc = 5678;
    snapshot.apu.pulse1.waveform_last_amp = 11;
    snapshot.apu.pulse2.waveform_last_amp = 12;
    snapshot.apu.triangle.waveform_last_amp = 13;
    snapshot.apu.noise.waveform_last_amp = 14;
    snapshot.apu.dmc.waveform_last_amp = 15;
    try std.testing.expectEqual(baseline, try digest(&snapshot));

    snapshot.apu.global_cycle = 1;
    const changed = try digest(&snapshot);
    try std.testing.expect(!std.mem.eql(u8, &baseline, &changed));
}

fn openStateDir(alloc: std.mem.Allocator, rom_name: []const u8) !std.fs.Dir {
    const data_dir_path = try paths.getDataDir(alloc);
    defer alloc.free(data_dir_path);

    std.fs.makeDirAbsolute(data_dir_path) catch |err| switch (err) {
        error.PathAlreadyExists => {},
        else => return err,
    };

    var data_dir = try std.fs.openDirAbsolute(data_dir_path, .{});
    defer data_dir.close();

    data_dir.makeDir(STATE_DIR_NAME) catch |err| switch (err) {
        error.PathAlreadyExists => {},
        else => return err,
    };

    var state_dir = try data_dir.openDir(STATE_DIR_NAME, .{});
    defer state_dir.close();

    state_dir.makeDir(rom_name) catch |err| switch (err) {
        error.PathAlreadyExists => {},
        else => return err,
    };

    return try state_dir.openDir(rom_name, .{});
}

fn openStateFile(alloc: std.mem.Allocator, rom_name: []const u8, slot: usize) !std.fs.File {
    var state_dir = try openStateDir(alloc, rom_name);
    defer state_dir.close();

    const slot_filename = try std.fmt.allocPrint(alloc, "slot-{}.{s}", .{ slot + 1, EXT });
    defer alloc.free(slot_filename);

    return try state_dir.openFile(slot_filename, .{});
}
