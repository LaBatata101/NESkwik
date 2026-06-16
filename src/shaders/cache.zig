const std = @import("std");
const builtin = @import("builtin");
const paths = @import("../utils/paths.zig");

const MAGIC: [4]u8 = "NSHC".*;
const VERSION: u32 = 1;

pub const SpirvPair = struct {
    vert: []u8,
    frag: []u8,

    pub fn deinit(self: SpirvPair, alloc: std.mem.Allocator) void {
        alloc.free(self.vert);
        alloc.free(self.frag);
    }
};

pub const ShaderCache = struct {
    alloc: std.mem.Allocator,
    dir: std.fs.Dir,

    const SHADERS_SUBDIR = "shaders";

    pub fn init(alloc: std.mem.Allocator) !ShaderCache {
        const cache_path = try paths.getCacheDir(alloc);
        defer alloc.free(cache_path);

        std.fs.makeDirAbsolute(cache_path) catch |err| switch (err) {
            error.PathAlreadyExists => {},
            else => return err,
        };

        var cache_dir = try std.fs.openDirAbsolute(cache_path, .{});
        defer cache_dir.close();

        cache_dir.makeDir(SHADERS_SUBDIR) catch |err| switch (err) {
            error.PathAlreadyExists => {},
            else => return err,
        };

        return .{ .alloc = alloc, .dir = try cache_dir.openDir(SHADERS_SUBDIR, .{}) };
    }

    pub fn deinit(self: *ShaderCache) void {
        self.dir.close();
    }

    /// Hash of (vk_version, glsl_version, vertex_src, fragment_src).
    pub fn computeKey(
        vk_version: u32,
        glsl_version: u32,
        vert_src: []const u8,
        frag_src: []const u8,
    ) [32]u8 {
        var h = std.crypto.hash.sha2.Sha256.init(.{});
        h.update(std.mem.asBytes(&vk_version));
        h.update(std.mem.asBytes(&glsl_version));
        h.update(vert_src);
        h.update(frag_src);
        var digest: [32]u8 = undefined;
        h.final(&digest);
        return digest;
    }

    /// Returns the cached SPIR-V pair, or null on miss. Caller owns the slices.
    pub fn lookup(self: *const ShaderCache, alloc: std.mem.Allocator, key: [32]u8) ?SpirvPair {
        const filename = self.spirv_filename(key) catch return null;
        defer self.alloc.free(filename);

        const file = self.dir.openFile(filename, .{}) catch return null;
        defer file.close();
        return readEntry(alloc, file) catch null;
    }

    /// Stores the SPIR-V pair.
    pub fn store(self: *const ShaderCache, key: [32]u8, vert: []const u8, frag: []const u8) !void {
        const filename = try self.spirv_filename(key);
        defer self.alloc.free(filename);

        const file = try self.dir.createFile(filename, .{});
        defer file.close();
        try writeEntry(file, vert, frag);
    }

    fn spirv_filename(self: *const ShaderCache, key: [32]u8) ![]u8 {
        const hex = std.fmt.bytesToHex(key, .lower);
        return std.fmt.allocPrint(self.alloc, "{s}.spirv", .{&hex});
    }
};

fn readEntry(alloc: std.mem.Allocator, file: std.fs.File) !SpirvPair {
    var buf: [4]u8 = undefined;

    if (try file.readAll(&buf) != 4) return error.UnexpectedEof;
    if (!std.mem.eql(u8, &buf, &MAGIC)) return error.InvalidMagic;

    if (try file.readAll(&buf) != 4) return error.UnexpectedEof;
    if (std.mem.readInt(u32, &buf, .little) != VERSION) return error.VersionMismatch;

    if (try file.readAll(&buf) != 4) return error.UnexpectedEof;
    const vert_len = std.mem.readInt(u32, &buf, .little);
    const vert = try alloc.alloc(u8, vert_len);
    errdefer alloc.free(vert);
    if (try file.readAll(vert) != vert_len) return error.UnexpectedEof;

    if (try file.readAll(&buf) != 4) return error.UnexpectedEof;
    const frag_len = std.mem.readInt(u32, &buf, .little);
    const frag = try alloc.alloc(u8, frag_len);
    errdefer alloc.free(frag);
    if (try file.readAll(frag) != frag_len) return error.UnexpectedEof;

    return .{ .vert = vert, .frag = frag };
}

fn writeEntry(file: std.fs.File, vert: []const u8, frag: []const u8) !void {
    var len_buf: [4]u8 = undefined;

    try file.writeAll(&MAGIC);

    std.mem.writeInt(u32, &len_buf, VERSION, .little);
    try file.writeAll(&len_buf);

    std.mem.writeInt(u32, &len_buf, @intCast(vert.len), .little);
    try file.writeAll(&len_buf);
    try file.writeAll(vert);

    std.mem.writeInt(u32, &len_buf, @intCast(frag.len), .little);
    try file.writeAll(&len_buf);
    try file.writeAll(frag);
}
