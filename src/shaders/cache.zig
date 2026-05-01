const std = @import("std");
const builtin = @import("builtin");

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
    dir: []u8,

    pub fn init(alloc: std.mem.Allocator) !ShaderCache {
        const dir = try cacheDir(alloc);
        errdefer alloc.free(dir);
        try makeDirAll(dir);
        return .{ .alloc = alloc, .dir = dir };
    }

    pub fn deinit(self: *ShaderCache) void {
        self.alloc.free(self.dir);
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
        const path = self.entryPath(key) catch return null;
        defer self.alloc.free(path);
        const file = std.fs.openFileAbsolute(path, .{}) catch return null;
        defer file.close();
        return readEntry(alloc, file) catch null;
    }

    /// Stores the SPIR-V pair.
    pub fn store(self: *const ShaderCache, key: [32]u8, vert: []const u8, frag: []const u8) !void {
        const path = try self.entryPath(key);
        defer self.alloc.free(path);
        const file = try std.fs.createFileAbsolute(path, .{ .truncate = true });
        defer file.close();
        try writeEntry(file, vert, frag);
    }

    fn entryPath(self: *const ShaderCache, key: [32]u8) ![]u8 {
        const hex = std.fmt.bytesToHex(key, .lower);
        return std.fmt.allocPrint(
            self.alloc,
            "{s}" ++ std.fs.path.sep_str ++ "{s}.spirv",
            .{ self.dir, &hex },
        );
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

/// Creates `path` and all missing parent directories.
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

/// Returns the OS-appropriate shader cache directory (owned by caller).
fn cacheDir(alloc: std.mem.Allocator) ![]u8 {
    // Subdirectory appended to the OS base cache path.
    const rel = "nesskwik" ++ std.fs.path.sep_str ++ "shaders";

    switch (builtin.os.tag) {
        .windows => {
            const base = std.process.getEnvVarOwned(alloc, "LOCALAPPDATA") catch
                try std.process.getEnvVarOwned(alloc, "APPDATA");
            defer alloc.free(base);
            return std.fmt.allocPrint(alloc, "{s}" ++ std.fs.path.sep_str ++ "{s}", .{ base, rel });
        },
        .macos => {
            const home = try std.process.getEnvVarOwned(alloc, "HOME");
            defer alloc.free(home);
            return std.fmt.allocPrint(alloc, "{s}/Library/Caches/{s}", .{ home, rel });
        },
        else => {
            // XDG_CACHE_HOME takes priority; fall back to ~/.cache.
            if (std.process.getEnvVarOwned(alloc, "XDG_CACHE_HOME") catch null) |xdg| {
                defer alloc.free(xdg);
                return std.fmt.allocPrint(alloc, "{s}/{s}", .{ xdg, rel });
            }
            const home = try std.process.getEnvVarOwned(alloc, "HOME");
            defer alloc.free(home);
            return std.fmt.allocPrint(alloc, "{s}/.cache/{s}", .{ home, rel });
        },
    }
}
