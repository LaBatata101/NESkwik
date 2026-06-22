const std = @import("std");
const generated = @import("border_shader_entries");

pub const border_shader_dir = "builtin://border-shaders";

pub const BorderShader = enum {
    none,
    snow,
    water,
    mudlord,
    bigblur,

    pub fn label(self: @This()) []const u8 {
        return switch (self) {
            .none => "None",
            .snow => "Snow",
            .water => "Water",
            .mudlord => "Mudlord",
            .bigblur => "Bigblur",
        };
    }

    pub fn presetPath(self: @This()) ?[]const u8 {
        if (self == .none) return null;

        inline for (@typeInfo(@This()).@"enum".fields) |field| {
            if (self == @field(@This(), field.name)) {
                return presetPathForName(field.name);
            }
        }
        unreachable;
    }
};

fn presetPathForName(comptime shader_name: []const u8) []const u8 {
    return border_shader_dir ++ "/" ++ shader_name ++ ".slangp";
}

const entries = generated.entries;

pub fn sourceForPath(path: []const u8) ?[]const u8 {
    for (entries) |entry| {
        if (std.mem.eql(u8, path, entry.path)) return entry.source;
    }
    return null;
}

pub fn isBorderShaderPath(path: []const u8) bool {
    return std.mem.startsWith(u8, path, border_shader_dir);
}

pub fn joinBorderShaderPath(alloc: std.mem.Allocator, filename: []const u8) ![]u8 {
    return std.fmt.allocPrint(alloc, "{s}/{s}", .{ border_shader_dir, filename });
}
