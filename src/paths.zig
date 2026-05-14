const std = @import("std");
const builtin = @import("builtin");

pub const APP_NAME = "neskwik";

pub fn getConfigDir(alloc: std.mem.Allocator) std.fs.GetAppDataDirError![]u8 {
    switch (builtin.os.tag) {
        .windows => {
            const local_app_data_dir = std.process.getEnvVarOwned(alloc, "LOCALAPPDATA") catch |err| switch (err) {
                error.OutOfMemory => |e| return e,
                else => return error.AppDataDirUnavailable,
            };
            defer alloc.free(local_app_data_dir);
            return std.fs.path.join(alloc, &.{ local_app_data_dir, APP_NAME });
        },
        .macos => {
            const home_dir = std.posix.getenv("HOME") orelse return error.AppDataDirUnavailable;
            return std.fs.path.join(alloc, &.{ home_dir, "Library", "Application Support", APP_NAME });
        },
        .linux, .serenity => {
            if (std.posix.getenv("XDG_CONFIG_HOME")) |xdg| {
                if (xdg.len > 0) {
                    return std.fs.path.join(alloc, &.{ xdg, APP_NAME });
                }
            }

            const home_dir = std.posix.getenv("HOME") orelse return error.AppDataDirUnavailable;
            return std.fs.path.join(alloc, &.{ home_dir, ".config", APP_NAME });
        },
        else => @compileError("Unsupported OS"),
    }
}

pub fn getDataDir(alloc: std.mem.Allocator) std.fs.GetAppDataDirError![]u8 {
    return std.fs.getAppDataDir(alloc, APP_NAME);
}
