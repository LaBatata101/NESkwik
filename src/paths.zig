const std = @import("std");
const builtin = @import("builtin");
const c = @import("root.zig").c;
const sdlError = @import("utils/sdl.zig").sdlError;

pub const APP_NAME = "neskwik";

pub fn getConfigDir(alloc: std.mem.Allocator) ![]u8 {
    switch (builtin.os.tag) {
        .windows => {
            const local_app_data_dir = std.process.getEnvVarOwned(alloc, "LOCALAPPDATA") catch |err| switch (err) {
                error.OutOfMemory => |e| return e,
                else => return error.AppConfigDirUnavailable,
            };
            defer alloc.free(local_app_data_dir);
            return std.fs.path.join(alloc, &.{ local_app_data_dir, APP_NAME });
        },
        .macos => {
            const home_dir = std.posix.getenv("HOME") orelse return error.AppConfigDirUnavailable;
            return std.fs.path.join(alloc, &.{ home_dir, "Library", "Application Support", APP_NAME });
        },
        .linux, .serenity => {
            if (builtin.abi.isAndroid()) {
                const path = sdlError(c.SDL_GetAndroidExternalStoragePath());
                return std.fs.path.join(alloc, &.{ std.mem.span(path), "config" });
            }

            if (std.posix.getenv("XDG_CONFIG_HOME")) |xdg| {
                if (xdg.len > 0) {
                    return std.fs.path.join(alloc, &.{ xdg, APP_NAME });
                }
            }

            const home_dir = std.posix.getenv("HOME") orelse return error.AppConfigDirUnavailable;
            return std.fs.path.join(alloc, &.{ home_dir, ".config", APP_NAME });
        },
        else => @compileError("Unsupported OS"),
    }
}

pub fn getDataDir(alloc: std.mem.Allocator) ![]u8 {
    if (builtin.abi.isAndroid()) {
        const path = sdlError(c.SDL_GetAndroidExternalStoragePath());
        return try alloc.dupe(u8, std.mem.span(path));
    } else {
        return std.fs.getAppDataDir(alloc, APP_NAME);
    }
}

pub fn getLogDir(alloc: std.mem.Allocator) ![]u8 {
    switch (builtin.os.tag) {
        .windows => {
            const local_app_data_dir = std.process.getEnvVarOwned(alloc, "LOCALAPPDATA") catch |err| switch (err) {
                error.OutOfMemory => |e| return e,
                else => return error.AppLogDirUnavailable,
            };
            defer alloc.free(local_app_data_dir);
            return std.fs.path.join(alloc, &.{ local_app_data_dir, APP_NAME, "logs" });
        },
        .macos => {
            const home_dir = std.posix.getenv("HOME") orelse return error.AppLogDirUnavailable;
            return std.fs.path.join(alloc, &.{ home_dir, "Library", "Logs", APP_NAME });
        },
        .linux, .serenity => {
            if (builtin.abi.isAndroid()) {
                const path = sdlError(c.SDL_GetAndroidExternalStoragePath());
                return std.fs.path.join(alloc, &.{ std.mem.span(path), "logs" });
            }

            if (std.posix.getenv("XDG_STATE_HOME")) |xdg| {
                if (xdg.len > 0) {
                    return std.fs.path.join(alloc, &.{ xdg, APP_NAME, "logs" });
                }
            }

            const home_dir = std.posix.getenv("HOME") orelse return error.AppLogDirUnavailable;
            return std.fs.path.join(alloc, &.{ home_dir, ".local", "state", APP_NAME, "logs" });
        },
        else => @compileError("Unsupported OS"),
    }
}

/// Returns the OS-appropriate shader cache directory (owned by caller).
pub fn getCacheDir(alloc: std.mem.Allocator) ![]u8 {
    switch (builtin.os.tag) {
        .windows => {
            const local_app_data_dir = std.process.getEnvVarOwned(alloc, "LOCALAPPDATA") catch |err| switch (err) {
                error.OutOfMemory => |e| return e,
                else => return error.AppCacheDirUnavailable,
            };
            defer alloc.free(local_app_data_dir);
            return std.fs.path.join(alloc, &.{ local_app_data_dir, APP_NAME, "cache" });
        },
        .macos => {
            const home_dir = std.posix.getenv("HOME") orelse return error.AppCacheDirUnavailable;
            return std.fs.path.join(alloc, &.{ home_dir, "Library", "Caches", APP_NAME });
        },
        .linux, .serenity => {
            if (builtin.abi.isAndroid()) {
                const path = sdlError(c.SDL_GetAndroidCachePath());
                return try alloc.dupe(u8, std.mem.span(path));
            }

            if (std.posix.getenv("XDG_CACHE_HOME")) |xdg| {
                return std.fs.path.join(alloc, &.{ xdg, APP_NAME });
            }

            const home_dir = std.posix.getenv("HOME") orelse return error.AppCacheDirUnavailable;
            return std.fs.path.join(alloc, &.{ home_dir, ".cache", APP_NAME });
        },
        else => @compileError("Unsupported OS"),
    }
}
