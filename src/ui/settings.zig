const std = @import("std");
const builtin = @import("builtin");

const bindings = @import("bindings.zig");
const utils = @import("core/utils.zig");
const EmulatorSettings = @import("state.zig").UIState.EmulatorSettings;

const APP_NAME = "ness";
const SETTINGS_FILENAME = "config.json";

pub const EmulationSpeed = enum {
    half,
    normal,
    double,
    triple,
    quadruple,

    pub fn label(self: @This()) []const u8 {
        return switch (self) {
            .half => "0.5x",
            .normal => "1x",
            .double => "2x",
            .triple => "3x",
            .quadruple => "4x",
        };
    }

    pub fn multiplier(self: @This()) f32 {
        return switch (self) {
            .half => 0.5,
            .normal => 1.0,
            .double => 2.0,
            .triple => 3.0,
            .quadruple => 4.0,
        };
    }
};

pub const ParamTarget = enum {
    main,
    border,
};

pub const ShaderParamSetting = struct {
    name: []const u8,
    value: f32,
};

const SettingsConfig = struct {
    version: u32 = 1,
    aspect_ratio: utils.AspectRatio = .@"4_3",
    shader_preset_path: ?[]const u8 = null,
    shader_params: []const ShaderParamSetting = &.{},
    border_shader_preset_path: ?[]const u8 = null,
    border_shader_params: []const ShaderParamSetting = &.{},
    hide_mouse_on_inactivity: bool = false,
    emulation_speed: EmulationSpeed = .normal,
    controller_bindings: bindings.ControllerKeyBindings = .{},
    general_bindings: bindings.GeneralKeyBindings = .{},
    gamepad_bindings: bindings.GamepadKeyBindings = .{},
    gamepad_deadzone: u8 = 25,
};

pub fn getSettingsDir(alloc: std.mem.Allocator) std.fs.GetAppDataDirError![]u8 {
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
        .linux, .freebsd, .netbsd, .dragonfly, .openbsd, .solaris, .illumos, .serenity => {
            if (std.posix.getenv("XDG_CONFIG_HOME")) |xdg| {
                if (xdg.len > 0) {
                    return std.fs.path.join(alloc, &.{ xdg, APP_NAME });
                }
            }

            const home_dir = std.posix.getenv("HOME") orelse return error.AppDataDirUnavailable;
            return std.fs.path.join(alloc, &.{ home_dir, ".config", APP_NAME });
        },
        .haiku => {
            var dir_path_buf: [std.fs.max_path_bytes]u8 = undefined;
            const rc = std.c.find_directory(.B_USER_SETTINGS_DIRECTORY, -1, true, &dir_path_buf, dir_path_buf.len);
            const settings_dir = try alloc.dupeZ(u8, std.mem.sliceTo(&dir_path_buf, 0));
            defer alloc.free(settings_dir);
            switch (rc) {
                0 => return std.fs.path.join(alloc, &.{ settings_dir, APP_NAME }),
                else => return error.AppDataDirUnavailable,
            }
        },
        else => @compileError("Unsupported OS"),
    }
}

pub fn load(alloc: std.mem.Allocator, config_dir: ?[]const u8, settings: *EmulatorSettings) !void {
    var arena = std.heap.ArenaAllocator.init(alloc);
    defer arena.deinit();
    const arena_alloc = arena.allocator();

    const dir = config_dir orelse return;
    const config_path = try std.fs.path.join(arena_alloc, &.{ dir, SETTINGS_FILENAME });

    const file = std.fs.openFileAbsolute(config_path, .{}) catch |err| switch (err) {
        error.FileNotFound => return,
        else => return err,
    };
    defer file.close();

    const json_bytes = try file.readToEndAlloc(arena_alloc, 256 * 1024);
    const parsed = try std.json.parseFromSlice(SettingsConfig, arena_alloc, json_bytes, .{
        .ignore_unknown_fields = true,
    });
    defer parsed.deinit();

    try applyConfig(alloc, settings, parsed.value);
}

pub fn save(alloc: std.mem.Allocator, config_dir: ?[]const u8, settings: EmulatorSettings) !void {
    var arena = std.heap.ArenaAllocator.init(alloc);
    defer arena.deinit();
    const arena_alloc = arena.allocator();

    const dir = config_dir orelse return;
    try std.fs.cwd().makePath(dir);

    const config_path = try std.fs.path.join(arena_alloc, &.{ dir, SETTINGS_FILENAME });
    const json_bytes = try std.json.Stringify.valueAlloc(arena_alloc, SettingsConfig{
        .aspect_ratio = settings.aspect_ratio,
        .shader_preset_path = settings.shader_preset_path,
        .shader_params = settings.shader_params.items,
        .border_shader_preset_path = settings.border_shader_preset_path,
        .border_shader_params = settings.border_shader_params.items,
        .hide_mouse_on_inactivity = settings.hide_mouse_on_inactivity,
        .emulation_speed = settings.emulation_speed,
        .controller_bindings = settings.controller_bindings,
        .general_bindings = settings.general_bindings,
        .gamepad_bindings = settings.gamepad_bindings,
        .gamepad_deadzone = settings.gamepad_deadzone,
    }, .{
        .whitespace = .indent_2,
        .emit_null_optional_fields = false,
    });

    const file = try std.fs.createFileAbsolute(config_path, .{});
    defer file.close();
    try file.writeAll(json_bytes);
}

fn applyConfig(alloc: std.mem.Allocator, settings: *EmulatorSettings, config: SettingsConfig) !void {
    settings.aspect_ratio = config.aspect_ratio;
    settings.hide_mouse_on_inactivity = config.hide_mouse_on_inactivity;
    settings.emulation_speed = config.emulation_speed;
    settings.controller_bindings = config.controller_bindings;
    settings.general_bindings = config.general_bindings;
    settings.gamepad_bindings = config.gamepad_bindings;
    settings.gamepad_deadzone = config.gamepad_deadzone;

    try replaceOptionalOwnedString(alloc, &settings.shader_preset_path, config.shader_preset_path);
    try replaceShaderParamSettings(alloc, &settings.shader_params, config.shader_params);
    if (settings.shader_preset_path == null) {
        clearShaderParamSettings(alloc, &settings.shader_params);
    }
    settings.should_load_shader = settings.shader_preset_path != null;

    try replaceOptionalOwnedString(alloc, &settings.border_shader_preset_path, config.border_shader_preset_path);
    try replaceShaderParamSettings(alloc, &settings.border_shader_params, config.border_shader_params);
    if (settings.border_shader_preset_path == null) {
        clearShaderParamSettings(alloc, &settings.border_shader_params);
    }
    settings.should_load_border_shader = settings.border_shader_preset_path != null;
}

pub fn setShaderParamSetting(
    alloc: std.mem.Allocator,
    param_settings: *std.ArrayList(ShaderParamSetting),
    name: []const u8,
    value: f32,
) !void {
    for (param_settings.items) |*item| {
        if (std.mem.eql(u8, item.name, name)) {
            item.value = value;
            return;
        }
    }

    const owned_name = try alloc.dupe(u8, name);
    errdefer alloc.free(owned_name);

    try param_settings.append(alloc, .{
        .name = owned_name,
        .value = value,
    });
}

fn replaceOptionalOwnedString(alloc: std.mem.Allocator, dest: *?[]u8, source: ?[]const u8) !void {
    if (dest.*) |old| {
        alloc.free(old);
        dest.* = null;
    }

    if (source) |value| {
        dest.* = try alloc.dupe(u8, value);
    }
}

pub fn clearShaderParamSettings(alloc: std.mem.Allocator, settings: *std.ArrayList(ShaderParamSetting)) void {
    for (settings.items) |item| {
        alloc.free(item.name);
    }
    settings.clearRetainingCapacity();
}

fn replaceShaderParamSettings(
    alloc: std.mem.Allocator,
    dest: *std.ArrayList(ShaderParamSetting),
    source: []const ShaderParamSetting,
) !void {
    clearShaderParamSettings(alloc, dest);
    errdefer clearShaderParamSettings(alloc, dest);

    for (source) |item| {
        const owned_name = try alloc.dupe(u8, item.name);
        errdefer alloc.free(owned_name);

        try dest.append(alloc, .{
            .name = owned_name,
            .value = item.value,
        });
    }
}
