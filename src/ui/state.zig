const std = @import("std");

const game_history = @import("../game_history.zig");
const c = @import("../root.zig").c;
const Key = @import("core/ui.zig").Key;
const UI = @import("core/ui.zig").UI;
const Rom = @import("../rom.zig").Rom;
const utils = @import("core/utils.zig");
const System = @import("../system.zig").System;
const bindings = @import("bindings.zig");
const settings = @import("settings.zig");
const paths = @import("../paths.zig");
const ness = @import("../root.zig");

const NES_WIDTH = ness.NES_WIDTH;
const OVERSCAN_TOP = ness.OVERSCAN_TOP;
const NES_VISIBLE_HEIGHT = ness.NES_VISIBLE_HEIGHT;
const OVERSCAN_PIXEL_OFFSET = OVERSCAN_TOP * NES_WIDTH * 4;
const NES_VISIBLE_PIXEL_BYTES = NES_WIDTH * NES_VISIBLE_HEIGHT * 4;
const NES_CONTROLLER_IMG = @embedFile("nes_controller_img");

const ControllerPlayer = bindings.ControllerPlayer;
const ControllerKeyBindings = bindings.ControllerKeyBindings;
const ControllerBindingTarget = bindings.ControllerBindingTarget;
const GeneralAction = bindings.GeneralAction;
const GeneralKeyBindings = bindings.GeneralKeyBindings;
const GamepadKeyBindings = bindings.GamepadKeyBindings;
const ParamTarget = settings.ParamTarget;
const ShaderParamSetting = settings.ShaderParamSetting;
const EmulationSpeed = settings.EmulationSpeed;

pub const SettingsCategory = enum {
    general,
    video,
    shader,
    controls,

    pub fn displayName(self: @This()) []const u8 {
        return switch (self) {
            .general => "General",
            .video => "Video",
            .shader => "Shader",
            .controls => "Controls",
        };
    }
};

pub const UIState = struct {
    alloc: std.mem.Allocator,
    /// Wheter to skip drawing the home screen.
    render_home_ui: bool = true,
    render_debug_ui: bool = false,
    emulation_running: bool = false,

    rom_bytes: ?[]u8 = null,
    rom: ?Rom = null,
    system: ?System = null,

    history: game_history.GameHistory = undefined,
    current_rom_path: ?[]u8 = null,
    game_start_time_ms: i64 = 0,

    paused: bool = false,

    settings: EmulatorSettings = .{},
    saved_settings: EmulatorSettings = .{},
    config_dir: ?[]u8 = null,
    controller_img: LoadedImage,

    const Self = @This();
    pub const LoadedImage = struct {
        raw: [*c]c.SDL_Surface,

        pub fn w(self: *const @This()) u32 {
            return @intCast(self.raw.*.w);
        }
        pub fn h(self: *const @This()) u32 {
            return @intCast(self.raw.*.h);
        }
        pub fn format(self: *const @This()) c.SDL_PixelFormat {
            return self.raw.*.format;
        }
        pub fn pixels(self: *const @This()) []const u8 {
            const len: usize = @as(usize, @intCast(self.raw.*.pitch)) * @as(usize, @intCast(self.raw.*.h));
            const ptr: [*]const u8 = @ptrCast(self.raw.*.pixels);
            return ptr[0..len];
        }
    };

    pub const EmulatorSettings = struct {
        aspect_ratio: utils.AspectRatio = .@"4_3",
        /// Path to the active shader preset (owned by this struct).
        shader_preset_path: ?[]u8 = null,
        /// Active shader parameter overrides (names are owned by this struct).
        shader_params: std.ArrayList(ShaderParamSetting) = .{},
        /// Set to true to trigger loading the shader preset on the next frame.
        should_load_shader: bool = false,
        /// Set to true to trigger clearing the shader preset on the next frame.
        should_clear_shader: bool = false,
        /// True while an async shader compile is in progress.
        shader_loading: bool = false,
        /// Last shader load error message to display in the settings window (owned).
        shader_error: ?[]u8 = null,
        /// Path to the active border shader preset (owned by this struct).
        border_shader_preset_path: ?[]u8 = null,
        /// Active border shader parameter overrides (names are owned by this struct).
        border_shader_params: std.ArrayList(ShaderParamSetting) = .{},
        /// Set to true to trigger loading the border shader preset on the next frame.
        should_load_border_shader: bool = false,
        /// Set to true to trigger clearing the border shader preset on the next frame.
        should_clear_border_shader: bool = false,
        /// True while an async border shader compile is in progress.
        border_shader_loading: bool = false,
        /// Last border shader load error message (owned).
        border_shader_error: ?[]u8 = null,
        /// Currently selected category in the settings sidebar.
        selected_category: SettingsCategory = .general,
        hide_mouse_on_inactivity: bool = false,
        emulation_speed: EmulationSpeed = .normal,
        selected_controller_player: ControllerPlayer = .one,
        controller_bindings: ControllerKeyBindings = .{},
        capture_binding: ?ControllerBindingTarget = null,
        general_bindings: GeneralKeyBindings = .{},
        capture_general_binding: ?GeneralAction = null,
        gamepad_bindings: GamepadKeyBindings = .{},
        capture_gamepad_binding: ?ControllerBindingTarget = null,
        gamepad_deadzone: u8 = 25,
    };

    pub fn init(alloc: std.mem.Allocator) Self {
        var hist = game_history.GameHistory.init(alloc);
        hist.load();

        const img_bytes = c.SDL_IOFromConstMem(NES_CONTROLLER_IMG, NES_CONTROLLER_IMG.len);
        const surface = c.SDL_LoadPNG_IO(img_bytes, true);

        const config_dir = paths.getConfigDir(alloc) catch |err| blk: {
            std.log.warn("settings directory unavailable: {s}", .{@errorName(err)});
            break :blk null;
        };

        var state = Self{
            .alloc = alloc,
            .history = hist,
            .config_dir = config_dir,
            .controller_img = .{ .raw = surface },
        };
        state.loadSettings();
        state.snapshotSettings() catch @panic("Failed to snapshot loaded settings");
        return state;
    }

    pub fn deinit(self: *Self) void {
        // Save before freeing the system.
        if (self.emulation_running) self.saveCurrentGame();
        self.history.deinit();

        if (self.config_dir) |path| self.alloc.free(path);
        if (self.current_rom_path) |p| self.alloc.free(p);
        if (self.rom_bytes) |rom_bytes| self.alloc.free(rom_bytes);
        deinitEmulatorSettings(self.alloc, &self.settings);
        deinitEmulatorSettings(self.alloc, &self.saved_settings);
        c.SDL_DestroySurface(self.controller_img.raw);

        if (self.emulation_running) {
            self.rom.?.deinit();
            self.system.?.deinit();
        }
    }

    pub fn loadRom(self: *Self, path: []const u8) !void {
        // Save previous game's progress before replacing it.
        if (self.emulation_running) self.saveCurrentGame();

        if (self.rom) |*rom| rom.deinit();
        if (self.system) |*system| system.deinit();

        const cwd = try std.process.getCwdAlloc(self.alloc);
        defer self.alloc.free(cwd);
        const rom_abs_path = try std.fs.path.resolve(self.alloc, &.{ cwd, path });
        defer self.alloc.free(rom_abs_path);

        const file = std.fs.openFileAbsolute(rom_abs_path, .{}) catch |err| switch (err) {
            else => {
                std.debug.print("Error while opening file: {any}\n", .{err});
                std.process.exit(1);
            },
        };
        defer file.close();

        const file_size = try file.getEndPos();
        try file.seekTo(0);

        std.log.debug("Reading file: {s}", .{rom_abs_path});
        if (self.rom_bytes) |bytes| self.alloc.free(bytes);
        self.rom_bytes = try self.alloc.alloc(u8, file_size);
        _ = try file.read(self.rom_bytes.?);

        self.rom = try Rom.init(self.alloc, path, self.rom_bytes.?);
        self.system = try System.init(self.alloc, &self.rom.?, .{});
        self.system.?.reset();
        self.emulation_running = true;
        self.render_home_ui = false;

        self.applyControllerBindings();

        if (self.current_rom_path) |p| self.alloc.free(p);
        self.current_rom_path = self.alloc.dupe(u8, path) catch null;
        self.game_start_time_ms = std.time.milliTimestamp();
    }

    pub fn unloadCurrentRom(self: *Self) void {
        self.saveCurrentGame();

        self.rom.?.deinit();
        self.system.?.deinit();
        self.alloc.free(self.current_rom_path.?);
        self.alloc.free(self.rom_bytes.?);

        self.rom = null;
        self.system = null;
        self.rom_bytes = null;
        self.current_rom_path = null;
        self.emulation_running = false;
        self.render_home_ui = true;
    }

    fn saveCurrentGame(self: *Self) void {
        const path = self.current_rom_path orelse return;
        const name = std.fs.path.stem(path);

        const elapsed_ms = std.time.milliTimestamp() - self.game_start_time_ms;
        const elapsed_secs: u64 = if (elapsed_ms > 0) @intCast(@divFloor(elapsed_ms, 1000)) else 0;

        var existing_secs: u64 = 0;
        for (self.history.entries.items) |entry| {
            if (std.mem.eql(u8, entry.rom_path, path)) {
                existing_secs = entry.play_time_secs;
                break;
            }
        }

        const pixels = self.system.?.frame_buffer()[OVERSCAN_PIXEL_OFFSET..][0..NES_VISIBLE_PIXEL_BYTES];
        self.history.save(name, path, existing_secs + elapsed_secs, pixels);

        // Reset so back-to-back saves (loadRom then deinit) don't double-count.
        self.game_start_time_ms = std.time.milliTimestamp();
    }

    pub fn applyControllerBindings(self: *Self) void {
        if (self.system) |*system| {
            bindings.applyBindingsToKeymap(&system.keymap1, self.settings.controller_bindings.player1);
            bindings.applyBindingsToKeymap(&system.keymap2, self.settings.controller_bindings.player2);
        }
    }

    pub fn generalBinding(self: *const Self, action: GeneralAction) Key {
        return self.settings.general_bindings.get(action);
    }

    pub fn hasSettingsChanges(self: *const Self) bool {
        return !isSettingsEqual(self.settings, self.saved_settings);
    }

    pub fn saveSettings(self: *Self) void {
        self.saveSettingsImpl() catch |err|
            std.log.err("settings save failed: {s}", .{@errorName(err)});
        self.snapshotSettings() catch @panic("Failed to snapshot saved settings");
    }

    pub fn restoreSavedSettings(self: *Self) void {
        const restored = clonePersistedSettings(self.alloc, self.saved_settings) catch
            @panic("Failed to restore loaded settings");
        deinitEmulatorSettings(self.alloc, &self.settings);
        self.settings = restored;

        self.settings.should_load_shader = self.settings.shader_preset_path != null;
        self.settings.should_clear_shader = self.settings.shader_preset_path == null;
        self.settings.shader_loading = false;
        self.settings.shader_error = null;

        self.settings.should_load_border_shader = self.settings.border_shader_preset_path != null;
        self.settings.should_clear_border_shader = self.settings.border_shader_preset_path == null;
        self.settings.border_shader_loading = false;
        self.settings.border_shader_error = null;

        self.applyControllerBindings();
    }

    fn loadSettings(self: *Self) void {
        settings.load(self.alloc, self.config_dir, &self.settings) catch |err|
            std.log.err("settings load failed: {s}", .{@errorName(err)});
    }

    fn saveSettingsImpl(self: *Self) !void {
        try settings.save(self.alloc, self.config_dir, self.settings);
    }

    fn snapshotSettings(self: *Self) !void {
        var snapshot = try clonePersistedSettings(self.alloc, self.settings);
        errdefer deinitEmulatorSettings(self.alloc, &snapshot);

        deinitEmulatorSettings(self.alloc, &self.saved_settings);
        self.saved_settings = snapshot;
    }

    pub fn applyShaderParamSettings(self: *const Self, ui: *UI, target: ParamTarget) void {
        const items = switch (target) {
            .main => self.settings.shader_params.items,
            .border => self.settings.border_shader_params.items,
        };

        for (items) |item| {
            switch (target) {
                .main => ui.setShaderParam(item.name, item.value),
                .border => ui.setBorderShaderParam(item.name, item.value),
            }
        }
    }

    pub fn setShaderParamSetting(self: *Self, target: ParamTarget, name: []const u8, value: f32) void {
        const param_settings = switch (target) {
            .main => &self.settings.shader_params,
            .border => &self.settings.border_shader_params,
        };

        settings.setShaderParamSetting(self.alloc, param_settings, name, value) catch {
            std.log.err("failed to persist shader parameter '{s}': out of memory", .{name});
            return;
        };
    }
};

fn clonePersistedSettings(
    alloc: std.mem.Allocator,
    source: UIState.EmulatorSettings,
) !UIState.EmulatorSettings {
    var result = UIState.EmulatorSettings{
        .aspect_ratio = source.aspect_ratio,
        .hide_mouse_on_inactivity = source.hide_mouse_on_inactivity,
        .emulation_speed = source.emulation_speed,
        .controller_bindings = source.controller_bindings,
        .general_bindings = source.general_bindings,
        .gamepad_bindings = source.gamepad_bindings,
        .gamepad_deadzone = source.gamepad_deadzone,
    };
    errdefer deinitEmulatorSettings(alloc, &result);

    if (source.shader_preset_path) |path| {
        result.shader_preset_path = try alloc.dupe(u8, path);
    }
    try cloneShaderParamSettings(alloc, &result.shader_params, source.shader_params.items);

    if (source.border_shader_preset_path) |path| {
        result.border_shader_preset_path = try alloc.dupe(u8, path);
    }
    try cloneShaderParamSettings(alloc, &result.border_shader_params, source.border_shader_params.items);

    return result;
}

fn cloneShaderParamSettings(
    alloc: std.mem.Allocator,
    dest: *std.ArrayList(ShaderParamSetting),
    source: []const ShaderParamSetting,
) !void {
    errdefer settings.clearShaderParamSettings(alloc, dest);

    for (source) |item| {
        const owned_name = try alloc.dupe(u8, item.name);
        errdefer alloc.free(owned_name);

        try dest.append(alloc, .{
            .name = owned_name,
            .value = item.value,
        });
    }
}

fn deinitShaderGroup(
    alloc: std.mem.Allocator,
    preset_path: *?[]u8,
    params: *std.ArrayList(ShaderParamSetting),
    err_msg: *?[]u8,
) void {
    if (preset_path.*) |path| {
        alloc.free(path);
        preset_path.* = null;
    }
    settings.clearShaderParamSettings(alloc, params);
    params.deinit(alloc);
    if (err_msg.*) |msg| {
        alloc.free(msg);
        err_msg.* = null;
    }
}

fn deinitEmulatorSettings(alloc: std.mem.Allocator, s: *UIState.EmulatorSettings) void {
    deinitShaderGroup(alloc, &s.shader_preset_path, &s.shader_params, &s.shader_error);
    deinitShaderGroup(alloc, &s.border_shader_preset_path, &s.border_shader_params, &s.border_shader_error);
}

fn isSettingsEqual(a: UIState.EmulatorSettings, b: UIState.EmulatorSettings) bool {
    return a.aspect_ratio == b.aspect_ratio and
        optionalStringsEqual(a.shader_preset_path, b.shader_preset_path) and
        shaderParamSettingsEqual(a.shader_params.items, b.shader_params.items) and
        optionalStringsEqual(a.border_shader_preset_path, b.border_shader_preset_path) and
        shaderParamSettingsEqual(a.border_shader_params.items, b.border_shader_params.items) and
        a.hide_mouse_on_inactivity == b.hide_mouse_on_inactivity and
        a.emulation_speed == b.emulation_speed and
        std.meta.eql(a.controller_bindings, b.controller_bindings) and
        std.meta.eql(a.general_bindings, b.general_bindings) and
        std.meta.eql(a.gamepad_bindings, b.gamepad_bindings) and
        a.gamepad_deadzone == b.gamepad_deadzone;
}

fn optionalStringsEqual(a: ?[]const u8, b: ?[]const u8) bool {
    if (a == null and b == null) return true;
    if (a == null or b == null) return false;
    return std.mem.eql(u8, a.?, b.?);
}

fn shaderParamSettingsEqual(a: []const ShaderParamSetting, b: []const ShaderParamSetting) bool {
    if (a.len != b.len) return false;
    for (a, b) |a_item, b_item| {
        if (!std.mem.eql(u8, a_item.name, b_item.name)) return false;
        if (a_item.value != b_item.value) return false;
    }
    return true;
}
