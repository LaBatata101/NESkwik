const std = @import("std");
const builtin = @import("builtin");

const game_history = @import("../game_history.zig");
const c = @import("../root.zig").c;
const Key = @import("core/ui.zig").Key;
const UI = @import("core/ui.zig").UI;
const Rom = @import("../rom.zig").Rom;
const utils = @import("core/utils.zig");
const System = @import("../system.zig").System;
const Frame = @import("../render.zig").Frame;
const PatternTableFrame = @import("../render.zig").PatternTableFrame;
const ProcessorStatus = @import("../cpu.zig").ProcessorStatus;
const ControllerButton = @import("../controller.zig").ControllerButton;
const bindings = @import("bindings.zig");
const settings = @import("settings.zig");
const paths = @import("../paths.zig");
const save_state = @import("../save_state.zig");
const file = @import("../utils/file.zig");
const ness = @import("../root.zig");
const sdlError = ness.sdlError;

const NES_WIDTH = ness.NES_WIDTH;
const NES_HEIGHT = ness.NES_HEIGHT;
const OVERSCAN_TOP = ness.OVERSCAN_TOP;
const NES_VISIBLE_HEIGHT = ness.NES_VISIBLE_HEIGHT;
const NES_PIXEL_BYTES = NES_WIDTH * NES_HEIGHT * 4;
const OVERSCAN_PIXEL_OFFSET = OVERSCAN_TOP * NES_WIDTH * 4;
const NES_VISIBLE_PIXEL_BYTES = NES_WIDTH * NES_VISIBLE_HEIGHT * 4;
const NES_CONTROLLER_IMG = @embedFile("nes_controller_img");
const NO_FRAME: u8 = std.math.maxInt(u8);
const CURSOR_HIDE_DELAY_MS = 3000;

const ControllerPlayer = bindings.ControllerPlayer;
const ControllerAction = bindings.ControllerAction;
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

pub const AppState = struct {
    alloc: std.mem.Allocator,
    /// Wheter to skip drawing the home screen.
    render_home_ui: bool = true,
    render_debug_ui: bool = false,
    emulation_running: bool = false,

    step_mode: bool = false,

    rom_bytes: ?[]u8 = null,
    rom: ?Rom = null,
    system: ?System = null,

    history: game_history.GameHistory = undefined,
    current_rom_path: ?[]u8 = null,
    game_start_time_ms: i64 = 0,

    paused: bool = false,
    lifecycle_suspended: std.atomic.Value(bool) = .init(false),

    last_mouse_activity_time: u64,
    is_cursor_hidden: bool = false,

    /// Set to true to trigger loading the shader preset on the next frame.
    should_load_shader: bool = false,
    /// Set to true to trigger clearing the shader preset on the next frame.
    should_clear_shader: bool = false,
    /// True while an async shader compile is in progress.
    shader_loading: bool = false,
    /// Last shader load error message to display in the settings window (owned).
    shader_error: ?[]u8 = null,
    /// Set to true to trigger loading the border shader preset on the next frame.
    should_load_border_shader: bool = false,
    /// Set to true to trigger clearing the border shader preset on the next frame.
    should_clear_border_shader: bool = false,
    /// True while an async border shader compile is in progress.
    border_shader_loading: bool = false,
    /// Last border shader load error message (owned).
    border_shader_error: ?[]u8 = null,

    settings: EmulatorSettings = .{},
    saved_settings: EmulatorSettings = .{},
    config_dir: ?[]u8 = null,
    controller_img: LoadedImage,
    emulation_thread: ?std.Thread = null,
    emulation_stop: std.atomic.Value(bool) = .init(false),
    emulation_lock: std.Thread.RwLock = .{},
    controller1_bits: std.atomic.Value(u8) = .init(0),
    controller2_bits: std.atomic.Value(u8) = .init(0),
    frame_buffers: [2]Frame = .{ Frame.init(), Frame.init() },
    published_frame_idx: std.atomic.Value(u8) = .init(0),
    ui_frame_idx: std.atomic.Value(u8) = .init(0),
    writing_frame_idx: std.atomic.Value(u8) = .init(NO_FRAME),
    render_frame_idx: u8 = 0,

    const Self = @This();
    pub const DebugSnapshot = struct {
        const CPU_MEMORY_SIZE = 0x10000;

        register_a: u8 = 0,
        register_x: u8 = 0,
        register_y: u8 = 0,
        pc: u16 = 0,
        sp: u8 = 0,
        status: ProcessorStatus = .{},
        scanline: u16 = 0,
        cycle: u16 = 0,
        global_cycle: u64 = 0,
        palette_ram: [32]u8 = [_]u8{0} ** 32,
        pattern_table_0: PatternTableFrame = .{},
        pattern_table_1: PatternTableFrame = .{},
        memory: [CPU_MEMORY_SIZE]u8 = [_]u8{0} ** CPU_MEMORY_SIZE,

        pub fn memPeek(self: *const @This(), addr: u16) u8 {
            return self.memory[addr];
        }

        pub fn memPeekU16(self: *const @This(), addr: u16) u16 {
            const lo = self.memPeek(addr);
            const hi = @as(u16, self.memPeek(addr +% 1));
            return (hi << 8) | lo;
        }

        fn capture(system: *const System) @This() {
            var snapshot: @This() = .{
                .register_a = system.cpu.register_a,
                .register_x = system.cpu.register_x,
                .register_y = system.cpu.register_y,
                .pc = system.cpu.pc,
                .sp = system.cpu.sp,
                .status = system.cpu.status,
                .scanline = system.ppu.scanline,
                .cycle = system.ppu.cycle,
                .global_cycle = system.ppu.global_cycle,
                .palette_ram = system.ppu.palette_table,
                .pattern_table_0 = system.ppu.get_pattern_table(0, 0),
                .pattern_table_1 = system.ppu.get_pattern_table(1, 0),
            };

            for (&snapshot.memory, 0..) |*value, addr| {
                value.* = system.cpu.bus.mem_peek(@intCast(addr));
            }

            return snapshot;
        }
    };

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
        /// Path to the active border shader preset (owned by this struct).
        border_shader_preset_path: ?[]u8 = null,
        /// Active border shader parameter overrides (names are owned by this struct).
        border_shader_params: std.ArrayList(ShaderParamSetting) = .{},
        /// Currently selected category in the settings sidebar.
        selected_category: SettingsCategory = .general,
        vsync: bool = true,
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
            .last_mouse_activity_time = c.SDL_GetTicks(),
        };
        state.loadSettings();
        state.snapshotSettings() catch @panic("Failed to snapshot loaded settings");
        return state;
    }

    pub fn deinit(self: *Self) void {
        self.stopEmulationThread();

        // Save before freeing the system.
        if (self.emulation_running) self.saveCurrentGame();
        self.history.deinit();

        if (self.config_dir) |path| self.alloc.free(path);
        if (self.current_rom_path) |p| self.alloc.free(p);
        if (self.rom_bytes) |rom_bytes| self.alloc.free(rom_bytes);
        deinitShaderRuntimeState(self.alloc, self);
        deinitEmulatorSettings(self.alloc, &self.settings);
        deinitEmulatorSettings(self.alloc, &self.saved_settings);
        c.SDL_DestroySurface(self.controller_img.raw);

        if (self.emulation_running) {
            self.rom.?.deinit();
            self.system.?.deinit();
        }
    }

    fn startEmulationThread(self: *Self) !void {
        self.stopEmulationThread();
        self.emulation_stop.store(false, .release);
        self.emulation_thread = try std.Thread.spawn(.{}, emulationThreadMain, .{self});
    }

    pub fn stopEmulationThread(self: *Self) void {
        if (self.emulation_thread) |thread| {
            self.emulation_stop.store(true, .release);
            thread.join();
            self.emulation_thread = null;
            self.emulation_stop.store(false, .release);
        }
    }

    fn emulationThreadMain(self: *Self) void {
        var frame_acc: f32 = 0.0;

        while (!self.emulation_stop.load(.acquire)) {
            self.emulation_lock.lock();
            const can_run = self.emulation_running and
                self.system != null and
                !self.lifecycle_suspended.load(.acquire) and
                !self.paused and
                !self.step_mode;
            if (can_run) {
                const speed = self.settings.emulation_speed;
                const multiplier = speed.multiplier();
                frame_acc += multiplier;
                const frames_to_run: u32 = @intFromFloat(frame_acc);
                frame_acc -= @floatFromInt(frames_to_run);

                self.system.?.apu.device.setSpeed(multiplier);
                for (0..frames_to_run) |_| {
                    self.system.?.applyControllerSnapshot(self.controllerSnapshot());
                    self.system.?.run_frame();
                    self.publishFrame(self.system.?.frame_buffer());
                }
            } else {
                frame_acc = 0.0;
            }
            self.emulation_lock.unlock();

            c.SDL_Delay(1);
        }
    }

    fn publishFrame(self: *Self, pixels: []const u8) void {
        while (true) {
            const read_idx = self.ui_frame_idx.load(.acquire);
            const write_idx: u8 = if (read_idx == 0) 1 else 0;

            self.writing_frame_idx.store(write_idx, .release);
            if (self.ui_frame_idx.load(.acquire) == write_idx) {
                self.writing_frame_idx.store(NO_FRAME, .release);
                continue;
            }

            @memcpy(self.frame_buffers[write_idx].data[0..], pixels);
            self.published_frame_idx.store(write_idx, .release);
            self.writing_frame_idx.store(NO_FRAME, .release);
            return;
        }
    }

    fn handleInput(self: *Self, ui: *UI) void {
        if (self.emulation_running) {
            const main_window_active = ui.current_window == ui.main_window;
            if (main_window_active) {
                self.syncControllers(ui);
                if (ui.isKeyPressed(self.generalBinding(.quit))) ui.quit = true;
                if (ui.isKeyPressed(self.generalBinding(.toggle_step_mode))) self.toggleDebug();
                if (ui.isKeyPressed(self.generalBinding(.restart))) self.resetSystem();
                if (ui.isKeyPressed(self.generalBinding(.toggle_pause))) self.togglePause();
                if (ui.isKeyPressed(self.generalBinding(.stop))) {
                    self.unloadCurrentRom();
                    ui.setWindowFullscreen(false);
                }
                if (self.step_mode and ui.isKeyPressed(self.generalBinding(.run_tick))) self.runSystemTick();
                if (self.step_mode and ui.isKeyPressed(self.generalBinding(.run_frame))) {
                    self.runSystemFrame();
                }
                if (ui.isKeyPressed(self.generalBinding(.toggle_fullscreen))) {
                    if (ui.isWindowFullscreen()) {
                        ui.setWindowFullscreen(false);
                    } else {
                        ui.setWindowFullscreen(true);
                    }
                }
            }

            if (ui.mouseMotion()) {
                self.last_mouse_activity_time = c.SDL_GetTicks();
                if (self.is_cursor_hidden) {
                    sdlError(c.SDL_ShowCursor());
                    self.is_cursor_hidden = false;
                }
            }

            if (self.settings.hide_mouse_on_inactivity and !self.render_debug_ui) {
                if (!self.is_cursor_hidden and ui.hasPassedSinceMS(self.last_mouse_activity_time, CURSOR_HIDE_DELAY_MS)) {
                    sdlError(c.SDL_HideCursor());
                    self.is_cursor_hidden = true;
                }
            } else if (self.is_cursor_hidden) { // Always show cursor if not in fullscreen
                sdlError(c.SDL_ShowCursor());
                self.is_cursor_hidden = false;
            }
        }
    }

    pub fn update(self: *Self, ui: *UI) void {
        self.handleInput(ui);

        while (true) {
            const idx = self.published_frame_idx.load(.acquire);
            self.ui_frame_idx.store(idx, .release);
            if (self.writing_frame_idx.load(.acquire) != idx) {
                self.render_frame_idx = idx;
                return;
            }
            std.atomic.spinLoopHint();
        }
    }

    pub fn framePixels(self: *Self, offset: usize, len: usize) []const u8 {
        return self.frame_buffers[self.render_frame_idx].data[offset..][0..len];
    }

    pub fn debugSnapshot(self: *Self) DebugSnapshot {
        self.emulation_lock.lockShared();
        defer self.emulation_lock.unlockShared();

        return DebugSnapshot.capture(&self.system.?);
    }

    pub fn syncControllers(self: *Self, ui: *UI) void {
        const snapshot = self.pollControllers(ui);
        self.controller1_bits.store(@bitCast(snapshot.player1), .release);
        self.controller2_bits.store(@bitCast(snapshot.player2), .release);
    }

    fn controllerSnapshot(self: *Self) System.ControllerSnapshot {
        return .{
            .player1 = @bitCast(self.controller1_bits.load(.acquire)),
            .player2 = @bitCast(self.controller2_bits.load(.acquire)),
        };
    }

    fn pollControllers(self: *Self, ui: *UI) System.ControllerSnapshot {
        return .{
            .player1 = self.pollController(ui, .one),
            .player2 = self.pollController(ui, .two),
        };
    }

    fn pollController(self: *Self, ui: *UI, player: ControllerPlayer) ControllerButton {
        var status: ControllerButton = .{};
        const key_bindings = self.settings.controller_bindings.forPlayerConst(player);

        inline for (@typeInfo(ControllerAction).@"enum".fields) |field| {
            const action = @field(ControllerAction, field.name);
            if (ui.isKeyDown(key_bindings.get(action))) {
                status.insert(action.button());
            }
        }

        if (ui.getGamepadCount() > player.value()) {
            self.pollGamepadButtons(ui, player, &status);
        }

        return status;
    }

    fn pollGamepadButtons(self: *Self, ui: *UI, player: ControllerPlayer, status: *ControllerButton) void {
        const gamepad_bindings = self.settings.gamepad_bindings.forPlayerConst(player);

        if (ui.isGamepadButtonDown(player, gamepad_bindings.a)) status.insert(.{ .BUTTON_A = true });
        if (ui.isGamepadButtonDown(player, gamepad_bindings.b)) status.insert(.{ .BUTTON_B = true });
        if (ui.isGamepadButtonDown(player, gamepad_bindings.start)) status.insert(.{ .START = true });
        if (ui.isGamepadButtonDown(player, gamepad_bindings.select)) status.insert(.{ .SELECT = true });
        if (ui.isGamepadButtonDown(player, gamepad_bindings.up)) status.insert(.{ .UP = true });
        if (ui.isGamepadButtonDown(player, gamepad_bindings.down)) status.insert(.{ .DOWN = true });
        if (ui.isGamepadButtonDown(player, gamepad_bindings.left)) status.insert(.{ .LEFT = true });
        if (ui.isGamepadButtonDown(player, gamepad_bindings.right)) status.insert(.{ .RIGHT = true });

        const threshold: i16 = @intCast(@as(u32, self.settings.gamepad_deadzone) * 32767 / 100);
        const lx = ui.getGamepadAxis(player, c.SDL_GAMEPAD_AXIS_LEFTX);
        const ly = ui.getGamepadAxis(player, c.SDL_GAMEPAD_AXIS_LEFTY);
        if (lx < -threshold) status.insert(.{ .LEFT = true });
        if (lx > threshold) status.insert(.{ .RIGHT = true });
        if (ly < -threshold) status.insert(.{ .UP = true });
        if (ly > threshold) status.insert(.{ .DOWN = true });
    }

    pub fn resetSystem(self: *Self) void {
        self.emulation_lock.lock();
        defer self.emulation_lock.unlock();
        self.system.?.reset();
    }

    pub fn runSystemTick(self: *Self) void {
        self.emulation_lock.lock();
        defer self.emulation_lock.unlock();
        self.system.?.tick();
    }

    pub fn runSystemFrame(self: *Self) void {
        self.emulation_lock.lock();
        defer self.emulation_lock.unlock();
        self.system.?.run_frame();
    }

    pub fn setEmulationSpeed(self: *Self, speed: EmulationSpeed) void {
        self.emulation_lock.lock();
        defer self.emulation_lock.unlock();
        self.settings.emulation_speed = speed;
    }

    pub fn saveStateSlot(self: *Self, slot: usize) void {
        const path = self.current_rom_path orelse return;
        self.emulation_lock.lock();
        defer self.emulation_lock.unlock();
        save_state.saveSlot(self.alloc, path, &self.system.?, slot) catch |err|
            std.log.err("save state slot {} failed: {s}", .{ slot + 1, @errorName(err) });
    }

    pub fn loadStateSlot(self: *Self, slot: usize) void {
        const path = self.current_rom_path orelse return;
        self.emulation_lock.lock();
        defer self.emulation_lock.unlock();
        save_state.loadSlot(self.alloc, path, &self.system.?, slot) catch |err| {
            std.log.err("load state slot {} failed: {s}", .{ slot + 1, @errorName(err) });
        };
    }

    pub fn saveStateSlotInfo(self: *Self, slot: usize) save_state.SlotInfo {
        const path = self.current_rom_path orelse return .{};
        return save_state.slotInfo(self.alloc, path, slot);
    }

    pub fn loadRom(self: *Self, path: []const u8) !void {
        // Save previous game's progress before replacing it.
        if (self.emulation_running) {
            self.stopEmulationThread();
            self.saveCurrentGame();
        }

        if (self.rom) |*rom| rom.deinit();
        if (self.system) |*system| system.deinit();

        const cwd = try std.process.getCwdAlloc(self.alloc);
        defer self.alloc.free(cwd);
        const rom_fullpath = try std.fs.path.resolve(self.alloc, &.{ cwd, path });
        defer self.alloc.free(rom_fullpath);

        const file = std.fs.openFileAbsolute(rom_fullpath, .{}) catch |err| switch (err) {
            else => {
                std.debug.print("Error while opening file: {any}\n", .{err});
                std.process.exit(1);
            },
        };
        defer file.close();

        const file_size = try file.getEndPos();
        try file.seekTo(0);

        std.log.debug("Reading file: {s}", .{rom_fullpath});
        if (self.rom_bytes) |bytes| self.alloc.free(bytes);
        self.rom_bytes = try self.alloc.alloc(u8, file_size);
        _ = try file.read(self.rom_bytes.?);

        self.rom = try Rom.init(self.alloc, rom_fullpath, self.rom_bytes.?);
        self.system = try System.init(self.alloc, &self.rom.?, .{});
        self.system.?.reset();
        self.publishFrame(self.system.?.frame_buffer());
        self.emulation_running = true;
        self.render_home_ui = false;

        if (self.current_rom_path) |p| self.alloc.free(p);
        self.current_rom_path = self.alloc.dupe(u8, rom_fullpath) catch null;
        self.game_start_time_ms = std.time.milliTimestamp();
        try self.startEmulationThread();
    }

    pub fn unloadCurrentRom(self: *Self) void {
        self.stopEmulationThread();
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

        const pixels = self.framePixels(OVERSCAN_PIXEL_OFFSET, NES_VISIBLE_PIXEL_BYTES);
        self.history.save(name, path, existing_secs + elapsed_secs, pixels);

        // Reset so back-to-back saves (loadRom then deinit) don't double-count.
        self.game_start_time_ms = std.time.milliTimestamp();
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

        self.emulation_lock.lock();
        defer self.emulation_lock.unlock();

        deinitEmulatorSettings(self.alloc, &self.settings);
        self.settings = restored;

        resetShaderRuntimeState(self);
    }

    fn loadSettings(self: *Self) void {
        settings.load(self.alloc, self.config_dir, &self.settings) catch |err|
            std.log.err("settings load failed: {s}", .{@errorName(err)});
        self.should_load_shader = self.settings.shader_preset_path != null;
        self.should_load_border_shader = self.settings.border_shader_preset_path != null;
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

    pub fn togglePause(self: *Self) void {
        self.emulation_lock.lock();
        defer self.emulation_lock.unlock();
        self.paused = !self.paused;
    }

    pub fn setLifecycleSuspended(self: *Self, suspended: bool) void {
        if (self.lifecycle_suspended.swap(suspended, .acq_rel) == suspended) return;

        if (suspended) {
            self.controller1_bits.store(0, .release);
            self.controller2_bits.store(0, .release);
        }

        if (self.system) |*system| {
            system.setAudioPaused(suspended);
        }
    }

    pub fn toggleDebug(self: *Self) void {
        self.emulation_lock.lock();
        defer self.emulation_lock.unlock();
        self.step_mode = !self.step_mode;
        self.render_debug_ui = !self.render_debug_ui;
    }

    pub fn toggleStepMode(self: *Self) void {
        self.emulation_lock.lock();
        defer self.emulation_lock.unlock();
        self.step_mode = !self.step_mode;
    }
};

fn clonePersistedSettings(
    alloc: std.mem.Allocator,
    source: AppState.EmulatorSettings,
) !AppState.EmulatorSettings {
    var result = AppState.EmulatorSettings{};
    errdefer deinitEmulatorSettings(alloc, &result);

    inline for (std.meta.fields(settings.SettingsConfig)) |field| {
        if (@hasField(AppState.EmulatorSettings, field.name)) {
            try cloneSettingsField(alloc, &@field(result, field.name), @field(source, field.name));
        }
    }

    return result;
}

fn cloneSettingsField(alloc: std.mem.Allocator, dest: anytype, source: anytype) !void {
    const Dest = @typeInfo(@TypeOf(dest)).pointer.child;
    const Source = @TypeOf(source);

    if (Dest == ?[]u8 and Source == ?[]u8) {
        if (source) |value| {
            dest.* = try alloc.dupe(u8, value);
        }
    } else if (Dest == std.ArrayList(ShaderParamSetting) and Source == std.ArrayList(ShaderParamSetting)) {
        try cloneShaderParamSettings(alloc, dest, source.items);
    } else {
        dest.* = source;
    }
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
) void {
    if (preset_path.*) |path| {
        alloc.free(path);
        preset_path.* = null;
    }
    settings.clearShaderParamSettings(alloc, params);
    params.deinit(alloc);
}

fn deinitEmulatorSettings(alloc: std.mem.Allocator, s: *AppState.EmulatorSettings) void {
    deinitShaderGroup(alloc, &s.shader_preset_path, &s.shader_params);
    deinitShaderGroup(alloc, &s.border_shader_preset_path, &s.border_shader_params);
}

fn resetShaderRuntimeState(app_state: *AppState) void {
    app_state.should_load_shader = app_state.settings.shader_preset_path != null;
    app_state.should_clear_shader = app_state.settings.shader_preset_path == null;
    app_state.shader_loading = false;
    if (app_state.shader_error) |old| {
        app_state.alloc.free(old);
        app_state.shader_error = null;
    }

    app_state.should_load_border_shader = app_state.settings.border_shader_preset_path != null;
    app_state.should_clear_border_shader = app_state.settings.border_shader_preset_path == null;
    app_state.border_shader_loading = false;
    if (app_state.border_shader_error) |old| {
        app_state.alloc.free(old);
        app_state.border_shader_error = null;
    }
}

fn deinitShaderRuntimeState(alloc: std.mem.Allocator, app_state: *AppState) void {
    if (app_state.shader_error) |msg| {
        alloc.free(msg);
        app_state.shader_error = null;
    }
    if (app_state.border_shader_error) |msg| {
        alloc.free(msg);
        app_state.border_shader_error = null;
    }
}

fn isSettingsEqual(a: AppState.EmulatorSettings, b: AppState.EmulatorSettings) bool {
    inline for (std.meta.fields(settings.SettingsConfig)) |field| {
        if (@hasField(AppState.EmulatorSettings, field.name)) {
            if (!settingsFieldEqual(@field(a, field.name), @field(b, field.name))) {
                return false;
            }
        }
    }

    return true;
}

fn settingsFieldEqual(a: anytype, b: @TypeOf(a)) bool {
    const T = @TypeOf(a);
    if (T == ?[]u8) {
        return optionalStringsEqual(a, b);
    } else if (T == std.ArrayList(ShaderParamSetting)) {
        return shaderParamSettingsEqual(a.items, b.items);
    } else {
        return std.meta.eql(a, b);
    }
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
