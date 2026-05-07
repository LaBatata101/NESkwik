const std = @import("std");

const game_history = @import("../game_history.zig");
const debug = @import("debug.zig");
const c = @import("../root.zig").c;
const sdlError = @import("../utils/sdl.zig").sdlError;
const UI = @import("core/ui.zig").UI;
const clay = @import("core/clay.zig");
const Rom = @import("../rom.zig").Rom;
const utils = @import("core/utils.zig");
const theme = @import("common.zig").theme;
const widgets = @import("core/widgets.zig");
const Color = @import("core/color.zig").Color;
const System = @import("../system.zig").System;
const pipeline = @import("../shaders/pipeline.zig");
const ness = @import("../root.zig");
const NES_WIDTH = ness.NES_WIDTH;
const NES_HEIGHT = ness.NES_HEIGHT;
const OVERSCAN_TOP = ness.OVERSCAN_TOP;
const OVERSCAN_BOTTOM = ness.OVERSCAN_BOTTOM;
const NES_VISIBLE_HEIGHT = ness.NES_VISIBLE_HEIGHT;
const OVERSCAN_PIXEL_OFFSET = OVERSCAN_TOP * NES_WIDTH * 4;
const NES_VISIBLE_PIXEL_BYTES = NES_WIDTH * NES_VISIBLE_HEIGHT * 4;

pub const UIState = struct {
    alloc: std.mem.Allocator,
    selected_rom_filepath: ?[]const u8 = null,
    /// Whether to load the selected ROM.
    should_load_rom: bool = false,
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

    settings: EmulatorSettings = .{},

    const Self = @This();
    const EmulatorSettings = struct {
        aspect_ratio: utils.AspectRatio = .@"4_3",
        /// Path to the active shader preset (owned by this struct).
        shader_preset_path: ?[]u8 = null,
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
    };

    pub fn init(alloc: std.mem.Allocator) Self {
        var hist = game_history.GameHistory.init(alloc);
        hist.load();
        return .{ .alloc = alloc, .history = hist };
    }

    pub fn deinit(self: *Self) void {
        // Save before freeing the system.
        if (self.emulation_running) self.saveCurrentGame();
        self.history.deinit();

        if (self.current_rom_path) |p| self.alloc.free(p);
        if (self.selected_rom_filepath) |filepath| self.alloc.free(filepath);
        if (self.rom_bytes) |rom_bytes| self.alloc.free(rom_bytes);
        if (self.settings.shader_preset_path) |path| self.alloc.free(path);
        if (self.settings.shader_error) |msg| self.alloc.free(msg);
        if (self.settings.border_shader_preset_path) |path| self.alloc.free(path);
        if (self.settings.border_shader_error) |msg| self.alloc.free(msg);

        if (self.emulation_running) { // TODO: add flag to check if a ROM was loaded
            self.rom.?.deinit();
            self.system.?.deinit();
        }
    }

    pub fn loadRom(self: *Self, path: []const u8, bytes: []u8) !void {
        // Save previous game's progress before replacing it.
        if (self.emulation_running) self.saveCurrentGame();

        if (self.rom) |*rom| rom.deinit();
        if (self.system) |*system| system.deinit();

        self.rom = try Rom.init(self.alloc, path, bytes);
        self.system = try System.init(self.alloc, &self.rom.?, .{});
        self.system.?.reset();
        self.emulation_running = true;
        self.render_home_ui = false;

        if (self.current_rom_path) |p| self.alloc.free(p);
        self.current_rom_path = self.alloc.dupe(u8, path) catch null;
        self.game_start_time_ms = std.time.milliTimestamp();
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

    pub fn setSelectedRom(self: *Self, filepath: []const u8) void {
        if (self.selected_rom_filepath) |path| self.alloc.free(path);
        self.selected_rom_filepath = self.alloc.dupe(u8, filepath) catch @panic("Failed to allocate!");
        self.should_load_rom = true;
    }

    pub fn getSelectedRom(self: *Self) []const u8 {
        self.should_load_rom = false;
        return self.selected_rom_filepath.?;
    }
};

const dialog_filter_list: [2]c.SDL_DialogFileFilter = [_]c.SDL_DialogFileFilter{
    .{ .name = "NES ROMs", .pattern = "nes" },
    .{ .name = "All files", .pattern = "*" },
};

pub fn drawGUI(ui: *UI, ui_state: *UIState) void {
    // Handle deferred shader preset load/clear requests.
    if (ui_state.settings.should_load_shader) {
        ui_state.settings.should_load_shader = false;
        if (ui_state.settings.shader_preset_path) |path| {
            if (ui_state.settings.shader_error) |old| {
                ui_state.alloc.free(old);
                ui_state.settings.shader_error = null;
            }
            ui.startShaderPreset(path) catch |err| {
                std.log.err("Failed to start shader load '{s}': {s}", .{ path, @errorName(err) });
                ui_state.settings.shader_error = std.fmt.allocPrint(
                    ui_state.alloc,
                    "Load failed: {s}",
                    .{@errorName(err)},
                ) catch null;
                // Clear the bad path so it doesn't show as "active".
                ui_state.alloc.free(path);
                ui_state.settings.shader_preset_path = null;
            };
            if (ui_state.settings.shader_error == null) {
                ui_state.settings.shader_loading = true;
            }
        }
    } else if (ui_state.settings.should_clear_shader) {
        ui_state.settings.should_clear_shader = false;
        ui.clearShaderPreset();
        ui_state.settings.shader_loading = false;
        if (ui_state.settings.shader_error) |old| {
            ui_state.alloc.free(old);
            ui_state.settings.shader_error = null;
        }
    }

    // Poll an in-progress async shader compile.
    if (ui_state.settings.shader_loading) {
        switch (ui.pollShaderLoad()) {
            .idle, .done => ui_state.settings.shader_loading = false,
            .compiling => {},
            .failed => |msg| {
                ui_state.settings.shader_loading = false;
                if (ui_state.settings.shader_error) |old| ui_state.alloc.free(old);
                ui_state.settings.shader_error = ui_state.alloc.dupe(u8, msg) catch null;
                if (ui_state.settings.shader_preset_path) |path| {
                    ui_state.alloc.free(path);
                    ui_state.settings.shader_preset_path = null;
                }
            },
        }
    }

    // Handle deferred border shader preset load/clear requests.
    if (ui_state.settings.should_load_border_shader) {
        ui_state.settings.should_load_border_shader = false;
        if (ui_state.settings.border_shader_preset_path) |path| {
            if (ui_state.settings.border_shader_error) |old| {
                ui_state.alloc.free(old);
                ui_state.settings.border_shader_error = null;
            }
            ui.startBorderShaderPreset(path) catch |err| {
                std.log.err("Failed to start border shader load '{s}': {s}", .{ path, @errorName(err) });
                ui_state.settings.border_shader_error = std.fmt.allocPrint(
                    ui_state.alloc,
                    "Load failed: {s}",
                    .{@errorName(err)},
                ) catch null;
                ui_state.alloc.free(path);
                ui_state.settings.border_shader_preset_path = null;
            };
            if (ui_state.settings.border_shader_error == null) {
                ui_state.settings.border_shader_loading = true;
            }
        }
    } else if (ui_state.settings.should_clear_border_shader) {
        ui_state.settings.should_clear_border_shader = false;
        ui.clearBorderShaderPreset();
        ui_state.settings.border_shader_loading = false;
        if (ui_state.settings.border_shader_error) |old| {
            ui_state.alloc.free(old);
            ui_state.settings.border_shader_error = null;
        }
    }

    // Poll an in-progress async border shader compile.
    if (ui_state.settings.border_shader_loading) {
        switch (ui.pollBorderShaderLoad()) {
            .idle, .done => ui_state.settings.border_shader_loading = false,
            .compiling => {},
            .failed => |msg| {
                ui_state.settings.border_shader_loading = false;
                if (ui_state.settings.border_shader_error) |old| ui_state.alloc.free(old);
                ui_state.settings.border_shader_error = ui_state.alloc.dupe(u8, msg) catch null;
                if (ui_state.settings.border_shader_preset_path) |path| {
                    ui_state.alloc.free(path);
                    ui_state.settings.border_shader_preset_path = null;
                }
            },
        }
    }

    const root = ui.column(.{});
    {
        if (!ui.isWindowFullscreen()) {
            const menubar = ui.menuBar(.{
                .bg_color = theme.bg_panel,
                .border_color = theme.border_dim,
            });
            {
                const sys_menu = ui.dropdownMenu(.{
                    .label = "System",
                    .bg_color = theme.bg_panel,
                    .hover_color = theme.bg_hover,
                    .text_color = theme.text_secondary,
                    .list_bg_color = theme.bg_section,
                    .list_border_color = theme.border,
                });
                if (ui.menuItem(.{
                    .label = "Open",
                    .bg_color = theme.bg_section,
                    .hover_color = theme.accent_blue,
                    .text_color = theme.text_secondary,
                }).clicked(ui.main_window.ctx)) {
                    const default_location = std.process.getCwdAlloc(ui.main_window.ctx.frameAlloc()) catch
                        @panic("Failed to allocate");
                    defer ui.main_window.ctx.frameAlloc().free(default_location);

                    c.SDL_ShowOpenFileDialog(
                        dialog_callback,
                        clay.anytypeToAnyopaquePtr(ui_state),
                        ui.main_window.ptr,
                        &dialog_filter_list,
                        dialog_filter_list.len,
                        default_location.ptr,
                        false,
                    );
                }
                sys_menu.end();

                const emulation_menu = ui.dropdownMenu(.{
                    .label = "Emulation",
                    .bg_color = theme.bg_panel,
                    .hover_color = theme.bg_hover,
                    .text_color = theme.text_secondary,
                    .list_bg_color = theme.bg_section,
                    .list_border_color = theme.border,
                });
                if (ui.menuItem(.{
                    .label = "Debug",
                    .enabled = ui_state.emulation_running,
                    .bg_color = theme.bg_section,
                    .hover_color = theme.accent_blue,
                    .text_color = theme.text_secondary,
                }).clicked(ui.main_window.ctx)) {
                    ui_state.render_debug_ui = !ui_state.render_debug_ui;
                }

                if (ui.menuItem(.{
                    .label = "Settings",
                    .bg_color = theme.bg_section,
                    .hover_color = theme.accent_blue,
                    .text_color = theme.text_secondary,
                }).clicked(ui.main_window.ctx)) {
                    ui.createWindow(
                        "Settings",
                        580,
                        480,
                        .{ .draw_fn = drawSettingsWindowUI, .draw_fn_data = @ptrCast(ui_state) },
                    );
                }
                emulation_menu.end();
            }
            menubar.end();
        }

        if (ui_state.render_home_ui) {
            drawHomeUI(ui, ui_state);
        } else if (ui_state.render_debug_ui) {
            debug.drawUI(ui, ui_state);
        } else {
            _ = ui.canvas(.{
                .pixel_format = c.SDL_PIXELFORMAT_ABGR8888,
                .pixels = ui_state.system.?.frame_buffer()[OVERSCAN_PIXEL_OFFSET..][0..NES_VISIBLE_PIXEL_BYTES],
                .w = NES_WIDTH,
                .h = NES_VISIBLE_HEIGHT,
                .aspect_ratio = ui_state.settings.aspect_ratio,
            });
        }
    }
    root.end();
}

fn openRomDialog(ui: *UI, ui_state: *UIState) void {
    const default_location = std.process.getCwdAlloc(ui.main_window.ctx.frameAlloc()) catch
        @panic("Failed to allocate");
    defer ui.main_window.ctx.frameAlloc().free(default_location);
    c.SDL_ShowOpenFileDialog(
        dialog_callback,
        clay.anytypeToAnyopaquePtr(ui_state),
        ui.main_window.ptr,
        &dialog_filter_list,
        dialog_filter_list.len,
        default_location.ptr,
        false,
    );
}

fn drawHomeUI(ui: *UI, ui_state: *UIState) void {
    const entries = ui_state.history.entries.items;

    const root = ui.column(.{
        .sizing = .grow,
        .bg_color = theme.bg_base,
        .child_alignment = .{ .x = .center, .y = .top },
    });
    {
        if (entries.len == 0) {
            // Empty state: open button.
            _ = ui.spacer(.{ .sizing = .grow });
            if (ui.button(.{
                .text = "Open ROM",
                .font_size = 15,
                .bg_color = theme.accent_blue,
                .text_color = Color.white,
                .padding = .{ .left = 28, .right = 28, .top = 10, .bottom = 10 },
                .corner_radius = 6,
                .elevation = 0,
            }).clicked(ui.main_window.ctx)) openRomDialog(ui, ui_state);
            _ = ui.spacer(.{ .sizing = .{ .w = .fixed(0), .h = .fixed(14) } });
            _ = ui.label(.{ .text = "or use System > Open", .font_size = 13, .color = theme.text_secondary });
            _ = ui.spacer(.{ .sizing = .grow });
        } else {
            const card_title_lines = maxGameCardTitleLines(ui, entries);
            const scroll = ui.scrollArea(.{ .sizing = .grow, .vertical = true });
            {
                const grid = ui.grid(.{
                    .id = "cards_grid",
                    .gap = 16,
                    .sizing = .grow,
                    .bg_color = theme.bg_base,
                    .child_alignment = .{ .x = .left, .y = .top },
                    .padding = .{ .left = 24, .right = 24, .top = 20, .bottom = 24 },
                });
                for (entries) |*entry| {
                    const slot = grid.item();
                    drawGameCard(ui, ui_state, entry, card_title_lines);
                    slot.end();
                }
                grid.end();
            }
            scroll.end();
        }
    }
    root.end();
}

const CARD_W: f32 = 200;
const CARD_THUMB_H: f32 = 175; // 200 * 224 / 256
const CARD_CONTENT_W: f32 = CARD_W - 20;
const CARD_TITLE_LINE_H: u16 = 16;
const CARD_TITLE_MAX_LINES: u16 = 3;
const CARD_PLAY_TIME_LINE_H: u16 = 14;
const CARD_CORNER_RADIUS: f32 = 6;
const PLACEHOLDER_THUMBNAIL_PIXEL = [_]u8{ 0, 0, 0, 255 };

fn cardTitleHeight(title_lines: u16) f32 {
    return @floatFromInt(@as(u32, CARD_TITLE_LINE_H) * @as(u32, title_lines));
}

fn cardInfoHeight(title_lines: u16) f32 {
    return 8 + cardTitleHeight(title_lines) + 4 + CARD_PLAY_TIME_LINE_H + 8;
}

fn cardHeight(title_lines: u16) f32 {
    return CARD_THUMB_H + cardInfoHeight(title_lines);
}

fn drawGameCard(ui: *UI, ui_state: *UIState, entry: *const game_history.GameEntry, title_lines: u16) void {
    const ctx = ui.main_window.ctx;
    const title_h = cardTitleHeight(title_lines);
    const info_h = cardInfoHeight(title_lines);

    const card = ui.column(.{
        .sizing = .{ .w = .fixed(CARD_W), .h = .fixed(cardHeight(title_lines)) },
        .bg_color = theme.bg_section,
        .hover_bg_color = theme.bg_hover,
        .corner_radius = CARD_CORNER_RADIUS,
        .border_width = 1,
        .border_color = theme.border_dim,
        .child_alignment = .{ .x = .left, .y = .top },
        .gap = 0,
    });
    {
        if (entry.thumbnail) |thumb| {
            _ = ui.canvas(.{
                .pixel_format = c.SDL_PIXELFORMAT_ABGR8888,
                .pixels = thumb,
                .w = game_history.THUMBNAIL_WIDTH,
                .h = game_history.THUMBNAIL_HEIGHT,
                .sizing = .{ .w = .fixed(CARD_W), .h = .fixed(CARD_THUMB_H) },
                .aspect_ratio = .none,
                .corner_radius = .{ .top_left = CARD_CORNER_RADIUS, .top_right = CARD_CORNER_RADIUS },
            });
        } else {
            _ = ui.canvas(.{
                .pixel_format = c.SDL_PIXELFORMAT_ABGR8888,
                .pixels = PLACEHOLDER_THUMBNAIL_PIXEL[0..],
                .w = 1,
                .h = 1,
                .sizing = .{ .w = .fixed(CARD_W), .h = .fixed(CARD_THUMB_H) },
                .bg_color = Color.rgb(10, 12, 16),
                .corner_radius = .{ .top_left = CARD_CORNER_RADIUS, .top_right = CARD_CORNER_RADIUS },
            });
        }

        const info = ui.column(.{
            .sizing = .{ .w = .fixed(CARD_W), .h = .fixed(info_h) },
            .padding = .{ .left = 10, .right = 10, .top = 8, .bottom = 8 },
            .gap = 4,
            .child_alignment = .{ .x = .left, .y = .top },
        });
        {
            const title = ui.scrollArea(.{
                .sizing = .{ .w = .fixed(CARD_CONTENT_W), .h = .fixed(title_h) },
                .vertical = true,
                .show_scrollbar = false,
            });
            {
                _ = ui.label(.{
                    .text = entry.name,
                    .font_size = 14,
                    .line_height = CARD_TITLE_LINE_H,
                    .color = theme.text_primary,
                });
            }
            title.end();
            _ = ui.label(.{
                .text = formatPlayTime(entry.play_time_secs, ctx.frameAlloc()),
                .font_size = 12,
                .line_height = CARD_PLAY_TIME_LINE_H,
                .color = theme.text_secondary,
            });
        }
        info.end();
    }
    card.end();

    if (clay.pointerOver(card.id) and ctx.frame.mouse_released) {
        ui_state.setSelectedRom(entry.rom_path);
    }
}

fn maxGameCardTitleLines(ui: *UI, entries: []const game_history.GameEntry) u16 {
    var max_lines: u16 = 1;
    for (entries) |entry| {
        max_lines = @max(max_lines, wrappedGameCardTitleLines(ui, entry.name));
    }
    return @min(max_lines, CARD_TITLE_MAX_LINES);
}

fn wrappedGameCardTitleLines(ui: *UI, title: []const u8) u16 {
    const space_w = measureGameCardTitleWidth(ui, " ");

    var lines: u16 = 1;
    var line_w: f32 = 0;
    var pos: usize = 0;

    while (pos < title.len) {
        while (pos < title.len and std.ascii.isWhitespace(title[pos])) : (pos += 1) {}
        if (pos >= title.len) break;

        const word_start = pos;
        while (pos < title.len and !std.ascii.isWhitespace(title[pos])) : (pos += 1) {}

        const word_w = measureGameCardTitleWidth(ui, title[word_start..pos]);
        if (line_w == 0) {
            line_w = word_w;
        } else if (line_w + space_w + word_w > CARD_CONTENT_W) {
            lines += 1;
            line_w = word_w;
        } else {
            line_w += space_w + word_w;
        }

        if (lines >= CARD_TITLE_MAX_LINES) return CARD_TITLE_MAX_LINES;
    }

    return lines;
}

fn measureGameCardTitleWidth(ui: *UI, text: []const u8) f32 {
    sdlError(c.TTF_SetFontSize(ui.font, @floatFromInt(14)));

    var w: c_int = 0;
    var h: c_int = 0;
    sdlError(c.TTF_GetStringSize(ui.font, text.ptr, text.len, &w, &h));

    return @floatFromInt(w);
}

fn formatPlayTime(secs: u64, alloc: std.mem.Allocator) []const u8 {
    if (secs < 60) return "< 1m";
    const minutes = secs / 60;
    const hours = minutes / 60;
    const mins_rem = minutes % 60;
    if (hours == 0) return std.fmt.allocPrint(alloc, "{d}m", .{minutes}) catch "?";
    if (mins_rem == 0) return std.fmt.allocPrint(alloc, "{d}h", .{hours}) catch "?";
    return std.fmt.allocPrint(alloc, "{d}h {d}m", .{ hours, mins_rem }) catch "?";
}

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

const SettingsCategory = enum {
    general,
    video,
    shader,

    fn displayName(self: @This()) []const u8 {
        return switch (self) {
            .general => "General",
            .video => "Video",
            .shader => "Shader",
        };
    }
};

const nav_active_bg = Color.rgb(28, 110, 90);
const nav_active_text = Color.rgb(210, 240, 232);
const nav_hover_bg = Color.rgb(35, 42, 52);

fn drawSettingsWindowUI(ui: *UI, user_data: ?*anyopaque) void {
    const ui_state: *UIState = @ptrCast(@alignCast(user_data));

    const root = ui.row(.{
        .sizing = .grow,
        .bg_color = theme.bg_base,
        .child_alignment = .{ .x = .left, .y = .top },
    });
    {
        drawSettingsSidebar(ui, ui_state);

        // Thin vertical divider
        const divider = ui.column(.{
            .sizing = .{ .w = .fixed(1), .h = .grow },
            .bg_color = theme.border_dim,
        });
        divider.end();

        drawSettingsContent(ui, ui_state);
    }
    root.end();
}

fn drawSettingsSidebar(ui: *UI, ui_state: *UIState) void {
    const sidebar = ui.column(.{
        .sizing = .{ .w = .fixed(130), .h = .grow },
        .bg_color = theme.bg_base,
        .padding = .{ .top = 6, .bottom = 6 },
        .gap = 0,
        .child_alignment = .{ .x = .left, .y = .top },
    });
    {
        const categories = [_]SettingsCategory{ .general, .video, .shader };
        for (categories) |category| {
            drawSidebarItem(ui, ui_state, category);
        }
    }
    sidebar.end();
}

fn drawSidebarItem(ui: *UI, ui_state: *UIState, category: SettingsCategory) void {
    const is_active = ui_state.settings.selected_category == category;
    if (ui.button(.{
        .text = category.displayName(),
        .font_size = 16,
        .text_color = if (is_active) nav_active_text else theme.text_secondary,
        .bg_color = if (is_active) nav_active_bg else theme.bg_base,
        .hover_color = if (is_active) nav_active_bg else nav_hover_bg,
        .padding = .{ .left = 14, .right = 14, .top = 9, .bottom = 9 },
        .corner_radius = 0,
        .elevation = 0,
        .sizing = .{ .w = .grow, .h = .fit },
        .text_alignment = .left,
    }).clicked(ui.current_window.ctx)) {
        ui_state.settings.selected_category = category;
    }
}

fn drawSettingsContent(ui: *UI, ui_state: *UIState) void {
    const scroll = ui.scrollArea(.{});
    {
        const content = ui.column(.{
            .sizing = .grow,
            .bg_color = theme.bg_panel,
            .child_alignment = .{ .x = .left, .y = .top },
            .padding = .all(5),
            .gap = 5,
        });

        {
            switch (ui_state.settings.selected_category) {
                .general => drawSettingsGeneralContent(ui, ui_state),
                .video => drawSettingsVideoContent(ui, ui_state),
                .shader => drawSettingsShaderContent(ui, ui_state),
            }
        }

        content.end();
    }
    scroll.end();
}

fn drawContentSectionHeader(ui: *UI, title: []const u8) void {
    _ = ui.label(.{
        .text = title,
        .font_size = 13,
        .color = theme.accent_green,
    });
}

fn drawContentSection(ui: *UI) *widgets.Container {
    return ui.column(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .bg_color = theme.bg_section,
        .padding = .{ .left = 14, .right = 14, .top = 12, .bottom = 12 },
        .gap = 12,
        .border_width = 1,
        .border_color = theme.border_dim,
        .corner_radius = 3,
        .child_alignment = .{ .x = .left, .y = .top },
    });
}

fn drawSettingsVideoContent(ui: *UI, ui_state: *UIState) void {
    drawContentSectionHeader(ui, "Display");
    {
        const section = drawContentSection(ui);
        drawAspectRatioRow(ui, ui_state);
        section.end();
    }
}

fn drawSettingsGeneralContent(ui: *UI, ui_state: *UIState) void {
    drawContentSectionHeader(ui, "General");
    const section = drawContentSection(ui);
    {
        const row = ui.row(.{
            .sizing = .{ .w = .grow, .h = .fit },
        });
        {
            _ = ui.label(.{
                .text = "Hide mouse on inactivity",
                .font_size = theme.LABEL_FONT,
                .color = theme.text_primary,
            });
            _ = ui.spacer(.{ .sizing = .grow });
            ui_state.settings.hide_mouse_on_inactivity = ui
                .toggle(.{ .value = ui_state.settings.hide_mouse_on_inactivity, .size = 22 })
                .value();
        }
        row.end();

        drawEmulationSpeedRow(ui, ui_state);
    }
    section.end();
}

fn drawEmulationSpeedRow(ui: *UI, ui_state: *UIState) void {
    const row = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .child_alignment = .{ .y = .center },
        .gap = 8,
    });
    {
        _ = ui.label(.{
            .text = "Emulation Speed",
            .font_size = theme.LABEL_FONT,
            .color = theme.text_primary,
        });

        _ = ui.spacer(.{ .sizing = .grow });

        const speed_opts = ui.combobox(EmulationSpeed, .{
            .selected = ui_state.settings.emulation_speed,
            .options = &.{ .half, .normal, .double, .triple, .quadruple },
            .bg_color = theme.bg_hover,
            .bg_color_on_hover = theme.bg_hover,
            .border_color = theme.border_dim,
            .border_color_on_open = theme.border_open,
            .border_color_on_hover = theme.border,
            .text_color = theme.text_primary,
            .float_panel = .{
                .bg_color = theme.bg_section,
                .border_color = theme.border_open,
            },
            .item = .{
                .bg_color_on_hover = theme.bg_hover,
                .bg_color = .{ .r = 0, .g = 0, .b = 0, .a = 0 },
                .text_color = theme.text_secondary,
                .text_color_on_hover = theme.text_accent,
            },
        });

        ui_state.settings.emulation_speed = speed_opts.selected();
    }
    row.end();
}

fn drawAspectRatioRow(ui: *UI, ui_state: *UIState) void {
    const row = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .child_alignment = .{ .y = .center },
        .gap = 8,
    });
    {
        _ = ui.label(.{
            .text = "Aspect Ratio",
            .font_size = theme.LABEL_FONT,
            .color = theme.text_primary,
        });

        _ = ui.spacer(.{ .sizing = .grow });

        const aspect_ratio_opts = ui.combobox(utils.AspectRatio, .{
            .id = "aspect_ratio_combo",
            .selected = ui_state.settings.aspect_ratio,
            .options = &.{ .none, .@"4_3", .@"16_9" },
            .bg_color = theme.bg_hover,
            .bg_color_on_hover = theme.bg_hover,
            .border_color = theme.border_dim,
            .border_color_on_open = theme.border_open,
            .border_color_on_hover = theme.border,
            .text_color = theme.text_primary,
            .float_panel = .{
                .bg_color = theme.bg_section,
                .border_color = theme.border_open,
            },
            .item = .{
                .bg_color_on_hover = theme.bg_hover,
                .bg_color = .{ .r = 0, .g = 0, .b = 0, .a = 0 },
                .text_color = theme.text_secondary,
                .text_color_on_hover = theme.text_accent,
            },
        });

        ui_state.settings.aspect_ratio = aspect_ratio_opts.selected();
    }
    row.end();
}

fn drawSettingsShaderContent(ui: *UI, ui_state: *UIState) void {
    drawContentSectionHeader(ui, "Shader");
    {
        const section = drawContentSection(ui);
        drawShaderPresetRow(ui, ui_state);
        if (ui_state.settings.shader_loading) {
            const progress = ui.getShaderProgress();
            const alloc = ui.current_window.ctx.frameAlloc();
            const progress_text = if (progress.total > 0)
                std.fmt.allocPrint(
                    alloc,
                    "Compiling... {d}/{d} passes",
                    .{ progress.completed, progress.total },
                ) catch "Compiling..."
            else
                "Compiling...";
            _ = ui.label(.{
                .text = progress_text,
                .font_size = 13,
                .color = theme.accent_purple,
            });
        } else if (ui_state.settings.shader_error) |err_msg| {
            _ = ui.label(.{
                .text = err_msg,
                .font_size = 13,
                .color = theme.accent_red,
            });
        }

        const param_infos = ui.getShaderParamInfos();
        if (!ui_state.settings.shader_loading and param_infos.len > 0) {
            drawShaderParamsSection(ui, "Parameters", param_infos, .main);
        }
        section.end();
    }

    drawContentSectionHeader(ui, "Border Shader");
    {
        const section = drawContentSection(ui);
        drawBorderShaderPresetRow(ui, ui_state);
        if (ui_state.settings.border_shader_loading) {
            _ = ui.label(.{
                .text = "Compiling...",
                .font_size = 13,
                .color = theme.accent_purple,
            });
        } else if (ui_state.settings.border_shader_error) |err_msg| {
            _ = ui.label(.{
                .text = err_msg,
                .font_size = 13,
                .color = theme.accent_red,
            });
        }

        const border_param_infos = ui.getBorderShaderParamInfos();
        if (!ui_state.settings.border_shader_loading and border_param_infos.len > 0) {
            drawShaderParamsSection(ui, "Border Parameters", border_param_infos, .border);
        }

        section.end();
    }
}

const ParamTarget = enum {
    main,
    border,
};

fn drawShaderParamsSection(
    ui: *UI,
    title: []const u8,
    param_infos: []const pipeline.ParamInfo,
    target: ParamTarget,
) void {
    drawContentSectionHeader(ui, title);

    const wrapper = ui.column(.{
        .sizing = .{ .w = .grow, .h = .fixed(220) },
        .bg_color = theme.bg_section,
        .border_width = 1,
        .border_color = theme.border_dim,
        .corner_radius = 3,
    });
    {
        const scroll = ui.scrollArea(.{
            .sizing = .grow,
            .vertical = true,
            .padding = .{ .left = 14, .right = 14, .top = 12, .bottom = 12 },
            .gap = 12,
        });
        {
            for (param_infos) |info| {
                drawParamRow(ui, info, target);
            }
        }
        scroll.end();
    }
    wrapper.end();
}

const PREVIEW_MAX_W: f32 = 320;
const PREVIEW_MAX_H: f32 = 270;
const PREVIEW_FRAME_PADDING: f32 = 2;
const PREVIEW_LABEL_HEIGHT: f32 = 22;
const black_pixel = [_]u8{ 0, 0, 0, 255 };

fn drawShaderPreview(
    ui: *UI,
    pixels: []const u8,
    px_w: u32,
    px_h: u32,
    shader: widgets.Canvas.ShaderPreview,
    aspect_ratio: utils.AspectRatio,
) void {
    const content_aspect = aspect_ratio.value() orelse @as(f32, @floatFromInt(4)) / 3;
    const preview_box_aspect = if (shader == .border)
        @max(content_aspect, @as(f32, 16.0) / 9.0)
    else
        content_aspect;
    const width_limited_height = PREVIEW_MAX_W / preview_box_aspect;
    const preview_w: f32 = if (width_limited_height <= PREVIEW_MAX_H)
        PREVIEW_MAX_W
    else
        PREVIEW_MAX_H * preview_box_aspect;
    const preview_h: f32 = if (width_limited_height <= PREVIEW_MAX_H)
        width_limited_height
    else
        PREVIEW_MAX_H;

    const wrapper = ui.column(.{
        .sizing = .{
            .w = .fixed(preview_w + PREVIEW_FRAME_PADDING),
            .h = .fixed(preview_h + PREVIEW_FRAME_PADDING + PREVIEW_LABEL_HEIGHT),
        },
        .bg_color = theme.border_dim,
        .padding = .all(1),
        .child_alignment = .center,
    });
    {
        _ = ui.label(.{ .text = "Preview", .color = theme.accent_green });
        _ = ui.canvas(.{
            .sizing = .grow,
            .pixel_format = c.SDL_PIXELFORMAT_ABGR8888,
            .pixels = pixels,
            .w = px_w,
            .h = px_h,
            .aspect_ratio = aspect_ratio,
            .shader_preview = shader,
        });
    }
    wrapper.end();
}

fn drawShaderPresetRow(ui: *UI, ui_state: *UIState) void {
    const can_clear_shader = ui_state.settings.shader_preset_path != null or ui_state.settings.shader_loading;

    const header_row = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .child_alignment = .{ .y = .center },
        .gap = 8,
    });
    {
        _ = ui.label(.{
            .text = "Shader",
            .font_size = theme.LABEL_FONT,
            .color = theme.text_primary,
        });

        _ = ui.spacer(.{ .sizing = .grow });

        if (ui.button(.{
            .text = "Load",
            .font_size = 15,
            .text_color = theme.text_primary,
            .bg_color = theme.bg_hover,
            .hover_color = theme.border,
            .padding = .{ .left = 10, .right = 10, .top = 5, .bottom = 5 },
            .elevation = 0,
        }).clicked(ui.current_window.ctx)) {
            const default_location = std.process.getCwdAlloc(ui.main_window.ctx.frameAlloc()) catch
                @panic("Failed to allocate");
            defer ui.main_window.ctx.frameAlloc().free(default_location);

            c.SDL_ShowOpenFileDialog(
                shader_dialog_callback,
                clay.anytypeToAnyopaquePtr(ui_state),
                ui.main_window.ptr,
                &shader_filter_list,
                shader_filter_list.len,
                default_location.ptr,
                false,
            );
        }

        if (ui.button(.{
            .text = "Clear",
            .font_size = 15,
            .enabled = can_clear_shader,
            .text_color = if (can_clear_shader) theme.text_primary else theme.text_secondary,
            .bg_color = if (can_clear_shader) theme.bg_hover else theme.bg_panel,
            .hover_color = if (can_clear_shader) theme.border else theme.bg_panel,
            .padding = .{ .left = 10, .right = 10, .top = 5, .bottom = 5 },
            .elevation = 0,
        }).clicked(ui.current_window.ctx)) {
            if (ui_state.settings.shader_preset_path) |path| {
                ui_state.alloc.free(path);
                ui_state.settings.shader_preset_path = null;
            }
            ui_state.settings.should_clear_shader = true;
        }
    }
    header_row.end();

    // Active preset filename.
    const active_path = ui.getShaderPresetPath();
    const filename = if (active_path) |p| blk: {
        const slash = std.mem.lastIndexOfScalar(u8, p, '/') orelse
            std.mem.lastIndexOfScalar(u8, p, '\\') orelse 0;
        break :blk if (slash > 0) p[slash + 1 ..] else p;
    } else "None";

    _ = ui.label(.{
        .text = filename,
        .font_size = 14,
        .color = theme.text_secondary,
    });

    if (ui_state.emulation_running) {
        const center = ui.row(.{ .sizing = .{ .w = .grow, .h = .fit }, .child_alignment = .{ .x = .center } });
        drawShaderPreview(ui, ui_state.system.?.frame_buffer()[OVERSCAN_PIXEL_OFFSET..][0..NES_VISIBLE_PIXEL_BYTES], NES_WIDTH, NES_VISIBLE_HEIGHT, .main, .none);
        center.end();
    }
}

fn getParamValue(ui: *UI, target: ParamTarget, name: []const u8) f32 {
    return switch (target) {
        .main => ui.getShaderParam(name),
        .border => ui.getBorderShaderParam(name),
    };
}

fn setParamValue(ui: *UI, target: ParamTarget, name: []const u8, value: f32) void {
    switch (target) {
        .main => ui.setShaderParam(name, value),
        .border => ui.setBorderShaderParam(name, value),
    }
}

fn drawParamRow(ui: *UI, info: pipeline.ParamInfo, target: ParamTarget) void {
    const alloc = ui.current_window.ctx.frameAlloc();

    // Section title: min == max == 0 (and step == 0 or null).
    // Shader authors use these entries to label groups of related params.
    const is_section = info.min == 0.0 and info.max == 0.0 and
        (info.step == null or info.step.? == 0.0);
    if (is_section) {
        const row = ui.row(.{
            .sizing = .{ .w = .grow, .h = .fit },
            .child_alignment = .{ .y = .center },
            .gap = 6,
        });
        {
            _ = ui.label(.{
                .text = info.display_name,
                .font_size = 12,
                .color = theme.accent_purple,
            });
            const sep = ui.row(.{ .sizing = .{ .w = .grow, .h = .fixed(1) } });
            sep.end();
            _ = ui.spacer(.{ .sizing = .{ .w = .fixed(0), .h = .fixed(0) } });
        }
        row.end();
        return;
    }

    // Toggle: range [0, 1] with step 1 — treat as boolean on/off switch.
    const is_toggle = info.min == 0.0 and info.max == 1.0 and
        info.step != null and info.step.? == 1.0;

    const current = getParamValue(ui, target, info.name);

    const row = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .child_alignment = .{ .y = .center },
        .gap = 8,
    });
    {
        _ = ui.label(.{
            .text = info.display_name,
            .font_size = 14,
            .color = theme.text_secondary,
        });
        _ = ui.spacer(.{ .sizing = .grow });

        if (is_toggle) {
            const tog = ui.toggle(.{
                .value = current != 0.0,
                .on_color = theme.accent_purple,
                .off_color = theme.text_muted,
                .thumb_color = theme.text_primary,
                .size = 18,
            });
            const new_val: f32 = if (tog.value()) 1.0 else 0.0;
            if (new_val != current) setParamValue(ui, target, info.name, new_val);
        } else {
            // Format value label: derive precision from step size.
            const decimals: usize = if (info.step) |s| (if (s >= 1.0) 0 else if (s >= 0.1) 1 else 2) else 2;
            const value_text = switch (decimals) {
                0 => std.fmt.allocPrint(alloc, "{d:.0}", .{current}) catch "?",
                1 => std.fmt.allocPrint(alloc, "{d:.1}", .{current}) catch "?",
                else => std.fmt.allocPrint(alloc, "{d:.2}", .{current}) catch "?",
            };
            _ = ui.label(.{
                .text = value_text,
                .font_size = 14,
                .color = theme.text_value,
            });
        }
    }
    row.end();

    if (!is_toggle) {
        // Draggable track bar below the label row.
        const drag = ui.slider(.{
            .value = current,
            .min = info.min,
            .max = info.max,
            .step = info.step,
            .height = 4,
            .fill_color = theme.accent_purple,
            .track_color = theme.bg_section,
            .thumb_color = theme.text_primary,
            .corner_radius = 2,
        });
        const new_val = drag.value();
        if (new_val != current) setParamValue(ui, target, info.name, new_val);
    }
}

const shader_filter_list: [2]c.SDL_DialogFileFilter = [_]c.SDL_DialogFileFilter{
    .{ .name = "RetroArch Shader Presets", .pattern = "slangp" },
    .{ .name = "All files", .pattern = "*" },
};

fn shader_dialog_callback(userdata: ?*anyopaque, filelist: [*c]const [*c]const u8, _: c_int) callconv(.c) void {
    const ui_state = clay.anyopaquePtrToType(*UIState, userdata);

    if (filelist == null) {
        std.debug.print("An error ocurred while selecting shader file: {s}\n", .{c.SDL_GetError()});
        return;
    }
    if (filelist.* == null) {
        return;
    }

    const filepath = std.mem.span(filelist.*);
    std.log.debug("User selected shader: {s}", .{filepath});

    if (ui_state.settings.shader_preset_path) |old_path| {
        ui_state.alloc.free(old_path);
    }
    ui_state.settings.shader_preset_path = ui_state.alloc.dupe(u8, filepath) catch @panic("Failed to allocate!");
    ui_state.settings.should_load_shader = true;
}

fn drawBorderShaderPresetRow(ui: *UI, ui_state: *UIState) void {
    const can_clear_border_shader = ui_state.settings.border_shader_preset_path != null or ui_state.settings.border_shader_loading;

    const header_row = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .child_alignment = .{ .y = .center },
        .gap = 8,
    });
    {
        _ = ui.label(.{
            .text = "Border Shader",
            .font_size = theme.LABEL_FONT,
            .color = theme.text_primary,
        });

        _ = ui.spacer(.{ .sizing = .grow });

        if (ui.button(.{
            .text = "Load",
            .font_size = 15,
            .text_color = theme.text_primary,
            .bg_color = theme.bg_hover,
            .hover_color = theme.border,
            .padding = .{ .left = 10, .right = 10, .top = 5, .bottom = 5 },
            .elevation = 0,
        }).clicked(ui.current_window.ctx)) {
            const default_location = std.process.getCwdAlloc(ui.main_window.ctx.frameAlloc()) catch
                @panic("Failed to allocate");
            defer ui.main_window.ctx.frameAlloc().free(default_location);

            c.SDL_ShowOpenFileDialog(
                border_shader_dialog_callback,
                clay.anytypeToAnyopaquePtr(ui_state),
                ui.main_window.ptr,
                &shader_filter_list,
                shader_filter_list.len,
                default_location.ptr,
                false,
            );
        }

        if (ui.button(.{
            .text = "Clear",
            .font_size = 15,
            .enabled = can_clear_border_shader,
            .text_color = if (can_clear_border_shader) theme.text_primary else theme.text_secondary,
            .bg_color = if (can_clear_border_shader) theme.bg_hover else theme.bg_panel,
            .hover_color = if (can_clear_border_shader) theme.border else theme.bg_panel,
            .padding = .{ .left = 10, .right = 10, .top = 5, .bottom = 5 },
            .elevation = 0,
        }).clicked(ui.current_window.ctx)) {
            if (ui_state.settings.border_shader_preset_path) |path| {
                ui_state.alloc.free(path);
                ui_state.settings.border_shader_preset_path = null;
            }
            ui_state.settings.should_clear_border_shader = true;
        }
    }
    header_row.end();

    // Active preset filename.
    const active_path = ui.getBorderShaderPresetPath();
    const filename = if (active_path) |p| blk: {
        const slash = std.mem.lastIndexOfScalar(u8, p, '/') orelse
            std.mem.lastIndexOfScalar(u8, p, '\\') orelse 0;
        break :blk if (slash > 0) p[slash + 1 ..] else p;
    } else "None";

    _ = ui.label(.{
        .text = filename,
        .font_size = 14,
        .color = theme.text_secondary,
    });

    const show_border_preview = ui_state.settings.border_shader_preset_path != null or
        ui_state.settings.border_shader_loading;
    if (show_border_preview) {
        const preview_aspect_ratio: utils.AspectRatio = switch (ui_state.settings.aspect_ratio) {
            .none => .@"4_3",
            else => ui_state.settings.aspect_ratio,
        };
        const preview_pixels: []const u8 = if (ui_state.emulation_running)
            ui_state.system.?.frame_buffer()[OVERSCAN_PIXEL_OFFSET..][0..NES_VISIBLE_PIXEL_BYTES]
        else
            &black_pixel;
        const px_w: u32 = if (ui_state.emulation_running) NES_WIDTH else 1;
        const px_h: u32 = if (ui_state.emulation_running) NES_VISIBLE_HEIGHT else 1;
        const center = ui.row(.{ .sizing = .{ .w = .grow, .h = .fit }, .child_alignment = .{ .x = .center } });
        drawShaderPreview(ui, preview_pixels, px_w, px_h, .border, preview_aspect_ratio);
        center.end();
    }
}

fn border_shader_dialog_callback(userdata: ?*anyopaque, filelist: [*c]const [*c]const u8, _: c_int) callconv(.c) void {
    const ui_state = clay.anyopaquePtrToType(*UIState, userdata);

    if (filelist == null) {
        std.debug.print("An error ocurred while selecting border shader file: {s}\n", .{c.SDL_GetError()});
        return;
    }
    if (filelist.* == null) {
        return;
    }

    const filepath = std.mem.span(filelist.*);
    std.log.debug("User selected border shader: {s}", .{filepath});

    if (ui_state.settings.border_shader_preset_path) |old_path| {
        ui_state.alloc.free(old_path);
    }
    ui_state.settings.border_shader_preset_path = ui_state.alloc.dupe(u8, filepath) catch @panic("Failed to allocate!");
    ui_state.settings.should_load_border_shader = true;
}

fn dialog_callback(userdata: ?*anyopaque, filelist: [*c]const [*c]const u8, _: c_int) callconv(.c) void {
    const ui_state = clay.anyopaquePtrToType(*UIState, userdata);

    if (filelist == null) {
        std.debug.print("An error ocurred while selecting the file: {s}\n", .{c.SDL_GetError()});
        return;
    }
    if (filelist.* == null) { // A pointer to NULL, the user either didn't choose any file or canceled the dialog.
        return;
    }

    const filepath = std.mem.span(filelist.*);
    std.log.debug("User selected file: {s}", .{filepath});
    ui_state.setSelectedRom(filepath);
}
