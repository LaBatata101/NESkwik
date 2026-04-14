const std = @import("std");

const debug = @import("debug.zig");
const c = @import("../root.zig").c;
const UI = @import("core/ui.zig").UI;
const clay = @import("core/clay.zig");
const Rom = @import("../rom.zig").Rom;
const utils = @import("core/utils.zig");
const theme = @import("common.zig").theme;
const Color = @import("core/color.zig").Color;
const System = @import("../system.zig").System;
const Shape = @import("core/widgets.zig").Shape;
const NES_WIDTH = @import("../root.zig").NES_WIDTH;
const NES_HEIGHT = @import("../root.zig").NES_HEIGHT;

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
    rom: Rom = undefined,
    system: System = undefined,

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
    };

    pub fn init(alloc: std.mem.Allocator) Self {
        return .{ .alloc = alloc };
    }

    pub fn deinit(self: *Self) void {
        if (self.selected_rom_filepath) |filepath| {
            self.alloc.free(filepath);
        }
        if (self.rom_bytes) |rom_bytes| {
            self.alloc.free(rom_bytes);
        }
        if (self.settings.shader_preset_path) |path| {
            self.alloc.free(path);
        }
        if (self.settings.shader_error) |msg| {
            self.alloc.free(msg);
        }

        if (self.emulation_running) { // TODO: add flag to check if a ROM was loaded
            self.rom.deinit();
            self.system.deinit();
        }
    }

    pub fn loadRom(self: *Self, path: []const u8, bytes: []u8) !void {
        self.rom = try Rom.init(self.alloc, path, bytes);
        self.system = try System.init(self.alloc, &self.rom, .{});
        self.system.reset();
        self.emulation_running = true;
        self.render_home_ui = false;
    }

    pub fn setSelectedRom(self: *Self, filepath: []const u8) void {
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

    const root = ui.column(.{});
    {
        if (!ui.isWindowFullscreen()) {
            const menubar = ui.menuBar(.{});
            {
                const sys_menu = ui.dropdownMenu(.{ .label = "System" });
                if (ui.menuItem(.{ .label = "Open" }).clicked(ui.main_window.ctx)) {
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

                const emulation_menu = ui.dropdownMenu(.{ .label = "Emulation" });
                if (ui.menuItem(.{ .label = "Debug" }).clicked(ui.main_window.ctx) and ui_state.emulation_running) {
                    ui_state.render_debug_ui = !ui_state.render_debug_ui;
                }

                if (ui.menuItem(.{ .label = "Settings" }).clicked(ui.main_window.ctx)) {
                    ui.createWindow(
                        "Settings",
                        400,
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
                .pixels = ui_state.system.frame_buffer(),
                .w = NES_WIDTH,
                .h = NES_HEIGHT,
                .aspect_ratio = ui_state.settings.aspect_ratio,
            });
        }
    }
    root.end();
}

fn drawHomeUI(ui: *UI, ui_state: *UIState) void {
    _ = ui_state; // autofix
    _ = ui.spacer(.{ .sizing = .grow });
}

fn drawSettingsWindowUI(ui: *UI, user_data: ?*anyopaque) void {
    const ui_state: *UIState = @ptrCast(@alignCast(user_data));
    const root = ui.column(.{
        .sizing = .grow,
        .bg_color = theme.bg_base,
        .gap = theme.PANEL_GAP,
        .child_alignment = .{ .x = .left, .y = .top },
    });
    {
        drawVideoSection(ui, ui_state);
        drawShaderSection(ui, ui_state);
    }
    root.end();
}

fn drawSectionHeader(ui: *UI, title: []const u8, accent: Color) void {
    const header = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .bg_color = theme.bg_section,
        .padding = theme.HEADER_PAD,
        .child_alignment = .{ .y = .center },
        .gap = 8,
        .border_width = 1,
        .border_color = theme.border_dim,
    });
    {
        _ = ui.shape(.{
            .vertices = &Shape.SQUARE,
            .sizing = .{ .w = .fixed(3), .h = .fixed(14) },
            .color = accent,
        });
        _ = ui.label(.{
            .text = title,
            .font_size = theme.LABEL_FONT,
            .color = theme.text_secondary,
        });
    }
    header.end();
}

fn drawVideoSection(ui: *UI, ui_state: *UIState) void {
    const col = ui.column(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .bg_color = theme.bg_panel,
        .gap = theme.PANEL_GAP,
        .child_alignment = .{ .y = .top },
    });
    {
        drawSectionHeader(ui, "VIDEO", theme.accent_blue);

        const body = ui.column(.{
            .sizing = .{ .w = .grow, .h = .fit },
            .bg_color = theme.bg_panel,
            .padding = theme.SECTION_PAD,
            .gap = 14,
        });
        {
            drawAspectRatioRow(ui, ui_state);
        }
        body.end();
    }
    col.end();
}

fn drawAspectRatioRow(ui: *UI, ui_state: *UIState) void {
    const col = ui.column(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .gap = 6,
        .child_alignment = .{ .x = .left },
    });
    {
        // Top row: label on the left, combobox pinned to the right.
        const header_row = ui.row(.{
            .sizing = .{ .w = .grow, .h = .fit },
            .child_alignment = .{ .y = .center },
            .gap = 8,
        });
        {
            _ = ui.label(.{
                .text = "ASPECT RATIO",
                .font_size = theme.LABEL_FONT,
                .color = theme.text_primary,
            });

            _ = ui.spacer(.{ .sizing = .grow });

            const aspect_ratio_opts = ui.combobox(utils.AspectRatio, .{
                .id = "aspect_ratio_combo",
                .selected = ui_state.settings.aspect_ratio,
                .options = &.{ .none, .@"4_3", .@"16_9" },
                .bg_color = theme.bg_section,
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
        header_row.end();

        _ = ui.label(.{
            .text = "Controls how the NES frame is scaled to fill the window.",
            .font_size = 14,
            .color = theme.text_secondary,
        });
    }
    col.end();
}

fn drawShaderSection(ui: *UI, ui_state: *UIState) void {
    const col = ui.column(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .bg_color = theme.bg_panel,
        .gap = theme.PANEL_GAP,
        .child_alignment = .{ .y = .top },
    });
    {
        drawSectionHeader(ui, "SHADER", theme.accent_purple);

        const body = ui.column(.{
            .sizing = .{ .w = .grow, .h = .fit },
            .bg_color = theme.bg_panel,
            .padding = theme.SECTION_PAD,
            .gap = 14,
        });
        {
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
        }
        body.end();

        // Parameters sub-section — only when a shader with params is loaded.
        const param_infos = ui.getShaderParamInfos();
        if (!ui_state.settings.shader_loading and param_infos.len > 0) {
            drawShaderParamsSection(ui, param_infos);
        }
    }
    col.end();
}

fn drawShaderParamsSection(ui: *UI, param_infos: []const @import("../shaders/pipeline.zig").ParamInfo) void {
    drawSectionHeader(ui, "PARAMETERS", theme.accent_purple);

    const scroll = ui.scrollArea(.{
        .id = "shader_params_scroll",
        .sizing = .{ .w = .grow, .h = .fixed(200) },
        .vertical = true,
        .padding = theme.SECTION_PAD,
        .gap = 12,
    });
    {
        for (param_infos, 0..) |info, i| {
            drawParamRow(ui, info, i);
        }
    }
    scroll.end();
}

fn drawShaderPresetRow(ui: *UI, ui_state: *UIState) void {
    const col = ui.column(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .gap = 6,
        .child_alignment = .{ .x = .left },
    });
    {
        // Top row: label on the left, buttons pinned to the right.
        const header_row = ui.row(.{
            .sizing = .{ .w = .grow, .h = .fit },
            .child_alignment = .{ .y = .center },
            .gap = 8,
        });
        {
            _ = ui.label(.{
                .text = "PRESET",
                .font_size = theme.LABEL_FONT,
                .color = theme.text_primary,
            });

            _ = ui.spacer(.{ .sizing = .grow });

            // "Load" button — opens file dialog
            if (ui.button(.{
                .text = "Load",
                .font_size = theme.LABEL_FONT,
                .text_color = theme.text_primary,
                .bg_color = theme.bg_section,
                .hover_color = theme.bg_hover,
                .padding = theme.HEADER_PAD,
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

            // "Clear" button — clears the active preset
            if (ui.button(.{
                .text = "Clear",
                .font_size = theme.LABEL_FONT,
                .text_color = theme.text_primary,
                .bg_color = theme.bg_section,
                .hover_color = theme.bg_hover,
                .padding = theme.HEADER_PAD,
            }).clicked(ui.current_window.ctx)) {
                if (ui_state.settings.shader_preset_path) |path| {
                    ui_state.alloc.free(path);
                    ui_state.settings.shader_preset_path = null;
                }
                ui_state.settings.should_clear_shader = true;
            }
        }
        header_row.end();

        // Filename on its own line so a long name never crowds the buttons.
        const active_path = ui.getShaderPresetPath();
        const preview_text = if (active_path) |p| blk: {
            const slash = std.mem.lastIndexOfScalar(u8, p, '/') orelse
                std.mem.lastIndexOfScalar(u8, p, '\\') orelse 0;
            break :blk if (slash > 0) p[slash + 1 ..] else p;
        } else "None";

        _ = ui.label(.{
            .text = preview_text,
            .font_size = 14,
            .color = theme.text_secondary,
        });
    }
    col.end();
}

fn drawParamRow(ui: *UI, info: @import("../shaders/pipeline.zig").ParamInfo, index: usize) void {
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

    const current = ui.getShaderParam(info.name);

    const widget_id = std.fmt.allocPrint(alloc, "param_widget_{d}", .{index}) catch info.name;

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
                .id = widget_id,
                .value = current != 0.0,
                .on_color = theme.accent_purple,
                .off_color = theme.text_muted,
                .thumb_color = theme.text_primary,
                .size = 18,
            });
            const new_val: f32 = if (tog.value()) 1.0 else 0.0;
            if (new_val != current) ui.setShaderParam(info.name, new_val);
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
            .id = widget_id,
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
        if (new_val != current) ui.setShaderParam(info.name, new_val);
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
