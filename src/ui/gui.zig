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
                        360,
                        120,
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
    const row = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .child_alignment = .{ .y = .center },
        .gap = 12,
    });
    {
        const label_col = ui.column(.{
            .sizing = .{ .w = .grow, .h = .fit },
            .gap = 3,
            .child_alignment = .{ .x = .left },
        });
        {
            _ = ui.label(.{
                .text = "ASPECT RATIO",
                .font_size = theme.LABEL_FONT,
                .color = theme.text_primary,
            });
            _ = ui.label(.{
                .text = "Controls how the NES frame is scaled to fill the window.",
                .font_size = 14,
                .color = theme.text_secondary,
            });
        }
        label_col.end();

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
    row.end();
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
