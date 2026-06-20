const std = @import("std");
const builtin = @import("builtin");

const game_history = @import("../game_history.zig");
const debug = @import("debug.zig");
const c = @import("../root.zig").c;
const ui_core = @import("core/ui.zig");
const UI = ui_core.UI;
const Key = ui_core.Key;
const clay = @import("core/clay.zig");
const viewport = @import("core/viewport.zig");
const theme = @import("common.zig").theme;
const widgets = @import("core/widgets.zig");
const Color = @import("core/color.zig").Color;
const android = @import("../utils/android.zig");
const pipeline = @import("../shaders/pipeline.zig");
const shader_download = @import("../shader_download.zig");
const bindings = @import("bindings.zig");
const settings = @import("settings.zig");
const utils = @import("../utils/misc.zig");
const save_state = @import("../save_state.zig");
const state = @import("state.zig");
const ness = @import("../root.zig");
const NES_WIDTH = ness.NES_WIDTH;
const NES_HEIGHT = ness.NES_HEIGHT;
const OVERSCAN_TOP = ness.OVERSCAN_TOP;
const OVERSCAN_BOTTOM = ness.OVERSCAN_BOTTOM;
const NES_VISIBLE_HEIGHT = ness.NES_VISIBLE_HEIGHT;
const OVERSCAN_PIXEL_OFFSET = OVERSCAN_TOP * NES_WIDTH * 4;
const NES_VISIBLE_PIXEL_BYTES = NES_WIDTH * NES_VISIBLE_HEIGHT * 4;

const ControllerPlayer = bindings.ControllerPlayer;
const ControllerAction = bindings.ControllerAction;
const ControllerBindingTarget = bindings.ControllerBindingTarget;
const GeneralAction = bindings.GeneralAction;
const GamepadButton = bindings.GamepadButton;
const ParamTarget = settings.ParamTarget;
const SettingsCategory = state.SettingsCategory;
pub const AppState = state.AppState;
const EmulationSpeed = settings.EmulationSpeed;

const dialog_filter_list: [2]c.SDL_DialogFileFilter = [_]c.SDL_DialogFileFilter{
    .{ .name = "NES ROMs", .pattern = "nes" },
    .{ .name = "All files", .pattern = "*" },
};

pub fn drawGUI(ui: *UI, app_state: *AppState) void {
    const root = ui.column(.{
        .bg_color = theme.bg_base,
    });
    {
        const safe_area_padding = ui.main_window.safeAreaPadding();

        if (builtin.abi.isAndroid()) {
            if (!ui.isWindowFullscreen()) drawAndroidHeader(ui, app_state);
        } else {
            if (!ui.isWindowFullscreen()) drawDesktopMenu(ui, app_state);
        }

        if (app_state.show_fps) {
            const f = ui.float(.{
                .attach_to = .to_element_with_id,
                .z_index = 5,
                .parentId = root.id.id,
                .attach_points = .{ .parent = .left_top, .element = .left_top },
                .offset = .{
                    .x = 5 + @as(f32, @floatFromInt(safe_area_padding.left)),
                    .y = 30 + @as(f32, @floatFromInt(safe_area_padding.bottom)),
                },
            });
            _ = ui.label(.{
                .text = std.fmt.allocPrint(
                    ui.main_window.ctx.frameAlloc(),
                    "FPS: {}\nSpeed: {}%",
                    .{ ui.fps_manager.getFPS(), app_state.currentEmulationSpeedPercent() },
                ) catch @panic("OOM"),
                .font_size = 20,
                .color = .red,
            });
            f.end();
        }

        if (builtin.abi.isAndroid() and app_state.show_android_settings_ui) {
            drawAndroidSettingsUI(ui, app_state, safe_area_padding);
        } else if (app_state.render_home_ui) {
            drawHomeUI(ui, app_state, safe_area_padding);
        } else if (app_state.render_debug_ui and !builtin.abi.isAndroid()) {
            debug.drawUI(ui, app_state);
        } else {
            const orientation: android.ScreenOrientation = if (builtin.abi.isAndroid())
                android.currentScreenOrientation().?
            else
                .unknown;
            const is_portrait = orientation == .portrait or orientation == .portrait_flipped;

            const canvas = ui.canvas(.{
                .pixel_format = c.SDL_PIXELFORMAT_ABGR8888,
                .pixels = app_state.framePixels(OVERSCAN_PIXEL_OFFSET, NES_VISIBLE_PIXEL_BYTES),
                .w = NES_WIDTH,
                .h = NES_VISIBLE_HEIGHT,
                .padding = if (builtin.abi.isAndroid()) safe_area_padding else .{},
                .aspect_ratio = if (is_portrait) .@"4_3" else app_state.settings.aspect_ratio,
                .viewport_alignment = if (is_portrait) .top else .center,
                .bg_color = Color.black,
                .apply_runtime_shaders = true,
            });

            if (builtin.abi.isAndroid()) {
                drawGamepad(ui, canvas.id, orientation);
            }

            if (app_state.paused) {
                const f = ui.float(.{
                    .attach_to = .to_element_with_id,
                    .attach_points = .{ .parent = .center_center, .element = .center_center },
                    .z_index = 5,
                    .sizing = .grow,
                    .parentId = canvas.id.id,
                });
                {
                    const col = ui.column(.{
                        .bg_color = Color.black.withAlpha(0.8),
                        .child_alignment = .center,
                    });
                    _ = ui.label(.{ .text = "Paused", .font_size = 42, .color = .white });
                    col.end();
                }
                f.end();
            }
        }

        if (app_state.show_save_state_toast) toast(ui, "State saved");
        if (app_state.show_load_state_toast) toast(ui, "State loaded");

        if (builtin.abi.isAndroid() and app_state.show_android_sidepanel) {
            drawAndroidSidepanel(ui, app_state, root.id);
        }
    }
    root.end();
}

fn toastTransition(state_: clay.TransitionData, _: clay.TransitionProperty) callconv(.c) clay.TransitionData {
    var s = state_;
    s.bounding_box.y += s.bounding_box.height;
    return s;
}

fn toast(ui: *UI, text: []const u8) void {
    const f = ui.float(.{
        .attach_to = .to_root,
        .attach_points = .{ .parent = .left_bottom, .element = .left_bottom },
        .z_index = 100,
        .sizing = .fit,
        .offset = .{ .x = 15, .y = -15 },
        .transition = .{
            .handler = clay.easeOut,
            .duration = 0.25,
            .properties = clay.TransitionProperty.position,
            .enter = .{
                .set_initial_state = toastTransition,
                .trigger = .trigger_on_first_parent_frame,
            },
            .exit = .{
                .set_final_state = toastTransition,
                .sibling_ordering = .natural_order,
            },
        },
    });
    {
        const col = ui.column(.{
            .bg_color = Color.black.withAlpha(0.8),
            .child_alignment = .center,
        });
        _ = ui.label(.{ .text = text, .font_size = 25, .color = .white });
        col.end();
    }
    f.end();
}

fn drawDesktopMenu(ui: *UI, app_state: *AppState) void {
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
            openRomDialog(ui, app_state);
        }
        if (ui.menuItem(.{
            .label = "Exit",
            .bg_color = theme.bg_section,
            .hover_color = theme.accent_blue,
            .text_color = theme.text_secondary,
            .shortcut = app_state.generalBinding(.quit).keyName(),
        }).clicked(ui.main_window.ctx)) {
            ui.quit = true;
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
            .label = if (app_state.paused) "Continue" else "Pause",
            .enabled = app_state.emulation_running,
            .bg_color = theme.bg_section,
            .hover_color = theme.accent_blue,
            .text_color = theme.text_secondary,
            .shortcut = app_state.generalBinding(.toggle_pause).keyName(),
        }).clicked(ui.main_window.ctx)) {
            app_state.togglePause();
        }
        if (ui.menuItem(.{
            .label = "Stop",
            .enabled = app_state.emulation_running,
            .bg_color = theme.bg_section,
            .hover_color = theme.accent_blue,
            .text_color = theme.text_secondary,
            .shortcut = app_state.generalBinding(.stop).keyName(),
        }).clicked(ui.main_window.ctx)) {
            app_state.unloadCurrentRom();
        }
        if (ui.menuItem(.{
            .label = "Restart",
            .enabled = app_state.emulation_running,
            .bg_color = theme.bg_section,
            .hover_color = theme.accent_blue,
            .text_color = theme.text_secondary,
            .shortcut = app_state.generalBinding(.restart).keyName(),
        }).clicked(ui.main_window.ctx)) {
            app_state.resetSystem();
        }

        _ = ui.separator(.{ .color = theme.border, .thickness = 2 });
        const save_state_item = ui.menuItem(.{
            .label = "Save State",
            .enabled = app_state.emulation_running,
            .bg_color = theme.bg_section,
            .hover_color = theme.accent_blue,
            .text_color = theme.text_secondary,
            .has_submenu = true,
        });
        {
            const save_state_submenu = save_state_item.submenu(.{
                .width = 270,
                .bg_color = theme.bg_section,
                .border_color = theme.border,
            });
            {
                drawStateSlotItems(ui, app_state, .save);
            }
            save_state_submenu.end();
        }
        save_state_item.end();

        const load_state_item = ui.menuItem(.{
            .label = "Load State",
            .enabled = app_state.emulation_running,
            .bg_color = theme.bg_section,
            .hover_color = theme.accent_blue,
            .text_color = theme.text_secondary,
            .has_submenu = true,
        });
        {
            const load_state_submenu = load_state_item.submenu(.{
                .width = 270,
                .bg_color = theme.bg_section,
                .border_color = theme.border,
            });
            {
                drawStateSlotItems(ui, app_state, .load);
            }
            load_state_submenu.end();
        }
        load_state_item.end();

        _ = ui.separator(.{ .color = theme.border, .thickness = 2 });
        if (ui.menuItem(.{
            .label = "Debug",
            .enabled = app_state.emulation_running,
            .bg_color = theme.bg_section,
            .hover_color = theme.accent_blue,
            .text_color = theme.text_secondary,
            .shortcut = app_state.generalBinding(.toggle_step_mode).keyName(),
        }).clicked(ui.main_window.ctx)) {
            app_state.toggleDebug();
        }
        if (ui.menuItem(.{
            .label = if (app_state.show_fps) "Hide FPS" else "Show FPS",
            .bg_color = theme.bg_section,
            .hover_color = theme.accent_blue,
            .text_color = theme.text_secondary,
        }).clicked(ui.main_window.ctx)) {
            app_state.show_fps = !app_state.show_fps;
        }
        _ = ui.separator(.{ .color = theme.border, .thickness = 2 });

        if (ui.menuItem(.{
            .label = "Settings",
            .bg_color = theme.bg_section,
            .hover_color = theme.accent_blue,
            .text_color = theme.text_secondary,
        }).clicked(ui.main_window.ctx)) {
            ui.createWindow(
                "Settings",
                680,
                640,
                .{ .draw_fn = drawSettingsWindowUI, .draw_fn_data = @ptrCast(app_state) },
            );
        }
        emulation_menu.end();
    }
    menubar.end();
}

fn drawAndroidHeader(ui: *UI, app_state: *AppState) void {
    const safe_padding = ui.main_window.safeAreaPadding();

    const header = ui.column(.{
        .bg_color = theme.bg_panel,
        .border_width = 1,
        .border_color = theme.border_dim,
        .sizing = .{ .h = .fit, .w = .grow },
    });
    {
        _ = ui.spacer(.{ .sizing = .{ .h = .fixed(@floatFromInt(safe_padding.top)) } });
        const inner = ui.row(.{
            .bg_color = theme.bg_panel,
            .padding = .{ .left = 8, .right = 12 },
            .gap = 8,
            .child_alignment = .{ .x = .left, .y = .center },
        });
        {
            if (ui.iconButton(.{
                .icon = ui.icons.get(.menu),
                .size = 28,
                .padding = .all(10),
                .bg_color = Color.transparent,
                .hover_color = theme.bg_hover,
                .tint = theme.text_primary,
                .corner_radius = 8,
            }).clicked(ui.main_window.ctx)) {
                app_state.show_android_sidepanel = true;
                // Set a timer of 250ms to avoid closing the sidepanel as soon as it's opened
                ui.setTimer("android_sidepanel", 250);
            }

            _ = ui.label(.{
                .text = "NESkwik",
                .font_size = 18,
                .line_height = 20,
                .color = theme.text_primary,
            });

            _ = ui.spacer(.{ .sizing = .{ .w = .grow } });

            if (!app_state.show_android_settings_ui) {
                if (ui.button(.{
                    .text = "Open ROM",
                    .font_size = 14,
                    .text_color = Color.white,
                    .bg_color = theme.accent_blue,
                    .hover_color = theme.accent_blue.lighten(0.12),
                    .padding = .{ .left = 14, .right = 14, .top = 9, .bottom = 9 },
                    .corner_radius = 6,
                    .elevation = 0,
                }).clicked(ui.main_window.ctx)) {
                    openRomDialog(ui, app_state);
                }
            }
        }
        inner.end();
    }
    header.end();
}

fn androidSidePanelTransitionState(state_: clay.TransitionData, _: clay.TransitionProperty) callconv(.c) clay.TransitionData {
    var s = state_;
    s.bounding_box.x -= s.bounding_box.width;
    return s;
}

fn drawAndroidSidepanel(ui: *UI, app_state: *AppState, root_id: clay.ElementId) void {
    const safe_padding = ui.main_window.safeAreaPadding();
    const dims = clay.getLayoutDimensions();
    const sidepanel_w = @min(340.0, dims.w * 0.86);
    const is_open = app_state.show_android_sidepanel;
    const sidepanel_transition = clay.TransitionElementConfig{
        .handler = clay.easeOut,
        .duration = 0.18,
        .properties = clay.TransitionProperty.position,
        .enter = .{
            .set_initial_state = androidSidePanelTransitionState,
            .trigger = .trigger_on_first_parent_frame,
        },
        .exit = .{
            .set_final_state = androidSidePanelTransitionState,
            .sibling_ordering = .natural_order,
        },
    };

    const sidepanel = ui.float(.{
        .id = "android_sidepanel",
        .attach_to = .to_element_with_id,
        .parentId = root_id.id,
        .attach_points = .{ .parent = .left_top, .element = .left_top },
        .z_index = 60,
        .sizing = .{ .w = .fixed(sidepanel_w), .h = .grow },
        .transition = sidepanel_transition,
        .no_render = !is_open,
    });
    {
        const col = ui.column(.{
            .sizing = .{ .w = .fixed(sidepanel_w), .h = .grow },
            .bg_color = theme.bg_panel,
            .border_width = 1,
            .border_color = theme.border,
            .child_alignment = .{ .x = .left, .y = .top },
            .padding = .{
                .top = safe_padding.top,
                .left = safe_padding.left,
                .right = safe_padding.right,
            },
        });
        {
            const scroll = ui.scrollArea(.{
                .sizing = .grow,
                .vertical = true,
                .padding = .{ .top = 8, .bottom = @intCast(@as(u16, safe_padding.bottom) + 8) },
            });
            {
                drawAndroidDrawerSectionLabel(ui, "Home");
                if (drawAndroidDrawerAction(ui, "Home", true)) {
                    app_state.show_android_settings_ui = false;
                    app_state.render_home_ui = true;
                    app_state.show_android_sidepanel = false;
                }
                if (drawAndroidDrawerAction(ui, "Open ROM", true)) {
                    app_state.show_android_sidepanel = false;
                    openRomDialog(ui, app_state);
                }

                drawAndroidDrawerSectionLabel(ui, "Emulation");
                if (drawAndroidDrawerAction(
                    ui,
                    if (app_state.paused) "Resume" else "Pause",
                    app_state.emulation_running,
                )) {
                    app_state.togglePause();
                    app_state.show_android_sidepanel = false;
                }
                if (drawAndroidDrawerAction(ui, "Restart", app_state.emulation_running)) {
                    app_state.resetSystem();
                    app_state.show_android_sidepanel = false;
                }
                if (drawAndroidDrawerAction(ui, "Stop", app_state.emulation_running)) {
                    app_state.unloadCurrentRom();
                    ui.setWindowFullscreen(false);
                }

                drawAndroidDrawerSectionLabel(ui, "State");
                if (drawAndroidDrawerAction(ui, "Save Slot 1", app_state.emulation_running)) {
                    app_state.saveStateSlot(0);
                    app_state.show_android_sidepanel = false;
                    app_state.show_save_state_toast = true;
                    ui.setTimer("save_state_toast", 1000);
                }
                if (drawAndroidDrawerAction(
                    ui,
                    "Load Slot 1",
                    app_state.emulation_running and app_state.saveStateSlotInfo(0) != null,
                )) {
                    app_state.loadStateSlot(0);
                    app_state.show_android_sidepanel = false;
                    app_state.show_load_state_toast = true;
                    ui.setTimer("load_state_toast", 1000);
                }

                drawAndroidDrawerSectionLabel(ui, "Tools");
                if (drawAndroidDrawerAction(ui, if (app_state.show_fps) "Hide FPS" else "Show FPS", true)) {
                    app_state.show_fps = !app_state.show_fps;
                    app_state.show_android_sidepanel = false;
                }

                if (drawAndroidDrawerAction(
                    ui,
                    std.fmt.allocPrint(
                        ui.main_window.ctx.frameAlloc(),
                        "Speed: {s}",
                        .{app_state.settings.emulation_speed.label()},
                    ) catch @panic("OOM"),
                    true,
                )) {
                    app_state.setEmulationSpeed(nextEmulationSpeed(app_state.settings.emulation_speed));
                }
                if (drawAndroidDrawerAction(ui, "Settings", true)) {
                    app_state.show_android_settings_ui = true;
                    app_state.render_home_ui = false;
                    app_state.show_android_sidepanel = false;
                    if (app_state.settings.selected_category != .shader) {
                        app_state.settings.selected_category = .video;
                    }
                }
                if (drawAndroidDrawerAction(ui, "Exit", true)) {
                    ui.quit = true;
                }
            }
            scroll.end();
        }
        col.end();
    }
    sidepanel.end();

    // close the sidepanel when a click happens outside of it
    if (is_open and !clay.pointerOver(sidepanel.id) and ui.current_window.ctx.frame.mouse_down) {
        if (ui.hasTimerExpired("android_sidepanel")) app_state.show_android_sidepanel = false;
    }
}

fn drawAndroidDrawerSectionLabel(ui: *UI, text: []const u8) void {
    const row = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fixed(34) },
        .padding = .{ .left = 20, .right = 20, .top = 12, .bottom = 4 },
        .child_alignment = .{ .x = .left, .y = .center },
    });
    {
        _ = ui.label(.{ .text = text, .font_size = 12, .line_height = 14, .color = theme.text_accent });
    }
    row.end();
}

fn drawAndroidDrawerAction(ui: *UI, text: []const u8, enabled: bool) bool {
    return ui.button(.{
        .text = text,
        .font_size = 16,
        .text_color = if (enabled) theme.text_primary else theme.text_muted,
        .bg_color = theme.bg_panel,
        .hover_color = theme.bg_hover,
        .padding = .{ .left = 20, .right = 20, .top = 14, .bottom = 14 },
        .corner_radius = 0,
        .elevation = 0,
        .sizing = .{ .w = .grow, .h = .fit },
        .text_alignment = .left,
        .enabled = enabled,
    }).clicked(ui.main_window.ctx);
}

fn nextEmulationSpeed(speed: EmulationSpeed) EmulationSpeed {
    return switch (speed) {
        .half => .normal,
        .normal => .double,
        .double => .triple,
        .triple => .quadruple,
        .quadruple => .half,
    };
}

fn drawAndroidSettingsUI(ui: *UI, app_state: *AppState, safe_area_padding: clay.Padding) void {
    const root = ui.column(.{
        .sizing = .grow,
        .bg_color = theme.bg_base,
        .child_alignment = .{ .x = .left, .y = .top },
        .padding = .{
            .bottom = safe_area_padding.bottom,
            .left = safe_area_padding.left,
            .right = safe_area_padding.right,
        },
    });
    {
        const col = ui.column(.{ .bg_color = theme.bg_panel, .sizing = .{ .w = .grow, .h = .fit } });
        {
            if (app_state.emulation_running) {
                _ = ui.spacer(.{ .sizing = .{ .h = .fixed(@floatFromInt(safe_area_padding.top)) } });
            }
            const tabs = ui.row(.{
                .sizing = .{ .w = .grow, .h = .fit },
                .padding = .{ .left = 12, .right = 12, .top = 10, .bottom = 10 },
                .gap = 8,
                .child_alignment = .{ .x = .left, .y = .center },
            });
            {
                drawAndroidSettingsTab(ui, app_state, .video);
                drawAndroidSettingsTab(ui, app_state, .shader);
            }
            tabs.end();
        }
        col.end();

        const scroll = ui.scrollArea(.{
            .sizing = .grow,
            .vertical = true,
            .padding = .{ .left = 10, .right = 10, .top = 10, .bottom = 10 },
            .gap = 8,
        });
        {
            const body = ui.column(.{
                .sizing = .{ .w = .grow, .h = .fit },
                .gap = 8,
                .child_alignment = .{ .x = .left, .y = .top },
            });
            {
                switch (app_state.settings.selected_category) {
                    .shader => drawSettingsShaderContent(ui, app_state),
                    else => drawSettingsVideoContent(ui, app_state),
                }
            }
            body.end();
        }
        scroll.end();

        const footer = ui.row(.{
            .sizing = .{ .w = .grow, .h = .fit },
            .bg_color = theme.bg_panel,
            .padding = .{ .left = 12, .right = 12, .top = 10, .bottom = 10 },
            .gap = 8,
            .child_alignment = .{ .x = .right, .y = .center },
        });
        {
            if (ui.button(.{
                .text = "Library",
                .font_size = 15,
                .text_color = theme.text_primary,
                .bg_color = theme.bg_hover,
                .hover_color = theme.border,
                .padding = .{ .left = 16, .right = 16, .top = 9, .bottom = 9 },
                .corner_radius = 6,
                .elevation = 0,
            }).clicked(ui.main_window.ctx)) {
                app_state.show_android_settings_ui = false;
                app_state.render_home_ui = true;
            }

            _ = ui.spacer(.{ .sizing = .grow });

            if (ui.button(.{
                .text = "Save",
                .font_size = 15,
                .text_color = Color.white,
                .bg_color = theme.accent_blue,
                .hover_color = theme.accent_blue.lighten(0.12),
                .padding = .{ .left = 18, .right = 18, .top = 9, .bottom = 9 },
                .corner_radius = 6,
                .elevation = 0,
            }).clicked(ui.main_window.ctx)) {
                app_state.show_android_settings_ui = false;
                app_state.render_home_ui = !app_state.emulation_running;

                app_state.saveSettings();
                ui.setVSync(app_state.settings.vsync);
            }
        }
        footer.end();

        if (app_state.show_custom_file_picker) {
            drawAndroidShaderFilePicker(ui, app_state, root.id);
        }
    }
    root.end();
}

fn drawAndroidSettingsTab(ui: *UI, app_state: *AppState, category: SettingsCategory) void {
    const is_active = app_state.settings.selected_category == category;
    if (ui.button(.{
        .text = category.displayName(),
        .font_size = 15,
        .text_color = if (is_active) Color.white else theme.text_secondary,
        .bg_color = if (is_active) theme.accent_blue else theme.bg_hover,
        .hover_color = if (is_active) theme.accent_blue.lighten(0.1) else theme.border,
        .padding = .{ .left = 16, .right = 16, .top = 9, .bottom = 9 },
        .corner_radius = 6,
        .elevation = 0,
    }).clicked(ui.main_window.ctx)) {
        app_state.settings.selected_category = category;
    }
}

fn drawAndroidShaderFilePicker(ui: *UI, app_state: *AppState, root_id: clay.ElementId) void {
    const overlay = ui.float(.{
        .attach_to = .to_element_with_id,
        .parentId = root_id.id,
        .attach_points = .{ .parent = .left_top, .element = .left_top },
        .z_index = 90,
        .sizing = .grow,
    });
    {
        const scrim = ui.column(.{
            .sizing = .grow,
            .bg_color = Color.black.withAlpha(0.72),
            .padding = .{ .left = 14, .right = 14, .top = 18, .bottom = 18 },
            .child_alignment = .center,
        });
        {
            const max_panel_w = @min(560.0, ui.main_window.logical_width - 28.0);
            const max_panel_h = @min(620.0, ui.main_window.logical_height - 36.0);
            const panel = ui.column(.{
                .sizing = .{
                    .w = .growMinMax(.{ .max = max_panel_w }),
                    .h = .fitMinMax(.{ .max = max_panel_h }),
                },
                .bg_color = theme.bg_section,
                .border_width = 1,
                .border_color = theme.border,
                .corner_radius = 4,
                .padding = .{ .left = 12, .right = 12, .top = 12, .bottom = 12 },
                .gap = 10,
                .child_alignment = .{ .x = .left, .y = .top },
            });
            {
                drawAndroidShaderFilePickerHeader(ui, app_state);
                drawAndroidShaderFilePickerBody(ui, app_state);
            }
            panel.end();
        }
        scrim.end();
    }
    overlay.end();
}

fn drawAndroidShaderFilePickerHeader(ui: *UI, app_state: *AppState) void {
    const row = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .gap = 8,
        .child_alignment = .{ .y = .center },
    });
    {
        _ = ui.label(.{
            .text = "Load Shader",
            .font_size = 18,
            .color = theme.text_primary,
        });
        _ = ui.spacer(.{ .sizing = .grow });
        if (ui.button(.{
            .text = "Close",
            .font_size = 15,
            .text_color = theme.text_primary,
            .bg_color = theme.bg_hover,
            .hover_color = theme.border,
            .padding = .{ .left = 10, .right = 10, .top = 6, .bottom = 6 },
            .corner_radius = 3,
            .elevation = 0,
        }).clicked(ui.main_window.ctx)) {
            app_state.closeShaderFilePicker();
        }
    }
    row.end();
}

fn drawAndroidShaderFilePickerBody(ui: *UI, app_state: *AppState) void {
    const frame_alloc = ui.current_window.ctx.frameAlloc();

    if (app_state.shader_file_picker_error) |msg| {
        const text = frame_alloc.dupe(u8, msg) catch @panic("OOM");
        _ = ui.label(.{
            .text = text,
            .font_size = 14,
            .color = theme.accent_red,
        });
        return;
    }

    const entries = app_state.shaderFilePickerEntries();
    const current_dir = app_state.shader_file_picker_current_dir;
    _ = ui.label(.{
        .text = if (current_dir.len == 0) "/" else frame_alloc.dupe(u8, current_dir) catch
            @panic("OOM"),
        .font_size = 13,
        .color = theme.text_secondary,
    });

    const scroll = ui.scrollArea(.{
        .sizing = .{ .w = .grow, .h = .fitMinMax(.{ .max = 520 }) },
        .gap = 5,
        .vertical = true,
        .padding = .{ .right = 4 },
    });
    blk: {
        if (app_state.shaderFilePickerCanGoUp()) {
            if (ui.button(.{
                .text = "../",
                .font_size = 14,
                .text_color = theme.text_primary,
                .bg_color = theme.bg_panel,
                .hover_color = theme.bg_hover,
                .padding = .{ .left = 10, .right = 10, .top = 8, .bottom = 8 },
                .corner_radius = 2,
                .elevation = 0,
                .sizing = .{ .w = .grow, .h = .fit },
                .text_alignment = .left,
            }).clicked(ui.main_window.ctx)) {
                app_state.shaderFilePickerGoUp();
                break :blk;
            }
        }

        if (entries.len == 0) {
            _ = ui.label(.{
                .text = "No folders or .slangp files found",
                .font_size = 14,
                .color = theme.text_secondary,
            });
        } else {
            for (entries, 0..) |entry, idx| {
                const is_dir = entry.kind == .directory;
                if (ui.button(.{
                    .text = frame_alloc.dupe(u8, entry.label) catch @panic("OOM"),
                    .font_size = 14,
                    .text_color = if (is_dir) theme.text_accent else theme.text_primary,
                    .bg_color = theme.bg_panel,
                    .hover_color = theme.bg_hover,
                    .padding = .{ .left = 10, .right = 10, .top = 8, .bottom = 8 },
                    .corner_radius = 2,
                    .elevation = 0,
                    .sizing = .{ .w = .grow, .h = .fit },
                    .text_alignment = .left,
                }).clicked(ui.main_window.ctx)) {
                    if (is_dir) {
                        app_state.openShaderFilePickerEntry(idx);
                    } else {
                        app_state.selectShaderFilePickerEntry(idx);
                    }
                    break :blk;
                }
            }
        }
    }
    scroll.end();
}

fn drawGamepad(ui: *UI, parent_id: clay.ElementId, screen_orientation: android.ScreenOrientation) void {
    const bottom_offset = -22;
    const arrows = ui.float(.{
        .attach_to = .to_element_with_id,
        .parentId = parent_id.id,
        .z_index = 1,
        .attach_points = .{ .parent = .left_bottom, .element = .left_bottom },
        .offset = .{ .x = 18, .y = bottom_offset - 6 },
    });
    {
        const col = ui.column(.{
            .child_alignment = .center,
            .sizing = .{ .w = .fixed(162), .h = .fixed(162) },
        });
        {
            const up_button = controllerButton(ui, "U", .{ .w = .fixed(54), .h = .fixed(54) });
            if (up_button.clickedOrHold(ui.main_window.ctx)) ui.pressOnScreenControllerButton(.up);

            const row = ui.row(.{ .gap = 54 });
            {
                const left_button = controllerButton(ui, "L", .{ .w = .fixed(54), .h = .fixed(54) });
                if (left_button.clickedOrHold(ui.main_window.ctx)) ui.pressOnScreenControllerButton(.left);

                const right_button = controllerButton(ui, "R", .{ .w = .fixed(54), .h = .fixed(54) });
                if (right_button.clickedOrHold(ui.main_window.ctx)) ui.pressOnScreenControllerButton(.right);
            }
            row.end();

            const down_button = controllerButton(ui, "D", .{ .w = .fixed(54), .h = .fixed(54) });
            if (down_button.clickedOrHold(ui.main_window.ctx)) ui.pressOnScreenControllerButton(.down);
        }
        col.end();
    }
    arrows.end();

    const buttons1 = ui.float(.{
        .attach_to = .to_element_with_id,
        .parentId = parent_id.id,
        .z_index = 1,
        .attach_points = .{ .parent = .center_bottom, .element = .center_bottom },
        .offset = .{ .x = 0, .y = if (screen_orientation == .portrait or screen_orientation == .portrait_flipped)
            bottom_offset - 182
        else
            bottom_offset },
    });
    {
        const row = ui.row(.{ .sizing = .fit, .gap = 10 });
        {
            const start_button = ui.button(.{
                .text = "START",
                .font_size = 13,
                .text_color = theme.text_primary,
                .bg_color = Color.black.withAlpha(0.58),
                .hover_color = theme.bg_hover.withAlpha(0.82),
                .padding = .{ .left = 14, .right = 14, .top = 10, .bottom = 10 },
                .corner_radius = 7,
                .elevation = 0,
                .border_width = 1,
                .border = .{ .color = Color.white.withAlpha(0.28).toClay(), .width = .outside(1) },
            });
            if (start_button.clickedOrHold(ui.main_window.ctx)) ui.pressOnScreenControllerButton(.start);

            const select_button = ui.button(.{
                .text = "SELECT",
                .font_size = 13,
                .text_color = theme.text_primary,
                .bg_color = Color.black.withAlpha(0.58),
                .hover_color = theme.bg_hover.withAlpha(0.82),
                .padding = .{ .left = 14, .right = 14, .top = 10, .bottom = 10 },
                .corner_radius = 7,
                .elevation = 0,
                .border_width = 1,
                .border = .{ .color = Color.white.withAlpha(0.28).toClay(), .width = .outside(1) },
            });
            if (select_button.clickedOrHold(ui.main_window.ctx)) ui.pressOnScreenControllerButton(.select);
        }
        row.end();
    }
    buttons1.end();

    const buttons2 = ui.float(.{
        .attach_to = .to_element_with_id,
        .parentId = parent_id.id,
        .z_index = 1,
        .attach_points = .{ .parent = .right_bottom, .element = .right_bottom },
        .offset = .{ .x = -18, .y = bottom_offset - 4 },
    });
    {
        const row = ui.row(.{ .sizing = .fit, .gap = 16 });
        {
            const b_button = controllerButton(ui, "B", .{ .w = .fixed(62), .h = .fixed(62) });
            if (b_button.clickedOrHold(ui.main_window.ctx)) ui.pressOnScreenControllerButton(.b);

            const a_button = controllerButton(ui, "A", .{ .w = .fixed(62), .h = .fixed(62) });
            if (a_button.clickedOrHold(ui.main_window.ctx)) ui.pressOnScreenControllerButton(.a);
        }
        row.end();
    }
    buttons2.end();
}

fn controllerButton(ui: *UI, text: []const u8, sizing: clay.Sizing) *widgets.Button {
    return ui.button(.{
        .text = text,
        .font_size = 18,
        .text_color = theme.text_primary,
        .sizing = sizing,
        .bg_color = Color.black.withAlpha(0.52),
        .hover_color = theme.accent_blue.withAlpha(0.72),
        .border_width = 1,
        .border = .{ .color = Color.white.withAlpha(0.28).toClay(), .width = .outside(1) },
        .corner_radius = 10,
        .padding = .all(0),
        .elevation = 0,
    });
}

const SaveStateMenuMode = enum { save, load };

fn drawStateSlotItems(ui: *UI, app_state: *AppState, mode: SaveStateMenuMode) void {
    for (0..save_state.SLOT_COUNT) |slot| {
        const slot_info = app_state.saveStateSlotInfo(slot);
        const enabled = mode == .save or slot_info != null;
        const label = if (slot_info) |info|
            std.fmt.allocPrint(
                ui.main_window.ctx.frameAlloc(),
                "Slot {d} - {s}",
                .{ slot + 1, &info.display_time },
            ) catch @panic("OOM")
        else
            std.fmt.allocPrint(
                ui.main_window.ctx.frameAlloc(),
                "Slot {d}",
                .{slot + 1},
            ) catch @panic("OOM");

        if (ui.menuItem(.{
            .label = label,
            .enabled = enabled,
            .bg_color = theme.bg_section,
            .hover_color = theme.accent_blue,
            .text_color = theme.text_secondary,
        }).clicked(ui.main_window.ctx)) {
            switch (mode) {
                .save => {
                    app_state.show_save_state_toast = true;
                    ui.setTimer("save_state_toast", 1000);
                    app_state.saveStateSlot(slot);
                },
                .load => {
                    app_state.show_load_state_toast = true;
                    ui.setTimer("load_state_toast", 1000);
                    app_state.loadStateSlot(slot);
                },
            }
        }
    }
}

const FilePickerCallbackData = struct { ui: *UI, app_state: *AppState };
fn openRomDialog(ui: *UI, app_state: *AppState) void {
    const default_location = std.process.getCwdAlloc(ui.main_window.ctx.frameAlloc()) catch
        @panic("Failed to allocate");
    defer ui.main_window.ctx.frameAlloc().free(default_location);

    const data = ui.main_window.ctx.persistent_arena.allocator()
        .create(FilePickerCallbackData) catch @panic("OOM");
    data.* = .{ .app_state = app_state, .ui = ui };

    c.SDL_ShowOpenFileDialog(
        file_picker_callback,
        clay.anytypeToAnyopaquePtr(data),
        ui.main_window.ptr,
        &dialog_filter_list,
        dialog_filter_list.len,
        default_location.ptr,
        false,
    );
}

fn drawHomeUI(ui: *UI, app_state: *AppState, safe_area_padding: clay.Padding) void {
    const entries = app_state.history.entries.items;

    const root = ui.column(.{
        .sizing = .grow,
        .bg_color = theme.bg_base,
        .child_alignment = .{ .x = .center, .y = .top },
        .padding = if (builtin.abi.isAndroid())
            .{
                .bottom = safe_area_padding.bottom,
                .left = safe_area_padding.left,
                .right = safe_area_padding.right,
            }
        else
            .{},
    });
    {
        // Empty state home
        if (entries.len == 0) {
            if (builtin.abi.isAndroid()) {
                _ = ui.spacer(.{ .sizing = .grow });
                _ = ui.label(.{ .text = "No ROMs added", .font_size = 16, .color = theme.text_secondary });
                _ = ui.spacer(.{ .sizing = .grow });
            } else {
                _ = ui.spacer(.{ .sizing = .grow });
                if (ui.button(.{
                    .text = "Open ROM",
                    .font_size = 15,
                    .bg_color = theme.accent_blue,
                    .text_color = Color.white,
                    .padding = .{ .left = 28, .right = 28, .top = 10, .bottom = 10 },
                    .corner_radius = 6,
                    .elevation = 0,
                }).clicked(ui.main_window.ctx)) openRomDialog(ui, app_state);
                _ = ui.spacer(.{ .sizing = .{ .w = .fixed(0), .h = .fixed(14) } });
                _ = ui.label(.{ .text = "or use System > Open", .font_size = 13, .color = theme.text_secondary });
                _ = ui.spacer(.{ .sizing = .grow });
            }
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
                    .item_transition = .{
                        .handler = clay.easeOut,
                        .duration = 0.22,
                        .properties = clay.TransitionProperty.position,
                        .interaction_handling = .allow_interactions_while_transitioning_position,
                    },
                });
                for (entries) |*entry| {
                    const slot = grid.item();
                    drawGameCard(ui, app_state, entry, card_title_lines);
                    slot.end();
                }
                grid.end();
            }
            scroll.end();
        }
    }
    root.end();
}

const CARD_W: f32 = if (builtin.abi.isAndroid()) 150 else 200;
const CARD_THUMB_H: f32 = CARD_W * 224 / 256;
const CARD_CONTENT_W: f32 = CARD_W - 20;
const CARD_TITLE_FONT_SIZE: u16 = 14;
const CARD_TITLE_LINE_H: u16 = 16;
const CARD_TITLE_MAX_LINES: u16 = 4;
const CARD_PLAY_TIME_LINE_H: u16 = 14;
const CARD_CORNER_RADIUS: f32 = 6;
const PLACEHOLDER_THUMBNAIL_PIXEL = [_]u8{ 0, 0, 0, 255 };

fn cardTitleHeight(title_lines: u16) f32 {
    return @floatFromInt(CARD_TITLE_LINE_H * title_lines);
}

fn cardInfoHeight(title_lines: u16) f32 {
    return 8 + cardTitleHeight(title_lines) + 4 + CARD_PLAY_TIME_LINE_H + 8;
}

fn cardHeight(title_lines: u16) f32 {
    return CARD_THUMB_H + cardInfoHeight(title_lines);
}

fn drawGameCard(ui: *UI, app_state: *AppState, entry: *const game_history.GameEntry, title_lines: u16) void {
    const ctx = ui.main_window.ctx;
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
            _ = ui.label(.{
                .text = entry.name,
                .font_size = CARD_TITLE_FONT_SIZE,
                .line_height = CARD_TITLE_LINE_H,
                .color = theme.text_primary,
            });
            _ = ui.spacer(.{ .sizing = .grow });
            _ = ui.label(.{
                .text = utils.formatPlayTime(entry.play_time_secs, ctx.frameAlloc()),
                .font_size = 13,
                .line_height = CARD_PLAY_TIME_LINE_H,
                .color = theme.text_secondary,
            });
        }
        info.end();
    }
    card.end();

    if (card.clicked(ui.main_window.ctx)) {
        app_state.loadRom(entry.rom_path) catch |err| std.debug.panic("Failed to load selected ROM: {any}\n", .{err});

        if (builtin.abi.isAndroid()) {
            ui.setWindowFullscreen(true);
        }
    }
}

fn maxGameCardTitleLines(ui: *UI, entries: []const game_history.GameEntry) u16 {
    const space_w = ui.measureTextWidth(" ", CARD_TITLE_FONT_SIZE);
    var max_lines: u16 = 1;
    for (entries) |entry| {
        max_lines = @max(max_lines, calculateGameCardTitleTotalLines(ui, entry.name, space_w));
    }
    return @min(max_lines, CARD_TITLE_MAX_LINES);
}

fn calculateGameCardTitleTotalLines(ui: *UI, title: []const u8, space_w: f32) u16 {
    var lines: u16 = 1;
    var line_w: f32 = 0;

    var it = std.mem.splitScalar(u8, title, ' ');
    while (it.next()) |word| {
        const word_w = ui.measureTextWidth(word, CARD_TITLE_FONT_SIZE);
        line_w += word_w + space_w;

        if (line_w > CARD_CONTENT_W) {
            lines += 1;
            line_w = word_w;
        }
    }

    return lines;
}

const nav_active_bg = Color.rgb(28, 110, 90);
const nav_active_text = Color.rgb(210, 240, 232);
const nav_hover_bg = Color.rgb(35, 42, 52);

fn drawSettingsWindowUI(ui: *UI, user_data: ?*anyopaque) void {
    const app_state: *AppState = @ptrCast(@alignCast(user_data));

    const root = ui.row(.{
        .sizing = .grow,
        .bg_color = theme.bg_base,
        .child_alignment = .{ .x = .left, .y = .top },
    });
    {
        drawSettingsSidebar(ui, app_state);
        _ = ui.separator(.{ .direction = .vertical, .color = theme.border_dim });

        const body = ui.column(.{
            .sizing = .{ .w = .grow, .h = .grow },
            .child_alignment = .{ .x = .left, .y = .top },
        });
        {
            drawSettingsContent(ui, app_state);
            drawSettingsFooter(ui, app_state);
        }
        body.end();
    }
    root.end();
}

fn drawSettingsFooter(ui: *UI, app_state: *AppState) void {
    const has_changes = app_state.hasSettingsChanges();

    const footer = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .bg_color = theme.bg_panel,
        .padding = .{ .left = 12, .right = 12, .top = 10, .bottom = 10 },
        .gap = 8,
        .child_alignment = .{ .x = .right, .y = .center },
    });
    {
        _ = ui.spacer(.{ .sizing = .grow });

        if (ui.button(.{
            .text = "Cancel",
            .font_size = 15,
            .text_color = theme.text_primary,
            .bg_color = theme.bg_hover,
            .hover_color = theme.border,
            .padding = .{ .left = 14, .right = 14, .top = 7, .bottom = 7 },
            .corner_radius = 3,
            .elevation = 0,
        }).clicked(ui.current_window.ctx)) {
            app_state.restoreSavedSettings();
            ui.closeCurrentWindow();
        }

        if (ui.button(.{
            .text = "Save",
            .font_size = 15,
            .enabled = has_changes,
            .text_color = if (has_changes) Color.white else theme.text_secondary,
            .bg_color = if (has_changes) theme.accent_blue else theme.bg_hover,
            .hover_color = if (has_changes) theme.accent_blue.lighten(0.12) else theme.bg_hover,
            .padding = .{ .left = 16, .right = 16, .top = 7, .bottom = 7 },
            .corner_radius = 3,
            .elevation = 0,
        }).clicked(ui.current_window.ctx)) {
            app_state.saveSettings();
            ui.closeCurrentWindow(); // close settings window
            ui.setVSync(app_state.settings.vsync);
        }
    }
    footer.end();
}

fn drawSettingsSidebar(ui: *UI, app_state: *AppState) void {
    const sidebar = ui.column(.{
        .sizing = .{ .w = .fixed(130), .h = .grow },
        .bg_color = theme.bg_base,
        .padding = .{ .top = 6, .bottom = 6 },
        .gap = 0,
        .child_alignment = .{ .x = .left, .y = .top },
    });
    {
        inline for (@typeInfo(SettingsCategory).@"enum".fields) |category| {
            drawSidebarItem(ui, app_state, @field(SettingsCategory, category.name));
        }
    }
    sidebar.end();
}

fn drawSidebarItem(ui: *UI, app_state: *AppState, category: SettingsCategory) void {
    const is_active = app_state.settings.selected_category == category;
    if (ui.button(.{
        .text = category.displayName(),
        .font_size = 19,
        .text_color = if (is_active) nav_active_text else theme.text_secondary,
        .bg_color = if (is_active) nav_active_bg else theme.bg_base,
        .hover_color = if (is_active) nav_active_bg else nav_hover_bg,
        .padding = .{ .left = 14, .right = 14, .top = 9, .bottom = 9 },
        .corner_radius = 0,
        .elevation = 0,
        .sizing = .{ .w = .grow, .h = .fit },
        .text_alignment = .left,
    }).clicked(ui.current_window.ctx)) {
        app_state.settings.selected_category = category;
    }
}

fn drawSettingsContent(ui: *UI, app_state: *AppState) void {
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
            switch (app_state.settings.selected_category) {
                .general => drawSettingsGeneralContent(ui, app_state),
                .video => drawSettingsVideoContent(ui, app_state),
                .shader => drawSettingsShaderContent(ui, app_state),
                .controls => drawSettingsControlsContent(ui, app_state),
            }
        }

        content.end();
    }
    scroll.end();
}

fn drawContentSectionHeader(ui: *UI, title: []const u8) void {
    _ = ui.label(.{
        .text = title,
        .font_size = 14,
        .color = theme.accent_green,
    });
}

fn drawContentSection(ui: *UI, params: struct { padding: ?clay.Padding = null, sizing: ?clay.Sizing = null }) *widgets.Container {
    return ui.column(.{
        .sizing = if (params.sizing) |sizing| sizing else .{ .w = .grow, .h = .fit },
        .bg_color = theme.bg_section,
        .padding = if (params.padding) |padding| padding else .{ .left = 14, .right = 14, .top = 12, .bottom = 12 },
        .gap = 12,
        .border_width = 1,
        .border_color = theme.border_dim,
        .corner_radius = 3,
        .child_alignment = .{ .x = .left, .y = .top },
    });
}

fn drawSettingsVideoContent(ui: *UI, app_state: *AppState) void {
    drawContentSectionHeader(ui, "General");
    const general_section = drawContentSection(ui, .{});
    {
        const row = ui.row(.{
            .sizing = .{ .w = .grow, .h = .fit },
        });
        {
            _ = ui.label(.{
                .text = "VSync",
                .font_size = theme.LABEL_FONT,
                .color = theme.text_primary,
            });
            _ = ui.spacer(.{ .sizing = .grow });
            app_state.settings.vsync = ui.toggle(.{ .value = app_state.settings.vsync, .size = 22 }).value();
        }
        row.end();
    }
    general_section.end();

    drawContentSectionHeader(ui, "Display");
    const display_section = drawContentSection(ui, .{});
    {
        drawAspectRatioRow(ui, app_state);
    }
    display_section.end();
}

fn drawSettingsGeneralContent(ui: *UI, app_state: *AppState) void {
    drawContentSectionHeader(ui, "General");
    const section = drawContentSection(ui, .{});
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
            app_state.settings.hide_mouse_on_inactivity = ui
                .toggle(.{ .value = app_state.settings.hide_mouse_on_inactivity, .size = 22 })
                .value();
        }
        row.end();

        drawEmulationSpeedRow(ui, app_state);
    }
    section.end();
}

fn drawSettingsControlsContent(ui: *UI, app_state: *AppState) void {
    drawContentSectionHeader(ui, "Controls");
    const controller_keymap_section = drawContentSection(ui, .{});
    {
        updateControllerBindingCapture(ui, app_state);
        updateGamepadBindingCapture(ui, app_state);
        updateGeneralBindingCapture(ui, app_state);
        drawControllerPlayerSelector(ui, app_state);
        drawConnectedGamepadRow(ui, app_state);
        drawControllerBindingOverlay(ui, app_state);

        if (ui.getGamepadCount() > 0) {
            drawContentSectionHeader(ui, "Analog Stick");
            const gamepad_section = drawContentSection(ui, .{});
            drawGamepadDeadzoneRow(ui, app_state);
            gamepad_section.end();
        }
    }
    controller_keymap_section.end();

    drawContentSectionHeader(ui, "General");
    const general_keymap_section = drawContentSection(ui, .{
        .padding = .{ .left = 14, .right = 5, .top = 12, .bottom = 12 },
    });
    {
        const scroll = ui.scrollArea(.{
            .sizing = .{ .h = .fixed(200), .w = .grow },
            .gap = 12,
        });
        {
            inline for (@typeInfo(GeneralAction).@"enum".fields) |action_field| {
                drawGeneralBindingRow(ui, app_state, @field(GeneralAction, action_field.name));
            }
        }
        scroll.end();
    }
    general_keymap_section.end();
}

fn drawGamepadDeadzoneRow(ui: *UI, app_state: *AppState) void {
    const alloc = ui.current_window.ctx.frameAlloc();
    const current: f32 = @floatFromInt(app_state.settings.gamepad_deadzone);

    const row = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .child_alignment = .{ .y = .center },
        .gap = 8,
    });
    {
        _ = ui.label(.{
            .text = "Deadzone",
            .font_size = theme.LABEL_FONT,
            .color = theme.text_primary,
        });
        _ = ui.spacer(.{ .sizing = .grow });
        _ = ui.label(.{
            .text = std.fmt.allocPrint(alloc, "{d}%", .{app_state.settings.gamepad_deadzone}) catch "?",
            .font_size = 14,
            .color = theme.text_value,
        });
    }
    row.end();

    const drag = ui.slider(.{
        .value = current,
        .min = 0.0,
        .max = 100.0,
        .step = 1.0,
        .height = 4,
        .fill_color = theme.accent_green,
        .track_color = theme.bg_hover,
        .thumb_color = theme.text_primary,
        .corner_radius = 2,
    });
    const new_val: u8 = @intFromFloat(drag.value());
    if (app_state.settings.gamepad_deadzone != new_val) {
        app_state.settings.gamepad_deadzone = new_val;
    }
}

fn drawConnectedGamepadRow(ui: *UI, app_state: *AppState) void {
    const row = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .child_alignment = .{ .y = .center },
        .gap = 8,
    });
    {
        if (ui.getConnectedGamepadName(app_state.settings.selected_controller_player.value())) |name| {
            _ = ui.label(.{
                .text = "Gamepad",
                .font_size = theme.LABEL_FONT,
                .color = theme.text_primary,
            });
            _ = ui.spacer(.{ .sizing = .grow });
            _ = ui.label(.{
                .text = name,
                .font_size = 15,
                .color = theme.text_primary,
            });
        }
    }
    row.end();
}

fn drawControllerPlayerSelector(ui: *UI, app_state: *AppState) void {
    const row = ui.row(.{
        .sizing = .{ .w = .fit, .h = .fit },
        .gap = 6,
        .child_alignment = .{ .y = .center },
    });
    {
        inline for (@typeInfo(ControllerPlayer).@"enum".fields) |player_field| {
            const player = @field(ControllerPlayer, player_field.name);
            const is_selected = app_state.settings.selected_controller_player == player;
            if (ui.button(.{
                .text = player.displayName(),
                .font_size = 14,
                .text_color = if (is_selected) nav_active_text else theme.text_secondary,
                .bg_color = if (is_selected) nav_active_bg else theme.bg_hover,
                .hover_color = if (is_selected) nav_active_bg else nav_hover_bg,
                .padding = .{ .left = 10, .right = 10, .top = 6, .bottom = 6 },
                .corner_radius = 3,
                .elevation = 0,
            }).clicked(ui.current_window.ctx)) {
                app_state.settings.selected_controller_player = player;
                app_state.settings.capture_binding = null;
                app_state.settings.capture_general_binding = null;
                app_state.settings.capture_gamepad_binding = null;
            }
        }
    }
    row.end();
}

fn updateControllerBindingCapture(ui: *UI, app_state: *AppState) void {
    const target = app_state.settings.capture_binding orelse return;
    const key = ui.getPressedKey() orelse return;
    if (key == .UNKNOWN) return;

    const player_bindings = app_state.settings.controller_bindings.forPlayer(target.player);
    if (player_bindings.get(target.action) != key) {
        player_bindings.set(target.action, key);
    }
    app_state.settings.capture_binding = null;
}

fn updateGamepadBindingCapture(ui: *UI, app_state: *AppState) void {
    const target = app_state.settings.capture_gamepad_binding orelse return;
    const pressed = ui.getPressedGamepadButton() orelse return;

    const player_bindings = app_state.settings.gamepad_bindings.forPlayer(target.player);
    if (player_bindings.get(target.action) != pressed.btn) {
        player_bindings.set(target.action, pressed.btn);
    }
    app_state.settings.capture_gamepad_binding = null;
}

fn updateGeneralBindingCapture(ui: *UI, app_state: *AppState) void {
    const action = app_state.settings.capture_general_binding orelse return;
    const key = ui.getPressedKey() orelse return;
    if (key == .UNKNOWN) return;

    if (app_state.settings.general_bindings.get(action) != key) {
        app_state.settings.general_bindings.set(action, key);
    }
    app_state.settings.capture_general_binding = null;
}

fn drawControllerBindingOverlay(ui: *UI, app_state: *AppState) void {
    const img_w = app_state.controller_img.w();
    const img_h = app_state.controller_img.h();
    const display_w: f32 = @min(500.0, @as(f32, @floatFromInt(img_w)));
    const display_h = display_w * @as(f32, @floatFromInt(img_h)) / @as(f32, @floatFromInt(img_w));

    const root = ui.column(.{
        .sizing = .{ .w = .fixed(display_w), .h = .fixed(display_h) },
    });
    _ = ui.canvas(.{
        .sizing = .{ .w = .fixed(display_w), .h = .fixed(display_h) },
        .pixel_format = app_state.controller_img.format(),
        .pixels = app_state.controller_img.pixels(),
        .w = img_w,
        .h = img_h,
    });

    const player = app_state.settings.selected_controller_player;
    inline for (@typeInfo(ControllerAction).@"enum".fields) |action_field| {
        const action = @field(ControllerAction, action_field.name);
        drawControllerBindingField(ui, app_state, root.id, player, action);
    }

    root.end();
}

fn drawControllerBindingField(
    ui: *UI,
    app_state: *AppState,
    parent_id: clay.ElementId,
    player: ControllerPlayer,
    action: ControllerAction,
) void {
    const has_gamepad = ui.getGamepadCount() > 0;

    const position = controllerBindingPosition(action);
    const target = ControllerBindingTarget{ .player = player, .action = action };

    const is_capturing_kb = !has_gamepad and isControllerBindingTarget(app_state.settings.capture_binding, target);
    const is_capturing_gp = has_gamepad and isControllerBindingTarget(app_state.settings.capture_gamepad_binding, target);
    const is_capturing = is_capturing_kb or is_capturing_gp;

    const label: []const u8 = if (is_capturing_gp)
        "Press btn"
    else if (is_capturing_kb)
        "Press key"
    else if (has_gamepad)
        app_state.settings.gamepad_bindings.forPlayerConst(player).get(action).displayName()
    else
        app_state.settings.controller_bindings.forPlayerConst(player).get(action).keyName();

    const float = ui.float(.{
        .attach_to = .to_element_with_id,
        .parentId = parent_id.id,
        .attach_points = .{ .element = .center_center, .parent = .left_top },
        .offset = .{ .x = position.x, .y = position.y },
        .z_index = 20,
    });
    {
        const btn = ui.button(.{
            .sizing = .{ .w = .fitMinMax(.{ .min = 50, .max = 65 }), .h = .fit },
            .padding = .all(4),
            .bg_color = if (is_capturing) theme.bg_active.withAlpha(0.85) else theme.bg_panel,
            .corner_radius = 0,
            .border = .{ .color = (if (is_capturing) theme.border_selected else theme.border).toClay(), .width = .outside(1) },
            .text = label,
            .font_size = 14,
            .tooltip = if (action == .select)
                .{ .text = "Select", .text_size = 16 }
            else if (action == .start)
                .{ .text = "Start", .text_size = 16 }
            else
                null,
        });
        if (btn.clicked(ui.current_window.ctx)) {
            if (has_gamepad) {
                app_state.settings.capture_gamepad_binding = target;
                app_state.settings.capture_binding = null;
            } else {
                app_state.settings.capture_binding = target;
                app_state.settings.capture_gamepad_binding = null;
            }
            app_state.settings.capture_general_binding = null;
        }
    }
    float.end();
}

fn isControllerBindingTarget(current: ?ControllerBindingTarget, target: ControllerBindingTarget) bool {
    const active = current orelse return false;
    return active.player == target.player and active.action == target.action;
}

fn controllerBindingPosition(action: ControllerAction) clay.Vector2 {
    return switch (action) {
        .up => .{ .x = 95, .y = 45 },
        .down => .{ .x = 95, .y = 195 },
        .left => .{ .x = 10, .y = 120 },
        .right => .{ .x = 180, .y = 120 },
        .select => .{ .x = 205, .y = 180 },
        .start => .{ .x = 275, .y = 180 },
        .b => .{ .x = 360, .y = 210 },
        .a => .{ .x = 430, .y = 210 },
    };
}

fn drawGeneralBindingRow(ui: *UI, app_state: *AppState, action: GeneralAction) void {
    const is_capturing = isGeneralBindingTarget(app_state.settings.capture_general_binding, action);
    const key = app_state.settings.general_bindings.get(action);
    const label = if (is_capturing) "Press key" else key.keyName();

    const row = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .child_alignment = .{ .y = .center },
        .gap = 10,
    });
    {
        _ = ui.label(.{
            .text = action.displayName(),
            .font_size = theme.LABEL_FONT,
            .color = theme.text_primary,
        });

        _ = ui.spacer(.{ .sizing = .grow });

        const btn = ui.button(.{
            .sizing = .{ .w = .fitMinMax(.{ .min = 95, .max = 130 }), .h = .fit },
            .padding = .{ .left = 10, .right = 10, .top = 6, .bottom = 6 },
            .bg_color = if (is_capturing) theme.bg_active else theme.bg_panel,
            .hover_color = if (is_capturing) theme.bg_active else theme.bg_hover,
            .text_color = if (is_capturing) theme.text_accent else theme.text_primary,
            .corner_radius = 0,
            .border = .{ .color = (if (is_capturing) theme.border_selected else theme.border).toClay(), .width = .outside(1) },
            .text = label,
            .font_size = 16,
        });
        if (btn.clicked(ui.current_window.ctx)) {
            app_state.settings.capture_general_binding = action;
            app_state.settings.capture_binding = null;
        }
    }
    row.end();
}

fn isGeneralBindingTarget(current: ?GeneralAction, action: GeneralAction) bool {
    const active = current orelse return false;
    return active == action;
}

fn drawEmulationSpeedRow(ui: *UI, app_state: *AppState) void {
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
            .selected = app_state.settings.emulation_speed,
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

        const selected_speed = speed_opts.selected();
        if (app_state.settings.emulation_speed != selected_speed) {
            app_state.setEmulationSpeed(selected_speed);
        }
    }
    row.end();
}

fn drawAspectRatioRow(ui: *UI, app_state: *AppState) void {
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

        const aspect_ratio_opts = ui.combobox(viewport.AspectRatio, .{
            .id = "aspect_ratio_combo",
            .selected = app_state.settings.aspect_ratio,
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

        const selected_aspect_ratio = aspect_ratio_opts.selected();
        if (app_state.settings.aspect_ratio != selected_aspect_ratio) {
            app_state.settings.aspect_ratio = selected_aspect_ratio;
        }
    }
    row.end();
}

fn drawSettingsShaderContent(ui: *UI, app_state: *AppState) void {
    if (builtin.abi.isAndroid()) {
        drawContentSectionHeader(ui, "Shader Library");
        {
            const section = drawContentSection(ui, .{});
            drawAndroidShaderDownload(ui, app_state);
            section.end();
        }
    }

    drawContentSectionHeader(ui, "Shader");
    {
        const section = drawContentSection(ui, .{});
        drawShaderPresetRow(ui, app_state);
        if (app_state.shader_loading) {
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
        } else if (app_state.shader_error) |err_msg| {
            _ = ui.label(.{
                .text = err_msg,
                .font_size = 13,
                .color = theme.accent_red,
            });
        }

        const param_infos = ui.getShaderParamInfos();
        if (!app_state.shader_loading and param_infos.len > 0) {
            drawShaderParamsSection(ui, app_state, "Parameters", param_infos, .main);
        }
        section.end();
    }

    drawContentSectionHeader(ui, "Border Shader");
    {
        const section = drawContentSection(ui, .{});
        drawBorderShaderPresetRow(ui, app_state);
        if (app_state.border_shader_loading) {
            _ = ui.label(.{
                .text = "Compiling...",
                .font_size = 13,
                .color = theme.accent_purple,
            });
        } else if (app_state.border_shader_error) |err_msg| {
            _ = ui.label(.{
                .text = err_msg,
                .font_size = 13,
                .color = theme.accent_red,
            });
        }

        const border_param_infos = ui.getBorderShaderParamInfos();
        if (!app_state.border_shader_loading and border_param_infos.len > 0) {
            drawShaderParamsSection(ui, app_state, "Border Parameters", border_param_infos, .border);
        }

        section.end();
    }
}

fn drawAndroidShaderDownload(ui: *UI, app_state: *AppState) void {
    var status = app_state.shaderDownloadStatus();
    const installed = app_state.shaderDownloadInstalled();
    const can_download = !status.active and !installed;

    const row = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .child_alignment = .{ .y = .center },
        .gap = 8,
    });
    {
        _ = ui.label(.{
            .text = "slang-shaders",
            .font_size = theme.LABEL_FONT,
            .color = theme.text_primary,
        });
        _ = ui.spacer(.{ .sizing = .grow });

        const button_text: []const u8 = if (installed) "Installed" else if (status.active) "Downloading" else "Download";
        if (ui.button(.{
            .text = button_text,
            .font_size = 15,
            .enabled = can_download,
            .text_color = if (can_download) theme.text_primary else theme.text_secondary,
            .bg_color = if (can_download) theme.bg_hover else theme.bg_panel,
            .hover_color = if (can_download) theme.border else theme.bg_panel,
            .padding = .{ .left = 10, .right = 10, .top = 5, .bottom = 5 },
            .elevation = 0,
        }).clicked(ui.current_window.ctx)) {
            app_state.startShaderDownload() catch |err| {
                std.log.err("failed to start shader download: {s}", .{@errorName(err)});
                if (app_state.shader_download_error) |old| app_state.alloc.free(old);
                app_state.shader_download_error = std.fmt.allocPrint(
                    app_state.alloc,
                    "Download failed: {s}",
                    .{@errorName(err)},
                ) catch null;
            };
            status = app_state.shaderDownloadStatus();
        }
    }
    row.end();

    switch (status.state) {
        .downloading => drawShaderDownloadProgress(ui, status.bytes, status.total_bytes),
        .extracting => {
            _ = ui.label(.{
                .text = "Extracting...",
                .font_size = 13,
                .color = theme.accent_purple,
            });
        },
        .failed => if (status.error_message) |msg| {
            _ = ui.label(.{
                .text = msg,
                .font_size = 13,
                .color = theme.accent_red,
            });
        },
        else => {},
    }
}

fn drawShaderDownloadProgress(ui: *UI, bytes: u64, total_bytes: u64) void {
    const alloc = ui.current_window.ctx.frameAlloc();
    const known_total = total_bytes != shader_download.unknown_total and total_bytes > 0;

    const progress_text = if (known_total)
        std.fmt.allocPrint(
            alloc,
            "Downloading... {s} / {s}",
            .{ utils.formatByteCount(alloc, bytes), utils.formatByteCount(alloc, total_bytes) },
        ) catch "Downloading..."
    else
        std.fmt.allocPrint(
            alloc,
            "Downloading... {s}",
            .{utils.formatByteCount(alloc, bytes)},
        ) catch "Downloading...";

    _ = ui.label(.{
        .text = progress_text,
        .font_size = 13,
        .color = theme.accent_purple,
    });
}

fn drawShaderParamsSection(
    ui: *UI,
    app_state: *AppState,
    title: []const u8,
    param_infos: []const pipeline.ParamInfo,
    target: ParamTarget,
) void {
    drawContentSectionHeader(ui, title);

    const section = drawContentSection(ui, .{
        .sizing = .{ .w = .growMinMax(.{ .max = 500 }), .h = .fit },
    });
    {
        const scroll = ui.scrollArea(.{
            .gap = 12,
            .sizing = .{ .w = .grow, .h = .fitMinMax(.{ .max = 350 }) },
        });
        {
            for (param_infos) |info| {
                drawParamRow(ui, app_state, info, target);
            }
        }
        scroll.end();
    }
    section.end();
}

const PREVIEW_MAX_W: f32 = 320;
const PREVIEW_MAX_H: f32 = 270;
const PREVIEW_FRAME_PADDING: f32 = 2;
const PREVIEW_LABEL_HEIGHT: f32 = 22;
const PreviewColor = struct {
    r: u8,
    g: u8,
    b: u8,
};

fn createStoppedPreviewPlaceholder(alloc: std.mem.Allocator) []const u8 {
    const pixels = alloc.alloc(u8, NES_VISIBLE_PIXEL_BYTES) catch @panic("OOM");

    var y: u32 = 0;
    while (y < NES_VISIBLE_HEIGHT) : (y += 1) {
        var x: u32 = 0;
        while (x < NES_WIDTH) : (x += 1) {
            const sky: PreviewColor = if (y < 120)
                .{ .r = 78, .g = 151, .b = 198 }
            else
                .{ .r = 64, .g = 130, .b = 176 };
            setPreviewPixel(pixels, x, y, sky);
        }
    }

    fillPreviewRect(pixels, 18, 28, 28, 8, .{ .r = 220, .g = 232, .b = 228 });
    fillPreviewRect(pixels, 26, 20, 18, 8, .{ .r = 220, .g = 232, .b = 228 });
    fillPreviewRect(pixels, 154, 38, 36, 8, .{ .r = 220, .g = 232, .b = 228 });
    fillPreviewRect(pixels, 166, 30, 18, 8, .{ .r = 220, .g = 232, .b = 228 });

    fillPreviewRect(pixels, 0, 134, 96, 38, .{ .r = 48, .g = 132, .b = 88 });
    fillPreviewRect(pixels, 96, 148, 86, 24, .{ .r = 42, .g = 112, .b = 78 });
    fillPreviewRect(pixels, 182, 128, 74, 44, .{ .r = 54, .g = 144, .b = 92 });

    drawPreviewBlock(pixels, 112, 86, .{ .r = 194, .g = 126, .b = 50 });
    drawPreviewBlock(pixels, 128, 86, .{ .r = 194, .g = 126, .b = 50 });
    drawPreviewBlock(pixels, 144, 86, .{ .r = 194, .g = 126, .b = 50 });
    drawPreviewBlock(pixels, 184, 62, .{ .r = 224, .g = 181, .b = 64 });
    fillPreviewRect(pixels, 188, 66, 4, 4, .{ .r = 86, .g = 61, .b = 38 });

    fillPreviewRect(pixels, 34, 120, 44, 10, .{ .r = 90, .g = 74, .b = 55 });
    fillPreviewRect(pixels, 34, 116, 44, 4, .{ .r = 116, .g = 161, .b = 82 });
    fillPreviewRect(pixels, 172, 106, 44, 10, .{ .r = 90, .g = 74, .b = 55 });
    fillPreviewRect(pixels, 172, 102, 44, 4, .{ .r = 116, .g = 161, .b = 82 });

    fillPreviewRect(pixels, 0, 172, NES_WIDTH, 52, .{ .r = 118, .g = 74, .b = 44 });
    fillPreviewRect(pixels, 0, 172, NES_WIDTH, 8, .{ .r = 72, .g = 161, .b = 76 });
    y = 184;
    while (y < NES_VISIBLE_HEIGHT) : (y += 16) {
        var x: u32 = 0;
        while (x < NES_WIDTH) : (x += 16) {
            fillPreviewRect(pixels, x, y, 14, 2, .{ .r = 92, .g = 55, .b = 34 });
        }
    }

    fillPreviewRect(pixels, 80, 140, 12, 18, .{ .r = 213, .g = 76, .b = 54 });
    fillPreviewRect(pixels, 82, 130, 10, 10, .{ .r = 240, .g = 184, .b = 112 });
    fillPreviewRect(pixels, 78, 142, 4, 8, .{ .r = 240, .g = 184, .b = 112 });
    fillPreviewRect(pixels, 92, 142, 4, 8, .{ .r = 240, .g = 184, .b = 112 });
    fillPreviewRect(pixels, 80, 158, 5, 8, .{ .r = 48, .g = 64, .b = 118 });
    fillPreviewRect(pixels, 88, 158, 5, 8, .{ .r = 48, .g = 64, .b = 118 });
    fillPreviewRect(pixels, 84, 134, 2, 2, .{ .r = 38, .g = 32, .b = 28 });

    fillPreviewRect(pixels, 148, 156, 12, 8, .{ .r = 92, .g = 66, .b = 45 });
    fillPreviewRect(pixels, 150, 150, 8, 6, .{ .r = 132, .g = 92, .b = 54 });
    fillPreviewRect(pixels, 142, 164, 24, 4, .{ .r = 72, .g = 45, .b = 31 });

    fillPreviewRect(pixels, 10, 8, 28, 4, .{ .r = 255, .g = 232, .b = 128 });
    fillPreviewRect(pixels, 42, 8, 6, 4, .{ .r = 255, .g = 232, .b = 128 });
    fillPreviewRect(pixels, 212, 8, 34, 4, .{ .r = 255, .g = 232, .b = 128 });

    return pixels;
}

fn drawPreviewBlock(pixels: []u8, x: u32, y: u32, color: PreviewColor) void {
    fillPreviewRect(pixels, x, y, 16, 16, color);
    fillPreviewRect(pixels, x, y, 16, 2, .{ .r = 240, .g = 170, .b = 84 });
    fillPreviewRect(pixels, x, y + 14, 16, 2, .{ .r = 112, .g = 70, .b = 38 });
    fillPreviewRect(pixels, x + 14, y, 2, 16, .{ .r = 112, .g = 70, .b = 38 });
}

fn fillPreviewRect(pixels: []u8, x: u32, y: u32, w: u32, h: u32, color: PreviewColor) void {
    const x_end = @min(x + w, NES_WIDTH);
    const y_end = @min(y + h, NES_VISIBLE_HEIGHT);
    var py = y;
    while (py < y_end) : (py += 1) {
        var px = x;
        while (px < x_end) : (px += 1) {
            setPreviewPixel(pixels, px, py, color);
        }
    }
}

fn setPreviewPixel(pixels: []u8, x: u32, y: u32, color: PreviewColor) void {
    const idx: usize = (@as(usize, y) * NES_WIDTH + x) * 4;
    pixels[idx + 0] = color.r;
    pixels[idx + 1] = color.g;
    pixels[idx + 2] = color.b;
    pixels[idx + 3] = 255;
}

fn drawShaderPreview(
    ui: *UI,
    pixels: []const u8,
    px_w: u32,
    px_h: u32,
    shader: widgets.Canvas.ShaderPreview,
    aspect_ratio: viewport.AspectRatio,
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

fn drawShaderPresetRow(ui: *UI, app_state: *AppState) void {
    const can_clear_shader = app_state.settings.shader_preset_path != null or app_state.shader_loading;

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
            if (builtin.abi.isAndroid()) {
                app_state.openShaderFilePicker(.main);
            } else {
                const default_location = std.process.getCwdAlloc(ui.main_window.ctx.frameAlloc()) catch @panic("OOM");
                defer ui.main_window.ctx.frameAlloc().free(default_location);

                c.SDL_ShowOpenFileDialog(
                    shader_dialog_callback,
                    clay.anytypeToAnyopaquePtr(app_state),
                    ui.main_window.ptr,
                    &shader_filter_list,
                    shader_filter_list.len,
                    default_location.ptr,
                    false,
                );
            }
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
            if (app_state.settings.shader_preset_path) |path| {
                app_state.alloc.free(path);
                app_state.settings.shader_preset_path = null;
            }
            settings.clearShaderParamSettings(app_state.alloc, &app_state.settings.shader_params);
            app_state.should_load_shader = false;
            app_state.should_clear_shader = true;
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

    if (app_state.emulation_running) {
        const center = ui.row(.{ .sizing = .{ .w = .grow, .h = .fit }, .child_alignment = .{ .x = .center } });
        drawShaderPreview(ui, app_state.framePixels(OVERSCAN_PIXEL_OFFSET, NES_VISIBLE_PIXEL_BYTES), NES_WIDTH, NES_VISIBLE_HEIGHT, .main, .none);
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

fn drawParamRow(ui: *UI, app_state: *AppState, info: pipeline.ParamInfo, target: ParamTarget) void {
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
            if (new_val != current) {
                setParamValue(ui, target, info.name, new_val);
                app_state.setShaderParamSetting(target, info.name, new_val);
            }
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
        if (new_val != current) {
            setParamValue(ui, target, info.name, new_val);
            app_state.setShaderParamSetting(target, info.name, new_val);
        }
    }
}

const shader_filter_list: [2]c.SDL_DialogFileFilter = [_]c.SDL_DialogFileFilter{
    .{ .name = "RetroArch Shader Presets", .pattern = "slangp" },
    .{ .name = "All files", .pattern = "*" },
};

fn shader_dialog_callback(userdata: ?*anyopaque, filelist: [*c]const [*c]const u8, _: c_int) callconv(.c) void {
    const app_state = clay.anyopaquePtrToType(*AppState, userdata);

    if (filelist == null) {
        std.log.err("An error ocurred while selecting shader file: {s}\n", .{c.SDL_GetError()});
        return;
    }
    if (filelist.* == null) {
        return;
    }

    const filepath = std.mem.span(filelist.*);
    std.log.debug("User selected shader: {s}", .{filepath});

    if (app_state.settings.shader_preset_path) |old_path| {
        app_state.alloc.free(old_path);
    }
    app_state.settings.shader_preset_path = app_state.alloc.dupe(u8, filepath) catch @panic("Failed to allocate!");
    settings.clearShaderParamSettings(app_state.alloc, &app_state.settings.shader_params);
    app_state.should_load_shader = true;
    app_state.should_clear_shader = false;
}

fn drawBorderShaderPresetRow(ui: *UI, app_state: *AppState) void {
    const can_clear_border_shader = app_state.settings.border_shader_preset_path != null or app_state.border_shader_loading;

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
            if (builtin.abi.isAndroid()) {
                app_state.openShaderFilePicker(.border);
            } else {
                const default_location = std.process.getCwdAlloc(ui.main_window.ctx.frameAlloc()) catch
                    @panic("Failed to allocate");
                defer ui.main_window.ctx.frameAlloc().free(default_location);

                c.SDL_ShowOpenFileDialog(
                    border_shader_dialog_callback,
                    clay.anytypeToAnyopaquePtr(app_state),
                    ui.main_window.ptr,
                    &shader_filter_list,
                    shader_filter_list.len,
                    default_location.ptr,
                    false,
                );
            }
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
            if (app_state.settings.border_shader_preset_path) |path| {
                app_state.alloc.free(path);
                app_state.settings.border_shader_preset_path = null;
            }
            settings.clearShaderParamSettings(app_state.alloc, &app_state.settings.border_shader_params);
            app_state.should_load_border_shader = false;
            app_state.should_clear_border_shader = true;
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

    const show_border_preview = app_state.settings.border_shader_preset_path != null or
        app_state.border_shader_loading;
    if (show_border_preview) {
        const preview_aspect_ratio: viewport.AspectRatio = switch (app_state.settings.aspect_ratio) {
            .none => .@"4_3",
            else => app_state.settings.aspect_ratio,
        };
        const preview_pixels: []const u8 = if (app_state.emulation_running)
            app_state.framePixels(OVERSCAN_PIXEL_OFFSET, NES_VISIBLE_PIXEL_BYTES)
        else
            createStoppedPreviewPlaceholder(ui.current_window.ctx.frameAlloc());
        const center = ui.row(.{ .sizing = .{ .w = .grow, .h = .fit }, .child_alignment = .{ .x = .center } });
        drawShaderPreview(ui, preview_pixels, NES_WIDTH, NES_VISIBLE_HEIGHT, .border, preview_aspect_ratio);
        center.end();
    }
}

fn border_shader_dialog_callback(userdata: ?*anyopaque, filelist: [*c]const [*c]const u8, _: c_int) callconv(.c) void {
    const app_state = clay.anyopaquePtrToType(*AppState, userdata);

    if (filelist == null) {
        std.log.err("An error ocurred while selecting border shader file: {s}\n", .{c.SDL_GetError()});
        return;
    }
    if (filelist.* == null) {
        return;
    }

    const filepath = std.mem.span(filelist.*);
    std.log.debug("User selected border shader: {s}", .{filepath});

    if (app_state.settings.border_shader_preset_path) |old_path| {
        app_state.alloc.free(old_path);
    }
    app_state.settings.border_shader_preset_path = app_state.alloc.dupe(u8, filepath) catch @panic("Failed to allocate!");
    settings.clearShaderParamSettings(app_state.alloc, &app_state.settings.border_shader_params);
    app_state.should_load_border_shader = true;
    app_state.should_clear_border_shader = false;
}

fn file_picker_callback(userdata: ?*anyopaque, filelist: [*c]const [*c]const u8, _: c_int) callconv(.c) void {
    if (filelist == null) {
        std.log.err("An error ocurred while selecting the file: {s}\n", .{c.SDL_GetError()});
        return;
    }
    if (filelist.* == null) { // A pointer to NULL, the user either didn't choose any file or canceled the dialog.
        return;
    }
    const data = clay.anyopaquePtrToType(*FilePickerCallbackData, userdata);
    const ui = data.ui;
    const app_state = data.app_state;
    defer ui.main_window.ctx.persistent_arena.allocator().destroy(data);

    const filepath = std.mem.span(filelist.*);
    std.log.debug("User selected file: {s}", .{filepath});
    app_state.loadRom(filepath) catch |err| std.debug.panic("Failed to load selected ROM: {any}\n", .{err});

    if (builtin.abi.isAndroid()) ui.setWindowFullscreen(true);
}
