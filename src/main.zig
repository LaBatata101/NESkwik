const std = @import("std");
const ness = @import("ness");

const c = ness.c;
const gui = ness.gui;
const Rom = ness.Rom;
const UI = ness.ui.UI;
const trace = ness.trace;
const debug = ness.debug;
const System = ness.System;
const widgets = ness.ui.widgets;
const FPSManager = ness.render.FPSManager;
const sdlError = ness.sdlError;

const CURSOR_HIDE_DELAY_MS = 3000;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}).init;
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var args = try std.process.argsWithAllocator(allocator);
    defer args.deinit();

    var ui = try UI.init(allocator, "NESkwik", 1280, 720);
    defer ui.deinit();
    var ui_state = gui.UIState.init(allocator);
    defer ui_state.deinit();

    var debug_mode = false;
    var step_mode = false;
    _ = args.skip();
    if (args.next()) |arg0| {
        if (std.mem.eql(u8, arg0, "--debug")) {
            debug_mode = true;

            if (args.next()) |arg1| {
                if (std.mem.eql(u8, arg1, "--step")) {
                    step_mode = true;
                    if (args.next()) |arg2| {
                        try ui_state.loadRom(arg2);
                    } else {
                        std.debug.print("ROM file path not provided\n", .{});
                        std.process.exit(1);
                    }
                } else {
                    try ui_state.loadRom(arg1);
                }
            } else {
                std.debug.print("ROM file path not provided\n", .{});
                std.process.exit(1);
            }
        } else {
            try ui_state.loadRom(arg0);
        }
        ui_state.render_home_ui = false;
    }

    ui.setFramerate(.{ .limited = 60 });

    var last_mouse_activity_time: u64 = c.SDL_GetTicks();
    var is_cursor_hidden: bool = false;
    var frame_acc: f32 = 0.0;

    while (!ui.shouldClose()) {
        ui.beginFrame();

        gui.drawGUI(ui, &ui_state);

        if (ui_state.emulation_running) {
            const main_window_active = ui.current_window == ui.main_window;
            if (main_window_active) {
                ui_state.system.?.sync_controllers(ui, &ui_state);
                if (ui.isKeyPressed(ui_state.generalBinding(.quit))) ui.quit = true;
                if (ui.isKeyPressed(ui_state.generalBinding(.toggle_step_mode))) step_mode = !step_mode;
                if (ui.isKeyPressed(ui_state.generalBinding(.reset))) ui_state.system.?.reset();
                if (step_mode and ui.isKeyPressed(ui_state.generalBinding(.run_tick))) ui_state.system.?.tick();
                if (step_mode and ui.isKeyPressed(ui_state.generalBinding(.run_frame))) ui_state.system.?.run_frame();
                if (ui.isKeyPressed(ui_state.generalBinding(.toggle_fullscreen))) {
                    if (ui.isWindowFullscreen()) {
                        ui.setWindowFullscreen(false);
                    } else {
                        ui.setWindowFullscreen(true);
                    }
                }
            }

            if (ui.mouseMotion()) {
                last_mouse_activity_time = c.SDL_GetTicks();
                if (is_cursor_hidden) {
                    sdlError(c.SDL_ShowCursor());
                    is_cursor_hidden = false;
                }
            }

            if (!step_mode and !ui_state.paused) {
                const speed = ui_state.settings.emulation_speed;
                frame_acc += speed.multiplier();
                const frames_to_run: u32 = @intFromFloat(frame_acc);
                frame_acc -= @floatFromInt(frames_to_run);
                ui_state.system.?.apu.device.setSpeed(speed.multiplier());
                for (0..frames_to_run) |_| {
                    ui_state.system.?.run_frame();
                }
            }

            if (ui_state.settings.hide_mouse_on_inactivity) {
                if (!is_cursor_hidden and ui.hasPassedSinceMS(last_mouse_activity_time, CURSOR_HIDE_DELAY_MS)) {
                    sdlError(c.SDL_HideCursor());
                    is_cursor_hidden = true;
                }
            } else if (is_cursor_hidden) { // Always show cursor if not in fullscreen
                sdlError(c.SDL_ShowCursor());
                is_cursor_hidden = false;
            }
        }

        ui.endFrame();
    }
}
