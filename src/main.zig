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

const SCALE = 3;
const CURSOR_HIDE_DELAY_MS = 3000;

fn init_sdl_systems() void {
    sdlError(c.SDL_Init(c.SDL_INIT_VIDEO | c.SDL_INIT_AUDIO));
    sdlError(c.TTF_Init());
}

fn deinit_sdl_systems() void {
    c.TTF_Quit();
    c.SDL_Quit();
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}).init;
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var args = try std.process.argsWithAllocator(allocator);
    defer args.deinit();

    init_sdl_systems();
    defer deinit_sdl_systems();

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
                        ui_state.setSelectedRom(arg2);
                    } else {
                        std.debug.print("ROM file path not provided\n", .{});
                        std.process.exit(1);
                    }
                } else {
                    ui_state.setSelectedRom(arg1);
                }
            } else {
                std.debug.print("ROM file path not provided\n", .{});
                std.process.exit(1);
            }
        } else {
            ui_state.setSelectedRom(arg0);
        }
        ui_state.render_home_screen = false;
    }

    ui.setFramerate(.{ .limited = 60 });

    var last_mouse_activity_time: u64 = c.SDL_GetTicks();
    var is_cursor_hidden: bool = false;

    while (!ui.shouldClose()) {
        ui.beginFrame();

        if (ui_state.should_load_rom) {
            const rom_path = ui_state.getSelectedRom();
            std.log.debug("Reading file: {s}", .{rom_path});
            const cwd = try std.process.getCwdAlloc(allocator);
            defer allocator.free(cwd);
            const rom_abs_path = try std.fs.path.resolve(allocator, &.{ cwd, rom_path });
            defer allocator.free(rom_abs_path);

            const file = std.fs.openFileAbsolute(rom_abs_path, .{}) catch |err| switch (err) {
                else => {
                    std.debug.print("Error while opening file: {any}\n", .{err});
                    std.process.exit(1);
                },
            };
            defer file.close();

            const file_size = try file.getEndPos();
            try file.seekTo(0);

            ui_state.rom_bytes = try allocator.alloc(u8, file_size);
            _ = try file.read(ui_state.rom_bytes.?);

            ui_state.loadRom(rom_path, ui_state.rom_bytes.?) catch |err| switch (err) {
                error.InvalidNesFormat => {
                    std.debug.print("Error: ROM format not supported!\n", .{});
                    std.process.exit(1);
                },
                error.OutOfMemory => {
                    std.debug.print("Error: Failed to allocate resources for ROM\n", .{});
                    std.process.exit(1);
                },
                error.UnsupportedMapper => {
                    std.debug.print("Error: Mapper not supported\n", .{});
                    std.process.exit(1);
                },
                else => {
                    std.debug.print("Another error ocurred: {any}\n", .{err});
                    std.process.exit(1);
                },
            };
        }

        gui.drawGUI(ui, &ui_state);

        if (ui_state.run_emu) {
            if (ui.getPressedKey()) |key| ui_state.system.controller_keydown(key);
            if (ui.getReleasedKey()) |key| ui_state.system.controller_keyup(key);
            if (ui.isKeyPressed(.ESCAPE)) std.process.exit(0);
            if (ui.isKeyPressed(.F9)) step_mode = !step_mode;
            if (ui.isKeyPressed(.R)) ui_state.system.reset();
            if (step_mode and ui.isKeyPressed(.F10)) ui_state.system.tick();
            if (step_mode and ui.isKeyPressed(.F11)) ui_state.system.run_frame();
            if (ui.isKeyPressed(.F)) {
                const flags = c.SDL_GetWindowFlags(ui.window);
                if (flags & c.SDL_WINDOW_FULLSCREEN != 0) {
                    sdlError(c.SDL_SetWindowFullscreen(ui.window, false));
                } else {
                    sdlError(c.SDL_SetWindowFullscreen(ui.window, true));
                }
            }
            if (ui.mouseMotion()) {
                last_mouse_activity_time = c.SDL_GetTicks();
                if (is_cursor_hidden) {
                    sdlError(c.SDL_ShowCursor());
                    is_cursor_hidden = false;
                }
            }

            ui_state.system.run_frame();

            if (c.SDL_GetWindowFlags(ui.window) & c.SDL_WINDOW_FULLSCREEN != 0) {
                if (!is_cursor_hidden and ui.ctx.hasPassedSinceMS(last_mouse_activity_time, CURSOR_HIDE_DELAY_MS)) {
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
