const std = @import("std");
const ness = @import("ness");

const c = ness.c;
const Rom = ness.Rom;
const trace = ness.trace;
const debug = ness.debug;
const System = ness.System;
const FPSManager = ness.render.FPSManager;
const TextRenderer = ness.gui.TextRenderer;
const sdlError = ness.utils.sdlError;

const SCALE = 3;
const CURSOR_HIDE_DELAY_MS = 3000;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}).init;
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var args = try std.process.argsWithAllocator(allocator);
    defer args.deinit();

    var debug_mode = false;
    var step_mode = false;
    var rom_filepath: []const u8 = undefined;
    _ = args.skip();
    if (args.next()) |arg0| {
        if (std.mem.eql(u8, arg0, "--debug")) {
            debug_mode = true;

            if (args.next()) |arg1| {
                if (std.mem.eql(u8, arg1, "--step")) {
                    step_mode = true;
                    if (args.next()) |arg2| {
                        rom_filepath = arg2;
                    } else {
                        std.debug.print("ROM file path not provided\n", .{});
                        std.process.exit(1);
                    }
                } else {
                    rom_filepath = arg1;
                }
            } else {
                std.debug.print("ROM file path not provided\n", .{});
                std.process.exit(1);
            }
        } else {
            rom_filepath = arg0;
        }
    } else {
        std.debug.print("ROM file path not provided\n", .{});
        std.process.exit(1);
    }

    const rom_abs_path = std.fs.cwd().realpathAlloc(allocator, rom_filepath) catch |err| switch (err) {
        error.FileNotFound => {
            std.debug.print("File not found!\n", .{});
            std.process.exit(1);
        },
        else => {
            std.debug.print("Another error ocurred: {any}\n", .{err});
            std.process.exit(1);
        },
    };
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

    const buffer = try allocator.alloc(u8, file_size);
    defer allocator.free(buffer);

    _ = try file.read(buffer);

    if (!c.SDL_Init(c.SDL_INIT_VIDEO | c.SDL_INIT_AUDIO)) {
        std.debug.print("Failed to initialize SDL: {s}\n", .{c.SDL_GetError()});
        return;
    }
    defer c.SDL_Quit();

    const window_width = (ness.NES_WIDTH + if (debug_mode) @as(c_int, ness.DEBUG_WIDTH) else 0) * SCALE;
    const window_height = ness.NES_HEIGHT * SCALE;
    const window = sdlError(c.SDL_CreateWindow("NESS 0.1", window_width, window_height, c.SDL_WINDOW_RESIZABLE | c.SDL_WINDOW_HIGH_PIXEL_DENSITY));
    defer c.SDL_DestroyWindow(window);

    const renderer = sdlError(c.SDL_CreateRenderer(window, null));
    defer c.SDL_DestroyRenderer(renderer);

    const logical_w = ness.NES_WIDTH + if (debug_mode) @as(c_int, ness.DEBUG_WIDTH) else 0;
    const logical_h = ness.NES_HEIGHT;

    sdlError(c.SDL_SetRenderLogicalPresentation(
        renderer,
        logical_w,
        logical_h,
        c.SDL_LOGICAL_PRESENTATION_LETTERBOX,
    ));

    const texture = c.SDL_CreateTexture(renderer, c.SDL_PIXELFORMAT_RGB24, c.SDL_TEXTUREACCESS_STREAMING, 256, 240);
    defer c.SDL_DestroyTexture(texture);

    const pt_texture0 = c.SDL_CreateTexture(renderer, c.SDL_PIXELFORMAT_RGB24, c.SDL_TEXTUREACCESS_STREAMING, 128, 128);
    defer c.SDL_DestroyTexture(pt_texture0);

    const pt_texture1 = c.SDL_CreateTexture(renderer, c.SDL_PIXELFORMAT_RGB24, c.SDL_TEXTUREACCESS_STREAMING, 128, 128);
    defer c.SDL_DestroyTexture(pt_texture1);

    sdlError(c.SDL_SetTextureScaleMode(texture, c.SDL_SCALEMODE_NEAREST));
    sdlError(c.SDL_SetTextureScaleMode(pt_texture0, c.SDL_SCALEMODE_NEAREST));
    sdlError(c.SDL_SetTextureScaleMode(pt_texture1, c.SDL_SCALEMODE_NEAREST));

    const src_rect = c.SDL_FRect{
        .x = 0.0,
        .y = 8.5, // Start 8 pixels DOWN
        .w = ness.NES_WIDTH, // Full width
        .h = ness.NES_HEIGHT - 16, // 240 - 8 (top) - 8 (bottom) = 224 lines
    };

    // Destination rect: Draw the 224-line image centered in the 240-line space (offset by 8 pixels)
    const dst_rect = c.SDL_FRect{
        .x = 0.0,
        .y = 0.0,
        .w = ness.NES_WIDTH,
        .h = if (debug_mode) 160.0 else ness.NES_HEIGHT,
    };

    var rom = Rom.init(allocator, rom_abs_path, buffer) catch |err| switch (err) {
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
    defer rom.deinit();

    var system = try System.init(allocator, &rom, .{ .trace_cpu = false });
    defer system.deinit();
    system.reset();

    var fps_manager = FPSManager.init();
    fps_manager.setFramerate(60);

    var text = TextRenderer.init(renderer);
    defer text.deinit();

    var last_mouse_activity_time: u64 = c.SDL_GetTicks();
    var is_cursor_hidden: bool = false;

    while (!system.quit) {
        process_input(window, &system, &step_mode, &last_mouse_activity_time, &is_cursor_hidden);
        if (!step_mode) {
            system.run_frame();
        }

        sdlError(c.SDL_RenderClear(renderer));
        sdlError(c.SDL_UpdateTexture(texture, null, system.frame_buffer(), ness.NES_WIDTH * SCALE));
        sdlError(c.SDL_RenderTexture(renderer, texture, &src_rect, &dst_rect));

        if (debug_mode) {
            debug.render_debug_mode(
                renderer,
                &system,
                &text,
                rom.prg_rom,
                pt_texture0,
                pt_texture1,
            );
        }

        sdlError(c.SDL_RenderPresent(renderer));
        _ = fps_manager.delay();

        const now = c.SDL_GetTicks();
        const flags = c.SDL_GetWindowFlags(window);
        const is_fullscreen = (flags & c.SDL_WINDOW_FULLSCREEN) != 0;

        if (is_fullscreen) {
            if (!is_cursor_hidden and (now - last_mouse_activity_time > CURSOR_HIDE_DELAY_MS)) {
                sdlError(c.SDL_HideCursor());
                is_cursor_hidden = true;
            }
        } else if (is_cursor_hidden) { // Always show cursor if not in fullscreen
            sdlError(c.SDL_ShowCursor());
            is_cursor_hidden = false;
            last_mouse_activity_time = now;
        }
    }
}

fn process_input(
    window: ?*c.SDL_Window,
    system: *System,
    step_mode: *bool,
    last_mouse_activity_time: *u64,
    is_cursor_hidden: *bool,
) void {
    var event: c.SDL_Event = undefined;
    while (c.SDL_PollEvent(&event)) {
        switch (event.type) {
            c.SDL_EVENT_QUIT => system.quit = true,
            c.SDL_EVENT_KEY_DOWN => switch (event.key.key) {
                c.SDLK_ESCAPE => system.quit = true,
                c.SDLK_F9 => step_mode.* = !step_mode.*,
                c.SDLK_F10 => system.tick(),
                c.SDLK_F11 => system.run_frame(),
                c.SDLK_R => system.reset(),
                c.SDLK_F => {
                    const flags = c.SDL_GetWindowFlags(window);
                    if (flags & c.SDL_WINDOW_FULLSCREEN != 0) {
                        sdlError(c.SDL_SetWindowFullscreen(window, false));
                    } else {
                        sdlError(c.SDL_SetWindowFullscreen(window, true));
                    }
                },
                else => |key_code| system.controller_keydown(key_code),
            },
            c.SDL_EVENT_MOUSE_MOTION => {
                last_mouse_activity_time.* = c.SDL_GetTicks();
                if (is_cursor_hidden.*) {
                    sdlError(c.SDL_ShowCursor());
                    is_cursor_hidden.* = false;
                }
            },
            c.SDL_EVENT_KEY_UP => system.controller_keyup(event.key.key),
            else => {},
        }
    }
}
