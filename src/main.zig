const std = @import("std");
const ness = @import("ness");

const c = ness.c;
const System = ness.System;
const Rom = ness.Rom;
const APU = ness.APU;
const Frame = ness.render.Frame;
const SDLAudioOut = ness.SDLAudioOut;
const FPSManager = ness.render.FPSManager;
const ControllerButton = ness.controller.ControllerButton;
const SYSTEM_PALLETE = ness.SYSTEM_PALLETE;
const trace = ness.trace;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}).init;
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var args = try std.process.argsWithAllocator(allocator);
    defer args.deinit();

    _ = args.skip();
    const rom_filepath = args.next() orelse {
        std.debug.print("ROM file path not provided\n", .{});
        std.process.exit(1);
    };

    const rom_abs_path = try std.fs.cwd().realpathAlloc(allocator, rom_filepath);
    defer allocator.free(rom_abs_path);

    const file = std.fs.openFileAbsolute(rom_abs_path, .{}) catch |err| switch (err) {
        error.FileNotFound => {
            std.debug.print("file not found!\n", .{});
            std.process.exit(1);
        },
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
        sdlPanic();
    }
    defer c.SDL_Quit();

    const window = c.SDL_CreateWindow("NESS 0.1", 256 * 3, 240 * 3, 0) orelse sdlPanic();
    defer c.SDL_DestroyWindow(window);

    const renderer = c.SDL_CreateRenderer(window, null) orelse sdlPanic();
    defer c.SDL_DestroyRenderer(renderer);

    _ = c.SDL_SetRenderScale(renderer, 3.0, 3.0);

    const texture = c.SDL_CreateTexture(renderer, c.SDL_PIXELFORMAT_RGB24, c.SDL_TEXTUREACCESS_TARGET, 256, 240);
    defer c.SDL_DestroyTexture(texture);

    _ = c.SDL_SetTextureScaleMode(texture, c.SDL_SCALEMODE_NEAREST);

    var rom = Rom.init(allocator, buffer) catch |err| switch (err) {
        error.InvalidNesFormat => {
            std.debug.print("ROM format not supported!\n", .{});
            std.process.exit(1);
        },
        error.OutOfMemory => {
            std.debug.print("Error allocating resources for ROM\n", .{});
            std.process.exit(1);
        },
    };
    defer rom.deinit();

    var cntrl1_keymap = std.AutoHashMap(u32, ControllerButton).init(allocator);
    defer cntrl1_keymap.deinit();

    try cntrl1_keymap.put(c.SDLK_DOWN, .{ .DOWN = true });
    try cntrl1_keymap.put(c.SDLK_UP, .{ .UP = true });
    try cntrl1_keymap.put(c.SDLK_RIGHT, .{ .RIGHT = true });
    try cntrl1_keymap.put(c.SDLK_LEFT, .{ .LEFT = true });
    try cntrl1_keymap.put(c.SDLK_RETURN, .{ .START = true });
    try cntrl1_keymap.put(c.SDLK_SPACE, .{ .SELECT = true });
    try cntrl1_keymap.put(c.SDLK_Q, .{ .BUTTON_A = true });
    try cntrl1_keymap.put(c.SDLK_E, .{ .BUTTON_B = true });

    var cntrl2_keymap = std.AutoHashMap(u32, ControllerButton).init(allocator);
    defer cntrl2_keymap.deinit();

    try cntrl2_keymap.put(c.SDLK_S, .{ .DOWN = true });
    try cntrl2_keymap.put(c.SDLK_W, .{ .UP = true });
    try cntrl2_keymap.put(c.SDLK_D, .{ .RIGHT = true });
    try cntrl2_keymap.put(c.SDLK_A, .{ .LEFT = true });
    try cntrl2_keymap.put(c.SDLK_P, .{ .START = true });
    try cntrl2_keymap.put(c.SDLK_U, .{ .SELECT = true });
    try cntrl2_keymap.put(c.SDLK_I, .{ .BUTTON_A = true });
    try cntrl2_keymap.put(c.SDLK_O, .{ .BUTTON_B = true });

    var apu = try APU.init(allocator, try SDLAudioOut.init(allocator));
    defer apu.deinit();

    var system = System.init(&rom, &apu);
    system.reset();

    var fps_manager = FPSManager.init();
    fps_manager.setFramerate(60);

    while (!system.quit) {
        process_input(&system, &cntrl1_keymap, &cntrl2_keymap);
        system.run_frame();

        _ = c.SDL_RenderClear(renderer);
        _ = c.SDL_UpdateTexture(texture, null, system.frame_buffer(), 256 * 3);
        _ = c.SDL_RenderTexture(renderer, texture, null, null);
        _ = c.SDL_RenderPresent(renderer);
        _ = fps_manager.delay();
    }
}

fn process_input(
    system: *System,
    cntrl1_keymap: *std.AutoHashMap(u32, ControllerButton),
    cntrl2_keymap: *std.AutoHashMap(u32, ControllerButton),
) void {
    var event: c.SDL_Event = undefined;
    while (c.SDL_PollEvent(&event)) {
        switch (event.type) {
            c.SDL_EVENT_QUIT => system.quit = true,
            c.SDL_EVENT_KEY_DOWN => switch (event.key.key) {
                c.SDLK_ESCAPE => system.quit = true,
                else => |key_code| {
                    if (cntrl1_keymap.get(key_code)) |key| {
                        system.cpu.controllers.cntrl1_status.insert(key);
                    }
                    if (cntrl2_keymap.get(key_code)) |key| {
                        system.cpu.controllers.cntrl2_status.insert(key);
                    }
                },
            },
            c.SDL_EVENT_KEY_UP => switch (event.key.key) {
                else => |key_code| {
                    if (cntrl1_keymap.get(key_code)) |key| {
                        system.cpu.controllers.cntrl1_status.remove(key);
                    }
                    if (cntrl2_keymap.get(key_code)) |key| {
                        system.cpu.controllers.cntrl2_status.remove(key);
                    }
                },
            },
            else => {},
        }
    }
}

fn sdlPanic() noreturn {
    const str = @as(?[*:0]const u8, c.SDL_GetError()) orelse "unknown error";
    @panic(std.mem.sliceTo(str, 0));
}
