const std = @import("std");
const ness = @import("ness");

const c = ness.c;
const Rom = ness.Rom;
const System = ness.System;
const FPSManager = ness.render.FPSManager;
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

    var src_rect = c.SDL_FRect{
        .x = 0.0,
        .y = 8.5, // Start 8 pixels DOWN
        .w = 256.0, // Full width
        .h = 224.0, // 240 - 8 (top) - 8 (bottom) = 224 lines
    };

    // Destination rect: Draw the 224-line image centered in the 240-line space (offset by 8 pixels)
    var dst_rect = c.SDL_FRect{
        .x = 0.0,
        .y = 0.0,
        .w = 256.0,
        .h = 240.0,
    };

    var rom = Rom.init(allocator, rom_abs_path, buffer) catch |err| switch (err) {
        error.InvalidNesFormat => {
            std.debug.print("ROM format not supported!\n", .{});
            std.process.exit(1);
        },
        error.OutOfMemory => {
            std.debug.print("Error allocating resources for ROM\n", .{});
            std.process.exit(1);
        },
        else => {
            std.debug.print("another error ocurred: {any}\n", .{err});
            std.process.exit(1);
        },
    };
    defer rom.deinit();

    var system = try System.init(allocator, &rom);
    defer system.deinit();
    system.reset();

    var fps_manager = FPSManager.init();
    fps_manager.setFramerate(60);

    while (!system.quit) {
        process_input(&system);
        system.run_frame();

        _ = c.SDL_RenderClear(renderer);
        _ = c.SDL_UpdateTexture(texture, null, system.frame_buffer(), 256 * 3);
        _ = c.SDL_RenderTexture(renderer, texture, &src_rect, &dst_rect);
        _ = c.SDL_RenderPresent(renderer);
        _ = fps_manager.delay();
    }
}

fn process_input(system: *System) void {
    var event: c.SDL_Event = undefined;
    while (c.SDL_PollEvent(&event)) {
        switch (event.type) {
            c.SDL_EVENT_QUIT => system.quit = true,
            c.SDL_EVENT_KEY_DOWN => switch (event.key.key) {
                c.SDLK_ESCAPE => system.quit = true,
                c.SDLK_R => system.reset(),
                else => |key_code| system.controller_keydown(key_code),
            },
            c.SDL_EVENT_KEY_UP => system.controller_keyup(event.key.key),
            else => {},
        }
    }
}

fn sdlPanic() noreturn {
    const str = @as(?[*:0]const u8, c.SDL_GetError()) orelse "unknown error";
    @panic(std.mem.sliceTo(str, 0));
}
