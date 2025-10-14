const std = @import("std");
const ness = @import("8bit_emulator");

const c = ness.c;
const System = ness.System;
const Bus = ness.Bus;
const PPU = ness.PPU;
const CPU = ness.CPU;
const Rom = ness.Rom;
const Frame = ness.render.Frame;
const FPSManager = ness.render.FPSManager;
const ControllerButton = ness.controller.ControllerButton;
const SYSTEM_PALLETE = ness.SYSTEM_PALLETE;
const trace = ness.trace;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
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

    if (!c.SDL_Init(c.SDL_INIT_VIDEO | c.SDL_INIT_EVENTS)) {
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

    const rom = Rom.load(buffer) catch |err| switch (err) {
        error.InvalidNesFormat => {
            std.debug.print("ROM format not supported!\n", .{});
            std.process.exit(1);
        },
    };

    var keymap = std.AutoHashMap(u32, ControllerButton).init(allocator);
    defer keymap.deinit();

    try keymap.put(c.SDLK_DOWN, .{ .DOWN = true });
    try keymap.put(c.SDLK_UP, .{ .UP = true });
    try keymap.put(c.SDLK_RIGHT, .{ .RIGHT = true });
    try keymap.put(c.SDLK_LEFT, .{ .LEFT = true });
    try keymap.put(c.SDLK_RETURN, .{ .START = true });
    try keymap.put(c.SDLK_SPACE, .{ .SELECT = true });
    try keymap.put(c.SDLK_A, .{ .BUTTON_A = true });
    try keymap.put(c.SDLK_S, .{ .BUTTON_B = true });

    var bus = Bus.init(rom);
    const cpu = CPU.init(&bus);
    var system = System.init(&bus, cpu);
    system.reset();

    var fps_manager = FPSManager.init();
    fps_manager.setFramerate(60);

    var continue_exec = true;
    while (continue_exec) {
        var event: c.SDL_Event = undefined;
        while (c.SDL_PollEvent(&event)) {
            switch (event.type) {
                c.SDL_EVENT_QUIT => continue_exec = false,
                c.SDL_EVENT_KEY_DOWN => switch (event.key.key) {
                    c.SDLK_ESCAPE => continue_exec = false,
                    else => |key_code| {
                        if (keymap.get(key_code)) |key| {
                            system.bus.controller1.button_status.insert(key);
                        }
                    },
                },
                c.SDL_EVENT_KEY_UP => switch (event.key.key) {
                    else => |key_code| {
                        if (keymap.get(key_code)) |key| {
                            system.bus.controller1.button_status.remove(key);
                        }
                    },
                },
                else => {},
            }
        }

        while (!system.is_frame_complete()) {
            system.tick();
        }

        _ = c.SDL_RenderClear(renderer);
        _ = c.SDL_UpdateTexture(texture, null, &bus.ppu.frame_buffer.data, 256 * 3);
        _ = c.SDL_RenderTexture(renderer, texture, null, null);
        _ = c.SDL_RenderPresent(renderer);
        _ = fps_manager.delay();
    }
}

fn sdlPanic() noreturn {
    const str = @as(?[*:0]const u8, c.SDL_GetError()) orelse "unknown error";
    @panic(std.mem.sliceTo(str, 0));
}
