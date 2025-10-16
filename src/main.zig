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

    const rom = Rom.load(buffer) catch |err| switch (err) {
        error.InvalidNesFormat => {
            std.debug.print("ROM format not supported!\n", .{});
            std.process.exit(1);
        },
    };

    var controller1_keymap = std.AutoHashMap(u32, ControllerButton).init(allocator);
    defer controller1_keymap.deinit();

    try controller1_keymap.put(c.SDLK_DOWN, .{ .DOWN = true });
    try controller1_keymap.put(c.SDLK_UP, .{ .UP = true });
    try controller1_keymap.put(c.SDLK_RIGHT, .{ .RIGHT = true });
    try controller1_keymap.put(c.SDLK_LEFT, .{ .LEFT = true });
    try controller1_keymap.put(c.SDLK_RETURN, .{ .START = true });
    try controller1_keymap.put(c.SDLK_SPACE, .{ .SELECT = true });
    try controller1_keymap.put(c.SDLK_Q, .{ .BUTTON_A = true });
    try controller1_keymap.put(c.SDLK_E, .{ .BUTTON_B = true });

    var controller2_keymap = std.AutoHashMap(u32, ControllerButton).init(allocator);
    defer controller2_keymap.deinit();

    try controller2_keymap.put(c.SDLK_S, .{ .DOWN = true });
    try controller2_keymap.put(c.SDLK_W, .{ .UP = true });
    try controller2_keymap.put(c.SDLK_D, .{ .RIGHT = true });
    try controller2_keymap.put(c.SDLK_A, .{ .LEFT = true });
    try controller2_keymap.put(c.SDLK_P, .{ .START = true });
    try controller2_keymap.put(c.SDLK_U, .{ .SELECT = true });
    try controller2_keymap.put(c.SDLK_I, .{ .BUTTON_A = true });
    try controller2_keymap.put(c.SDLK_O, .{ .BUTTON_B = true });

    var bus = Bus.init(allocator, rom);
    const cpu = CPU.init(&bus);
    var system = System.init(&bus, cpu);
    system.reset();

    var fps_manager = FPSManager.init();
    fps_manager.setFramerate(60);

    var spec = c.SDL_AudioSpec{};
    spec.freq = 48000;
    spec.format = c.SDL_AUDIO_F32;
    spec.channels = 1; // mono

    const audio_stream = c.SDL_OpenAudioDeviceStream(
        c.SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK,
        &spec,
        audio_callback,
        null,
    ) orelse sdlPanic();
    defer c.SDL_DestroyAudioStream(audio_stream);

    _ = c.SDL_ResumeAudioStreamDevice(audio_stream);

    var continue_exec = true;
    while (continue_exec) {
        var event: c.SDL_Event = undefined;
        while (c.SDL_PollEvent(&event)) {
            switch (event.type) {
                c.SDL_EVENT_QUIT => continue_exec = false,
                c.SDL_EVENT_KEY_DOWN => switch (event.key.key) {
                    c.SDLK_ESCAPE => continue_exec = false,
                    else => |key_code| {
                        if (controller1_keymap.get(key_code)) |key| {
                            system.bus.controllers.cntrl1_status.insert(key);
                        }
                        if (controller2_keymap.get(key_code)) |key| {
                            system.bus.controllers.cntrl2_status.insert(key);
                        }
                    },
                },
                c.SDL_EVENT_KEY_UP => switch (event.key.key) {
                    else => |key_code| {
                        if (controller1_keymap.get(key_code)) |key| {
                            system.bus.controllers.cntrl1_status.remove(key);
                        }
                        if (controller2_keymap.get(key_code)) |key| {
                            system.bus.controllers.cntrl2_status.remove(key);
                        }
                    },
                },
                else => {},
            }
        }

        // while (!system.is_frame_complete()) {
        //     system.tick();
        // }

        _ = c.SDL_RenderClear(renderer);
        _ = c.SDL_UpdateTexture(texture, null, &bus.ppu.frame_buffer.data, 256 * 3);
        _ = c.SDL_RenderTexture(renderer, texture, null, null);
        _ = c.SDL_RenderPresent(renderer);
        _ = fps_manager.delay();
    }
}

var current_sine_sample: u32 = 0;

fn audio_callback(
    userdata: ?*anyopaque,
    audio_stream: ?*c.SDL_AudioStream,
    additional_amount: c_int,
    total_amount: c_int,
) callconv(.c) void {
    _ = userdata;
    _ = total_amount;

    const bytes_per_sample: c_int = @sizeOf(f32);
    var samples_to_generate: usize = @intCast(@divTrunc(additional_amount, bytes_per_sample));
    while (samples_to_generate > 0) {
        var samples: [128]f32 = undefined;
        const total: usize = @intCast(@min(samples_to_generate, samples.len));
        for (0..total) |i| {
            const freq: u32 = 440;
            const phase = @as(f32, @floatFromInt(current_sine_sample * freq)) / @as(f32, 48000.0);
            samples[i] = @sin(phase * 2 * std.math.pi);
            current_sine_sample += 1;
        }

        current_sine_sample %= 48000;
        _ = c.SDL_PutAudioStreamData(audio_stream, &samples, @as(c_int, @intCast(total)) * bytes_per_sample);
        samples_to_generate -= total;
    }
}

fn sdlPanic() noreturn {
    const str = @as(?[*:0]const u8, c.SDL_GetError()) orelse "unknown error";
    @panic(std.mem.sliceTo(str, 0));
}
