const std = @import("std");

const ness = @import("8bit_emulator");
const Bus = ness.Bus;
const PPU = ness.PPU;
const CPU = ness.CPU;
const Rom = ness.Rom;
const Frame = ness.render.Frame;
const ControllerButton = ness.controller.ControllerButton;
const SYSTEM_PALLETE = ness.SYSTEM_PALLETE;
const trace = ness.trace;

const c = @cImport({
    @cInclude("SDL3/SDL.h");
    @cInclude("SDL3/SDL_main.h");
});

pub const FPSManager = struct {
    framecount: u32,
    rateticks: f32,
    baseticks: u32,
    lastticks: u32,
    rate: u32,

    const Self = @This();
    const FPS_DEFAULT = 30;
    const FPS_LOWER_LIMIT = 1;

    fn getTicks() u32 {
        const ticks: u32 = @intCast(c.SDL_GetTicks());
        return if (ticks == 0) 1 else ticks;
    }

    pub fn init() Self {
        const baseticks = Self.getTicks();
        return .{
            .framecount = 0,
            .rate = FPS_DEFAULT,
            .rateticks = @divTrunc(1000.0, FPS_DEFAULT),
            .baseticks = baseticks,
            .lastticks = baseticks,
        };
    }

    pub fn setFramerate(self: *Self, rate: u32) void {
        if (rate < FPS_LOWER_LIMIT) {
            @panic("Framerate can't be lower than 1.");
        }

        self.framecount = 0;
        self.rate = rate;
        self.rateticks = @divExact(1000.0, @as(f32, @floatFromInt(rate)));
    }

    pub fn delay(self: *Self) u32 {
        self.framecount += 1;

        const current_ticks = Self.getTicks();
        const time_passed = current_ticks - self.lastticks;
        const target_ticks = self.baseticks + self.framecount * @as(u32, @intFromFloat(self.rateticks));

        if (current_ticks <= target_ticks) {
            c.SDL_Delay(target_ticks - current_ticks);
        } else {
            self.framecount = 0;
            self.baseticks = Self.getTicks();
        }

        return time_passed;
    }
};

const System = struct {
    cpu: CPU,
    bus: *Bus,
    system_clock_counter: usize,

    const Self = @This();

    fn init(bus: *Bus, cpu: CPU) Self {
        return .{
            .bus = bus,
            .cpu = cpu,
            .system_clock_counter = 0,
        };
    }

    pub fn tick(self: *Self) void {
        self.bus.ppu.tick();

        // The CPU runs 3 times slower than the PPU, so only execute the CPU every 3 times.
        if (self.system_clock_counter % 3 == 0) {
            // if (self.bus.dma_transfer) {} else {
            self.cpu.tick();
            // }
        }

        if (self.bus.ppu.nmi_interrupt) {
            self.bus.ppu.nmi_interrupt = false;
            self.cpu.interrupt(CPU.NMI);
        }

        self.system_clock_counter += 1;
    }

    pub fn reset(self: *Self) void {
        self.cpu.reset();
        self.system_clock_counter = 0;
    }
};

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

        while (!system.bus.ppu.frame_complete) {
            system.tick();
        }
        system.bus.ppu.frame_complete = false;

        _ = c.SDL_RenderClear(renderer);
        _ = c.SDL_UpdateTexture(texture, null, &bus.ppu.frame_buffer.data, 256 * 3);
        _ = c.SDL_RenderTexture(renderer, texture, null, null);
        _ = c.SDL_RenderPresent(renderer);
        _ = fps_manager.delay();
    }
}

fn processInput(cpu: *CPU) void {
    var event: c.SDL_Event = undefined;
    while (c.SDL_PollEvent(&event)) {
        switch (event.type) {
            c.SDL_EVENT_QUIT => std.process.exit(0),
            c.SDL_EVENT_KEY_DOWN => switch (event.key.key) {
                c.SDLK_ESCAPE => std.process.exit(0),
                c.SDLK_W => cpu.mem_write(0xFF, 0x77),
                c.SDLK_S => cpu.mem_write(0xFF, 0x73),
                c.SDLK_A => cpu.mem_write(0xFF, 0x61),
                c.SDLK_D => cpu.mem_write(0xFF, 0x64),
                else => {},
            },
            else => {},
        }
    }
}

fn sdlPanic() noreturn {
    const str = @as(?[*:0]const u8, c.SDL_GetError()) orelse "unknown error";
    @panic(std.mem.sliceTo(str, 0));
}
