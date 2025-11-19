const std = @import("std");

const c = @import("root.zig").c;
const Rom = @import("rom.zig").Rom;
const Bus = @import("bus.zig").Bus;
const CPU = @import("cpu.zig").CPU;
const PPU = @import("ppu.zig").PPU;
const APU = @import("apu/apu.zig").APU;
const SDLAudioOut = @import("sdl_audio.zig").SDLAudioOut;
const ControllerButton = @import("controller.zig").ControllerButton;
const trace = @import("trace.zig").trace;

pub const Settings = struct {
    trace_cpu: bool = false,
    disable_audio: bool = false,
};

pub const System = struct {
    allocator: std.mem.Allocator,

    bus: *Bus,
    cpu: *CPU,
    apu: *APU,
    ppu: *PPU,

    // Keymap for controller 1.
    keymap1: std.AutoHashMap(u32, ControllerButton),
    // Keymap for controller 2.
    keymap2: std.AutoHashMap(u32, ControllerButton),

    quit: bool,
    settings: Settings,

    trace_file: ?std.fs.File,
    trace_file_writer: ?std.fs.File.Writer,
    trace_file_buffer: [4096]u8,

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator, rom: *Rom, settings: Settings) !Self {
        const cpu = try allocator.create(CPU);
        const apu = try allocator.create(APU);
        const ppu = try allocator.create(PPU);
        const bus = try allocator.create(Bus);

        ppu.* = PPU.init(rom);
        apu.* = try APU.init(allocator, try SDLAudioOut.init(allocator));
        bus.* = Bus.init(rom, ppu, apu);
        cpu.* = CPU.init(bus);

        apu.device.disable = settings.disable_audio;

        const keymap1, const keymap2 = try Self.init_keymaps(allocator);

        var trace_file: ?std.fs.File = null;
        var trace_file_writer: ?std.fs.File.Writer = null;
        // const trace_file_buffer = try allocator.alloc(u8, 4096);
        if (settings.trace_cpu) {
            trace_file = try std.fs.cwd().createFile("cpu.trace", .{});
            trace_file_writer = .initStreaming(trace_file.?, &.{});
        }

        return .{
            .allocator = allocator,
            .bus = bus,
            .cpu = cpu,
            .ppu = ppu,
            .apu = apu,
            .quit = false,
            .settings = settings,
            .keymap1 = keymap1,
            .keymap2 = keymap2,
            .trace_file = trace_file,
            .trace_file_writer = trace_file_writer,
            .trace_file_buffer = undefined,
        };
    }

    pub fn deinit(self: *Self) void {
        self.keymap1.deinit();
        self.keymap2.deinit();
        self.apu.deinit();

        self.allocator.destroy(self.cpu);
        self.allocator.destroy(self.ppu);
        self.allocator.destroy(self.apu);
        self.allocator.destroy(self.bus);

        if (self.settings.trace_cpu) {
            self.trace_file.?.close();
        }
    }

    fn init_keymaps(allocator: std.mem.Allocator) !struct {
        std.AutoHashMap(u32, ControllerButton),
        std.AutoHashMap(u32, ControllerButton),
    } {
        var keymap1 = std.AutoHashMap(u32, ControllerButton).init(allocator);

        try keymap1.put(c.SDLK_DOWN, .{ .DOWN = true });
        try keymap1.put(c.SDLK_UP, .{ .UP = true });
        try keymap1.put(c.SDLK_RIGHT, .{ .RIGHT = true });
        try keymap1.put(c.SDLK_LEFT, .{ .LEFT = true });
        try keymap1.put(c.SDLK_RETURN, .{ .START = true });
        try keymap1.put(c.SDLK_SPACE, .{ .SELECT = true });
        try keymap1.put(c.SDLK_Z, .{ .BUTTON_A = true });
        try keymap1.put(c.SDLK_X, .{ .BUTTON_B = true });

        var keymap2 = std.AutoHashMap(u32, ControllerButton).init(allocator);

        try keymap2.put(c.SDLK_S, .{ .DOWN = true });
        try keymap2.put(c.SDLK_W, .{ .UP = true });
        try keymap2.put(c.SDLK_D, .{ .RIGHT = true });
        try keymap2.put(c.SDLK_A, .{ .LEFT = true });
        try keymap2.put(c.SDLK_P, .{ .START = true });
        try keymap2.put(c.SDLK_U, .{ .SELECT = true });
        try keymap2.put(c.SDLK_I, .{ .BUTTON_A = true });
        try keymap2.put(c.SDLK_O, .{ .BUTTON_B = true });

        return .{ keymap1, keymap2 };
    }

    pub fn tick(self: *Self) void {
        if (self.apu.irq_interrupt) {
            self.cpu.interrupt(CPU.IRQ);
        }

        self.cpu.tick();

        if (self.ppu.requested_run_cycle() <= self.bus.cycles) {
            self.run_ppu();
        }
        if (self.apu.requested_run_cycle() <= self.bus.cycles) {
            self.run_apu();
        }
    }

    pub fn run_frame(self: *Self) void {
        while (!self.is_frame_complete()) {
            if (self.settings.trace_cpu) {
                var writer = self.trace_file_writer.?;
                trace(&writer.interface, self.cpu) catch @panic("Failed to trace CPU");
            }
            self.tick();
        }
    }

    fn run_ppu(self: *Self) void {
        self.ppu.run_to(self.bus.cycles);
        if (self.ppu.nmi_interrupt) {
            self.ppu.nmi_interrupt = false;
            self.cpu.interrupt(CPU.NMI);
        }
    }

    fn run_apu(self: *Self) void {
        self.apu.run_to(self.bus.cycles);
        if (self.apu.irq_interrupt) {
            self.cpu.interrupt(CPU.IRQ);
        }
    }

    pub fn reset(self: *Self) void {
        self.cpu.reset();
        self.bus.reset();
        self.quit = false;
    }

    pub fn is_frame_complete(self: *Self) bool {
        if (self.ppu.frame_complete) {
            self.ppu.frame_complete = false;
            if (self.settings.trace_cpu) {
                self.dump_trace();
            }
            return true;
        }
        return false;
    }

    fn dump_trace(self: *const Self) void {
        var writer = self.trace_file_writer.?;
        writer.interface.flush() catch |err| std.debug.panic("Flush failed: {any}\n", .{err});
    }

    pub fn frame_buffer(self: *Self) *const u8 {
        return @ptrCast(&self.ppu.frame_buffer.data);
    }

    pub fn controller_keydown(self: *Self, key_code: u32) void {
        if (self.keymap1.get(key_code)) |key| {
            self.bus.controllers.cntrl1_status.insert(key);
        }
        if (self.keymap2.get(key_code)) |key| {
            self.bus.controllers.cntrl2_status.insert(key);
        }
    }

    pub fn controller_keyup(self: *Self, key_code: u32) void {
        if (self.keymap1.get(key_code)) |key| {
            self.bus.controllers.cntrl1_status.remove(key);
        }
        if (self.keymap2.get(key_code)) |key| {
            self.bus.controllers.cntrl2_status.remove(key);
        }
    }
};
