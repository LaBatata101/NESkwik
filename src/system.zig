const std = @import("std");

const c = @import("root.zig").c;
const Rom = @import("rom.zig").Rom;
const Bus = @import("bus.zig").Bus;
const CPU = @import("cpu.zig").CPU;
const PPU = @import("ppu.zig").PPU;
const APU = @import("apu/apu.zig").APU;
const Keys = @import("ui/core/ui.zig").Key;
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
    keymap1: std.AutoHashMap(Keys, ControllerButton),
    // Keymap for controller 2.
    keymap2: std.AutoHashMap(Keys, ControllerButton),

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
        apu.* = try APU.init(allocator, try SDLAudioOut.init(allocator), rom);
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
        std.AutoHashMap(Keys, ControllerButton),
        std.AutoHashMap(Keys, ControllerButton),
    } {
        var keymap1 = std.AutoHashMap(Keys, ControllerButton).init(allocator);

        try keymap1.put(.DOWN, .{ .DOWN = true });
        try keymap1.put(.UP, .{ .UP = true });
        try keymap1.put(.RIGHT, .{ .RIGHT = true });
        try keymap1.put(.LEFT, .{ .LEFT = true });
        try keymap1.put(.RETURN, .{ .START = true });
        try keymap1.put(.SPACE, .{ .SELECT = true });
        try keymap1.put(.Z, .{ .BUTTON_A = true });
        try keymap1.put(.X, .{ .BUTTON_B = true });

        var keymap2 = std.AutoHashMap(Keys, ControllerButton).init(allocator);

        try keymap2.put(.S, .{ .DOWN = true });
        try keymap2.put(.W, .{ .UP = true });
        try keymap2.put(.D, .{ .RIGHT = true });
        try keymap2.put(.A, .{ .LEFT = true });
        try keymap2.put(.P, .{ .START = true });
        try keymap2.put(.U, .{ .SELECT = true });
        try keymap2.put(.I, .{ .BUTTON_A = true });
        try keymap2.put(.O, .{ .BUTTON_B = true });

        return .{ keymap1, keymap2 };
    }

    pub fn tick(self: *Self) void {
        self.ppu.tick();
        self.ppu.tick();
        self.ppu.tick();

        if (self.bus.rom.mapper_irq_active() or self.apu.irq_triggered()) {
            self.cpu.interrupt(CPU.IRQ);
        }

        if (self.ppu.nmi_interrupt) {
            self.ppu.nmi_interrupt = false;
            self.cpu.interrupt(CPU.NMI);
        }

        self.cpu.step();
        self.apu.step();
        self.bus.cycles += 1;
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
        self.apu.step(self.bus.cycles);
        if (self.apu.irq_interrupt) {
            self.cpu.interrupt(CPU.IRQ);
        }
    }

    pub fn reset(self: *Self) void {
        self.cpu.reset();
        self.bus.reset();
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

    pub fn frame_buffer(self: *Self) []const u8 {
        return &self.ppu.frame_buffer.data;
    }

    pub fn controller_keydown(self: *Self, key_code: Keys) void {
        if (self.keymap1.get(key_code)) |key| {
            self.bus.controllers.cntrl1_status.insert(key);
        }
        if (self.keymap2.get(key_code)) |key| {
            self.bus.controllers.cntrl2_status.insert(key);
        }
    }

    pub fn controller_keyup(self: *Self, key_code: Keys) void {
        if (self.keymap1.get(key_code)) |key| {
            self.bus.controllers.cntrl1_status.remove(key);
        }
        if (self.keymap2.get(key_code)) |key| {
            self.bus.controllers.cntrl2_status.remove(key);
        }
    }
};
