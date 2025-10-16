const std = @import("std");
const CPU = @import("cpu.zig").CPU;
const APU = @import("apu/apu.zig").APU;
const SDLAudioOut = @import("sdl_audio.zig").SDLAudioOut;
const Rom = @import("rom.zig").Rom;

pub const System = struct {
    cpu: CPU,
    // apu: APU,
    quit: bool,

    const Self = @This();

    pub fn init(rom: Rom, apu: *APU) Self {
        // const device = try SDLAudioOut.init(alloc);
        return .{
            .cpu = CPU.init(rom, apu),
            // .apu = try APU.init(alloc, device),
            .quit = false,
        };
    }

    // pub fn deinit(self: *Self) void {
    //     self.apu.deinit();
    // }

    pub fn tick(self: *Self) void {
        if (self.cpu.ppu.requested_run_cycle() <= self.cpu.cycles) {
            self.cpu.run_ppu();
        }
        if (self.cpu.apu.requested_run_cycle() <= self.cpu.cycles) {
            self.cpu.run_apu();
        }

        self.cpu.tick();
    }

    pub fn run_frame(self: *Self) void {
        while (!self.is_frame_complete()) {
            self.tick();
        }
    }

    pub fn reset(self: *Self) void {
        self.cpu.reset();
        self.quit = false;
    }

    pub fn is_frame_complete(self: *Self) bool {
        if (self.cpu.ppu.frame_complete) {
            self.cpu.ppu.frame_complete = false;
            return true;
        }
        return false;
    }

    pub fn frame_buffer(self: *Self) *const u8 {
        return @ptrCast(&self.cpu.ppu.frame_buffer.data);
    }
};
