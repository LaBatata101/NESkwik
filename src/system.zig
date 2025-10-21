const std = @import("std");
const CPU = @import("cpu.zig").CPU;
const Rom = @import("rom.zig").Rom;

pub const System = struct {
    cpu: CPU,
    quit: bool,
    next_interrupt: u64,

    const Self = @This();

    pub fn init(rom: Rom) Self {
        return .{
            .cpu = CPU.init(rom),
            .quit = false,
            .next_interrupt = 0,
        };
    }

    pub fn tick(self: *Self) void {
        if (self.cpu.cycles >= self.cpu.next_interrupt) {
            self.cpu.update_next_interrupt();
        }

        if (self.cpu.ppu.requested_run_cycle() <= self.cpu.cycles) {
            self.cpu.run_ppu();
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
