const std = @import("std");

const cpu = @import("cpu.zig");
const rom = @import("rom.zig");
const bus = @import("bus.zig");
pub const controller = @import("controller.zig");
pub const render = @import("render.zig");

pub const System = @import("system.zig").System;
pub const SYSTEM_PALLETE = render.SYSTEM_PALETTE;
pub const CPU = cpu.CPU;
pub const Rom = rom.Rom;
pub const Bus = bus.Bus;
pub const PPU = @import("ppu.zig").PPU;
pub const opcodes = @import("opcodes.zig");
pub const trace = @import("trace.zig").trace;

pub const c = @cImport({
    @cInclude("SDL3/SDL.h");
    @cInclude("SDL3/SDL_main.h");
    @cInclude("blip_buf.h");
});

test {
    std.testing.refAllDecls(@This());
}
