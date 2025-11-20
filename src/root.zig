const std = @import("std");

const cpu = @import("cpu.zig");
const rom = @import("rom.zig");
pub const gui = @import("gui.zig");
pub const debug = @import("debug.zig");
pub const render = @import("render.zig");
pub const opcodes = @import("opcodes.zig");
pub const controller = @import("controller.zig");

pub const PPU = @import("ppu.zig").PPU;
pub const APU = @import("apu/apu.zig").APU;
pub const trace = @import("trace.zig");
pub const System = @import("system.zig").System;
pub const SDLAudioOut = @import("sdl_audio.zig").SDLAudioOut;

pub const CPU = cpu.CPU;
pub const Rom = rom.Rom;
pub const SYSTEM_PALLETE = render.SYSTEM_PALETTE;

pub const utils = @import("utils/sdl.zig");

pub const c = @cImport({
    @cInclude("SDL3/SDL.h");
    @cInclude("SDL3/SDL_main.h");
    @cInclude("blip_buf.h");
});

pub const NES_WIDTH = 256;
pub const NES_HEIGHT = 240;
pub const DEBUG_WIDTH = 250;

test {
    std.testing.refAllDecls(@This());
}
