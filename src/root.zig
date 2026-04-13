const std = @import("std");

const cpu = @import("cpu.zig");
const rom = @import("rom.zig");
pub const render = @import("render.zig");
pub const opcodes = @import("opcodes.zig");
pub const controller = @import("controller.zig");
pub const ui = @import("ui/core/ui.zig");
pub const gui = @import("ui/gui.zig");

pub const PPU = @import("ppu.zig").PPU;
pub const APU = @import("apu/apu.zig").APU;
pub const trace = @import("trace.zig");
pub const System = @import("system.zig").System;
pub const SDLAudioOut = @import("sdl_audio.zig").SDLAudioOut;

pub const CPU = cpu.CPU;
pub const Rom = rom.Rom;
pub const SYSTEM_PALLETE = render.SYSTEM_PALETTE;

pub const sdlError = @import("utils/sdl.zig").sdlError;
pub const mmap = @import("utils/mmap.zig");
pub const ThreadPool = @import("utils/pool.zig");
pub const vulkan = @import("utils/vulkan.zig");

pub const c = @cImport({
    @cInclude("SDL3/SDL.h");
    @cInclude("blip_buf.h");
    @cInclude("SDL3_ttf/SDL_ttf.h");
    @cInclude("glslang/Include/glslang_c_interface.h");
    @cInclude("glslang/Public/resource_limits_c.h");
    @cInclude("vulkan/vulkan.h");
    @cInclude("spirv_cross_c.h");
});

pub const NES_WIDTH = 256;
pub const NES_HEIGHT = 240;
pub const DEBUG_WIDTH = 250;

test {
    std.testing.refAllDecls(@This());
}
