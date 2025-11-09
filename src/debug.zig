const std = @import("std");
const c = @import("root.zig").c;
const System = @import("system.zig").System;
const TextRenderer = @import("gui.zig").TextRenderer;
const opcodes = @import("opcodes.zig");
const ProcessorStatus = @import("cpu.zig").ProcessorStatus;
const trace = @import("trace.zig");
const Color = @import("render.zig").Color;

const NES_WIDTH = @import("root.zig").NES_WIDTH;
const NES_HEIGHT = @import("root.zig").NES_HEIGHT;
const DEBUG_WIDTH = @import("root.zig").DEBUG_WIDTH;

pub fn render_debug_mode(renderer: *c.SDL_Renderer, system: *System, text: *TextRenderer, prg_rom: []const u8) void {
    _ = c.SDL_SetRenderDrawColor(renderer, 43, 40, 40, 255);
    var debug_bg_rect = c.SDL_FRect{
        .x = @floatFromInt(NES_WIDTH),
        .y = 0.0,
        .w = @floatFromInt(DEBUG_WIDTH),
        .h = @floatFromInt(NES_HEIGHT),
    };
    _ = c.SDL_RenderFillRect(renderer, &debug_bg_rect);

    _ = c.SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    _ = c.SDL_RenderLine(renderer, @floatFromInt(NES_WIDTH), 0.0, @floatFromInt(NES_WIDTH), @floatFromInt(NES_HEIGHT));

    var buffer: [23]u8 = undefined;
    var addr_mode_buffer: [27]u8 = undefined;
    var instructions_to_show: [580]u8 = undefined;
    var addr_buffer: [7]u8 = undefined;
    var fba = std.io.fixedBufferStream(&addr_buffer);

    var pos: usize = 0;
    var op_size: u8 = 0;
    for (0..10) |_| {
        const code = prg_rom[(system.cpu.pc + op_size) % prg_rom.len];
        const opcode = opcodes.OP_CODES[code];

        std.fmt.format(fba.writer(), "${X:04}  ", .{system.cpu.pc + op_size}) catch @panic("Failed to format");
        const addr_str = fba.getWritten();

        op_size += opcode.size();

        const opcode_str = trace.format_opcode(
            system.cpu,
            code,
            system.cpu.pc + op_size,
            &buffer,
        ) catch @panic("Failed to format opcode");

        const addr_mode_str = trace.format_addressing_mode(
            system.cpu,
            opcode,
            system.cpu.pc + op_size + 1,
            &addr_mode_buffer,
        ) catch @panic("Failed to format addressing mode");

        @memcpy(instructions_to_show[pos..][0..addr_str.len], addr_str);
        pos += addr_str.len;
        @memcpy(instructions_to_show[pos..][0..opcode_str.len], opcode_str);
        pos += opcode_str.len;
        @memcpy(instructions_to_show[pos..][0..addr_mode_str.len], addr_mode_str);
        pos += addr_mode_str.len;

        instructions_to_show[pos] = '\n';
        pos += 1;

        fba.reset();
    }

    text.render_fmt(NES_HEIGHT + 20, 5,
        \\A: ${X:02} X: ${X:02} Y: ${X:02}
        \\Status: {s}
        \\
        \\PC: ${X:04} SP: ${X:02}
        \\
        \\> {s}
        \\
        \\================================
        \\PPU:
        \\ - SCANLINE: {}
        \\ - CYCLE: {}
    , .{
        system.cpu.register_a,
        system.cpu.register_x,
        system.cpu.register_y,
        format_status(system.cpu.status),
        system.cpu.pc,
        system.cpu.sp,
        instructions_to_show[0..pos],
        system.bus.ppu.scanline,
        system.bus.ppu.cycle,
    });
}

fn format_status(status: ProcessorStatus) [8]u8 {
    var buffer: [8]u8 = [_]u8{'-'} ** 8;
    if (status.carry_flag) buffer[0] = 'C';
    if (status.zero_flag) buffer[1] = 'Z';
    if (status.interrupt_disable) buffer[2] = 'I';
    if (status.break_command) buffer[4] = 'B';
    if (status.break2) buffer[5] = 'B';
    if (status.overflow_flag) buffer[6] = 'O';
    if (status.negative_flag) buffer[7] = 'N';
    return buffer;
}
