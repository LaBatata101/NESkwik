const std = @import("std");
const c = @import("root.zig").c;
const System = @import("system.zig").System;
const TextRenderer = @import("gui.zig").TextRenderer;
const opcodes = @import("opcodes.zig");
const ProcessorStatus = @import("cpu.zig").ProcessorStatus;
const trace = @import("trace.zig");
const Color = @import("render.zig").Color;
const SYSTEM_PALETTE = @import("render.zig").SYSTEM_PALETTE;

const NES_WIDTH = @import("root.zig").NES_WIDTH;
const NES_HEIGHT = @import("root.zig").NES_HEIGHT;
const DEBUG_WIDTH = @import("root.zig").DEBUG_WIDTH;

pub fn render_debug_mode(
    renderer: *c.SDL_Renderer,
    system: *System,
    text: *TextRenderer,
    prg_rom: []const u8,
    pt_texture0: *c.SDL_Texture,
    pt_texture1: *c.SDL_Texture,
) void {
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

        op_size += opcode.size();

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
        \\ - GLOBAL CYCLE: {}
        \\
        \\SYSTEM CYCLE: {}
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
        system.bus.ppu.global_cycle,
        system.bus.cycles,
    });

    _ = c.SDL_SetRenderDrawColor(renderer, 43, 40, 40, 255);
    const pt_size = 64.0;
    const pt_y = 172.5;
    const pt_x_1 = 10.0;
    const pt_x_2 = pt_x_1 + pt_size + 10.0;

    text.render(pt_x_1, 162.5, "Pattern Tables:");
    const frame_pt0 = system.ppu.get_pattern_table(0, 0);
    const frame_pt1 = system.ppu.get_pattern_table(1, 0);

    const pitch = NES_WIDTH * 3;
    _ = c.SDL_UpdateTexture(pt_texture0, null, &frame_pt0.data, pitch);
    _ = c.SDL_UpdateTexture(pt_texture1, null, &frame_pt1.data, pitch);

    const dest_rect0 = c.SDL_FRect{
        .x = pt_x_1,
        .y = pt_y,
        .w = pt_size,
        .h = pt_size,
    };
    _ = c.SDL_RenderTexture(renderer, pt_texture0, null, &dest_rect0);

    const dest_rect1 = c.SDL_FRect{
        .x = pt_x_2,
        .y = pt_y,
        .w = pt_size,
        .h = pt_size,
    };
    _ = c.SDL_RenderTexture(renderer, pt_texture1, null, &dest_rect1);

    const palette_start_x = pt_x_2 + pt_size + 10.0;
    render_palettes(renderer, system, text, palette_start_x, 162.5);
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

fn render_palettes(
    renderer: *c.SDL_Renderer,
    system: *System,
    text: *TextRenderer,
    start_x: f32,
    start_y: f32,
) void {
    const palette_ram = system.ppu.palette_table;
    const swatch_size = 7.0;
    const padding = 0.0;

    text.render_with_color(start_x, start_y, "BG Palettes:", Color.WHITE);

    const bg_grid_y = start_y + 12.0;
    for (0..4) |palette_num| {
        for (0..4) |color_num| {
            const i = (palette_num * 4) + color_num;

            const palette_index = palette_ram[i];
            const color = SYSTEM_PALETTE[palette_index & 0x3F];
            _ = c.SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

            const grid_x: f32 = @floatFromInt(palette_num % 2);
            const grid_y: f32 = @floatFromInt(palette_num / 2);

            const palette_width = (swatch_size * 4) + (padding * 3);
            const palette_height = swatch_size;

            const x_pos = start_x + (grid_x * (palette_width + padding * 3)) + (@as(f32, @floatFromInt(color_num)) * (swatch_size + padding));
            const y_pos = bg_grid_y + (grid_y * (palette_height + padding * 3));

            var rect = c.SDL_FRect{
                .x = x_pos,
                .y = y_pos,
                .w = swatch_size,
                .h = swatch_size,
            };
            _ = c.SDL_RenderFillRect(renderer, &rect);
        }
    }

    const sp_grid_y = bg_grid_y + (swatch_size * 2) + (padding * 6) + 12.0;
    text.render_with_color(start_x, sp_grid_y - 12.0, "Sprite Palettes:", Color.WHITE);

    for (0..4) |palette_num| {
        for (0..4) |color_num| {
            const i = (palette_num * 4) + color_num;

            var palette_addr: u8 = 0x10 + @as(u8, @truncate(i));
            palette_addr = switch (palette_addr) {
                0x10, 0x14, 0x18, 0x1C => palette_addr - 0x10,
                else => palette_addr,
            };

            const palette_index = palette_ram[palette_addr];
            const color = SYSTEM_PALETTE[palette_index & 0x3F];
            _ = c.SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

            const grid_x: f32 = @floatFromInt(palette_num % 2);
            const grid_y: f32 = @floatFromInt(palette_num / 2);

            const palette_width = (swatch_size * 4) + (padding * 3);
            const palette_height = swatch_size;

            const x_pos = start_x + (grid_x * (palette_width + padding * 3)) + (@as(f32, @floatFromInt(color_num)) * (swatch_size + padding));
            const y_pos = sp_grid_y + (grid_y * (palette_height + padding * 3));

            var rect = c.SDL_FRect{
                .x = x_pos,
                .y = y_pos,
                .w = swatch_size,
                .h = swatch_size,
            };
            _ = c.SDL_RenderFillRect(renderer, &rect);
        }
    }

    _ = c.SDL_SetRenderDrawColor(renderer, 43, 40, 40, 255);
}
