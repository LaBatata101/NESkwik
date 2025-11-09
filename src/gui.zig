const std = @import("std");
const c = @import("root.zig").c;
const Color = @import("render.zig").Color;

const FONT = @embedFile("./resources/font.bmp");
const CHAR_WIDTH: f32 = 5;
const CHAR_HEIGTH: f32 = 7;

pub const TextRenderer = struct {
    renderer: *c.SDL_Renderer,
    font_texture: *c.SDL_Texture,
    writer: std.Io.Writer,

    const Self = @This();

    pub fn init(renderer: *c.SDL_Renderer) Self {
        const font_bytes = c.SDL_IOFromConstMem(FONT, FONT.len);
        const font_surface = c.SDL_LoadBMP_IO(font_bytes, true);
        const font_texture = c.SDL_CreateTextureFromSurface(renderer, font_surface);
        c.SDL_DestroySurface(font_surface);

        // _ = c.SDL_SetTextureColorMod(font_texture, 0, 255, 0);
        _ = c.SDL_SetTextureScaleMode(font_texture, c.SDL_SCALEMODE_NEAREST);

        return .{
            .renderer = renderer,
            .font_texture = font_texture,
            .writer = .fixed(&.{}),
        };
    }

    pub fn deinit(self: *Self) void {
        c.SDL_DestroyTexture(self.font_texture);
    }
    pub fn render(self: *Self, x: f32, y: f32, text: []const u8) void {
        self.render_with_color(x, y, text, Color.WHITE);
    }

    pub fn render_with_color(self: *Self, x: f32, y: f32, text: []const u8, color: Color) void {
        var pos_x: f32 = x;
        var pos_y: f32 = y;

        const x_padding = 1;
        const y_padding = 1.5;
        const scale = 1;

        var old_r: u8 = undefined;
        var old_g: u8 = undefined;
        var old_b: u8 = undefined;
        _ = c.SDL_GetTextureColorMod(self.font_texture, &old_r, &old_g, &old_b);
        _ = c.SDL_SetTextureColorMod(self.font_texture, color.r, color.g, color.b);

        for (text) |char| {
            if (char == '\n') {
                pos_x = x;
                pos_y += CHAR_HEIGTH + y_padding;
                continue;
            }

            const src = c.SDL_FRect{
                .x = @floatFromInt(((char - 32) % 18) * 7 + 1),
                .y = @floatFromInt(((char - 32) / 18) * 9 + 1),
                .w = CHAR_WIDTH,
                .h = CHAR_HEIGTH,
            };
            const dest = c.SDL_FRect{
                .x = pos_x,
                .y = pos_y,
                .w = CHAR_WIDTH * scale,
                .h = CHAR_HEIGTH * scale,
            };
            _ = c.SDL_RenderTexture(
                self.renderer,
                self.font_texture,
                &src,
                &dest,
            );
            pos_x += CHAR_WIDTH * scale + x_padding;
        }

        _ = c.SDL_SetTextureColorMod(self.font_texture, old_r, old_g, old_b);
    }

    pub fn render_fmt(self: *Self, x: f32, y: f32, comptime fmt: []const u8, args: anytype) void {
        self.render_fmt_with_color(x, y, fmt, args, Color.WHITE);
    }

    pub fn render_fmt_with_color(
        self: *Self,
        x: f32,
        y: f32,
        comptime fmt: []const u8,
        args: anytype,
        color: Color,
    ) void {
        var debug_text_buffer: [1024]u8 = undefined;
        var fba = std.io.fixedBufferStream(&debug_text_buffer);
        std.fmt.format(fba.writer(), fmt, args) catch @panic("Failed to format");
        self.render_with_color(x, y, fba.getWritten(), color);
    }
};
