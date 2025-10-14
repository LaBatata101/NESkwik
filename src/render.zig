const std = @import("std");

const c = @import("root.zig").c;

const Mirroring = @import("rom.zig").Mirroring;
const PPU = @import("ppu.zig").PPU;

// zig fmt: off
pub const SYSTEM_PALETTE: [64]Color = [_]Color{
   Color.RGB(0x80, 0x80, 0x80), Color.RGB(0x00, 0x3D, 0xA6), Color.RGB(0x00, 0x12, 0xB0), Color.RGB(0x44, 0x00, 0x96), Color.RGB(0xA1, 0x00, 0x5E),
   Color.RGB(0xC7, 0x00, 0x28), Color.RGB(0xBA, 0x06, 0x00), Color.RGB(0x8C, 0x17, 0x00), Color.RGB(0x5C, 0x2F, 0x00), Color.RGB(0x10, 0x45, 0x00),
   Color.RGB(0x05, 0x4A, 0x00), Color.RGB(0x00, 0x47, 0x2E), Color.RGB(0x00, 0x41, 0x66), Color.RGB(0x00, 0x00, 0x00), Color.RGB(0x05, 0x05, 0x05),
   Color.RGB(0x05, 0x05, 0x05), Color.RGB(0xC7, 0xC7, 0xC7), Color.RGB(0x00, 0x77, 0xFF), Color.RGB(0x21, 0x55, 0xFF), Color.RGB(0x82, 0x37, 0xFA),
   Color.RGB(0xEB, 0x2F, 0xB5), Color.RGB(0xFF, 0x29, 0x50), Color.RGB(0xFF, 0x22, 0x00), Color.RGB(0xD6, 0x32, 0x00), Color.RGB(0xC4, 0x62, 0x00),
   Color.RGB(0x35, 0x80, 0x00), Color.RGB(0x05, 0x8F, 0x00), Color.RGB(0x00, 0x8A, 0x55), Color.RGB(0x00, 0x99, 0xCC), Color.RGB(0x21, 0x21, 0x21),
   Color.RGB(0x09, 0x09, 0x09), Color.RGB(0x09, 0x09, 0x09), Color.RGB(0xFF, 0xFF, 0xFF), Color.RGB(0x0F, 0xD7, 0xFF), Color.RGB(0x69, 0xA2, 0xFF),
   Color.RGB(0xD4, 0x80, 0xFF), Color.RGB(0xFF, 0x45, 0xF3), Color.RGB(0xFF, 0x61, 0x8B), Color.RGB(0xFF, 0x88, 0x33), Color.RGB(0xFF, 0x9C, 0x12),
   Color.RGB(0xFA, 0xBC, 0x20), Color.RGB(0x9F, 0xE3, 0x0E), Color.RGB(0x2B, 0xF0, 0x35), Color.RGB(0x0C, 0xF0, 0xA4), Color.RGB(0x05, 0xFB, 0xFF),
   Color.RGB(0x5E, 0x5E, 0x5E), Color.RGB(0x0D, 0x0D, 0x0D), Color.RGB(0x0D, 0x0D, 0x0D), Color.RGB(0xFF, 0xFF, 0xFF), Color.RGB(0xA6, 0xFC, 0xFF),
   Color.RGB(0xB3, 0xEC, 0xFF), Color.RGB(0xDA, 0xAB, 0xEB), Color.RGB(0xFF, 0xA8, 0xF9), Color.RGB(0xFF, 0xAB, 0xB3), Color.RGB(0xFF, 0xD2, 0xB0),
   Color.RGB(0xFF, 0xEF, 0xA6), Color.RGB(0xFF, 0xF7, 0x9C), Color.RGB(0xD7, 0xE8, 0x95), Color.RGB(0xA6, 0xED, 0xAF), Color.RGB(0xA2, 0xF2, 0xDA),
   Color.RGB(0x99, 0xFF, 0xFC), Color.RGB(0xDD, 0xDD, 0xDD), Color.RGB(0x11, 0x11, 0x11), Color.RGB(0x11, 0x11, 0x11)
};
// zig fmt: on

pub const Color = struct {
    r: u8,
    g: u8,
    b: u8,
    a: u8,

    fn RGBA(r: u8, g: u8, b: u8, a: u8) @This() {
        return .{ .r = r, .g = g, .b = b, .a = a };
    }

    fn RGB(r: u8, g: u8, b: u8) @This() {
        return Color.RGBA(r, g, b, 255);
    }
};

const Rect = struct {
    x1: usize,
    y1: usize,
    x2: usize,
    y2: usize,
};

pub const Frame = struct {
    data: [WIDTH * HEIGHT * 3]u8,

    const Self = @This();
    const WIDTH: usize = 256;
    const HEIGHT: usize = 240;

    pub fn init() Self {
        return .{ .data = [_]u8{0} ** (WIDTH * HEIGHT * 3) };
    }

    pub fn set_pixel(self: *Self, x: usize, y: usize, rgb: Color) void {
        const base = y * 3 * WIDTH + x * 3;
        if (base + 2 < self.data.len) {
            self.data[base] = rgb.r;
            self.data[base + 1] = rgb.g;
            self.data[base + 2] = rgb.b;
        }
    }
};

pub const FPSManager = struct {
    framecount: u32,
    rateticks: f32,
    baseticks: u32,
    lastticks: u32,
    rate: u32,

    const Self = @This();
    const FPS_DEFAULT = 30;
    const FPS_LOWER_LIMIT = 1;

    fn getTicks() u32 {
        const ticks: u32 = @intCast(c.SDL_GetTicks());
        return if (ticks == 0) 1 else ticks;
    }

    pub fn init() Self {
        const baseticks = Self.getTicks();
        return .{
            .framecount = 0,
            .rate = FPS_DEFAULT,
            .rateticks = @divTrunc(1000.0, FPS_DEFAULT),
            .baseticks = baseticks,
            .lastticks = baseticks,
        };
    }

    pub fn setFramerate(self: *Self, rate: u32) void {
        if (rate < FPS_LOWER_LIMIT) {
            @panic("Framerate can't be lower than 1.");
        }

        self.framecount = 0;
        self.rate = rate;
        self.rateticks = @divExact(1000.0, @as(f32, @floatFromInt(rate)));
    }

    pub fn delay(self: *Self) u32 {
        self.framecount += 1;

        const current_ticks = Self.getTicks();
        const time_passed = current_ticks - self.lastticks;
        const target_ticks = self.baseticks + self.framecount * @as(u32, @intFromFloat(self.rateticks));

        if (current_ticks <= target_ticks) {
            c.SDL_Delay(target_ticks - current_ticks);
        } else {
            self.framecount = 0;
            self.baseticks = Self.getTicks();
        }

        return time_passed;
    }
};

fn bg_pallette(ppu: *const PPU, attr_table: *const [64]u8, tile_column: usize, tile_row: usize) struct { u8, u8, u8, u8 } {
    const attr_table_idx = tile_row / 4 * 8 + tile_column / 4;
    const attr_byte = attr_table[attr_table_idx];

    const a = tile_column % 4 / 2;
    const b = tile_row % 4 / 2;

    var pallet_idx: u8 = undefined;
    if (a == 0 and b == 0) {
        pallet_idx = attr_byte & 0b11;
    } else if (a == 1 and b == 0) {
        pallet_idx = (attr_byte >> 2) & 0b11;
    } else if (a == 0 and b == 1) {
        pallet_idx = (attr_byte >> 4) & 0b11;
    } else if (a == 1 and b == 1) {
        pallet_idx = (attr_byte >> 6) & 0b11;
    } else {
        std.debug.panic("Should not happen {}, {}\n", .{ a, b });
    }

    const pallet_start = pallet_idx * 4 + 1;

    return .{
        ppu.palette_table[0],
        ppu.palette_table[pallet_start],
        ppu.palette_table[pallet_start + 1],
        ppu.palette_table[pallet_start + 2],
    };
}

fn sprite_pallette(ppu: *const PPU, pallette_idx: u8) struct { u8, u8, u8, u8 } {
    const start = 0x11 + pallette_idx * 4;
    return .{
        0,
        ppu.palette_table[start],
        ppu.palette_table[start + 1],
        ppu.palette_table[start + 2],
    };
}

pub fn render(ppu: *const PPU, frame: *Frame) void {
    const scroll_x: usize = ppu.scroll_register.fine_x;
    const scroll_y: usize = ppu.scroll_register.fine_y;

    const main_nametable = switch (ppu.mirroring) {
        .VERTICAL => switch (ppu.ctrl_register.nametable_addr()) {
            0x2000, 0x2400 => ppu.vram[0..0x400],
            0x2800, 0x2C00 => ppu.vram[0x400..0x800],
            else => |addr| std.debug.panic(
                "Not supported address 0x{X:04} for VERTICAL mirroring type\n",
                .{addr},
            ),
        },
        .HORIZONTAL => switch (ppu.ctrl_register.nametable_addr()) {
            0x2000, 0x2800 => ppu.vram[0..0x400],
            0x2400, 0x2C00 => ppu.vram[0x400..0x800],
            else => |addr| std.debug.panic(
                "Not supported address 0x{X:04} for HORIZONTAL mirroring type\n",
                .{addr},
            ),
        },
        else => |mirroring| std.debug.panic("Not supported mirroring type: {any}\n", .{mirroring}),
    };
    const second_nametable = switch (ppu.mirroring) {
        .VERTICAL => switch (ppu.ctrl_register.nametable_addr()) {
            0x2000, 0x2800 => ppu.vram[0x400..0x800],
            0x2400, 0x2C00 => ppu.vram[0..0x400],
            else => |addr| std.debug.panic(
                "Not supported address 0x{X:04} for VERTICAL mirroring type\n",
                .{addr},
            ),
        },
        .HORIZONTAL => switch (ppu.ctrl_register.nametable_addr()) {
            0x2000, 0x2400 => ppu.vram[0x400..0x800],
            0x2800, 0x2C00 => ppu.vram[0..0x400],
            else => |addr| std.debug.panic(
                "Not supported address 0x{X:04} for HORIZONTAL mirroring type\n",
                .{addr},
            ),
        },
        else => |mirroring| std.debug.panic("Not supported mirroring type: {any}\n", .{mirroring}),
    };

    // Render background
    render_nametable(
        ppu,
        frame,
        main_nametable,
        .{ .x1 = scroll_x, .y1 = scroll_y, .x2 = 256, .y2 = 240 },
        -@as(isize, @bitCast(scroll_x)),
        -@as(isize, @bitCast(scroll_y)),
    );
    if (scroll_x > 0) {
        render_nametable(
            ppu,
            frame,
            second_nametable,
            .{ .x1 = 0, .y1 = 0, .x2 = scroll_x, .y2 = 240 },
            256 - @as(isize, @bitCast(scroll_x)),
            0,
        );
    } else if (scroll_y > 0) {
        render_nametable(
            ppu,
            frame,
            second_nametable,
            .{ .x1 = 0, .y1 = 0, .x2 = 256, .y2 = scroll_y },
            0,
            240 - @as(isize, @bitCast(scroll_y)),
        );
    }

    // Render sprites
    var i: isize = @bitCast(ppu.oam_data_register.len - 4);
    while (i >= 0) : (i -= 4) {
        const i_usize: usize = @bitCast(i);
        const tile_idx: u16 = ppu.oam_data_register[i_usize + 1];
        const tile_x: usize = ppu.oam_data_register[i_usize + 3];
        const tile_y: usize = ppu.oam_data_register[i_usize];

        const flip_vertical = ppu.oam_data_register[i_usize + 2] >> 7 & 1 != 0;
        const flip_horizontal = ppu.oam_data_register[i_usize + 2] >> 6 & 1 != 0;
        const pallette_idx = ppu.oam_data_register[i_usize + 2] & 0b11;
        const sprite_palette = sprite_pallette(ppu, pallette_idx);
        const bank = ppu.ctrl_register.sprt_pattern_addr();

        const tile = ppu.chr_rom[bank + tile_idx * 16 .. bank + tile_idx * 16 + 15 + 1];

        for (0..8) |y| {
            var upper = tile[y];
            var lower = tile[y + 8];

            var x: isize = 7;
            while (x >= 0) : (x -= 1) {
                const x_usize: usize = @bitCast(x);
                const value = (1 & lower) << 1 | (1 & upper);
                upper = upper >> 1;
                lower = lower >> 1;
                const rgb = switch (value) {
                    0 => continue, // skip coloring the pixel
                    1 => SYSTEM_PALETTE[sprite_palette[1]],
                    2 => SYSTEM_PALETTE[sprite_palette[2]],
                    3 => SYSTEM_PALETTE[sprite_palette[3]],
                    else => std.debug.panic("Incorrect value {}\n", .{value}),
                };

                if (flip_horizontal == false and flip_vertical == false) {
                    frame.set_pixel(tile_x + x_usize, tile_y + y, rgb);
                } else if (flip_horizontal == true and flip_vertical == false) {
                    frame.set_pixel(tile_x + 7 - x_usize, tile_y + y, rgb);
                } else if (flip_horizontal == false and flip_vertical == true) {
                    frame.set_pixel(tile_x + x_usize, tile_y + 7 - y, rgb);
                } else if (flip_horizontal == true and flip_vertical == true) {
                    frame.set_pixel(tile_x + 7 - x_usize, tile_y + 7 - y, rgb);
                }
            }
        }
    }
}

fn render_nametable(
    ppu: *const PPU,
    frame: *Frame,
    name_table: []const u8,
    view_port: Rect,
    shift_x: isize,
    shift_y: isize,
) void {
    const bank = ppu.ctrl_register.bg_pattern_addr();

    const attribute_table = name_table[0x3C0..0x400];
    for (0..0x3C0) |i| {
        const tile_column = i % 32;
        const tile_row = i / 32;
        const tile_idx: u16 = name_table[i];

        const tile = ppu.chr_rom[(bank + tile_idx * 16)..(bank + tile_idx * 16 + 15 + 1)];
        const pallette = bg_pallette(ppu, attribute_table, tile_column, tile_row);

        for (0..8) |y| {
            var upper = tile[y];
            var lower = tile[y + 8];

            var x: isize = 7;
            while (x >= 0) : (x -= 1) {
                const value = (1 & lower) << 1 | (1 & upper);
                upper >>= 1;
                lower >>= 1;
                const rgb = switch (value) {
                    0 => SYSTEM_PALETTE[pallette[0]],
                    1 => SYSTEM_PALETTE[pallette[1]],
                    2 => SYSTEM_PALETTE[pallette[2]],
                    3 => SYSTEM_PALETTE[pallette[3]],
                    else => std.debug.panic("Incorrect value {}\n", .{value}),
                };
                const pixel_x = tile_column * 8 + @as(usize, @bitCast(x));
                const pixel_y = tile_row * 8 + y;
                if (pixel_x >= view_port.x1 and pixel_x < view_port.x2 and pixel_y >= view_port.y1 and pixel_y < view_port.y2) {
                    frame.set_pixel(
                        @as(usize, @bitCast(shift_x + @as(isize, @bitCast(pixel_x)))),
                        @as(usize, @bitCast(shift_y + @as(isize, @bitCast(pixel_y)))),
                        rgb,
                    );
                }
            }
        }
    }
}

pub fn show_tile(chr_rom: []u8, bank: usize, tile_n: usize) Frame {
    std.debug.assert(bank <= 1);

    var frame = Frame.init();
    const bank_idx: usize = bank * 0x1000;

    const tile = chr_rom[(bank_idx + tile_n * 16)..(bank_idx + tile_n * 16 + 15 + 1)];

    for (0..8) |y| {
        var upper = tile[y];
        var lower = tile[y + 8];

        var x: usize = 7;
        while (x > 0) : (x -= 1) {
            const value = (1 & upper) << 1 | (1 & lower);
            upper >>= 1;
            lower >>= 1;
            const rgb = switch (value) {
                0 => SYSTEM_PALETTE[0x01],
                1 => SYSTEM_PALETTE[0x23],
                2 => SYSTEM_PALETTE[0x27],
                3 => SYSTEM_PALETTE[0x30],
                else => std.debug.panic("Incorrect value {}\n", .{value}),
            };
            frame.set_pixel(x, y, rgb);
        }
    }

    return frame;
}

pub fn show_tile_bank(chr_rom: []u8, bank: usize, ppu: *const PPU, frame: *Frame) void {
    std.debug.assert(bank <= 1);

    var tile_y: usize = 0;
    var tile_x: usize = 0;
    const bank_idx = bank * 0x1000;

    for (0..255) |tile_n| {
        if (tile_n != 0 and tile_n % 20 == 0) {
            tile_y += 10;
            tile_x = 0;
        }
        const tile = chr_rom[(bank_idx + tile_n * 16)..(bank_idx + tile_n * 16 + 15 + 1)];
        const pallette = bg_pallette(ppu, tile_x, tile_y);

        for (0..8) |y| {
            var upper = tile[y];
            var lower = tile[y + 8];

            var x: usize = 8;
            while (x > 0) : (x -= 1) {
                const value = (1 & lower) << 1 | (1 & upper);
                upper = upper >> 1;
                lower = lower >> 1;
                const rgb = switch (value) {
                    0 => SYSTEM_PALETTE[pallette[0]],
                    1 => SYSTEM_PALETTE[pallette[1]],
                    2 => SYSTEM_PALETTE[pallette[2]],
                    3 => SYSTEM_PALETTE[pallette[3]],
                    else => @panic("AAAAAAAAAAAA"),
                };
                frame.set_pixel(tile_x + x - 1, tile_y + y, rgb);
            }
        }

        tile_x += 10;
    }
}
