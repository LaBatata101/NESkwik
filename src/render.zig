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
