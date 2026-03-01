const std = @import("std");

const c = @import("root.zig").c;

pub const Color = @import("ui/core/color.zig").Color;
const Mirroring = @import("rom.zig").Mirroring;
const PPU = @import("ppu.zig").PPU;

// zig fmt: off
pub const SYSTEM_PALETTE: [64]Color = [_]Color{
   Color.rgb(0x80, 0x80, 0x80), Color.rgb(0x00, 0x3D, 0xA6), Color.rgb(0x00, 0x12, 0xB0), Color.rgb(0x44, 0x00, 0x96), Color.rgb(0xA1, 0x00, 0x5E),
   Color.rgb(0xC7, 0x00, 0x28), Color.rgb(0xBA, 0x06, 0x00), Color.rgb(0x8C, 0x17, 0x00), Color.rgb(0x5C, 0x2F, 0x00), Color.rgb(0x10, 0x45, 0x00),
   Color.rgb(0x05, 0x4A, 0x00), Color.rgb(0x00, 0x47, 0x2E), Color.rgb(0x00, 0x41, 0x66), Color.rgb(0x00, 0x00, 0x00), Color.rgb(0x05, 0x05, 0x05),
   Color.rgb(0x05, 0x05, 0x05), Color.rgb(0xC7, 0xC7, 0xC7), Color.rgb(0x00, 0x77, 0xFF), Color.rgb(0x21, 0x55, 0xFF), Color.rgb(0x82, 0x37, 0xFA),
   Color.rgb(0xEB, 0x2F, 0xB5), Color.rgb(0xFF, 0x29, 0x50), Color.rgb(0xFF, 0x22, 0x00), Color.rgb(0xD6, 0x32, 0x00), Color.rgb(0xC4, 0x62, 0x00),
   Color.rgb(0x35, 0x80, 0x00), Color.rgb(0x05, 0x8F, 0x00), Color.rgb(0x00, 0x8A, 0x55), Color.rgb(0x00, 0x99, 0xCC), Color.rgb(0x21, 0x21, 0x21),
   Color.rgb(0x09, 0x09, 0x09), Color.rgb(0x09, 0x09, 0x09), Color.rgb(0xFF, 0xFF, 0xFF), Color.rgb(0x0F, 0xD7, 0xFF), Color.rgb(0x69, 0xA2, 0xFF),
   Color.rgb(0xD4, 0x80, 0xFF), Color.rgb(0xFF, 0x45, 0xF3), Color.rgb(0xFF, 0x61, 0x8B), Color.rgb(0xFF, 0x88, 0x33), Color.rgb(0xFF, 0x9C, 0x12),
   Color.rgb(0xFA, 0xBC, 0x20), Color.rgb(0x9F, 0xE3, 0x0E), Color.rgb(0x2B, 0xF0, 0x35), Color.rgb(0x0C, 0xF0, 0xA4), Color.rgb(0x05, 0xFB, 0xFF),
   Color.rgb(0x5E, 0x5E, 0x5E), Color.rgb(0x0D, 0x0D, 0x0D), Color.rgb(0x0D, 0x0D, 0x0D), Color.rgb(0xFF, 0xFF, 0xFF), Color.rgb(0xA6, 0xFC, 0xFF),
   Color.rgb(0xB3, 0xEC, 0xFF), Color.rgb(0xDA, 0xAB, 0xEB), Color.rgb(0xFF, 0xA8, 0xF9), Color.rgb(0xFF, 0xAB, 0xB3), Color.rgb(0xFF, 0xD2, 0xB0),
   Color.rgb(0xFF, 0xEF, 0xA6), Color.rgb(0xFF, 0xF7, 0x9C), Color.rgb(0xD7, 0xE8, 0x95), Color.rgb(0xA6, 0xED, 0xAF), Color.rgb(0xA2, 0xF2, 0xDA),
   Color.rgb(0x99, 0xFF, 0xFC), Color.rgb(0xDD, 0xDD, 0xDD), Color.rgb(0x11, 0x11, 0x11), Color.rgb(0x11, 0x11, 0x11)
};
// zig fmt: on

pub const Frame = struct {
    data: [WIDTH * HEIGHT * 4]u8,

    const Self = @This();
    const WIDTH: usize = 256;
    const HEIGHT: usize = 240;

    pub fn init() Self {
        return .{ .data = [_]u8{0} ** (WIDTH * HEIGHT * 4) };
    }

    pub fn set_pixel(self: *Self, x: usize, y: usize, rgb: Color) void {
        const base = y * 4 * WIDTH + x * 4;
        if (base + 2 < self.data.len) {
            self.data[base] = rgb.r;
            self.data[base + 1] = rgb.g;
            self.data[base + 2] = rgb.b;
            self.data[base + 3] = 255;
        }
    }

    pub fn clear(self: *Self) void {
        @memset(&self.data, 0);
    }
};

pub const PatternTableFrame = struct {
    data: [128 * 128 * 4]u8 = [_]u8{0} ** (128 * 128 * 4),

    pub fn set_pixel(self: *@This(), x: usize, y: usize, rgb: Color) void {
        const base = y * 4 * 128 + x * 4;
        self.data[base] = rgb.r;
        self.data[base + 1] = rgb.g;
        self.data[base + 2] = rgb.b;
        self.data[base + 3] = 255;
    }
};

pub const FPSManager = struct {
    target_ticks_per_frame: u32,
    last_frame_end: u64,
    dt: u64,
    mode: FramerateMode,

    const Self = @This();
    pub const FramerateMode = union(enum) {
        limited: u32,
        unlimited,
    };
    const FPS_DEFAULT = 60;

    fn getTicks() u64 {
        const ticks = c.SDL_GetTicks();
        return if (ticks == 0) 1 else ticks;
    }

    pub fn init() Self {
        return .{
            .target_ticks_per_frame = c.SDL_MS_PER_SECOND / FPS_DEFAULT,
            .last_frame_end = Self.getTicks(),
            .dt = 0,
            .mode = .unlimited,
        };
    }

    pub fn setFramerate(self: *Self, mode: FramerateMode) void {
        self.mode = mode;
    }

    /// Limits the frame rate by sleeping the remaining time in the frame slot.
    /// Returns the delta time since the last frame in milliseconds.
    pub fn delay(self: *Self) u64 {
        const now = Self.getTicks();
        const frame_duration = now -% self.last_frame_end;

        switch (self.mode) {
            .limited => |rate| {
                if (rate < 1) {
                    @panic("Framerate can't be lower than 1.");
                }

                const target_ticks_per_frame = @divTrunc(@as(u32, @intCast(c.SDL_MS_PER_SECOND)), rate);
                if (frame_duration < target_ticks_per_frame) {
                    c.SDL_Delay(@intCast(self.target_ticks_per_frame - frame_duration));
                }
            },
            .unlimited => {},
        }

        self.last_frame_end = Self.getTicks();
        self.dt = self.last_frame_end -% now + frame_duration;
        return self.dt;
    }

    pub fn getFPS(self: *const Self) u64 {
        return @divTrunc(c.SDL_MS_PER_SECOND, @max(self.dt, 1));
    }
};
