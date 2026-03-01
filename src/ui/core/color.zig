const clay = @import("clay.zig");

pub const Color = struct {
    r: u8,
    g: u8,
    b: u8,
    a: u8,

    pub const white = Color{ .r = 255, .g = 255, .b = 255, .a = 255 };
    pub const black = Color{ .r = 0, .g = 0, .b = 0, .a = 255 };
    pub const red = Color{ .r = 220, .g = 60, .b = 60, .a = 255 };
    pub const green = Color{ .r = 60, .g = 220, .b = 60, .a = 255 };
    pub const blue = Color{ .r = 60, .g = 120, .b = 220, .a = 255 };
    pub const gray = Color{ .r = 150, .g = 150, .b = 150, .a = 255 };
    pub const lightGray = Color{ .r = 200, .g = 200, .b = 200, .a = 255 };
    pub const darkGray = Color{ .r = 80, .g = 80, .b = 80, .a = 255 };

    pub fn rgb(r: u8, g: u8, b: u8) Color {
        return .{ .r = r, .g = g, .b = b, .a = 255 };
    }

    pub fn toClay(self: Color) clay.Color {
        return .{
            @floatFromInt(self.r),
            @floatFromInt(self.g),
            @floatFromInt(self.b),
            @floatFromInt(self.a),
        };
    }

    /// Darken a color by a percentage (0.0 to 1.0)
    pub fn darken(self: Color, amount: f32) Color {
        return .{
            .r = @intFromFloat(@as(f32, @floatFromInt(self.r)) * (1.0 - amount)),
            .g = @intFromFloat(@as(f32, @floatFromInt(self.g)) * (1.0 - amount)),
            .b = @intFromFloat(@as(f32, @floatFromInt(self.b)) * (1.0 - amount)),
            .a = self.a,
        };
    }

    /// Lighten a color by a percentage (0.0 to 1.0)
    pub fn lighten(self: Color, amount: f32) Color {
        return .{
            .r = @intFromFloat(@min(255, @as(f32, @floatFromInt(self.r)) + (255 - @as(f32, @floatFromInt(self.r))) * amount)),
            .g = @intFromFloat(@min(255, @as(f32, @floatFromInt(self.g)) + (255 - @as(f32, @floatFromInt(self.g))) * amount)),
            .b = @intFromFloat(@min(255, @as(f32, @floatFromInt(self.b)) + (255 - @as(f32, @floatFromInt(self.b))) * amount)),
            .a = self.a,
        };
    }

    /// Create a semi-transparent version of this color
    pub fn withAlpha(self: Color, alpha: u8) Color {
        return .{
            .r = self.r,
            .g = self.g,
            .b = self.b,
            .a = alpha,
        };
    }
};
