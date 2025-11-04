const std = @import("std");
/// `false` is released; `true` is pressed
/// https://www.nesdev.org/wiki/Standard_controller
pub const ControllerButton = packed struct(u8) {
    BUTTON_A: bool = false,
    BUTTON_B: bool = false,
    SELECT: bool = false,
    START: bool = false,
    UP: bool = false,
    DOWN: bool = false,
    LEFT: bool = false,
    RIGHT: bool = false,

    pub fn insert(self: *@This(), other: ControllerButton) void {
        self.* = @bitCast(@as(u8, @bitCast(self.*)) | @as(u8, @bitCast(other)));
    }

    pub fn remove(self: *@This(), other: ControllerButton) void {
        self.* = @bitCast(@as(u8, @bitCast(self.*)) & ~@as(u8, @bitCast(other)));
    }
};

pub const Controllers = struct {
    /// - strobe bit on - controller reports only status of the button A on every read
    /// - strobe bit off - controller cycles through all buttons
    strobe: bool,
    cntrl1_index: u8,
    cntrl2_index: u8,

    cntrl1_status: ControllerButton,
    cntrl2_status: ControllerButton,

    const Self = @This();

    pub fn init() Self {
        return .{
            .strobe = false,
            .cntrl1_index = 0,
            .cntrl2_index = 0,
            .cntrl1_status = .{},
            .cntrl2_status = .{},
        };
    }

    /// Set the state of `strobe`.
    pub fn set_strobe(self: *Self, data: u8) void {
        self.strobe = data & 1 != 0;
        if (self.strobe) {
            self.cntrl1_index = 0;
            self.cntrl2_index = 0;
        }
    }

    /// Each read reports one bit at a time. The first 8 reads will indicate which buttons or directions are pressed.
    /// The buttons are reported in the following order:
    ///     `A` -> `B` -> `SELECT` -> `START` -> `UP` -> `DOWN` -> `LEFT` -> `RIGHT`
    ///
    /// All subsequent reads will return 1. To reset the button read state back to `A` we need to set the `strobe`.
    pub fn cntrl1_read(self: *Self) u8 {
        if (self.cntrl1_index > 7) {
            return 1;
        }

        const pressed_button = @as(u8, @bitCast(self.cntrl1_status)) & std.math.shl(u8, 1, self.cntrl1_index);
        const result = std.math.shr(u8, pressed_button, self.cntrl1_index);

        if (!self.strobe and self.cntrl1_index <= 7) {
            self.cntrl1_index += 1;
        }

        // return the nth bit (`button_index`) in `pressed_button`
        return result;
    }

    pub fn cntrl2_read(self: *Self) u8 {
        if (self.cntrl2_index > 7) {
            return 1;
        }

        const pressed_button = @as(u8, @bitCast(self.cntrl2_status)) & std.math.shl(u8, 1, self.cntrl2_index);
        const result = std.math.shr(u8, pressed_button, self.cntrl2_index);

        if (!self.strobe and self.cntrl2_index <= 7) {
            self.cntrl2_index += 1;
        }

        // return the nth bit (`button_index`) in `pressed_button`
        return result;
    }

    pub fn cntrl1_peek(self: *const Self) u8 {
        if (self.cntrl1_index > 7) {
            return 1;
        }
        const pressed_button = @as(u8, @bitCast(self.cntrl1_status)) & std.math.shl(u8, 1, self.cntrl1_index);
        return std.math.shr(u8, pressed_button, self.cntrl1_index);
    }

    pub fn cntrl2_peek(self: *const Self) u8 {
        if (self.cntrl2_index > 7) {
            return 1;
        }

        const pressed_button = @as(u8, @bitCast(self.cntrl2_status)) & std.math.shl(u8, 1, self.cntrl2_index);
        return std.math.shr(u8, pressed_button, self.cntrl2_index);
    }
};

test "joypad strobe mode ON" {
    var joypad = Controllers.init();

    joypad.set_strobe(1);
    joypad.cntrl1_status.insert(.{ .BUTTON_A = true });

    for (0..10) |_| {
        try std.testing.expectEqual(1, joypad.cntrl1_read());
    }
}

test "joypad strobe mode ON/OFF" {
    var joypad = Controllers.init();

    joypad.set_strobe(0);
    joypad.cntrl1_status.insert(.{ .RIGHT = true });
    joypad.cntrl1_status.insert(.{ .LEFT = true });
    joypad.cntrl1_status.insert(.{ .SELECT = true });
    joypad.cntrl1_status.insert(.{ .BUTTON_B = true });

    for (0..2) |_| {
        try std.testing.expectEqual(0, joypad.cntrl1_read());
        try std.testing.expectEqual(1, joypad.cntrl1_read());
        try std.testing.expectEqual(1, joypad.cntrl1_read());
        try std.testing.expectEqual(0, joypad.cntrl1_read());
        try std.testing.expectEqual(0, joypad.cntrl1_read());
        try std.testing.expectEqual(0, joypad.cntrl1_read());
        try std.testing.expectEqual(1, joypad.cntrl1_read());
        try std.testing.expectEqual(1, joypad.cntrl1_read());

        for (0..10) |_| {
            try std.testing.expectEqual(1, joypad.cntrl1_read());
        }

        joypad.set_strobe(1);
        joypad.set_strobe(0);
    }
}
