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

pub const Controller = struct {
    /// - strobe bit on - controller reports only status of the button A on every read
    /// - strobe bit off - controller cycles through all buttons
    strobe: bool,
    button_index: u8,
    button_status: ControllerButton,

    const Self = @This();

    pub fn init() Self {
        return .{
            .strobe = false,
            .button_index = 0,
            .button_status = .{},
        };
    }

    /// Set the state of `strobe`.
    pub fn write(self: *Self, data: u8) void {
        self.strobe = data & 1 != 0;
        if (self.strobe) {
            self.button_index = 0;
        }
    }

    /// Each read reports one bit at a time. The first 8 reads will indicate which buttons or directions are pressed.
    /// The buttons are reported in the following order:
    ///     `A` -> `B` -> `SELECT` -> `START` -> `UP` -> `DOWN` -> `LEFT` -> `RIGHT`
    ///
    /// All subsequent reads will return 1. To reset the button read state back to `A` we need to set the `strobe`.
    pub fn read(self: *Self) u8 {
        if (self.button_index > 7) {
            return 1;
        }

        const pressed_button = @as(u8, @bitCast(self.button_status)) & std.math.shl(u8, 1, self.button_index);
        const result = std.math.shr(u8, pressed_button, self.button_index);

        if (!self.strobe and self.button_index <= 7) {
            self.button_index += 1;
        }

        // return the nth bit (`button_index`) in `pressed_button`
        return result;
    }
};

test "joypad strobe mode ON" {
    var joypad = Controller.init();

    joypad.write(1);
    joypad.button_status.insert(.{ .BUTTON_A = true });

    for (0..10) |_| {
        try std.testing.expectEqual(1, joypad.read());
    }
}

test "joypad strobe mode ON/OFF" {
    var joypad = Controller.init();

    joypad.write(0);
    joypad.button_status.insert(.{ .RIGHT = true });
    joypad.button_status.insert(.{ .LEFT = true });
    joypad.button_status.insert(.{ .SELECT = true });
    joypad.button_status.insert(.{ .BUTTON_B = true });

    for (0..2) |_| {
        try std.testing.expectEqual(0, joypad.read());
        try std.testing.expectEqual(1, joypad.read());
        try std.testing.expectEqual(1, joypad.read());
        try std.testing.expectEqual(0, joypad.read());
        try std.testing.expectEqual(0, joypad.read());
        try std.testing.expectEqual(0, joypad.read());
        try std.testing.expectEqual(1, joypad.read());
        try std.testing.expectEqual(1, joypad.read());

        for (0..10) |_| {
            try std.testing.expectEqual(1, joypad.read());
        }

        joypad.write(1);
        joypad.write(0);
    }
}
