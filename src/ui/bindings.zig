const std = @import("std");

const ControllerButton = @import("../controller.zig").ControllerButton;
const Key = @import("core/ui.zig").Key;

pub const GamepadButton = enum {
    south,
    east,
    west,
    north,
    back,
    guide,
    start,
    left_stick,
    right_stick,
    left_shoulder,
    right_shoulder,
    dpad_up,
    dpad_down,
    dpad_left,
    dpad_right,

    pub fn displayName(self: @This()) []const u8 {
        return switch (self) {
            .south => "A",
            .east => "B",
            .west => "X",
            .north => "Y",
            .back => "Back",
            .guide => "Home",
            .start => "Start",
            .left_stick => "L3",
            .right_stick => "R3",
            .left_shoulder => "LB",
            .right_shoulder => "RB",
            .dpad_up => "D-Up",
            .dpad_down => "D-Down",
            .dpad_left => "D-Left",
            .dpad_right => "D-Right",
        };
    }
};

pub const GamepadPlayerBindings = struct {
    up: GamepadButton = .dpad_up,
    down: GamepadButton = .dpad_down,
    left: GamepadButton = .dpad_left,
    right: GamepadButton = .dpad_right,
    select: GamepadButton = .back,
    start: GamepadButton = .start,
    b: GamepadButton = .east,
    a: GamepadButton = .south,

    pub fn get(self: @This(), action: ControllerAction) GamepadButton {
        return switch (action) {
            inline else => |act| @field(self, @tagName(act)),
        };
    }

    pub fn set(self: *@This(), action: ControllerAction, btn: GamepadButton) void {
        switch (action) {
            inline else => |act| @field(self, @tagName(act)) = btn,
        }
    }
};

pub const GamepadKeyBindings = struct {
    player1: GamepadPlayerBindings = .{},
    player2: GamepadPlayerBindings = .{},

    pub fn forPlayer(self: *@This(), player: ControllerPlayer) *GamepadPlayerBindings {
        return switch (player) {
            .one => &self.player1,
            .two => &self.player2,
        };
    }

    pub fn forPlayerConst(self: *const @This(), player: ControllerPlayer) *const GamepadPlayerBindings {
        return switch (player) {
            .one => &self.player1,
            .two => &self.player2,
        };
    }
};

pub const ControllerPlayer = enum {
    one,
    two,

    pub fn displayName(self: @This()) []const u8 {
        return switch (self) {
            .one => "Player 1",
            .two => "Player 2",
        };
    }

    pub fn value(self: @This()) usize {
        return switch (self) {
            .one => 0,
            .two => 1,
        };
    }
};

pub const ControllerAction = enum {
    up,
    down,
    left,
    right,
    select,
    start,
    b,
    a,

    pub fn displayName(self: @This()) []const u8 {
        return switch (self) {
            .up => "Up",
            .down => "Down",
            .left => "Left",
            .right => "Right",
            .select => "Select",
            .start => "Start",
            .b => "B",
            .a => "A",
        };
    }

    fn button(self: @This()) ControllerButton {
        return switch (self) {
            .up => .{ .UP = true },
            .down => .{ .DOWN = true },
            .left => .{ .LEFT = true },
            .right => .{ .RIGHT = true },
            .select => .{ .SELECT = true },
            .start => .{ .START = true },
            .b => .{ .BUTTON_B = true },
            .a => .{ .BUTTON_A = true },
        };
    }
};

pub const ControllerPlayerBindings = struct {
    up: Key,
    down: Key,
    left: Key,
    right: Key,
    select: Key,
    start: Key,
    b: Key,
    a: Key,

    pub fn defaults(player: ControllerPlayer) @This() {
        return switch (player) {
            .one => .{
                .up = .UP,
                .down = .DOWN,
                .left = .LEFT,
                .right = .RIGHT,
                .select = .SPACE,
                .start = .RETURN,
                .b = .X,
                .a = .Z,
            },
            .two => .{
                .up = .W,
                .down = .S,
                .left = .A,
                .right = .D,
                .select = .U,
                .start = .P,
                .b = .O,
                .a = .I,
            },
        };
    }

    pub fn get(self: @This(), action: ControllerAction) Key {
        return switch (action) {
            inline else => |a| @field(self, @tagName(a)),
        };
    }

    pub fn set(self: *@This(), action: ControllerAction, key: Key) void {
        switch (action) {
            inline else => |a| @field(self, @tagName(a)) = key,
        }
    }
};

pub const ControllerKeyBindings = struct {
    player1: ControllerPlayerBindings = ControllerPlayerBindings.defaults(.one),
    player2: ControllerPlayerBindings = ControllerPlayerBindings.defaults(.two),

    pub fn forPlayer(self: *@This(), player: ControllerPlayer) *ControllerPlayerBindings {
        return switch (player) {
            .one => &self.player1,
            .two => &self.player2,
        };
    }

    pub fn forPlayerConst(self: *const @This(), player: ControllerPlayer) *const ControllerPlayerBindings {
        return switch (player) {
            .one => &self.player1,
            .two => &self.player2,
        };
    }
};

pub const ControllerBindingTarget = struct {
    player: ControllerPlayer,
    action: ControllerAction,
};

pub const GeneralAction = enum {
    quit,
    toggle_step_mode,
    reset,
    run_tick,
    run_frame,
    toggle_fullscreen,

    pub fn displayName(self: @This()) []const u8 {
        return switch (self) {
            .quit => "Quit",
            .toggle_step_mode => "Toggle Step Mode",
            .reset => "Reset",
            .run_tick => "Run Tick",
            .run_frame => "Run Frame",
            .toggle_fullscreen => "Toggle Fullscreen",
        };
    }
};

pub const GeneralKeyBindings = struct {
    quit: Key = .ESCAPE,
    toggle_step_mode: Key = .F9,
    reset: Key = .R,
    run_tick: Key = .F10,
    run_frame: Key = .F11,
    toggle_fullscreen: Key = .F,

    pub fn get(self: @This(), action: GeneralAction) Key {
        return switch (action) {
            inline else => |a| @field(self, @tagName(a)),
        };
    }

    pub fn set(self: *@This(), action: GeneralAction, key: Key) void {
        switch (action) {
            inline else => |a| @field(self, @tagName(a)) = key,
        }
    }
};

pub fn applyBindingsToKeymap(
    keymap: *std.AutoHashMap(Key, ControllerButton),
    player_bindings: ControllerPlayerBindings,
) void {
    keymap.clearRetainingCapacity();
    inline for (@typeInfo(ControllerAction).@"enum".fields) |f| {
        const action = @field(ControllerAction, f.name);
        const key = player_bindings.get(action);
        const entry = keymap.getOrPut(key) catch @panic("Failed to update controller binding");
        if (entry.found_existing) {
            entry.value_ptr.insert(action.button());
        } else {
            entry.value_ptr.* = action.button();
        }
    }
}
