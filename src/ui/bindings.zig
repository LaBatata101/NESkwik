const std = @import("std");

const ControllerButton = @import("../controller.zig").ControllerButton;
const System = @import("../system.zig").System;
const Key = @import("core/ui.zig").Key;

pub const ControllerPlayer = enum {
    one,
    two,

    pub fn displayName(self: @This()) []const u8 {
        return switch (self) {
            .one => "Player 1",
            .two => "Player 2",
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

pub fn applyControllerBindings(system: *System, controller_bindings: ControllerKeyBindings) void {
    applyBindingsToKeymap(&system.keymap1, controller_bindings.player1);
    applyBindingsToKeymap(&system.keymap2, controller_bindings.player2);
}

fn applyBindingsToKeymap(
    keymap: *std.AutoHashMap(Key, ControllerButton),
    controller_bindings: ControllerPlayerBindings,
) void {
    keymap.clearRetainingCapacity();
    inline for (@typeInfo(ControllerAction).@"enum".fields) |f| {
        const action = @field(ControllerAction, f.name);
        putControllerBinding(keymap, controller_bindings.get(action), action);
    }
}

fn putControllerBinding(
    keymap: *std.AutoHashMap(Key, ControllerButton),
    key: Key,
    action: ControllerAction,
) void {
    const entry = keymap.getOrPut(key) catch @panic("Failed to update controller binding");
    if (entry.found_existing) {
        entry.value_ptr.insert(action.button());
    } else {
        entry.value_ptr.* = action.button();
    }
}
