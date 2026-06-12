const clay = @import("clay.zig");

pub const AspectRatio = enum {
    none,
    @"4_3",
    @"16_9",

    pub fn label(self: @This()) []const u8 {
        return switch (self) {
            .none => "None",
            .@"4_3" => "4:3",
            .@"16_9" => "16:9",
        };
    }

    pub fn value(self: @This()) ?f32 {
        return switch (self) {
            .none => null,
            .@"4_3" => @as(f32, @floatFromInt(4)) / 3,
            .@"16_9" => @as(f32, @floatFromInt(16)) / 9,
        };
    }
};

pub const ViewportAlignment = enum {
    center,
    top,
};

const Viewport = struct { x: f32, y: f32, w: f32, h: f32 };
pub fn calculateViewport(
    win_x: f32,
    win_y: f32,
    win_w: f32,
    win_h: f32,
    aspect: AspectRatio,
    alignment: ViewportAlignment,
) Viewport {
    const desired_aspect = aspect.value() orelse return .{
        .x = win_x,
        .y = win_y,
        .w = win_w,
        .h = win_h,
    };
    const device_aspect = win_w / win_h;

    var delta: f32 = 0;
    var x: f32 = win_x;
    var y: f32 = win_y;
    var viewport_w: f32 = win_w;
    var viewport_h: f32 = win_h;

    if (@abs(device_aspect - desired_aspect) < 0.0001) {
        // Assume that the aspect ratios are equal
    } else if (device_aspect > desired_aspect) {
        delta = (desired_aspect / device_aspect - 1.0) / 2.0 + 0.5;
        x += win_w * (0.5 - delta);
        viewport_w = 2.0 * win_w * delta;
    } else {
        delta = (device_aspect / desired_aspect - 1.0) / 2.0 + 0.5;
        y += switch (alignment) {
            .center => win_h * (0.5 - delta),
            .top => 0,
        };
        viewport_h = 2.0 * win_h * delta;
    }

    return .{ .x = x, .y = y, .w = viewport_w, .h = viewport_h };
}

pub fn canvasContentBounds(bounds: clay.BoundingBox, padding: clay.Padding) clay.BoundingBox {
    const left: f32 = @floatFromInt(padding.left);
    const right: f32 = @floatFromInt(padding.right);
    const top: f32 = @floatFromInt(padding.top);
    const bottom: f32 = @floatFromInt(padding.bottom);

    return .{
        .x = bounds.x + left,
        .y = bounds.y + top,
        .width = @max(0, bounds.width - left - right),
        .height = @max(0, bounds.height - top - bottom),
    };
}
