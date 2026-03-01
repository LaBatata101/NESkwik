pub const AspectRatio = enum {
    @"4_3",
    @"16_9",

    fn value(self: @This()) f32 {
        return switch (self) {
            .@"4_3" => 4 / 3,
            .@"16_9" => 16 / 9,
        };
    }
};

const Viewport = struct { x: f32, y: f32, w: f32, h: f32 };
pub fn calculateViewport(win_x: f32, win_y: f32, win_w: f32, win_h: f32, aspect: AspectRatio) Viewport {
    const desired_aspect = aspect.value();
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
        y += win_h * (0.5 - delta);
        viewport_h = 2.0 * win_h * delta;
    }

    return .{ .x = x, .y = y, .w = viewport_w, .h = viewport_h };
}
