const std = @import("std");
const ness = @import("ness");

const c = ness.c;
const gui = ness.gui;
const Rom = ness.Rom;
const UI = ness.ui.UI;
const trace = ness.trace;
const debug = ness.debug;
const System = ness.System;
const widgets = ness.ui.widgets;
const sdlError = ness.sdlError;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}).init;
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var args = try std.process.argsWithAllocator(allocator);
    defer args.deinit();

    var ui = try UI.init(allocator, "NESkwik", 1280, 720);
    defer ui.deinit();
    var app_state = gui.AppState.init(allocator);
    defer app_state.deinit();

    ui.setVSync(app_state.settings.vsync);

    _ = args.skip();
    if (args.next()) |arg0| {
        if (std.mem.eql(u8, arg0, "--debug")) {
            app_state.toggleDebug();

            if (args.next()) |arg1| {
                try app_state.loadRom(arg1);
            } else {
                std.debug.print("ROM file path not provided\n", .{});
                std.process.exit(1);
            }
        } else {
            try app_state.loadRom(arg0);
        }
        app_state.render_home_ui = false;
    }

    ui.setFramerate(.unlimited);

    while (!ui.shouldClose()) {
        ui.beginFrame();

        app_state.update(ui);
        gui.drawGUI(ui, &app_state);

        ui.endFrame();
    }
}
