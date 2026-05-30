const std = @import("std");
const builtin = @import("builtin");
const android = if (builtin.abi.isAndroid()) @import("android") else struct {};
const ness = @import("ness");
const logging = ness.logging;

const c = ness.c;
const gui = ness.gui;
const Rom = ness.Rom;
const UI = ness.ui.UI;
const debug = ness.debug;
const System = ness.System;
const widgets = ness.ui.widgets;
const sdlError = ness.sdlError;

pub const std_options: std.Options = .{
    .logFn = if (builtin.abi.isAndroid()) android.logFn else logging.logFn,
};

pub const panic = std.debug.FullPanic(customPanic);

comptime {
    if (builtin.abi.isAndroid()) {
        @export(&SDL_main, .{ .name = "SDL_main", .linkage = .strong });
    }
}

fn SDL_main() callconv(.c) void {
    if (!comptime builtin.abi.isAndroid()) {
        @compileError("SDL_main should not be called outside of Android builds");
    }
    main() catch |err| {
        std.log.err("{t}", .{err});
        if (@errorReturnTrace()) |trace| {
            std.debug.dumpStackTrace(trace.*);
        }
    };
}

fn customPanic(msg: []const u8, first_trace_addr: ?usize) noreturn {
    const alloc = std.heap.page_allocator;
    var trace: std.Io.Writer.Allocating = .init(alloc);
    defer trace.deinit();
    const trace_writer = &trace.writer;

    var buffer: std.Io.Writer.Allocating = .init(alloc);
    defer buffer.deinit();
    const writer = &buffer.writer;

    if (builtin.abi.isAndroid()) {
        writer.print("{s}\n", .{msg}) catch {};
    } else {
        writer.print("The program crashed with the following message:\n\"{s}\"\n\n", .{msg}) catch {};
        if (logging.path()) |log_path| {
            writer.print("Check the log file at {s}\n\n", .{log_path}) catch {};
        } else {
            writer.print("Log file unavailable.\n\n", .{}) catch {};
        }
    }

    if (std.debug.getSelfDebugInfo()) |debug_info| {
        std.debug.writeCurrentStackTrace(trace_writer, debug_info, .no_color, first_trace_addr) catch |err| {
            writer.print("Unable to dump stack trace: {s}\n", .{@errorName(err)}) catch {};
        };
        const stacktrace = trace.written();
        if (stacktrace.len > 1024) {
            writer.print("Stacktrace:\n{s}...\n", .{stacktrace[0..1024]}) catch {};
        } else {
            writer.print("Stacktrace:\n{s}\n", .{stacktrace}) catch {};
        }
    } else |err| {
        writer.print("Unable to dump stack trace:\n\tUnable to open debug info: {s}\n", .{@errorName(err)}) catch {};
    }

    const panic_msg = alloc.dupeZ(u8, buffer.written()) catch unreachable;
    defer alloc.free(panic_msg);

    logging.writePanic(buffer.written());
    std.debug.print("{s}", .{panic_msg});

    _ = c.SDL_ShowSimpleMessageBox(c.SDL_MESSAGEBOX_ERROR, "NESkwik Error", panic_msg.ptr, null);

    std.process.exit(1);
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}).init;
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    logging.init(allocator) catch |err| {
        std.debug.print("Failed to initialize log file: {s}\n", .{@errorName(err)});
    };
    defer logging.deinit(allocator);

    var args = try std.process.argsWithAllocator(allocator);
    defer args.deinit();

    var ui = try UI.init(allocator, "NESkwik", 1280, 720);
    defer ui.deinit();
    var app_state = gui.AppState.init(allocator);
    defer app_state.deinit();

    ui.setVSync(app_state.settings.vsync);

    if (!builtin.abi.isAndroid()) {
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
    }
    ui.setFramerate(.unlimited);

    while (!ui.shouldClose()) {
        ui.beginFrame();

        app_state.update(ui);
        gui.drawGUI(ui, &app_state);

        ui.endFrame();
    }
}
