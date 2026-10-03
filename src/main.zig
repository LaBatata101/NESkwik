const std = @import("std");
const builtin = @import("builtin");
const android = if (builtin.abi.isAndroid()) @import("android") else struct {};
const ness = @import("ness");
const logging = ness.logging;

const c = ness.c;
const gui = ness.gui;
const Rom = ness.Rom;
const UI = ness.ui.UI;
const System = ness.System;
const widgets = ness.ui.widgets;
const sdlError = ness.sdlError;
const customPanic = ness.customPanic;

pub const std_options: std.Options = .{
    .logFn = if (builtin.abi.isAndroid()) androidAndFileLogFn else logging.logFn,
    // Emscripten has no sigaltstack (ENOSYS), which std.Thread treats as
    // unreachable when attaching the segfault-handler stack to new threads.
    .signal_stack_size = if (ness.features.wasm) null else (std.Options{}).signal_stack_size,
};

/// std.debug defaults to page_allocator, whose multithreaded WASM backend is
/// not implemented in Zig 0.16. Emscripten's libc allocator is thread-safe.
pub const debug = if (ness.features.wasm) struct {
    pub fn getDebugInfoAllocator() std.mem.Allocator {
        return std.heap.c_allocator;
    }
} else struct {};

pub const panic = std.debug.FullPanic(customPanic);

// Handles window resizes on Windows.
const CallbackParams = struct { ui: *UI, app_state: *gui.AppState };
fn handleWindowsResize(userdata: ?*anyopaque, event: [*c]c.SDL_Event) callconv(.c) bool {
    if (event == null or event.*.type != c.SDL_EVENT_WINDOW_EXPOSED) return true;

    const ctx: *CallbackParams = @ptrCast(@alignCast(userdata.?));
    if (event.*.window.windowID != ctx.ui.main_window.id()) return true;

    ctx.ui.beginFrameNoSDLEvents();
    gui.drawGUI(ctx.ui, ctx.app_state);
    ctx.ui.endFrame();
    return true;
}

fn androidAndFileLogFn(
    comptime message_level: std.log.Level,
    comptime scope: @EnumLiteral(),
    comptime format: []const u8,
    args: anytype,
) void {
    logging.logFn(message_level, scope, format, args);
    android.logFn(message_level, scope, format, args);
}

comptime {
    if (builtin.abi.isAndroid()) {
        @export(&SDL_main, .{ .name = "SDL_main" });
    }
    if (ness.features.wasm) {
        @export(&ness.wasm.exportedLoadRom, .{ .name = "neskwik_request_rom_load" });
        @export(&ness.wasm.exportedUnloadRom, .{ .name = "neskwik_request_rom_unload" });
        @export(&ness.wasm.shaderDirectoryImported, .{ .name = "neskwik_shader_directory_imported" });
    }
}

fn SDL_main() callconv(.c) void {
    if (!comptime builtin.abi.isAndroid()) {
        @compileError("SDL_main should not be called outside of Android builds");
    }

    var threaded: std.Io.Threaded = .init_single_threaded;
    defer threaded.deinit();

    appMain(std.heap.smp_allocator, threaded.io(), null) catch |err| {
        std.debug.panic("{any}", .{err});
    };
}

const Init = if (ness.features.wasm) std.process.Init.Minimal else std.process.Init;
pub fn main(init: Init) !void {
    const args = if (!ness.features.wasm) blk: {
        ness.env.init(init.environ_map);

        var args = try init.minimal.args.iterateAllocator(init.gpa);
        defer args.deinit();
        break :blk args;
    } else null;

    var threaded: std.Io.Threaded = .init_single_threaded;
    defer threaded.deinit();

    try appMain(std.heap.c_allocator, threaded.io(), if (ness.features.wasm) null else @constCast(&args));
}

fn appMain(allocator: std.mem.Allocator, io: std.Io, cli_args: ?*std.process.Args.Iterator) !void {
    logging.init(allocator, io) catch |err| {
        std.debug.print("Failed to initialize log file: {s}\n", .{@errorName(err)});
    };
    defer logging.deinit(allocator);

    var ui = try UI.init(allocator, io, "NESkwik", 1280, 720);
    defer if (!ness.features.wasm) ui.deinit();
    var app_state = try gui.AppState.init(allocator, io, ui);
    defer if (!ness.features.wasm) app_state.deinit();

    if (ness.features.wasm) {
        ness.wasm.wasm_app_state = app_state;
    }

    var cb_params = CallbackParams{ .ui = ui, .app_state = app_state };
    if (builtin.os.tag == .windows) sdlError(c.SDL_AddEventWatch(handleWindowsResize, &cb_params));
    defer if (builtin.os.tag == .windows) c.SDL_RemoveEventWatch(handleWindowsResize, &cb_params);

    ui.setVSync(app_state.settings.vsync);
    ui.setFramerate(.unlimited);

    if (ness.features.wasm) {
        std.os.emscripten.emscripten_set_main_loop_arg(mainloop, @ptrCast(&cb_params), 0, 1);
    } else {
        if (cli_args) |args| {
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

        while (!ui.shouldClose()) {
            app_state.update();

            ui.beginFrame();
            gui.drawGUI(ui, app_state);
            ui.endFrame();
        }
    }
}

fn mainloop(args: ?*anyopaque) callconv(.c) void {
    const params: *CallbackParams = @ptrCast(@alignCast(args));

    if (params.ui.shouldClose()) {
        ness.wasm.requestShutdown();
    }

    const shutdown_ready = ness.wasm.poll();
    if (ness.wasm.isShutdownRequested()) {
        if (shutdown_ready) {
            ness.wasm.wasm_app_state = null;
            std.os.emscripten.emscripten_cancel_main_loop();
            params.app_state.deinit();
            params.ui.deinit();
        }
        return;
    }

    params.app_state.update();

    params.ui.beginFrame();
    gui.drawGUI(params.ui, params.app_state);
    params.ui.endFrame();
}
