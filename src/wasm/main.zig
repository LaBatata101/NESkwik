const std = @import("std");
const AppState = @import("../ui/gui.zig").AppState;

pub var wasm_app_state: ?*AppState = null;

var pending_rom_path: ?[]u8 = null;
var shutdown_requested = false;

/// Load a ROM. A running game is stopped first, which completes
/// asynchronously in `poll`.
pub fn loadRom(path: []const u8) !void {
    const state = wasm_app_state orelse return error.ShuttingDown;
    if (shutdown_requested) return error.ShuttingDown;

    if (!state.hasLoadedGame()) {
        try state.loadRom(path);
        if (state.ui.isWasmMobile()) state.ui.setWindowFullscreen(true);
        return;
    }

    const owned_path = try state.alloc.dupe(u8, path);
    if (pending_rom_path) |superseded| state.alloc.free(superseded);
    pending_rom_path = owned_path;
    state.requestGameStop();
}

pub fn unloadRom() void {
    const state = wasm_app_state orelse return;
    if (shutdown_requested) return;
    cancelPendingLoad(state);
    state.requestGameStop();
}

/// `neskwik_request_rom_load`, called by web/bridge.js.
pub fn exportedLoadRom(path: [*:0]const u8) callconv(.c) c_int {
    const requested_path = std.mem.span(path);
    loadRom(requested_path) catch |err| {
        std.log.err("Failed to load ROM '{s}': {s}", .{ requested_path, @errorName(err) });
        return 0;
    };
    return 1;
}

/// `neskwik_request_rom_unload`
pub fn exportedUnloadRom() callconv(.c) void {
    unloadRom();
}

/// `neskwik_shader_directory_imported`, called by web/bridge.js once a shader
/// folder was copied into the library (`/shaders/<folder>`).
pub fn shaderDirectoryImported(folder: [*:0]const u8) callconv(.c) c_int {
    const state = wasm_app_state orelse return 0;
    if (shutdown_requested) return 0;
    state.showImportedShaderFolder(std.mem.span(folder));
    return 1;
}

pub fn requestShutdown() void {
    if (shutdown_requested) return;
    shutdown_requested = true;
    const state = wasm_app_state orelse return;
    cancelPendingLoad(state);
    state.requestGameStop();
}

pub fn isShutdownRequested() bool {
    return shutdown_requested;
}

/// Polls detached worker teardown from the browser main loop. Returns true
/// only when a requested shutdown may safely destroy the application state.
pub fn poll() bool {
    const state = wasm_app_state orelse return shutdown_requested;

    if (state.hasLoadedGame()) {
        if (state.isEmulationRunning() or !state.finishGameStop()) return false;
    }

    if (shutdown_requested) return true;

    if (pending_rom_path) |path| {
        pending_rom_path = null;
        defer state.alloc.free(path);
        state.loadRom(path) catch |err| {
            std.log.err("Deferred ROM load failed for '{s}': {s}", .{ path, @errorName(err) });
        };
    }
    return false;
}

fn cancelPendingLoad(state: *AppState) void {
    if (pending_rom_path) |path| state.alloc.free(path);
    pending_rom_path = null;
}
