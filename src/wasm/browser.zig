//! Functions implemented in JavaScript (web/lib.js).

extern fn neskwik_wasm_open_file() void;
extern fn neskwik_wasm_open_shader_file() void;
extern fn wasm_has_touch_input() bool;
extern fn neskwik_wasm_panic_stack() ?[*:0]u8;
extern fn neskwik_wasm_show_panic(message: [*]const u8, message_len: usize, stack: [*:0]const u8) void;

pub fn openRomPicker() void {
    neskwik_wasm_open_file();
}

pub fn openShaderPicker() void {
    neskwik_wasm_open_shader_file();
}

pub fn hasTouchInput() bool {
    return wasm_has_touch_input();
}

/// Stack trace of the calling thread, from the browser's JavaScript stack.
/// It is never freed: the caller is about to crash.
pub fn panicStack() [*:0]const u8 {
    return neskwik_wasm_panic_stack() orelse "";
}

/// Shows the crash dialog and stops the app. Blocks until the main thread
/// has shown it.
pub fn showPanic(message: []const u8, stack: [*:0]const u8) void {
    neskwik_wasm_show_panic(message.ptr, message.len, stack);
}
