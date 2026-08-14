//! Functions implemented in JavaScript (web/lib.js).

extern fn neskwik_wasm_open_file() void;
extern fn neskwik_wasm_open_shader_file() void;
extern fn wasm_has_touch_input() bool;

pub fn openRomPicker() void {
    neskwik_wasm_open_file();
}

pub fn openShaderPicker() void {
    neskwik_wasm_open_shader_file();
}

pub fn hasTouchInput() bool {
    return wasm_has_touch_input();
}
