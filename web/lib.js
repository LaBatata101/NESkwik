addToLibrary({
  neskwik_wasm_open_file: function () {
    document.getElementById("rom-picker").click();
  },
  neskwik_wasm_open_shader_file: function () {
    document.getElementById("shader-picker").click();
  },
  wasm_has_touch_input: function () {
    return window.matchMedia("(pointer: coarse)").matches;
  },
});
