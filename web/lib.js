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

  // Runs on the panicking thread: Zig cannot unwind the wasm stack, but the
  // browser's JavaScript stack of this thread includes its wasm frames.
  neskwik_wasm_panic_stack__deps: ["$stringToNewUTF8"],
  neskwik_wasm_panic_stack: function () {
    const limit = Error.stackTraceLimit;
    Error.stackTraceLimit = 100;
    const stack = new Error().stack || "";
    Error.stackTraceLimit = limit;

    const frames = stack
      .split("\n")
      .filter((line) => line.includes("wasm-function"))
      .map((line) =>
        line
          .trim()
          .replace(/^at /, "")
          .replace(/^neskwik\.wasm\./, "")
          .replace(/[^\s(@]*neskwik\.wasm:/, ""),
      );
    // Start at the code that panicked, like `first_trace_addr` does natively.
    // Release builds strip function names, so nothing matches there.
    const panicFrame = /^(wasm\.browser\.panicStack|wasm\.panic\.|debug\.)/;
    while (frames.length > 1 && panicFrame.test(frames[0])) frames.shift();
    return stringToNewUTF8(frames.join("\n"));
  },

  // Proxied to the main thread, which owns the page. The app stops first: the
  // panicking thread may hold locks the next frame would block on forever,
  // and a blocked main thread never paints the dialog.
  neskwik_wasm_show_panic__proxy: "sync",
  neskwik_wasm_show_panic__deps: ["$MainLoop", "$UTF8ToString"],
  neskwik_wasm_show_panic: function (message, message_len, stack) {
    MainLoop.pause();
    Module["SDL3"]?.audioContext?.suspend();
    Module.showPanic(UTF8ToString(message, message_len), UTF8ToString(stack));
  },
});
