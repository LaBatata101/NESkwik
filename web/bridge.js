Module.preRun = Module.preRun || [];
Module.preRun.push(() => {
  FS.mkdir("/storage");
  FS.mount(IDBFS, { autoPersist: true }, "/storage");
  FS.mkdir("/roms");
  FS.mount(IDBFS, { autoPersist: true }, "/roms");
  FS.mkdir("/shaders");
  FS.mount(IDBFS, { autoPersist: true }, "/shaders");

  addRunDependency("idbfs");
  FS.syncfs(true, (err) => {
    if (err) console.error("IDBFS initialization failed:", err);
    removeRunDependency("idbfs");
  });
});

(function () {
  if (typeof document === "undefined") return;

  const MAX_ROM_BYTES = 2 * 1024 * 1024;
  const MAX_SHADER_RESOURCE_BYTES = 8 * 1024 * 1024;
  Module.canvas = document.getElementById("canvas");

  function reportImportError(error) {
    console.error(error);
    const status = document.getElementById("status");
    if (status) status.textContent = error.message || String(error);
  }

  async function loadRom(file) {
    if (file.size > MAX_ROM_BYTES) {
      throw new Error("ROM is larger than the supported 2 MiB limit.");
    }

    const path = "/roms/" + file.name;
    FS.writeFile(path, new Uint8Array(await file.arrayBuffer()));
    const accepted = Module.ccall("neskwik_request_rom_load", "number", ["string"], [path]);
    if (!accepted) throw new Error("NESkwik could not accept this ROM load request.");
  }

  function shaderRelativePath(file) {
    const path = (file.webkitRelativePath || file.name).replaceAll("\\", "/");
    const parts = path.split("/").filter(Boolean);
    if (parts.length === 0 || parts.some(part => part === "." || part === "..")) {
      throw new Error("Invalid shader file path.");
    }
    return parts.join("/");
  }

  // Presets, shader sources, the files they `#include`, and LUT images.
  const SHADER_RESOURCE_EXTENSIONS = [".slangp", ".slang", ".inc", ".h", ".params", ".png", ".jpg"];

  // Shader folders are kept in the persistent "/shaders" library, one
  // directory per imported folder, so they only need to be imported once.
  const SHADER_LIBRARY = "/shaders";

  function removeTree(path) {
    if (!FS.analyzePath(path).exists) return;
    if (FS.isDir(FS.stat(path).mode)) {
      for (const name of FS.readdir(path)) {
        if (name !== "." && name !== "..") removeTree(path + "/" + name);
      }
      FS.rmdir(path);
    } else {
      FS.unlink(path);
    }
  }

  async function importShaderDirectory(fileList) {
    const files = Array.from(fileList || []).filter(file => {
      const name = file.name.toLowerCase();
      return SHADER_RESOURCE_EXTENSIONS.some(extension => name.endsWith(extension));
    });
    if (!files.some(file => file.name.toLowerCase().endsWith(".slangp"))) {
      throw new Error("The selected directory does not contain any .slangp presets.");
    }
    const oversized = files.find(file => file.size > MAX_SHADER_RESOURCE_BYTES);
    if (oversized) throw new Error(oversized.name + " is larger than the supported 8 MiB limit.");

    // Relative paths start with the name of the selected folder.
    const folder = shaderRelativePath(files[0]).split("/")[0];
    // Importing a folder again replaces it, picking up added or removed files.
    removeTree(SHADER_LIBRARY + "/" + folder);
    for (const file of files) {
      const path = SHADER_LIBRARY + "/" + shaderRelativePath(file);
      FS.mkdirTree(path.slice(0, path.lastIndexOf("/")));
      FS.writeFile(path, new Uint8Array(await file.arrayBuffer()));
    }

    const accepted = Module.ccall("neskwik_shader_directory_imported", "number", ["string"], [folder]);
    if (!accepted) throw new Error("NESkwik could not open the imported shader directory.");
  }


  Module.onRuntimeInitialized = function () {
    const loading = document.getElementById("loading");
    if (loading) loading.hidden = true;
  };

  addEventListener("DOMContentLoaded", function () {
    const picker = document.getElementById("rom-picker");
    picker.addEventListener("change", function () {
      const file = picker.files && picker.files[0];
      picker.value = "";
      if (file) loadRom(file).catch(reportImportError);
    });

    const shaderPicker = document.getElementById("shader-picker");
    shaderPicker.addEventListener("change", function () {
      const files = Array.from(shaderPicker.files || []);
      shaderPicker.value = "";
      if (files.length > 0) importShaderDirectory(files).catch(reportImportError);
    });

    for (const eventName of ["dragenter", "dragover"]) {
      document.addEventListener(eventName, event => { event.preventDefault(); event.dataTransfer.dropEffect = "copy"; });
    }
    document.addEventListener("drop", function (event) {
      event.preventDefault();
      const file = event.dataTransfer.files && event.dataTransfer.files[0];
      if (file) loadRom(file).catch(reportImportError);
    });
  });
})();
