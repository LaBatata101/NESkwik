const std = @import("std");
const android = @import("android");

const android_api_level: android.ApiLevel = .android15;

pub fn build(b: *std.Build) !void {
    const exe_name = "neskwik";
    const package_name = "com.labatata.neskwik";

    var target = b.standardTargetOptions(.{});
    if (isWasmTarget(target)) {
        target.query.cpu_features_add.addFeatureSet(std.Target.wasm.featureSet(&.{
            .atomics,
            .bulk_memory,
        }));
        target = b.resolveTargetQuery(target.query);
    }
    const optimize = b.standardOptimizeOption(.{});
    const iroh_lib_dir = b.option([]const u8, "iroh-lib-dir", "Directory containing a prebuilt iroh-ffi static archive");
    const wasm = isWasmTarget(target);
    if (wasm) try applyZigStdlibPatches(b);
    const wasm_sdk = if (wasm) resolveWasmSdk(b) else null;
    const android_targets = android.standardTargets(b, target, android_api_level);

    var root_target_single = [_]std.Build.ResolvedTarget{target};
    const targets: []std.Build.ResolvedTarget = if (android_targets.len == 0)
        root_target_single[0..]
    else
        android_targets;

    const android_apk: ?*android.Apk = blk: {
        if (android_targets.len == 0) break :blk null;

        const android_sdk = android.Sdk.create(b, .{});
        const apk = android_sdk.createApk(.{
            .name = exe_name,
            .api_level = android_api_level,
            .build_tools_version = "36.1.0",
            .ndk_version = "28.2.13676358",
        });

        apk.setKeyStore(android_sdk.createKeyStore(.example));
        apk.setAndroidManifest(b.path("android/AndroidManifest.xml"));
        apk.addResourceDirectory(b.path("android/res"));
        apk.addJavaSourceFile(.{ .file = b.path("android/src/NeskwikActivity.java") });
        addAndroidLibcxxShared(b, apk, android_targets);

        const sdl_java_dep = b.dependency("sdl", .{
            .target = android_targets[0],
            .optimize = optimize,
            .preferred_linkage = .static,
        });
        const sdl_java_files = sdl_java_dep.namedWriteFiles("sdljava");
        for (sdl_java_files.files.items) |file| {
            apk.addJavaSourceFile(.{ .file = file.contents.copy });
        }

        break :blk apk;
    };

    for (targets) |resolved_target| {
        if (resolved_target.result.cpu.arch == .x86 or resolved_target.result.cpu.arch == .arm) continue;

        const deps = try createNessModule(b, resolved_target, optimize, iroh_lib_dir, wasm_sdk);

        const app_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = resolved_target,
            .optimize = optimize,
            .single_threaded = false,
            .strip = optimize == .ReleaseFast,
            .imports = &.{
                .{ .name = "ness", .module = deps.mod },
            },
        });

        if (resolved_target.result.abi.isAndroid()) {
            const apk = android_apk orelse @panic("Android APK should be initialized");
            const android_dep = b.dependency("android", .{
                .optimize = optimize,
                .target = resolved_target,
            });
            app_module.addImport("android", android_dep.module("android"));

            const app_lib = b.addLibrary(.{
                .name = "main",
                .root_module = app_module,
                .linkage = .dynamic,
                .use_llvm = true,
            });
            linkAppLibraries(app_lib, deps);
            app_lib.root_module.linkSystemLibrary("c++_shared", .{});
            apk.addArtifact(app_lib);
        } else if (wasm) {
            try addWasmApp(b, app_module, optimize, wasm_sdk.?);
        } else {
            const exe = b.addExecutable(.{
                .name = exe_name,
                .use_llvm = true,
                .root_module = app_module,
            });

            if (resolved_target.result.os.tag == .windows) {
                exe.subsystem = .Windows;
            }

            linkAppLibraries(exe, deps);

            if (resolved_target.result.os.tag == .macos) {
                exe.root_module.addObjectFile(b.path("third-party/MoltenVK/libMoltenVK.a"));

                // SDL finds a statically linked MoltenVK using:
                // dlsym(RTLD_DEFAULT, "vkGetInstanceProcAddr").
                //
                // Ensure the archive member is included and the symbol is visible
                // in the executable's dynamic symbol table.
                exe.forceUndefinedSymbol("_vkGetInstanceProcAddr");
                exe.rdynamic = true;

                exe.root_module.linkFramework("IOSurface", .{});
            }

            b.installArtifact(exe);

            const run_step = b.step("run", "Run the app");
            const run_cmd = b.addRunArtifact(exe);
            run_step.dependOn(&run_cmd.step);

            run_cmd.step.dependOn(b.getInstallStep());

            if (b.args) |args| {
                run_cmd.addArgs(args);
            }

            const profiler_exe = b.addExecutable(.{
                .name = "profiler",
                .use_llvm = true,
                .root_module = b.createModule(.{
                    .root_source_file = b.path("src/profiler.zig"),
                    .target = resolved_target,
                    .optimize = optimize,
                    .imports = &.{
                        .{ .name = "ness", .module = deps.mod },
                    },
                    .link_libc = true,
                }),
            });
            profiler_exe.root_module.addIncludePath(b.path("third-party/blip_buf-1.1.0"));
            profiler_exe.root_module.linkLibrary(deps.sdl_lib);
            profiler_exe.root_module.linkLibrary(deps.blip_buf_lib);

            const install_profiler = b.addInstallArtifact(profiler_exe, .{});
            b.getInstallStep().dependOn(&install_profiler.step);

            const profiler_step = b.step("profiler", "Run the deterministic 30-second profiler");
            const run_profiler = b.addRunArtifact(profiler_exe);
            run_profiler.setCwd(b.path("."));
            run_profiler.step.dependOn(&install_profiler.step);
            if (b.args) |args| {
                run_profiler.addArgs(args);
            }
            profiler_step.dependOn(&run_profiler.step);

            addTestStep(b, resolved_target, optimize, deps, exe);
        }
    }

    if (android_apk) |apk| {
        const installed_apk = apk.addInstallApk();
        b.getInstallStep().dependOn(&installed_apk.step);

        const run_step = b.step("run", "Install and run the app on an Android device");
        const adb_install = apk.sdk.addAdbInstall(installed_apk.source);
        const adb_start = apk.sdk.addAdbStart(package_name ++ "/" ++ package_name ++ ".NeskwikActivity");
        adb_start.step.dependOn(&adb_install.step);
        run_step.dependOn(&adb_start.step);
    }
}

fn addAndroidLibcxxShared(b: *std.Build, apk: *android.Apk, targets: []const std.Build.ResolvedTarget) void {
    for (targets) |target| {
        if (target.result.cpu.arch == .x86 or target.result.cpu.arch == .arm) continue;

        const system_triple = androidSystemTriple(b, target);
        const libcxx_path: std.Build.LazyPath = .{
            .cwd_relative = b.fmt("{s}/usr/lib/{s}/libc++_shared.so", .{
                apk.ndk.sysroot_path,
                system_triple,
            }),
        };

        switch (target.result.cpu.arch) {
            .aarch64 => apk.addLibraryFile(.arm64_v8a, libcxx_path),
            .x86_64 => apk.addLibraryFile(.x86_64, libcxx_path),
            else => @panic(b.fmt("unsupported Android target arch: {s}", .{@tagName(target.result.cpu.arch)})),
        }
    }
}

fn androidSystemTriple(b: *std.Build, target: std.Build.ResolvedTarget) []const u8 {
    if (!target.result.abi.isAndroid()) {
        @panic("expected Android target");
    }
    return switch (target.result.cpu.arch) {
        .aarch64 => "aarch64-linux-android",
        .x86_64 => "x86_64-linux-android",
        else => @panic(b.fmt("unsupported Android target arch: {s}", .{@tagName(target.result.cpu.arch)})),
    };
}

const NessDeps = struct {
    mod: *std.Build.Module,
    sdl_lib: *std.Build.Step.Compile,
    blip_buf_lib: *std.Build.Step.Compile,
    clay_lib: *std.Build.Step.Compile,
    font_raster_lib: *std.Build.Step.Compile,
    glslang_lib: *std.Build.Step.Compile,
    spirv_cross_lib: *std.Build.Step.Compile,
};

const WasmSdk = struct {
    /// Emscripten's system headers.
    include_dir: []const u8,
    /// Project-local Emscripten cache holding the sysroot, unless the sysroot
    /// was given with `--sysroot`.
    cache_dir: ?[]const u8,
    /// Generates the sysroot in `cache_dir`; everything compiled against its
    /// headers must wait for it.
    prepare: ?*std.Build.Step.Run,
};

/// Flags for C++ libraries using threads. Zig does not configure libc++ for
/// Emscripten's pthreads on its own.
const wasm_cxx_thread_flags = [_][]const u8{
    "-pthread",
    "-D__EMSCRIPTEN_PTHREADS__=1",
    "-D_LIBCPP_HAS_THREADS=1",
    "-D_LIBCPP_HAS_THREAD_API_PTHREAD=1",
    "-D_LIBCPP_HAS_MONOTONIC_CLOCK=1",
};

fn isWasmTarget(target: std.Build.ResolvedTarget) bool {
    return target.result.cpu.arch == .wasm32 and target.result.os.tag == .emscripten;
}

fn applyZigStdlibPatches(b: *std.Build) !void {
    const io = b.graph.io;
    const zig_exe = if (std.fs.path.isAbsolute(b.graph.zig_exe))
        try std.Io.Dir.realPathFileAbsoluteAlloc(io, b.graph.zig_exe, b.allocator)
    else
        try std.Io.Dir.cwd().realPathFileAlloc(io, b.graph.zig_exe, b.allocator);
    const zig_dir = std.fs.path.dirname(zig_exe) orelse return error.InvalidZigExecutablePath;
    const std_dir = b.pathJoin(&.{ zig_dir, "lib", "std" });
    const stdlib_dir = try std.Io.Dir.openDirAbsolute(io, std_dir, .{});
    defer stdlib_dir.close(io);

    try replaceInFile(b, stdlib_dir, "Io/Threaded.zig", &.{
        .{
            .old = "            const to: i64 = if (timeout_ns) |ns| ns else -1;",
            .new = "            const to: i64 = if (timeout_ns) |ns| std.math.cast(i64, ns) orelse std.math.maxInt(i64) else -1;",
        },
        .{
            .old = "fn doNothingSignalHandler(_: posix.SIG) callconv(.c) void {}",
            .new = "const SignalHandler = if (builtin.os.tag == .emscripten) c_int else posix.SIG;\n" ++
                "fn doNothingSignalHandler(_: SignalHandler) callconv(.c) void {}",
        },
    });
    try replaceInFile(b, stdlib_dir, "os/emscripten.zig", &.{
        .{
            .old = "    pub fn STOPSIG(s: u32) u32 {",
            .new = "    pub fn STOPSIG(s: u32) SIG {",
        },
    });
}

const Replacement = struct {
    old: []const u8,
    new: []const u8,
};

fn replaceInFile(b: *std.Build, dir: std.Io.Dir, path: []const u8, replacements: []const Replacement) !void {
    var contents = try dir.readFileAlloc(b.graph.io, path, b.allocator, .unlimited);
    var changed = false;

    for (replacements) |replacement| {
        if (std.mem.indexOf(u8, contents, replacement.old) != null) {
            contents = try std.mem.replaceOwned(u8, b.allocator, contents, replacement.old, replacement.new);
            changed = true;
        } else if (std.mem.indexOf(u8, contents, replacement.new) == null) {
            std.log.err("Zig stdlib patch does not match '{s}'", .{path});
            return error.ZigStdlibPatchDoesNotApply;
        }
    }

    if (changed) {
        try dir.writeFile(b.graph.io, .{ .sub_path = path, .data = contents });
        std.log.info("patched Zig stdlib file '{s}'", .{path});
    }
}

fn resolveWasmSdk(b: *std.Build) WasmSdk {
    if (b.sysroot) |path| return .{
        .include_dir = b.pathJoin(&.{ path, "include" }),
        .cache_dir = null,
        .prepare = null,
    };

    _ = b.findProgram(&.{"em-config"}, &.{}) catch
        @panic("wasm32-emscripten requires an activated Emscripten installation (em-config was not found)");
    const embuilder = b.findProgram(&.{"embuilder"}, &.{}) catch
        @panic("wasm32-emscripten requires embuilder in PATH");
    const cache = b.pathFromRoot(".zig-cache/emscripten");
    const prepare = b.addSystemCommand(&.{ embuilder, "build", "sysroot" });
    prepare.setEnvironmentVariable("EM_CACHE", cache);
    return .{
        .include_dir = b.pathJoin(&.{ cache, "sysroot", "include" }),
        .cache_dir = cache,
        .prepare = prepare,
    };
}

fn addCFlags(
    b: *std.Build,
    compile: *std.Build.Step.Compile,
    flags: []const []const u8,
) void {
    for (compile.root_module.link_objects.items) |link_object| switch (link_object) {
        .c_source_file => |source| source.flags = appendStrings(b, source.flags, flags),
        .c_source_files => |sources| sources.flags = appendStrings(b, sources.flags, flags),
        .other_step => |dependency| addCFlags(b, dependency, flags),
        else => {},
    };
}

fn addWasmApp(
    b: *std.Build,
    app_module: *std.Build.Module,
    optimize: std.builtin.OptimizeMode,
    sdk: WasmSdk,
) !void {
    app_module.link_libc = true;
    app_module.addIncludePath(b.path("third-party/SDL/include"));

    const app_object = b.addLibrary(.{
        .name = "neskwik-wasm",
        .root_module = app_module,
        .linkage = .static,
    });
    if (optimize == .Debug) {
        // Debug C code calls into the UBSan runtime, which Zig only links
        // itself. emcc links here, so ship it in the archive, with Zig's
        // compiler-rt for the f80 conversions it uses that Emscripten's
        // builtins lack. Zig exports compiler-rt weakly, so the two do not clash.
        app_object.bundle_ubsan_rt = true;
        app_object.bundle_compiler_rt = true;
    }

    // Zig only compiles; emcc links the app and every archive it depends on
    // and generates the JavaScript runtime.
    const emcc = b.findProgram(&.{"emcc"}, &.{}) catch
        @panic("wasm32-emscripten requires emcc in PATH");
    const link = b.addSystemCommand(&.{emcc});
    if (sdk.cache_dir) |cache| link.setEnvironmentVariable("EM_CACHE", cache);

    var graph: WasmGraph = .{ .b = b, .sdk = sdk, .link = link };
    try graph.visitCompile(app_object);

    link.addArg("-o");
    const js_output = link.addOutputFileArg("neskwik.js");
    link.addArgs(&.{
        switch (optimize) {
            .Debug => "-O0",
            .ReleaseSafe => "-O2",
            .ReleaseFast => "-O3",
            .ReleaseSmall => "-Oz",
        },
        if (optimize == .Debug) "-sASSERTIONS=1" else "-sASSERTIONS=0",
        "-pthread",
        // The emulation thread plus the shader presets compiling at once; see
        // `compileJobs` in src/wasm/gles_backend.zig.
        "-sPTHREAD_POOL_SIZE=8",
        "-sALLOW_MEMORY_GROWTH=1",
        "-sSTACK_SIZE=1048576",
        "-sFORCE_FILESYSTEM=1",
        "-sSUPPORT_LONGJMP=emscripten",
        "-sDISABLE_EXCEPTION_CATCHING=0",
        "-sENVIRONMENT=web",
        "-sMAX_WEBGL_VERSION=2",
        "-sDEFAULT_TO_CXX=1",
        "-sEXPORTED_FUNCTIONS=['_main', '_neskwik_request_rom_load', '_neskwik_request_rom_unload', '_neskwik_shader_directory_imported']",
        "-sEXPORTED_RUNTIME_METHODS=['ccall']",
        "-lidbfs.js",
        "--pre-js",
    });
    link.addFileArg(b.path("web/bridge.js"));
    link.addArg("--js-library");
    link.addFileArg(b.path("web/lib.js"));
    if (optimize == .Debug) link.addArgs(&.{
        // The default 16 MiB does not fit the static data of an unoptimized build.
        "-sINITIAL_MEMORY=33554432",
        // Keep function names so wasm stack traces in the browser are readable.
        "--profiling-funcs",
    });

    const install_assets = b.addInstallDirectory(.{
        .source_dir = js_output.dirname(),
        .install_dir = .prefix,
        .install_subdir = "web",
        .include_extensions = &.{ ".js", ".wasm" },
    });
    const install_html = b.addInstallFileWithDir(b.path("web/index.html"), .prefix, "web/index.html");
    const install_favicon = b.addInstallFileWithDir(b.path("resources/icons/nes-icon-64x64.png"), .prefix, "web/favicon.png");
    b.getInstallStep().dependOn(&install_assets.step);
    b.getInstallStep().dependOn(&install_html.step);
    b.getInstallStep().dependOn(&install_favicon.step);

    const run_step = b.step("run", "Serve the browser build with emrun");
    const emrun = b.findProgram(&.{"emrun"}, &.{}) catch
        @panic("zig build run for wasm32-emscripten requires emrun in PATH");
    const serve = b.addSystemCommand(&.{ emrun, b.getInstallPath(.prefix, "web/index.html") });
    serve.step.dependOn(b.getInstallStep());
    run_step.dependOn(&serve.step);
}

/// Walks the module graph of the app to prepare every module and library for
/// Emscripten: all of them compile against the sysroot headers (and so wait
/// for embuilder to generate them), and every library is passed to emcc.
const WasmGraph = struct {
    b: *std.Build,
    sdk: WasmSdk,
    link: *std.Build.Step.Run,
    visited_modules: std.AutoHashMapUnmanaged(*std.Build.Module, void) = .empty,
    visited_compiles: std.AutoHashMapUnmanaged(*std.Build.Step.Compile, void) = .empty,

    fn visitCompile(self: *WasmGraph, compile: *std.Build.Step.Compile) std.mem.Allocator.Error!void {
        if ((try self.visited_compiles.getOrPut(self.b.allocator, compile)).found_existing) return;
        if (self.sdk.prepare) |prepare| compile.step.dependOn(&prepare.step);
        self.link.addFileArg(compile.getEmittedBin());
        try self.visitModule(compile.root_module);
    }

    fn visitModule(self: *WasmGraph, module: *std.Build.Module) std.mem.Allocator.Error!void {
        if ((try self.visited_modules.getOrPut(self.b.allocator, module)).found_existing) return;
        module.addSystemIncludePath(.{ .cwd_relative = self.sdk.include_dir });
        for (module.link_objects.items) |link_object| switch (link_object) {
            .other_step => |compile| try self.visitCompile(compile),
            .static_path => |path| self.link.addFileArg(path),
            else => {},
        };
        for (module.import_table.values()) |import| try self.visitModule(import);
    }
};

fn appendStrings(
    b: *std.Build,
    existing: []const []const u8,
    additions: []const []const u8,
) []const []const u8 {
    const result = b.allocator.alloc([]const u8, existing.len + additions.len) catch @panic("OOM");
    @memcpy(result[0..existing.len], existing);
    @memcpy(result[existing.len..], additions);
    return result;
}

fn createNessModule(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    iroh_lib_dir: ?[]const u8,
    wasm_sdk: ?WasmSdk,
) !NessDeps {
    const wasm = isWasmTarget(target);
    const preferred_linkage: std.builtin.LinkMode = if (target.result.abi.isAndroid()) .dynamic else .static;
    const enable_pic: ?bool = if (target.result.abi.isAndroid()) true else null;

    const mod = b.createModule(.{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .single_threaded = false,
    });
    const feature_options = b.addOptions();
    feature_options.addOption(bool, "wasm", wasm);
    mod.addOptions("features", feature_options);
    addAndroidCImportMacros(b, target, mod);

    const sdl_dep = if (wasm_sdk) |sdk|
        b.dependency("sdl", .{
            .target = target,
            .optimize = optimize,
            .preferred_linkage = preferred_linkage,
            .system_include_path = std.Build.LazyPath{ .cwd_relative = sdk.include_dir },
            .emscripten_pthreads = false,
        })
    else
        b.dependency("sdl", .{
            .target = target,
            .optimize = optimize,
            .preferred_linkage = preferred_linkage,
        });
    const sdl_lib = sdl_dep.artifact("SDL3");
    mod.linkLibrary(sdl_lib);

    const blip_buf_lib = b.addLibrary(
        .{
            .name = "blip_buf",
            .linkage = .static,
            .root_module = b.createModule(
                .{
                    .target = target,
                    .optimize = optimize,
                    .pic = enable_pic,
                    .link_libc = true,
                },
            ),
        },
    );
    blip_buf_lib.root_module.addCSourceFile(.{ .file = b.path("third-party/blip_buf-1.1.0/blip_buf.c") });
    mod.addIncludePath(b.path("third-party/blip_buf-1.1.0"));
    mod.linkLibrary(blip_buf_lib);

    const clay_mod = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .pic = enable_pic,
        .link_libc = true,
    });
    clay_mod.addIncludePath(b.path("third-party/clay"));

    const clay_lib = b.addLibrary(.{
        .name = "clay",
        .linkage = .static,
        .root_module = clay_mod,
    });
    clay_lib.root_module.addCSourceFile(.{ .file = b.path("third-party/clay/clay_impl.c") });
    mod.linkLibrary(clay_lib);

    const font_raster_dep = b.dependency("font_raster", .{
        .target = target,
        .optimize = optimize,
        .pic = true,
        .@"enable-harfbuzz" = false,
        .@"enable-libpng" = false,
    });
    const font_raster_mod = font_raster_dep.module("font_raster");
    if (target.result.abi.isAndroid()) {
        const freetype_mod = font_raster_mod.import_table.get("freetype") orelse
            @panic("font_raster does not expose its freetype import");
        addAndroidCImportMacros(b, target, freetype_mod);
    }
    mod.addImport("font_raster", font_raster_mod);
    const font_raster_lib = blk: {
        for (font_raster_mod.link_objects.items) |link_object| {
            switch (link_object) {
                .other_step => |compile| break :blk compile,
                else => {},
            }
        }
        @panic("font_raster module did not link its FreeType artifact");
    };

    const glslang_lib = b.dependency("glslang", .{
        .target = target,
        .optimize = optimize,
        .enable_pic = true,
    }).artifact("glslang");
    mod.linkLibrary(glslang_lib);

    const spirv_cross_lib = b.dependency("spirv_cross", .{
        .target = target,
        .optimize = optimize,
        .pic = true,
    }).artifact("spirv-cross-c");
    mod.linkLibrary(spirv_cross_lib);

    if (wasm) {
        // These C archives bypass emcc's compile driver, so apply the
        // transforms emcc would: FreeType's setjmp/longjmp calls, and the C++
        // exceptions SPIRV-Cross reports unsupported shaders with (its C API
        // turns them into error codes instead of aborting the program).
        addCFlags(b, font_raster_lib, &.{ "-mllvm", "-enable-emscripten-sjlj" });
        addCFlags(b, glslang_lib, &wasm_cxx_thread_flags);
        addCFlags(b, spirv_cross_lib, &(wasm_cxx_thread_flags ++ [_][]const u8{ "-mllvm", "-enable-emscripten-cxx-exceptions" }));
    }

    const zeit_dep = b.dependency("zeit", .{
        .target = target,
        .optimize = optimize,
    });
    mod.addImport("zeit", zeit_dep.module("zeit"));

    if (!wasm) {
        const vk_headers = b.dependency("vulkan_headers", .{});
        mod.addIncludePath(vk_headers.path("include"));

        const iroh_dep = if (iroh_lib_dir) |lib_dir|
            b.dependency("iroh", .{
                .target = target,
                .optimize = optimize,
                .iroh_lib_dir = lib_dir,
            })
        else
            b.dependency("iroh", .{
                .target = target,
                .optimize = optimize,
            });
        const iroh_mod = iroh_dep.module("iroh");
        addAndroidCImportMacros(b, target, iroh_mod);
        mod.addImport("iroh", iroh_mod);
        if (target.result.os.tag == .linux and !target.result.abi.isAndroid()) {
            // Rust's static standard library uses the platform unwinder. Shader
            // builds also pull this in through libc++, but netplay must not rely
            // on that incidental dependency.
            mod.linkSystemLibrary("unwind", .{});
        }
    }

    mod.addAnonymousImport("pixeloid_font", .{ .root_source_file = b.path("resources/fonts/PixeloidSans.ttf") });
    mod.addAnonymousImport("nes_controller_img", .{ .root_source_file = b.path("resources/images/nes-controller.png") });
    mod.addAnonymousImport("app_icon", .{ .root_source_file = b.path("resources/icons/nes-icon-256x256.png") });
    mod.addAnonymousImport("fast_forward_icon", .{ .root_source_file = b.path("resources/icons/fast_forward_32x32.png") });
    mod.addAnonymousImport("skip_next_icon", .{ .root_source_file = b.path("resources/icons/skip_next_32x32.png") });
    mod.addAnonymousImport("play_icon", .{ .root_source_file = b.path("resources/icons/play_arrow_32x32.png") });
    mod.addAnonymousImport("stop_icon", .{ .root_source_file = b.path("resources/icons/stop_32x32.png") });
    mod.addAnonymousImport("menu_icon", .{ .root_source_file = b.path("resources/icons/menu_32x32.png") });
    mod.addAnonymousImport("controller_icon", .{ .root_source_file = b.path("resources/icons/controller_32x32.png") });
    mod.addAnonymousImport("keyboard_icon", .{ .root_source_file = b.path("resources/icons/keyboard_32x32.png") });
    mod.addAnonymousImport("dpad_up_icon", .{ .root_source_file = b.path("resources/icons/dpad_up_32x32.png") });
    mod.addAnonymousImport("dpad_down_icon", .{ .root_source_file = b.path("resources/icons/dpad_down_32x32.png") });
    mod.addAnonymousImport("dpad_left_icon", .{ .root_source_file = b.path("resources/icons/dpad_left_32x32.png") });
    mod.addAnonymousImport("dpad_right_icon", .{ .root_source_file = b.path("resources/icons/dpad_right_32x32.png") });
    mod.addAnonymousImport("copy_icon", .{ .root_source_file = b.path("resources/icons/copy_32x32.png") });

    try addBorderShaderImports(b, mod);

    return .{
        .mod = mod,
        .sdl_lib = sdl_lib,
        .blip_buf_lib = blip_buf_lib,
        .clay_lib = clay_lib,
        .font_raster_lib = font_raster_lib,
        .glslang_lib = glslang_lib,
        .spirv_cross_lib = spirv_cross_lib,
    };
}

fn addAndroidCImportMacros(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    module: *std.Build.Module,
) void {
    if (!target.result.abi.isAndroid()) return;

    const api_level = b.fmt("{d}", .{android_api_level});
    module.addCMacro("__ANDROID_API__", api_level);
    module.addCMacro("__ANDROID_MIN_SDK_VERSION__", api_level);

    // Zig 0.16's C translator rejects Bionic's nullability annotations
    // inside array parameter declarators. They do not affect the ABI.
    module.addCMacro("_Nullable", "");
    module.addCMacro("_Nonnull", "");
    module.addCMacro("_Null_unspecified", "");
}

const BorderShaderFile = struct {
    rel_path: []const u8,
    import_name: []const u8,
};

fn addBorderShaderImports(b: *std.Build, mod: *std.Build.Module) !void {
    const shader_dir_path = "resources/shaders";

    var files: std.ArrayList(BorderShaderFile) = .empty;
    try collectBorderShaderFiles(b, &files, shader_dir_path, "");

    std.mem.sort(BorderShaderFile, files.items, {}, struct {
        fn lessThan(_: void, lhs: BorderShaderFile, rhs: BorderShaderFile) bool {
            return std.mem.lessThan(u8, lhs.rel_path, rhs.rel_path);
        }
    }.lessThan);

    const generated_source = try renderBorderShaderEntries(b, files.items);
    const generated_path = b.addWriteFiles().add("border_shader_entries.zig", generated_source);
    const generated_mod = b.createModule(.{
        .root_source_file = generated_path,
    });

    for (files.items) |file| {
        const file_path = try std.fs.path.join(b.allocator, &.{ shader_dir_path, file.rel_path });
        generated_mod.addAnonymousImport(
            file.import_name,
            .{ .root_source_file = b.path(file_path) },
        );
    }

    mod.addImport("border_shader_entries", generated_mod);
}

fn collectBorderShaderFiles(
    b: *std.Build,
    files: *std.ArrayList(BorderShaderFile),
    shader_dir_path: []const u8,
    rel_dir: []const u8,
) !void {
    const dir_path = if (rel_dir.len == 0)
        shader_dir_path
    else
        try std.fs.path.join(b.allocator, &.{ shader_dir_path, rel_dir });

    var shader_dir = b.build_root.handle.openDir(b.graph.io, dir_path, .{ .iterate = true }) catch |err|
        @panic(b.fmt("failed to open border shader directory '{s}': {s}", .{ dir_path, @errorName(err) }));
    defer shader_dir.close(b.graph.io);

    var iter = shader_dir.iterate();
    while (try iter.next(b.graph.io)) |entry| {
        const rel_path = if (rel_dir.len == 0)
            try b.allocator.dupe(u8, entry.name)
        else
            try std.fmt.allocPrint(b.allocator, "{s}/{s}", .{ rel_dir, entry.name });

        if (entry.kind == .directory) {
            try collectBorderShaderFiles(b, files, shader_dir_path, rel_path);
            continue;
        }
        if (entry.kind != .file) continue;

        const ext = std.fs.path.extension(entry.name);
        if (!isBorderShaderSourceFile(ext)) continue;

        try files.append(b.allocator, .{
            .rel_path = rel_path,
            .import_name = try borderShaderImportName(b, rel_path, ext),
        });
    }
}

fn isBorderShaderSourceFile(ext: []const u8) bool {
    return std.mem.eql(u8, ext, ".slang") or
        std.mem.eql(u8, ext, ".slangp") or
        std.mem.eql(u8, ext, ".h") or
        std.mem.eql(u8, ext, ".inc");
}

fn borderShaderImportName(b: *std.Build, rel_path: []const u8, ext: []const u8) ![]const u8 {
    var import_name: std.ArrayList(u8) = .empty;
    try import_name.appendSlice(b.allocator, "border_shader_");
    for (rel_path) |ch| {
        try import_name.append(b.allocator, if (std.ascii.isAlphanumeric(ch) or ch == '_') ch else '_');
    }

    const suffix = if (std.mem.eql(u8, ext, ".slangp"))
        "_preset"
    else if (std.mem.eql(u8, ext, ".slang"))
        "_source"
    else if (std.mem.eql(u8, ext, ".h") or std.mem.eql(u8, ext, ".inc"))
        "_header"
    else
        unreachable;

    try import_name.appendSlice(b.allocator, suffix);
    return import_name.toOwnedSlice(b.allocator);
}

fn renderBorderShaderEntries(b: *std.Build, files: []const BorderShaderFile) ![]const u8 {
    var source: std.ArrayList(u8) = .empty;
    try source.appendSlice(b.allocator,
        \\pub const Entry = struct {
        \\    path: []const u8,
        \\    source: []const u8,
        \\};
        \\
        \\pub const entries = [_]Entry{
        \\
    );

    for (files) |file| {
        try source.appendSlice(b.allocator, "    .{ .path = \"builtin://border-shaders/");
        try appendZigStringContent(b, &source, file.rel_path);
        try source.appendSlice(b.allocator, "\", .source = @embedFile(\"");
        try appendZigStringContent(b, &source, file.import_name);
        try source.appendSlice(b.allocator, "\") },\n");
    }

    try source.appendSlice(b.allocator, "};\n");
    return source.toOwnedSlice(b.allocator);
}

fn appendZigStringContent(b: *std.Build, source: *std.ArrayList(u8), value: []const u8) !void {
    for (value) |ch| {
        switch (ch) {
            '\\' => try source.appendSlice(b.allocator, "\\\\"),
            '"' => try source.appendSlice(b.allocator, "\\\""),
            '\n' => try source.appendSlice(b.allocator, "\\n"),
            '\r' => try source.appendSlice(b.allocator, "\\r"),
            '\t' => try source.appendSlice(b.allocator, "\\t"),
            else => try source.append(b.allocator, ch),
        }
    }
}

fn linkAppLibraries(compile: *std.Build.Step.Compile, deps: NessDeps) void {
    compile.root_module.linkLibrary(deps.sdl_lib);
    compile.root_module.linkLibrary(deps.blip_buf_lib);
    compile.root_module.linkLibrary(deps.clay_lib);
    compile.root_module.linkLibrary(deps.font_raster_lib);
    compile.root_module.linkLibrary(deps.glslang_lib);
    compile.root_module.linkLibrary(deps.spirv_cross_lib);
}

fn addTestStep(
    b: *std.Build,
    target: std.Build.ResolvedTarget,
    optimize: std.builtin.OptimizeMode,
    deps: NessDeps,
    exe: *std.Build.Step.Compile,
) void {
    const test_filters = b.option([]const []const u8, "test-filter", "Skip tests that do not match any filter") orelse &[0][]const u8{};
    const no_run = b.option(bool, "no-run", "Don't run the test") orelse false;
    const rom_tests_enabled = b.option(bool, "rom-tests", "Run ROM tests. This is very slow!") orelse false;
    const skipped_rom_tests = b.option([]const []const u8, "skip-rom-test", "Skip ROM tests matching any pattern") orelse &[0][]const u8{};

    const test_step = b.step("test", "Run tests");
    if (!no_run and rom_tests_enabled) {
        const test_exe = b.addExecutable(.{
            .name = "rom-tests",
            .use_llvm = true,
            .root_module = b.createModule(.{
                .root_source_file = b.path("src/test_runners/rom_test_runner.zig"),
                .target = target,
                .optimize = optimize,
                .imports = &.{
                    .{ .name = "ness", .module = deps.mod },
                },
            }),
        });
        const rom_mod_test_artifacts = b.addInstallArtifact(test_exe, .{
            .dest_dir = .{
                .override = .{
                    .custom = "tests",
                },
            },
        });
        test_exe.root_module.addIncludePath(b.path("third-party/blip_buf-1.1.0"));

        test_exe.root_module.linkLibrary(deps.sdl_lib);
        test_exe.root_module.linkLibrary(deps.blip_buf_lib);
        test_exe.root_module.link_libc = true;

        const run_rom_tests = b.addRunArtifact(test_exe);

        for (test_filters) |filter| {
            run_rom_tests.addArg(filter);
        }
        for (skipped_rom_tests) |skip| {
            run_rom_tests.addArg("--skip");
            run_rom_tests.addArg(skip);
        }

        test_step.dependOn(&rom_mod_test_artifacts.step);
        test_step.dependOn(&run_rom_tests.step);
    } else {
        const mod_tests = b.addTest(.{
            .name = "ness-mod-test",
            .root_module = deps.mod,
            .filters = test_filters,
            .use_llvm = true,
            .test_runner = .{ .path = b.path("src/test_runners/test_runner.zig"), .mode = .simple },
        });
        const mod_test_artifacts = b.addInstallArtifact(mod_tests, .{
            .dest_dir = .{
                .override = .{
                    .custom = "tests",
                },
            },
        });

        const run_mod_tests = b.addRunArtifact(mod_tests);
        const exe_tests = b.addTest(.{
            .name = "exe-test",
            .root_module = exe.root_module,
            .filters = test_filters,
            .use_llvm = true,
            .test_runner = .{ .path = b.path("src/test_runners/test_runner.zig"), .mode = .simple },
        });
        b.installArtifact(exe_tests);

        const run_exe_tests = b.addRunArtifact(exe_tests);

        test_step.dependOn(&mod_test_artifacts.step);
        if (!no_run) {
            test_step.dependOn(&run_mod_tests.step);
            test_step.dependOn(&run_exe_tests.step);
        }
    }
}
