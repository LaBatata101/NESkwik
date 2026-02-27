const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const mod = b.addModule("ness", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
    });

    const sdl_dep = b.dependency("sdl", .{
        .target = target,
        .optimize = optimize,
    });
    const sdl_lib = sdl_dep.artifact("SDL3");
    mod.linkLibrary(sdl_lib);

    const blip_buf_lib = b.addLibrary(
        .{ .name = "blip_buf", .linkage = .static, .root_module = b.createModule(
            .{
                .target = target,
                .optimize = optimize,
                .link_libc = true,
            },
        ) },
    );
    blip_buf_lib.addCSourceFile(.{ .file = b.path("third-party/blip_buf-1.1.0/blip_buf.c") });
    mod.addIncludePath(b.path("third-party/blip_buf-1.1.0"));
    mod.linkLibrary(blip_buf_lib);

    const clay_mod = b.createModule(.{
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });
    clay_mod.addIncludePath(b.path("third-party/clay"));

    const clay_lib = b.addLibrary(.{
        .name = "clay",
        .linkage = .static,
        .root_module = clay_mod,
    });
    clay_lib.addCSourceFile(.{ .file = b.path("third-party/clay/clay_impl.c") });
    mod.linkLibrary(clay_lib);

    const sdl_ttf_dep = b.dependency("sdl_ttf", .{
        .target = target,
        .optimize = optimize,
    });
    const sdl_ttf_lib = sdl_ttf_dep.artifact("SDL3_ttf");
    mod.linkLibrary(sdl_ttf_lib);

    const glslang_dep = b.dependency("glslang", .{
        .target = target,
        .optimize = optimize,
    });
    const glslang_lib = glslang_dep.artifact("glslang");
    mod.linkLibrary(glslang_lib);

    const spirv_cross_dep = b.dependency("spirv_cross", .{
        .target = target,
        .optimize = optimize,
    });
    const spirv_cross_lib = spirv_cross_dep.artifact("spirv-cross-c");
    mod.linkLibrary(spirv_cross_lib);

    mod.linkSystemLibrary("vulkan", .{});

    mod.addAnonymousImport("pixeloid_font", .{ .root_source_file = b.path("fonts/PixeloidSans.ttf") });

    const exe = b.addExecutable(.{
        .name = "ness",
        .use_llvm = true,
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .strip = optimize == .ReleaseFast,
            .imports = &.{
                .{ .name = "ness", .module = mod },
            },
        }),
    });
    exe.linkLibrary(clay_lib);
    exe.linkLibrary(sdl_ttf_lib);
    exe.linkLibrary(glslang_lib);
    exe.linkLibrary(spirv_cross_lib);

    b.installArtifact(exe);

    const run_step = b.step("run", "Run the app");
    const run_cmd = b.addRunArtifact(exe);
    run_step.dependOn(&run_cmd.step);

    run_cmd.step.dependOn(b.getInstallStep());

    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const test_filters = b.option([]const []const u8, "test-filter", "Skip tests that do not match any filter") orelse &[0][]const u8{};
    const no_run = b.option(bool, "no-run", "Don't run the test") orelse false;
    const rom_tests_enabled = b.option(bool, "rom-tests", "Run ROM tests. This is very slow!") orelse false;

    const test_step = b.step("test", "Run tests");
    if (!no_run and rom_tests_enabled) {
        const test_exe = b.addExecutable(.{
            .name = "rom-tests",
            .root_module = b.createModule(.{
                .root_source_file = b.path("src/test_runners/rom_test_runner.zig"),
                .target = target,
                .optimize = optimize,
                .imports = &.{
                    .{ .name = "ness", .module = mod },
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

        test_exe.root_module.linkLibrary(sdl_lib);
        test_exe.root_module.linkLibrary(blip_buf_lib);
        test_exe.root_module.link_libc = true;

        const run_rom_tests = b.addRunArtifact(test_exe);

        for (test_filters) |filter| {
            run_rom_tests.addArg(filter);
        }

        test_step.dependOn(&rom_mod_test_artifacts.step);
        test_step.dependOn(&run_rom_tests.step);
    } else {
        const mod_tests = b.addTest(.{
            .name = "ness-mod-test",
            .root_module = mod,
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
