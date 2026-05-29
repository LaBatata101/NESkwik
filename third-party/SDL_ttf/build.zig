const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const preferred_link_mode = b.option(
        std.builtin.LinkMode,
        "preferred_link_mode",
        "Prefer building SDL as a statically or dynamically linked library (default: static)",
    ) orelse .static;

    var windows = false;
    var linux = false;
    var macos = false;
    switch (target.result.os.tag) {
        .windows => {
            windows = true;
        },
        .linux => {
            linux = true;
        },
        .macos => {
            macos = true;
        },
        else => {},
    }

    const sdl_ttf_mod = b.createModule(.{
        .target = target,
        .optimize = optimize,
    });
    sdl_ttf_mod.addCSourceFiles(.{
        .files = &.{
            "src/SDL_hashtable.c",
            "src/SDL_hashtable_ttf.c",
            "src/SDL_gpu_textengine.c",
            "src/SDL_renderer_textengine.c",
            "src/SDL_surface_textengine.c",
            "src/SDL_ttf.c",
        },
        .flags = &.{},
    });
    sdl_ttf_mod.addIncludePath(b.path("include/"));

    const sdl_ttf = b.addLibrary(.{
        .name = "SDL3_ttf",
        .linkage = preferred_link_mode,
        .root_module = sdl_ttf_mod,
    });
    sdl_ttf.linkLibC();
    sdl_ttf.installHeadersDirectory(b.path("include/SDL3_ttf"), "SDL3_ttf", .{});

    {
        // SDL Dependency
        const sdl_dep = b.dependency("sdl", .{
            .target = target,
            .optimize = optimize,
        });
        sdl_ttf_mod.linkLibrary(sdl_dep.artifact("SDL3"));
        // const sdl_test_lib = sdl_dep.artifact("SDL3_test");
    }

    {
        // Freetype
        const ft_dep = b.dependency("freetype", .{
            .target = target,
            .optimize = optimize,
            .use_system_zlib = false,
            .enable_brotli = true,
        });
        sdl_ttf_mod.linkLibrary(ft_dep.artifact("freetype"));
    }

    const install_sdl_ttf_lib = b.addInstallArtifact(sdl_ttf, .{});

    const install_sdl_ttf_step = b.step("install_sdl_ttf", "Install SDL_ttf");
    install_sdl_ttf_step.dependOn(&install_sdl_ttf_lib.step);

    b.getInstallStep().dependOn(&install_sdl_ttf_lib.step);
}
