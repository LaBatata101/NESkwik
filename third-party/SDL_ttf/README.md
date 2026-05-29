# SDL_ttf but with the zig build system

This is a port of [SDL_ttf](https://github.com/libsdl-org/SDL_ttf) to the zig build system, to use it with the zig package manager.

This is ***not*** a wrapper.

## Usage

Requires zig version `0.14.0`, higher versions not tested.

Fetch it with:
```bash
zig fetch --save git+https://github.com/boubl/SDL_ttf.git
```

And link it in your `build.zig`:
```zig
{
    // SDL_ttf Dependency
    const sdl_ttf_dep = b.dependency("sdl_ttf", .{
        .target = target,
        .optimize = optimize,
    });
    const sdl_ttf_lib = sdl_ttf_dep.artifact("SDL_ttf");

    exe_mod.linkLibrary(sdl_ttf_lib);
}
```

> [!NOTE]
> This was only tested on macOS with 0.14.0 for my personal use. PR are welcome if this does not work on other platforms.

