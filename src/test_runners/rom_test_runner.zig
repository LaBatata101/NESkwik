const std = @import("std");
const ness = @import("ness");

const c = ness.c;
const Rom = ness.Rom;
const System = ness.System;

const ADDR_STATUS = 0x6000;
const ADDR_MAGIC_START = 0x6001; // $DE $B0 $61
const ADDR_TEXT_START = 0x6004;

const MAGIC_BYTES = [_]u8{ 0xDE, 0xB0, 0x61 };

const STATUS_RUNNING = 0x80;
const STATUS_NEEDS_RESET = 0x81;
const STATUS_PASSED = 0x00;

const CPU_ONE_SEC_CYCLES: u64 = 1_790_000;

const TESTS_DIR = "test_roms";

pub fn suppressLog(
    comptime message_level: std.log.Level,
    comptime scope: @Type(.enum_literal),
    comptime format: []const u8,
    args: anytype,
) void {
    _ = message_level;
    _ = scope;
    _ = format;
    _ = args;
}

pub const std_options: std.Options = .{
    .log_level = .err,
    .logFn = suppressLog,
};

const TestState = struct {
    mutex: std.Thread.Mutex = .{},
    passed_count: usize = 0,
    total_count: usize = 0,
    failed_tests: std.ArrayList(FailedTest),
    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator) TestState {
        return .{
            .failed_tests = .empty,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *TestState) void {
        // Clean up the failed test data
        for (self.failed_tests.items) |failed| {
            self.allocator.free(failed.output);
            // The name is the path, which was also allocated
            self.allocator.free(failed.name);
        }
        self.failed_tests.deinit(self.allocator);
    }
};

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    var filter: ?[]const u8 = null;
    if (args.len > 1) {
        filter = args[1];
    }

    _ = c.SDL_SetHint(c.SDL_HINT_NO_SIGNAL_HANDLERS, "1");

    if (!c.SDL_Init(c.SDL_INIT_AUDIO)) {
        std.debug.print("Failed to initialize SDL for headless tests!\n", .{});
        return error.SDLInitFailed;
    }
    defer c.SDL_Quit();

    const cwd = std.fs.cwd();
    var dir = cwd.openDir(TESTS_DIR, .{ .iterate = true }) catch |err| {
        std.debug.print("Error: Couldn't find '{s}'.\nError: {}\n", .{ TESTS_DIR, err });
        return;
    };
    defer dir.close();

    // Initialize Thread Pool
    var pool: std.Thread.Pool = undefined;
    // .allocator is required for the pool's internal structures
    try pool.init(.{ .allocator = allocator });

    // Initialize Shared Test State
    var state = TestState.init(allocator);
    defer state.deinit();

    std.debug.print("\n=== RUNNING TEST ROMS ===\n\n", .{});

    var iterator = dir.iterate();
    while (try iterator.next()) |entry| {
        if (entry.kind != .file) continue;
        if (!std.mem.endsWith(u8, entry.name, ".nes")) continue;

        if (filter) |f| {
            if (std.mem.indexOf(u8, entry.name, f) == null) continue;
        }

        // We must duplicate the path because the thread will run after the
        // iterator moves to the next entry or closes.
        const rom_path = try std.fs.path.join(allocator, &.{ TESTS_DIR, entry.name });

        // Spawn the job
        try pool.spawn(run_test_wrapper, .{ &state, rom_path });
    }

    var timer = std.time.Timer.start() catch @panic("failed to start timer");
    pool.deinit(); // This will wait for all threads to finish
    const exec_time = timer.lap();

    if (state.failed_tests.items.len > 0) {
        std.debug.print("\nFailed Tests:\n", .{});
        for (state.failed_tests.items) |failed_test| {
            std.debug.print("\n{s}", .{failed_test.name});
            if (failed_test.output.len > 0) {
                std.debug.print(" - output: {s}\n{s}", .{ failed_test.output, "=" ** 15 });
            }
            std.debug.print("\n", .{});
        }
        std.debug.print("Failed: {}\n", .{state.failed_tests.items.len});
    }

    std.debug.print("\nPassed: {}/{}\n", .{ state.passed_count, state.total_count });
    std.debug.print("Total time: {d:.2}ms\n", .{
        exec_time / std.time.ns_per_ms,
    });

    if (state.failed_tests.items.len > 0) {
        std.process.exit(1);
    }
}

// Wrapper function executed by the thread pool
fn run_test_wrapper(state: *TestState, rom_path: []const u8) void {
    // Ensure we free the path we allocated in the main loop,
    // UNLESS it's saved into failed_tests (handled below)
    var should_free_path = true;
    defer if (should_free_path) state.allocator.free(rom_path);

    var timer = std.time.Timer.start() catch @panic("failed to start timer");
    const result = run_test_rom(state.allocator, rom_path) catch |err| {
        // Handle unexpected runtime errors (file not found, etc)
        state.mutex.lock();
        defer state.mutex.unlock();
        std.debug.print("{s}... \x1b[31mERROR ({})\x1b[0m\n", .{ std.fs.path.stem(rom_path), err });
        return;
    };
    const exec_time = timer.lap();

    // Lock mutex to safely print to stdout and update stats
    state.mutex.lock();
    defer state.mutex.unlock();

    state.total_count += 1;

    const name_stem = std.fs.path.stem(rom_path);

    switch (result) {
        .passed => {
            state.passed_count += 1;
            std.debug.print("{s: <30} \x1b[32mOK\x1b[0m \ttook {d:.2}ms\n", .{
                name_stem,
                exec_time / std.time.ns_per_ms,
            });
        },
        .failed => |fail_data| {
            std.debug.print("{s: <30} \x1b[31mFAILED\x1b[0m\n", .{name_stem});

            // We transfer ownership of the allocated rom_path string to the struct
            // so it can be printed in the summary later.
            should_free_path = false;

            // We need to construct a FailedTest that owns its data
            const stored_failure = FailedTest{
                .code = fail_data.code,
                .name = rom_path, // Store full path/buffer ownership
                .output = fail_data.output,
            };

            state.failed_tests.append(state.allocator, stored_failure) catch {
                std.debug.print("CRITICAL: Failed to append test failure to list.\n", .{});
            };
        },
    }
}

const FailedTest = struct {
    code: u8,
    name: []const u8,
    output: []const u8,
};

const TestResult = union(enum) {
    passed,
    failed: FailedTest,
};

fn run_test_rom(allocator: std.mem.Allocator, path: []const u8) !TestResult {
    const file = try std.fs.cwd().openFile(path, .{});
    defer file.close();

    const file_size = try file.getEndPos();
    try file.seekTo(0);
    const buffer = try allocator.alloc(u8, file_size);
    defer allocator.free(buffer);
    _ = try file.read(buffer);

    var rom = try Rom.init(allocator, path, buffer);
    defer rom.deinit();

    var system = try System.init(allocator, &rom, .{ .disable_audio = true });
    defer system.deinit();
    system.reset();

    var test_started = false;
    var timestamp: u64 = 0;
    var status = system.bus.mem_read(ADDR_STATUS);
    while (true) {
        while (system.bus.cycles <= (CPU_ONE_SEC_CYCLES / 2) + timestamp) {
            system.run_frame();
        }
        timestamp = system.bus.cycles;

        if (!test_started) {
            const m1 = system.bus.mem_read(ADDR_MAGIC_START);
            const m2 = system.bus.mem_read(ADDR_MAGIC_START + 1);
            const m3 = system.bus.mem_read(ADDR_MAGIC_START + 2);

            test_started = m1 == MAGIC_BYTES[0] and m2 == MAGIC_BYTES[1] and m3 == MAGIC_BYTES[2];
        }

        if (test_started and status == STATUS_NEEDS_RESET) {
            system.reset();
            continue;
        }

        status = system.bus.mem_read(ADDR_STATUS);
        if (status != STATUS_RUNNING) {
            break;
        }
    }

    if (status != STATUS_PASSED) {
        var output_buffer = std.ArrayList(u8).empty;
        var char_idx: u16 = 0;
        while (true) : (char_idx += 1) {
            const char = system.bus.mem_read(ADDR_TEXT_START + char_idx);
            if (char == 0) break;

            if (char_idx >= output_buffer.items.len) {
                try output_buffer.append(allocator, char);
            }
        }
        return .{
            .failed = .{
                .code = status,
                .name = std.fs.path.stem(path),
                .output = try output_buffer.toOwnedSlice(allocator),
            },
        };
    } else {
        return .passed;
    }
}
