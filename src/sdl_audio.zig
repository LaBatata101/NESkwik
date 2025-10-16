const std = @import("std");

const c = @import("root.zig").c;
const Sample = @import("apu/buffer.zig").Sample;

const OUT_SAMPLE_RATE: i32 = 44100;
const BUFFER_SIZE: usize = @divExact(@as(usize, @intCast(OUT_SAMPLE_RATE)), 15);

const BufferOut = struct {
    samples: [BUFFER_SIZE]Sample,
    input_counter: usize,
    playback_counter: usize,
    input_samples: usize,
    too_slow: bool,
};

pub const SDLAudioOut = struct {
    buffer: BufferOut,
    mutex: std.Thread.Mutex,
    cond: std.Thread.Condition,
    stream: *c.SDL_AudioStream,

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator) !*Self {
        const self = try allocator.create(Self);
        errdefer allocator.destroy(self);

        self.* = .{
            .buffer = .{
                .samples = [_]Sample{0} ** BUFFER_SIZE,
                .input_counter = 0,
                .playback_counter = 0,
                .input_samples = 0,
                .too_slow = false,
            },
            .mutex = .{},
            .cond = .{},
            .stream = undefined,
        };

        var desired: c.SDL_AudioSpec = .{};
        desired.freq = OUT_SAMPLE_RATE;
        desired.format = c.SDL_AUDIO_S16LE;
        desired.channels = 1;

        self.stream = c.SDL_OpenAudioDeviceStream(c.SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &desired, audio_stream_callback, self) orelse return error.SDLInitFailed;
        errdefer _ = c.SDL_DestroyAudioStream(self.stream);

        _ = c.SDL_ResumeAudioStreamDevice(self.stream);

        return self;
    }

    pub fn deinit(self: *Self, alloc: std.mem.Allocator) void {
        _ = c.SDL_DestroyAudioStream(self.stream);
        alloc.destroy(self);
    }

    pub fn play(self: *Self, buffer: []const Sample) void {
        self.wait(buffer.len);
        self.mutex.lock();
        defer self.mutex.unlock();

        if (self.buffer.too_slow) {
            std.log.warn("Audio transfer can't keep up\n", .{});
            self.buffer.too_slow = false;
        }

        var in_index: usize = 0;
        var out_index = self.buffer.input_counter;
        const out_len = BUFFER_SIZE;
        const in_len = buffer.len;

        while (in_index < in_len) {
            self.buffer.samples[out_index] = buffer[in_index];
            in_index += 1;
            out_index += 1;
            if (out_index == out_len) {
                out_index = 0;
            }
        }
        self.buffer.input_counter = (self.buffer.input_counter + in_len) % out_len;
        self.buffer.input_samples += in_len;
    }

    pub fn sampleRate(self: *const Self) f64 {
        _ = self;
        return @as(f64, @floatFromInt(OUT_SAMPLE_RATE));
    }

    pub fn wait(self: *Self, in_size: usize) void {
        self.mutex.lock();
        defer self.mutex.unlock();

        while (self.buffer.input_samples + in_size > BUFFER_SIZE) {
            self.cond.wait(&self.mutex);
        }
    }
};

fn audio_stream_callback(
    userdata: ?*anyopaque,
    stream_arg: ?*c.SDL_AudioStream,
    additional_amount: c_int,
    total_amount: c_int,
) callconv(.c) void {
    _ = additional_amount;
    const stream = @as(*c.SDL_AudioStream, @ptrCast(stream_arg.?));
    const this = @as(*SDLAudioOut, @ptrCast(@alignCast(userdata.?)));

    this.mutex.lock();
    defer this.mutex.unlock();

    const sample_size: usize = @sizeOf(Sample);
    const total_bytes = total_amount;
    const max_bytes = this.buffer.input_samples * sample_size;
    const transferred_bytes_usize: usize = @intCast(@min(max_bytes, total_bytes));

    const transferred_bytes: c_int = @as(c_int, @intCast(transferred_bytes_usize));
    const transferred_samples: usize = @divExact(transferred_bytes_usize, sample_size);

    if (transferred_bytes < total_bytes) {
        this.buffer.too_slow = true;
    }

    if (transferred_samples > 0) {
        const first_samples = BUFFER_SIZE - this.buffer.playback_counter;
        if (transferred_samples <= first_samples) {
            const src_ptr = @as([*]const u8, @ptrCast(&this.buffer.samples[this.buffer.playback_counter]));
            _ = c.SDL_PutAudioStreamData(stream, src_ptr, transferred_bytes);
        } else {
            // First part
            const first_bytes: c_int = @as(c_int, @intCast(first_samples * sample_size));
            const src1_ptr = @as([*]const u8, @ptrCast(&this.buffer.samples[this.buffer.playback_counter]));
            _ = c.SDL_PutAudioStreamData(stream, src1_ptr, first_bytes);

            // Second part
            const second_samples = transferred_samples - first_samples;
            const second_bytes: c_int = @as(c_int, @intCast(second_samples * sample_size));
            const src2_ptr = @as([*]const u8, @ptrCast(&this.buffer.samples[0]));
            _ = c.SDL_PutAudioStreamData(stream, src2_ptr, second_bytes);
        }

        this.buffer.input_samples -= transferred_samples;
        this.buffer.playback_counter = (this.buffer.playback_counter + transferred_samples) % BUFFER_SIZE;
    }

    if (this.buffer.too_slow) {
        this.buffer.input_counter = this.buffer.playback_counter;
    }

    this.cond.signal();
}
