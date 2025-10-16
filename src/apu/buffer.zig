const std = @import("std");
const c = @import("../root.zig").c;

const NES_CLOCK_RATE: u64 = 1789773;
const NES_FPS: u32 = 60;
const FRAMES_PER_BUFFER: u32 = 1;

pub const Sample = i16;

pub const SampleBuffer = struct {
    blip: ?*c.blip_t,
    samples: std.ArrayList(Sample),
    transfer_samples: u32,

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator, out_rate: f64) !*Self {
        const self = try allocator.create(Self);
        errdefer allocator.destroy(self);

        const samples_per_frame = @as(u32, @intFromFloat(out_rate)) / NES_FPS;
        const transfer_samples = samples_per_frame * FRAMES_PER_BUFFER;

        const buf = c.blip_new(transfer_samples);
        c.blip_set_rates(buf, @as(f64, @floatFromInt(NES_CLOCK_RATE)), out_rate);

        self.* = .{
            .blip = buf,
            .samples = std.ArrayList(Sample).initCapacity(allocator, transfer_samples),
            .transfer_samples = transfer_samples,
        };

        return self;
    }

    pub fn read(self: *Self) []i16 {
        const samples_read = c.blip_read_samples(self.blip, self.samples.items, 0, 0);
        return self.samples.items[0..samples_read];
    }

    pub fn add_delta(self: *Self, clock_time: u32, delta: i32) void {
        c.blip_add_delta(self.blip, clock_time, delta);
    }

    pub fn end_frame(self: *Self, clock_duration: u32) void {
        c.blip_end_frame(self.blip, clock_duration);
    }

    pub fn clocks_needed(self: Self) u32 {
        return c.blip_clocks_needed(self.blip, self.transfer_samples);
    }
};

pub const Waveform = struct {
    buffer: *SampleBuffer,
    last_amp: Sample,
    volume_mult: i32,

    const Self = @This();

    pub fn init(buffer: *SampleBuffer, volume_mult: i32) Self {
        return .{
            .buffer = buffer,
            .last_amp = 0,
            .volume_mult = volume_mult,
        };
    }

    pub fn set_amplitude(self: *Self, amp: Sample, cycle: u32) void {
        const last_amp = self.last_amp;
        const delta: i32 = amp - last_amp;
        if (delta == 0) {
            return;
        }

        self.buffer.add_delta(cycle, delta);
        self.last_amp = amp;
    }
};
