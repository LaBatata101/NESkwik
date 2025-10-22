const std = @import("std");

const buffer = @import("buffer.zig");
const SDLAudioOut = @import("../sdl_audio.zig").SDLAudioOut;
const Pulse = @import("channels/pulse.zig").Pulse;
const Triangle = @import("channels/triangle.zig").Triangle;
const SampleBuffer = buffer.SampleBuffer;
const Waveform = buffer.Waveform;

const NTSC_TICK_LENGTH_TABLE: [2][6]u64 = [_][6]u64{
    .{ 7459, 7456, 7458, 7458, 7458, 0 },
    .{ 1, 7458, 7456, 7458, 7458, 7452 },
};

const VOLUME_MULT: i32 = ((@as(i16, 32767) / 16) / 3);

const Frame = packed struct(u8) {
    /// `false` = 4-step, `true` = 5-step
    mode: bool = false,
    suppress_irq: bool = false,
    _: u6 = 0,
};

const Jitter = union(enum) {
    Delay: struct { u64, u8 },
    None,
};

pub const APU = struct {
    alloc: std.mem.Allocator,

    pulse1: Pulse,
    pulse2: Pulse,
    triangle: Triangle,
    frame: Frame,

    pulse_buffer: *SampleBuffer,
    tnd_buffer: *SampleBuffer,

    device: *SDLAudioOut,

    global_cyc: u64,
    tick_cycle: u8,
    next_tick_cyc: u64,
    next_transfer_cyc: u64,
    last_frame_cyc: u64,

    irq_interrupt: bool,

    jitter: Jitter,

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator, device: *SDLAudioOut) !Self {
        const sample_rate = device.sampleRate();

        const pulse_buffer = try SampleBuffer.init(allocator, sample_rate);
        const tnd_buffer = try SampleBuffer.init(allocator, sample_rate);
        const clocks_needed = pulse_buffer.clocks_needed();

        return .{
            .alloc = allocator,

            .pulse1 = Pulse.init(false, Waveform.init(pulse_buffer, VOLUME_MULT)),
            .pulse2 = Pulse.init(true, Waveform.init(pulse_buffer, VOLUME_MULT)),
            .triangle = Triangle.init(Waveform.init(tnd_buffer, VOLUME_MULT)),
            .frame = .{},

            .pulse_buffer = pulse_buffer,
            .tnd_buffer = tnd_buffer,

            .device = device,

            .global_cyc = 0,
            .tick_cycle = 0,
            .next_tick_cyc = NTSC_TICK_LENGTH_TABLE[0][0],
            .next_transfer_cyc = clocks_needed,
            .last_frame_cyc = 0,

            .irq_interrupt = false,
            .jitter = Jitter.None,
        };
    }

    pub fn deinit(self: *Self) void {
        self.device.deinit(self.alloc);
        self.pulse_buffer.deinit(self.alloc);
        self.tnd_buffer.deinit(self.alloc);
    }

    pub fn run_to(self: *Self, cpu_cycle: u64) void {
        while (self.global_cyc < cpu_cycle) {
            const current_cycle = self.global_cyc;
            var next_step = @min(cpu_cycle, self.next_tick_cyc);
            next_step = @min(next_step, self.next_transfer_cyc);

            switch (self.jitter) {
                .Delay => |data| {
                    const time, _ = data;
                    next_step = @min(next_step, time);
                },
                else => {},
            }

            self.play(current_cycle, next_step);
            self.global_cyc = next_step;

            switch (self.jitter) {
                .Delay => |data| {
                    const time, const value = data;
                    if (self.global_cyc == time) {
                        self.set_4017(value);
                        self.jitter = Jitter.None;
                    }
                },
                else => {},
            }
            if (self.global_cyc == self.next_tick_cyc) {
                self.tick();
            }
            if (self.global_cyc == self.next_transfer_cyc) {
                self.transfer();
            }
        }
    }

    fn tick(self: *Self) void {
        self.tick_cycle += 1;
        self.next_tick_cyc = self.global_cyc + NTSC_TICK_LENGTH_TABLE[@intFromBool(self.frame.mode)][self.tick_cycle];
        if (self.frame.mode) {
            switch (self.tick_cycle) {
                1 => {
                    self.envelope_tick();
                    self.length_tick();
                },
                2 => self.envelope_tick(),
                3 => {
                    self.envelope_tick();
                    self.length_tick();
                },
                4 => self.envelope_tick(),
                5 => self.tick_cycle = 0, //4 is the actual last tick in the cycle.
                else => self.tick_cycle = 0,
            }
        } else {
            switch (self.tick_cycle) {
                1 => self.envelope_tick(),
                2 => {
                    self.envelope_tick();
                    self.length_tick();
                },
                3 => self.envelope_tick(),
                4 => {
                    self.tick_cycle = 0;
                    self.envelope_tick();
                    self.length_tick();
                    return self.raise_irq();
                },
                else => self.tick_cycle = 0,
            }
        }
    }

    fn envelope_tick(self: *Self) void {
        self.pulse1.envelope_tick();
        self.pulse2.envelope_tick();
        self.triangle.envelope_tick();
        // TODO: tick other channels here
    }

    fn length_tick(self: *Self) void {
        self.pulse1.length_tick();
        self.pulse2.length_tick();
        self.triangle.length_tick();
        // TODO: tick other channels here
    }

    fn raise_irq(self: *Self) void {
        if (!self.frame.suppress_irq) {
            self.irq_interrupt = true;
        }
    }

    fn play(self: *Self, from_cyc: u64, to_cyc: u64) void {
        const from: u32 = @intCast(from_cyc - self.last_frame_cyc);
        const to: u32 = @intCast(to_cyc - self.last_frame_cyc);
        self.pulse1.play(from, to);
        self.pulse2.play(from, to);
        self.triangle.play(from, to);
        // TODO play other channels...
    }

    fn transfer(self: *Self) void {
        const cpu_cyc = self.global_cyc;
        const cycles_since_last_frame: u32 = @intCast(cpu_cyc - self.last_frame_cyc);
        self.last_frame_cyc = cpu_cyc;

        const pulse_buffer = self.pulse_buffer;
        const tnd_buffer = self.tnd_buffer;
        pulse_buffer.end_frame(cycles_since_last_frame);
        tnd_buffer.end_frame(cycles_since_last_frame);

        const pulse_buffer_samples = pulse_buffer.read();
        const tnd_buffer_samples = tnd_buffer.read();

        std.debug.assert(pulse_buffer_samples.len == tnd_buffer_samples.len);

        var samples = std.ArrayList(buffer.Sample).initCapacity(
            self.alloc,
            // Can be either buffer length since they have the same size
            pulse_buffer_samples.len,
        ) catch @panic("Failed to initialize samples buffer\n");
        defer samples.deinit(self.alloc);

        for (pulse_buffer_samples, tnd_buffer_samples) |pulse_sample, tnd_sample| {
            samples.append(self.alloc, pulse_sample +| tnd_sample) catch
                @panic("Failed to append item to samples buffer\n");
        }

        self.next_transfer_cyc = cpu_cyc + pulse_buffer.clocks_needed();
        self.device.play(samples.items);
    }

    /// Returns the cycle number representing the next time the CPU should run
    /// the APU.
    /// Min of the next APU IRQ, the next DMC IRQ, and the next tick time. When
    /// the CPU cycle reaches this number, the CPU must run the APU.
    pub fn requested_run_cycle(self: Self) u64 {
        return self.next_tick_cyc;
    }

    fn set_4017(self: *Self, value: u8) void {
        self.frame = @bitCast(value);
        if (self.frame.suppress_irq) {
            self.irq_interrupt = false;
        }

        self.tick_cycle = 0;
        self.next_tick_cyc = self.global_cyc + NTSC_TICK_LENGTH_TABLE[@intFromBool(self.frame.mode)][0];
    }

    pub fn read_status(self: *Self, cycle: u64) u8 {
        self.run_to(cycle - 1);
        var status: u8 = 0;
        status |= self.pulse1.length_counter.active();
        status |= self.pulse2.length_counter.active() << 1;
        status |= self.triangle.length_counter.active() << 2;
        // TODO: other channels here...
        status |= if (self.irq_interrupt) @as(u8, 1 << 6) else 0;
        self.irq_interrupt = false;

        self.run_to(cycle);
        return status;
    }

    pub fn write(self: *Self, addr: u16, value: u8) void {
        switch (addr) {
            0x4000, 0x4001, 0x4002, 0x4003 => self.pulse1.write(addr, value),
            0x4004, 0x4005, 0x4006, 0x4007 => self.pulse2.write(addr, value),
            0x4008, 0x4009, 0x400A, 0x400B => self.triangle.write(addr, value),
            0x400C, 0x400D, 0x400E, 0x400F => {
                // TODO: noise channel
            },
            0x4010, 0x4011, 0x4012, 0x4023 => {
                // TODO: DMC
            },
            0x4015 => {
                self.triangle.length_counter.set_enable(value & 0b0000_0100 != 0);
                self.pulse1.length_counter.set_enable(value & 0b0000_0001 != 0);
                self.pulse2.length_counter.set_enable(value & 0b0000_0010 != 0);
            },
            0x4017 => {
                if (self.global_cyc % 2 == 0) {
                    self.set_4017(value);
                } else {
                    self.jitter = .{ .Delay = .{ self.global_cyc + 1, value } };
                }
            },
            else => {
                std.log.warn("Attempt to write to addr 0x{X:04}", .{addr});
            },
        }
    }
};
