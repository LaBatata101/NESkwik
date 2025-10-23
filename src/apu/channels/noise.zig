const Timer = @import("../components.zig").Timer;
const Waveform = @import("../buffer.zig").Waveform;
const Envelope = @import("../components.zig").Envelope;
const LengthCounter = @import("../components.zig").LengthCounter;

// zig fmt: off
const PERIOD_TABLE: [16]u16  = .{
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
};
// zig fmt: on

const LinearFeedbackShiftRegister = struct {
    value: u16,
    mode: bool,

    const Self = @This();

    fn init() Self {
        return .{
            .value = 1,
            .mode = false,
        };
    }

    fn shift(self: *Self) bool {
        const bit0 = self.value & 0x01;
        const bit1 = self.other_bit();
        const new_bit = bit0 ^ bit1;

        self.value = (self.value >> 1) | (new_bit << 14);
        return self.value & 0x01 == 1;
    }

    fn other_bit(self: Self) u16 {
        if (self.mode) {
            return (self.value & (0x01 << 1)) >> 1;
        } else {
            return (self.value & (0x01 << 6)) >> 6;
        }
    }

    fn set_mode(self: *Self, mode: u8) void {
        self.mode = mode == 0;
    }
};

pub const Noise = struct {
    envelope: Envelope,
    length_counter: LengthCounter,
    timer: Timer,
    shifter: LinearFeedbackShiftRegister,
    waveform: Waveform,

    const Self = @This();

    pub fn init(waveform: Waveform) Self {
        return .{
            .envelope = Envelope.init(),
            .length_counter = LengthCounter.init(5),
            .timer = Timer.init(1),
            .waveform = waveform,
            .shifter = LinearFeedbackShiftRegister.init(),
        };
    }

    pub fn length_tick(self: *Self) void {
        self.length_counter.tick();
    }

    pub fn envelope_tick(self: *Self) void {
        self.envelope.tick();
    }

    pub fn play(self: *Self, from_cycle: u32, to_cycle: u32) void {
        if (!self.length_counter.audible()) {
            self.waveform.set_amplitude(0, from_cycle);
            return;
        }
        const volume = self.envelope.volume();

        var current_cycle = from_cycle;
        while (true) {
            switch (self.timer.run(&current_cycle, to_cycle)) {
                .Clock => {
                    const enabled = self.shifter.shift();
                    const amp = if (enabled) volume else 0;
                    self.waveform.set_amplitude(amp, current_cycle);
                },
                else => break,
            }
        }
    }

    pub fn write(self: *Self, addr: u16, value: u8) void {
        switch (addr % 4) {
            0 => {
                self.length_counter.write_halt(value);
                self.envelope.write(value);
            },
            2 => {
                const mode = (value & 0b1000_0000) >> 7;
                self.shifter.set_mode(mode);
                const period_index = value & 0b0000_1111;
                self.timer.period = PERIOD_TABLE[period_index];
            },
            3 => self.length_counter.write_counter(value),
            else => {},
        }
    }
};
