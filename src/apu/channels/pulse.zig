const components = @import("../components.zig");
const Timer = components.Timer;
const Envelope = components.Envelope;
const LengthCounter = components.LengthCounter;
const Waveform = @import("../buffer.zig").Waveform;

const PULSE_DUTY_CYCLES: [4][8]i16 = [_][8]i16{
    .{ 0, 1, -1, 0, 0, 0, 0, 0 },
    .{ 0, 1, 0, -1, 0, 0, 0, 0 },
    .{ 0, 1, 0, 0, 0, -1, 0, 0 },
    .{ 0, -1, 0, 1, 0, 0, 0, 0 },
};

const Sweep = struct {
    enable: bool,
    period: u8,
    negate: bool,
    shift: u3,
    is_square2: bool,
    divider: u8,
    reload: bool,

    const Self = @This();

    fn init(is_square2: bool) Self {
        return .{
            .enable = false,
            .period = 0,
            .negate = false,
            .shift = 0,
            .is_square2 = is_square2,
            .divider = 0,
            .reload = false,
        };
    }

    fn write(self: *Self, value: u8) void {
        self.enable = (value & 0b1000_0000) != 0;
        self.period = (value & 0b0111_0000) >> 4;
        self.negate = (value & 0b0000_1000) != 0;
        self.shift = @truncate(value & 0b0000_0111);
        self.reload = true;
    }

    fn audible(self: Self) bool {
        _ = self;
        return true;
    }

    fn period_shift(self: Self, timer: *const Timer) i16 {
        var shift: i16 = @bitCast(timer.period);
        shift >>= self.shift;

        if (self.negate) {
            shift = -shift;
            if (self.is_square2) {
                shift += 1;
            }
        }

        return shift;
    }

    fn tick(self: *Self, timer: *Timer) void {
        if (!self.enable) {
            return;
        }

        self.divider -|= 1;
        if (self.divider == 0) {
            self.divider = self.period;
            timer.add_period_shifter(self.period_shift(timer));
        }

        if (self.reload) {
            self.divider = self.period;
            self.reload = false;
        }
    }
};

pub const Pulse = struct {
    duty: usize,
    duty_index: usize,
    envelope: Envelope,
    sweep: Sweep,
    timer: Timer,
    length_counter: LengthCounter,
    waveform: Waveform,

    const Self = @This();

    pub fn init(is_square2: bool, waveform: Waveform) Self {
        return .{
            .duty = 0,
            .duty_index = 0,
            .envelope = Envelope.init(),
            .sweep = Sweep.init(is_square2),
            .timer = Timer.init(2),
            .length_counter = LengthCounter.init(5),
            .waveform = waveform,
        };
    }

    pub fn length_tick(self: *Self) void {
        self.length_counter.tick();
        self.sweep.tick(&self.timer);
    }

    pub fn envelope_tick(self: *Self) void {
        self.envelope.tick();
    }

    pub fn play(self: *Self, from_cyc: u32, to_cyc: u32) void {
        if (!self.sweep.audible() or !self.length_counter.audible()) {
            self.waveform.set_amplitude(0, from_cyc);
            return;
        }

        const volume = self.envelope.volume();
        var current_cyc = from_cyc;
        while (true) {
            switch (self.timer.run(&current_cyc, to_cyc)) {
                .NoClock => break,
                .Clock => {
                    self.duty_index = (self.duty_index + 1) % 8;
                    switch (PULSE_DUTY_CYCLES[self.duty][self.duty_index]) {
                        -1 => self.waveform.set_amplitude(0, current_cyc),
                        0 => {},
                        1 => self.waveform.set_amplitude(volume, current_cyc),
                        else => {},
                    }
                },
            }
        }
    }

    pub fn write(self: *Self, idx: u16, value: u8) void {
        switch (idx % 4) {
            0 => {
                self.duty = value >> 6;
                self.length_counter.write_halt(value);
                self.envelope.write(value);
            },
            1 => self.sweep.write(value),
            2 => self.timer.write_low(value),
            3 => {
                self.length_counter.write_counter(value);
                self.timer.write_high(value);
            },
            else => {},
        }
    }
};
