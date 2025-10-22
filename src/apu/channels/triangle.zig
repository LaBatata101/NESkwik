const Timer = @import("../components.zig").Timer;
const LengthCounter = @import("../components.zig").LengthCounter;
const Waveform = @import("../buffer.zig").Waveform;

// zig fmt: off
const TRIANGLE_VOLUME: [32]i16 = .{
    0xF, 0xE, 0xD, 0xC, 0xB, 0xA, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0,
    0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xE, 0xF
};
// zig fmt: on

const LinearCounter = struct {
    control: bool,
    reload: bool,
    value: u8,
    counter: u8,

    const Self = @This();

    fn init() Self {
        return .{
            .control = false,
            .reload = false,
            .value = 0,
            .counter = 0,
        };
    }

    fn write(self: *Self, value: u8) void {
        self.value = value & 0b0111_1111;
        self.control = value & 0b1000_0000 != 0;
    }

    fn tick(self: *Self) void {
        if (self.reload) {
            self.counter = self.value;
        } else {
            self.counter -|= 1;
        }
        if (!self.control) {
            self.reload = false;
        }
    }

    fn audible(self: Self) bool {
        return self.counter > 0;
    }
};

pub const Triangle = struct {
    counter: LinearCounter,
    timer: Timer,
    length_counter: LengthCounter,
    waveform: Waveform,
    volume_index: usize,

    const Self = @This();

    pub fn init(waveform: Waveform) Self {
        return .{
            .counter = LinearCounter.init(),
            .timer = Timer.init(1),
            .length_counter = LengthCounter.init(7),
            .waveform = waveform,
            .volume_index = 0,
        };
    }

    pub fn length_tick(self: *Self) void {
        self.length_counter.tick();
    }

    pub fn envelope_tick(self: *Self) void {
        self.counter.tick();
    }

    pub fn play(self: *Self, from_cycle: u32, to_cycle: u32) void {
        if (!self.counter.audible() or !self.length_counter.audible()) {
            self.waveform.set_amplitude(0, from_cycle);
            return;
        }

        var current_cycle = from_cycle;
        while (true) {
            switch (self.timer.run(&current_cycle, to_cycle)) {
                .Clock => {
                    self.volume_index = (self.volume_index + 1) % 32;
                    const volume = TRIANGLE_VOLUME[self.volume_index];
                    self.waveform.set_amplitude(volume, current_cycle);
                },
                else => break,
            }
        }
    }

    pub fn write(self: *Self, addr: u16, value: u8) void {
        switch (addr % 4) {
            0 => {
                self.length_counter.write_halt(value);
                self.counter.write(value);
            },
            1 => {},
            2 => self.timer.write_low(value),
            3 => {
                self.length_counter.write_counter(value);
                self.timer.write_high(value);
                self.counter.reload = true;
            },
            else => {},
        }
    }
};
