// zig fmt: off
// Lookup table taken from "Length Counter" section at https://www.nesdev.org/apu_ref.txt
const LENGTH_TABLE = [32]u8{
    0x0A, 0xFE,
    0x14, 0x02,
    0x28, 0x04,
    0x50, 0x06,
    0xA0, 0x08,
    0x3C, 0x0A,
    0x0E, 0x0C,
    0x1A, 0x0E,
    0x0C, 0x10,
    0x18, 0x12,
    0x30, 0x14,
    0x60, 0x16,
    0xC0, 0x18,
    0x48, 0x1A,
    0x10, 0x1C,
    0x20, 0x1E,
};
// zig fmt: on

pub const LengthCounter = struct {
    halt_bit: usize,
    halted: bool,
    enabled: bool,
    remaining: u8,

    const Self = @This();

    pub fn init(halt_bit: usize) Self {
        return .{
            .halt_bit = halt_bit,
            .halted = false,
            .enabled = false,
            .remaining = 0,
        };
    }

    pub fn write_halt(self: *Self, value: u8) void {
        self.halted = (value >> self.halt_bit) & 0x01 != 0;
    }

    pub fn write_counter(self: *Self, value: u8) void {
        if (self.enabled) {
            self.remaining = LENGTH_TABLE[value >> 3];
        }
    }

    pub fn tick(self: *Self) void {
        if (!self.halted) {
            self.remaining -%= 1;
        }
    }

    pub fn audible(self: Self) void {
        return self.remaining > 0;
    }

    pub fn active(self: Self) u8 {
        return if (self.audible()) 1 else 0;
    }

    pub fn set_enable(self: *Self, enable: bool) void {
        self.enabled = enable;
        if (!enable) {
            self.remaining = 0;
        }
    }
};

pub const Envelope = struct {
    should_loop: bool,
    constant_volume: bool,
    n: u8,
    divider: u8,
    counter: u8,

    const Self = @This();

    pub fn init() Self {
        return .{
            .should_loop = false,
            .constant_volume = false,
            .n = 0,
            .divider = 0,
            .counter = 0,
        };
    }

    pub fn write(self: *Self, value: u8) void {
        self.should_loop = (value >> 5) & 0x01 != 0;
        self.constant_volume = (value >> 4) & 0x01 != 0;
        self.n = value & 0x0F;
        self.divider = self.n;
        self.counter = 15;
    }

    pub fn tick(self: *Self) void {
        if (self.divider == 0) {
            self.envelope_tick();
            self.divider = self.n;
        } else {
            self.divider -= 1;
        }
    }

    pub fn envelope_tick(self: *Self) void {
        if (self.should_loop and self.counter == 0) {
            self.counter = 15;
        } else {
            self.counter -%= 1;
        }
    }

    pub fn volume(self: Self) i16 {
        if (self.constant_volume) {
            return @as(i16, self.n);
        } else {
            return @as(i16, self.counter);
        }
    }
};

pub const TimerClock = enum {
    Clock,
    NoClock,
};

pub const Timer = struct {
    period: u16,
    divider: u16,
    remaining: u16,

    const Self = @This();

    pub fn init(divider: u32) Self {
        return .{
            .period = 0,
            .divider = divider,
            .remaining = 0,
        };
    }

    pub fn write_low(self: *Self, value: u8) void {
        self.period = (self.period & 0xFF00) | value;
    }

    pub fn write_high(self: *Self, value: u8) void {
        self.period = (self.period & 0x00FF) | (value & 0x07) << 8;
    }

    pub fn add_period_shifter(self: *Self, shift: i16) void {
        const new_period = @as(i16, @bitCast(self.period)) +% shift;
        self.period = @as(u16, @bitCast(new_period));
    }

    pub fn wavelen(self: Self) u32 {
        return (@as(u32, self.period) + 1) * self.divider;
    }

    pub fn run(self: *Self, current_cycle: *u32, to_cycle: u32) TimerClock {
        const end_wavelen = current_cycle.* + self.remaining;
        if (end_wavelen < to_cycle) {
            current_cycle.* += self.remaining;
            self.remaining = self.wavelen();
            return .Clock;
        } else {
            self.remaining -= to_cycle - current_cycle.*;
            current_cycle.* = to_cycle;
            return .NoClock;
        }
    }
};
