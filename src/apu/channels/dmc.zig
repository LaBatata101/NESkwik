const std = @import("std");
const Timer = @import("../components.zig").Timer;
const Waveform = @import("../buffer.zig").Waveform;
const Rom = @import("../../rom.zig").Rom;

// zig fmt: off
const RATE_TABLE: [16]u16  = .{
    428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54
};
// zig fmt: on

pub const DMC = struct {
    enabled: bool,
    volume: u8,

    // Memory Reader state
    sample_addr: u16,
    sample_length: u16,
    current_addr: u16,
    bytes_remaining: u16,
    sample_buffer: ?u8,

    // Output Unit state
    shift_register: u8,
    bits_remaining: u8,
    silence: bool,

    // Registers
    irq_enabled: bool,
    loop_flag: bool,
    irq_pending: bool,

    timer: Timer,
    waveform: Waveform,

    // Used to read PRG-ROM
    rom: *Rom,

    const Self = @This();

    pub fn init(waveform: Waveform, rom: *Rom) Self {
        return .{
            .enabled = false,
            .volume = 0,
            .sample_addr = 0xC000,
            .sample_length = 1,
            .current_addr = 0xC000,
            .bytes_remaining = 0,
            .sample_buffer = null,
            .shift_register = 0,
            .bits_remaining = 8,
            .silence = true,
            .irq_enabled = false,
            .loop_flag = false,
            .irq_pending = false,
            .timer = Timer.init(1),
            .waveform = waveform,
            .rom = rom,
        };
    }

    pub fn set_enabled(self: *Self, enabled: bool) void {
        self.irq_pending = false;

        if (enabled) {
            if (self.bytes_remaining == 0) {
                self.restart();
            }
        } else {
            self.bytes_remaining = 0;
        }
        self.enabled = enabled;
    }

    fn restart(self: *Self) void {
        self.current_addr = self.sample_addr;
        self.bytes_remaining = self.sample_length;
        self.fetch_sample();
    }

    fn fetch_sample(self: *Self) void {
        if (self.sample_buffer == null and self.bytes_remaining > 0) {
            self.sample_buffer = self.rom.prg_rom_read(self.current_addr);

            self.current_addr +|= 1;
            if (self.current_addr == 0) {
                self.current_addr = 0x8000;
            }

            self.bytes_remaining -= 1;
            if (self.bytes_remaining == 0) {
                if (self.loop_flag) {
                    self.restart();
                } else if (self.irq_enabled) {
                    self.irq_pending = true;
                }
            }
        }
    }

    pub fn play(self: *Self, from_cycle: u32, to_cycle: u32) void {
        var current_cycle = from_cycle;

        self.fetch_sample();

        while (true) {
            switch (self.timer.run(&current_cycle, to_cycle)) {
                .Clock => {
                    if (!self.silence) {
                        const bit = self.shift_register & 0x01;
                        if (bit != 0 and self.volume <= 125) {
                            self.volume += 2;
                        } else if (bit == 0 and self.volume >= 2) {
                            self.volume -= 2;
                        }
                    }

                    self.shift_register >>= 1;
                    self.bits_remaining -|= 1;

                    if (self.bits_remaining == 0) {
                        self.bits_remaining = 8;
                        if (self.sample_buffer) |byte| {
                            self.silence = false;
                            self.shift_register = byte;
                            self.sample_buffer = null;
                            self.fetch_sample();
                        } else {
                            self.silence = true;
                        }
                    }

                    self.waveform.set_amplitude(self.volume, current_cycle);
                },
                else => break,
            }
        }
    }

    pub fn write(self: *Self, addr: u16, value: u8) void {
        switch (addr) {
            0x4010 => {
                self.irq_enabled = value & 0b1000_0000 != 0;
                self.loop_flag = value & 0b0100_0000 != 0;
                self.timer.period = RATE_TABLE[value & 0x0F];

                if (!self.irq_enabled) {
                    self.irq_pending = false;
                }
            },
            0x4011 => self.volume = value & 0x7F,
            0x4012 => self.sample_addr = 0xC000 | (@as(u16, value) << 6),
            0x4013 => self.sample_length = (@as(u16, value) << 4) + 1,
            else => {},
        }
    }
};
