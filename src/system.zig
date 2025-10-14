const CPU = @import("cpu.zig").CPU;
const Bus = @import("bus.zig").Bus;

pub const System = struct {
    cpu: CPU,
    bus: *Bus,
    system_clock_counter: usize,

    const Self = @This();

    pub fn init(bus: *Bus, cpu: CPU) Self {
        return .{
            .bus = bus,
            .cpu = cpu,
            .system_clock_counter = 0,
        };
    }

    pub fn tick(self: *Self) void {
        self.bus.ppu.tick();

        // The CPU runs 3 times slower than the PPU, so only execute the CPU every 3 times.
        if (self.system_clock_counter % 3 == 0) {
            const dma_handled = self.bus.handle_dma_transfer(self.system_clock_counter);
            if (!dma_handled) {
                // the DMA isn't transferring data, the CPU is allowed to execute
                self.cpu.tick();
            }
        }

        if (self.bus.ppu.nmi_interrupt) {
            self.bus.ppu.nmi_interrupt = false;
            self.cpu.interrupt(CPU.NMI);
        }

        self.system_clock_counter += 1;
    }

    pub fn reset(self: *Self) void {
        self.cpu.reset();
        self.system_clock_counter = 0;
    }

    pub fn is_frame_complete(self: *Self) bool {
        if (self.bus.ppu.frame_complete) {
            self.bus.ppu.frame_complete = false;
            return true;
        }
        return false;
    }
};
