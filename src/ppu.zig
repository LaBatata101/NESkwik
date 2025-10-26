const std = @import("std");

const Mirroring = @import("rom.zig").Mirroring;
const render = @import("render.zig");
const Rom = @import("rom.zig").Rom;
const Frame = render.Frame;

const CYCLES_PER_SCANLINE: u64 = 341;
const SCANLINES_PER_FRAME: u64 = 262;
const CYCLES_PER_FRAME: u64 = CYCLES_PER_SCANLINE * SCANLINES_PER_FRAME;

const ControlRegister = packed struct(u8) {
    /// Base nametable address (0 = 0x2000; 1 = 0x2400; 2 = 0x2800; 3 = 0x2C00)
    name_table_addr: u2 = 0,

    /// VRAM address increment per CPU read/write of `PPUDATA` (`false`: add 1, going across; `true`: add 32, going down)
    vram_add_increment: bool = false,

    /// Sprite pattern table address for 8x8 sprites (`false`: 0x0000; `true`: 0x1000; ignored in 8x16 mode)
    sprite_pattern_addr: bool = false,

    /// Background pattern table address (`false`: 0x0000; `true`: 0x1000)
    background_pattern_addr: bool = false,

    /// Sprite size (`false`: 8x8 pixels; `true`: 8x16 pixels)
    sprite_size: bool = false,

    /// PPU master/slave select (`false`: read backdrop from EXT pins; `true`: output color on EXT pins)
    master_slave_select: bool = false,

    /// Generate an NMI at the start of the vertical blanking interval (`false`: off; `true`: on)
    generate_nmi: bool = false,

    fn vram_addr_increment(self: @This()) u8 {
        return if (self.vram_add_increment) 32 else 1;
    }

    pub fn bg_pattern_addr(self: @This()) u16 {
        return if (self.background_pattern_addr) 0x1000 else 0;
    }

    pub fn sprt_pattern_addr(self: @This()) u16 {
        return if (self.sprite_pattern_addr) 0x1000 else 0;
    }

    pub fn nametable_addr(self: @This()) u16 {
        return switch (self.name_table_addr) {
            0 => 0x2000,
            1 => 0x2400,
            2 => 0x2800,
            3 => 0x2C00,
        };
    }
};

const MaskRegister = packed struct(u8) {
    /// (`false`: normal color, `true`: greyscale)
    greyscale: bool = false,

    /// `true`: Show background in leftmost 8 pixels of screen, `false`: Hide
    render_background_left: bool = false,

    /// `true`: Show sprites in leftmost 8 pixels of screen, `false`: Hide
    render_sprite_left: bool = false,

    /// `true`: Enable background rendering
    render_background: bool = false,

    /// `true`: Enable sprite rendering
    render_sprite: bool = false,

    /// (green on PAL/Dendy)
    emphasize_red: bool = false,

    /// (red on PAL/Dendy)
    emphasize_green: bool = false,
    emphasize_blue: bool = false,
};

const StatusRegister = packed struct(u8) {
    open_bus: u5 = 0,
    sprite_overflow: bool = false,
    sprite_zero_hit: bool = false,
    /// Vblank flag, cleared on read.
    vblank: bool = false,
};

const ScrollRegister = struct {
    fine_x: u8 = 0,
    fine_y: u8 = 0,
};

const AddrRegister = packed struct(u16) {
    coarse_x: u5 = 0,
    coarse_y: u5 = 0,
    nametable_x: u1 = 0,
    nametable_y: u1 = 0,
    fine_y: u3 = 0,
    unused: u1 = 0,

    fn addr(self: @This()) u16 {
        return @bitCast(self);
    }
};

const Nametable = struct {
    data: [960]u8,
    /// Controls which palette is assigned to each part of the background.
    attribute_table: [64]u8,

    fn get_palette_idx(self: @This(), tile_x: u8, tile_y: u8) u8 {
        const block_x = tile_x / 4;
        const block_y = tile_y / 4;
        const attribute_idx = block_y * 8 + block_x;
        const attribute_byte = self.attribute_table[attribute_idx];

        // Determine which quadrant within the block tile belongs to
        const sub_x = tile_x % 4 / 2;
        const sub_y = tile_y % 4 / 2;

        var palette_idx: u8 = undefined;
        if (sub_x == 0 and sub_y == 0) { // upper-left
            palette_idx = attribute_byte & 0b11;
        } else if (sub_x == 1 and sub_y == 0) { // upper-right
            palette_idx = (attribute_byte >> 2) & 0b11;
        } else if (sub_x == 0 and sub_y == 1) { // lower-left
            palette_idx = (attribute_byte >> 4) & 0b11;
        } else if (sub_x == 1 and sub_y == 1) { // lower-right
            palette_idx = (attribute_byte >> 6) & 0b11;
        } else {
            unreachable;
        }

        return palette_idx;
    }
};

/// The PPU reserves 32 bytes in its VRAM address space (range $3F00 to $3F1F) for palette memory.
const Palette = struct {
    /// Background palettes: $3F00–$3F0F
    background: [16]u8,
    /// Sprite palettes: $3F10–$3F1F
    sprite: [16]u8,

    fn init(palette_data: [32]u8) @This() {
        return .{
            .background = palette_data[0..16],
            .sprite = palette_data[16..],
        };
    }

    fn get_background(self: @This(), palette_idx: u8) struct { u8, u8, u8, u8 } {
        const base = palette_idx * 4 + 1;
        return .{
            // The first entry of background palette 0 (the byte at $3F00) is the universal background color / backdrop.
            // It is used as the “background behind everything else” color, when both background + sprites are
            // transparent or disabled.
            self.background[0],
            self.background[base],
            self.background[base + 1],
            self.background[base + 2],
        };
    }

    fn get_foreground(self: @This(), palette_idx: u8) struct { u8, u8, u8, u8 } {
        const base = palette_idx * 4;
        return .{
            0,
            self.sprite[base],
            self.sprite[base + 1],
            self.sprite[base + 2],
        };
    }
};

const Tile = struct {
    low_bitplane: [8]u8,
    high_bitplane: [8]u8,
    pixel_values: [8]u2,
};

// - The PPU has 0x0000–0x1FFF in its address space reserved for the pattern tables (aka CHR memory). That’s 8 KB total.
// - Those 8 KB are typically split into two 4 KB pattern tables (often called “pattern table 0” and “pattern table 1”).
// Whether a tile is drawn from table 0 or table 1 is controlled by PPU control bits (for background vs sprites).
// - Each pattern table holds 256 tiles (because each tile uses 16 bytes, and 256×16=4096).
const PatternTable = struct {
    // Each tile in the pattern table is 16 bytes; 256 is the total number of tile in a 4KB pattern table.
    tiles: [256]Tile,

    fn init(data: []u8) @This() {
        std.debug.assert(data.len == 4096);

        var tiles: [256]Tile = undefined;
        for (0..256) |i| {
            // We multiply by 16 to skip the last two biplanes.
            var tile = Tile{
                .low_bitplane = data[i * 16 .. i * 16 + 8],
                .high_bitplane = data[i * 16 + 8 .. i * 16 + 16],
                .pixel_values = undefined,
            };
            for (0..8) |y| {
                tile.pixel_values[y] = tile.high_bitplane << 1 | tile.low_bitplane;
            }
            tiles[i] = tile;
        }

        return .{ .tiles = tiles };
    }
};

const BgData = struct {
    next_tile_id: u8 = 0,
    next_tile_attr: u8 = 0,
    next_tile_lsb: u8 = 0,
    next_tile_msb: u8 = 0,
    shifter_pattern_lo: u16 = 0,
    shifter_pattern_hi: u16 = 0,
    shifter_attr_lo: u16 = 0,
    shifter_attr_hi: u16 = 0,
};

const OamEntry = struct {
    /// Y position of sprite
    y: u8 = 0,
    /// ID of tile from pattern memory
    id: u8 = 0,
    /// Flags define how sprite should be rendered
    attr: u8 = 0,
    /// X position of sprite
    x: u8 = 0,

    fn init(data: []const u8) @This() {
        return .{
            .y = data[0],
            .id = data[1],
            .attr = data[2],
            .x = data[3],
        };
    }
};

const FgData = struct {
    sprite_scanline: [8]OamEntry = [_]OamEntry{.{}} ** 8,
    sprite_count: u8 = 0,
    shifter_pattern_lo: [8]u8 = [_]u8{0} ** 8,
    shifter_pattern_hi: [8]u8 = [_]u8{0} ** 8,

    sprite_zero_hit_possible: bool = false,
    sprite_zero_being_rendered: bool = false,
};

fn div_rem(num: u64, den: u64) struct { u64, u64 } {
    return .{ @divFloor(num, den), num % den };
}

fn ppu_to_cpu_cycle(ppu_cyc: u64) u64 {
    const div, const rem = div_rem(ppu_cyc, 3);
    return if (rem == 0) div else div + 1;
}

fn cpu_to_ppu_cycle(cpu_cyc: u64) u64 {
    return cpu_cyc * 3;
}

/// - PPU memory map
/// The PPU addresses a 14-bit (16kB) address space, `0x0000-0x3FFF`, completely separate from the CPU's address bus. It
/// is either directly accessed by the PPU itself, or via the CPU with memory mapped registers at `0x2006` and `0x2007`.
///
/// *Address range*       *Size*    Description             Mapped by
/// `$0000-$0FFF`         `$1000`   Pattern table 0         Cartridge
/// `$1000-$1FFF`         `$1000`   Pattern table 1         Cartridge
/// `$2000-$23BF`         `$03C0`   Nametable 0             Cartridge
/// `$23C0-$23FF`         `$0040`   Attribute table 0       Cartridge
/// `$2400-$27BF`         `$03C0`   Nametable 1             Cartridge
/// `$27C0-$27FF`         `$0040`   Attribute table 1       Cartridge
/// `$2800-$2BBF`         `$03C0`   Nametable 2             Cartridge
/// `$2BC0-$2BFF`         `$0040`   Attribute table 2       Cartridge
/// `$2C00-$2FBF`         `$03C0`   Nametable 3             Cartridge
/// `$2FC0-$2FFF`         `$0040`   Attribute table 3       Cartridge
/// `$3000-$3EFF`         `$0F00`   Unused                  Cartridge
/// `$3F00-$3F1F`         `$0020`   Palette RAM indexes     Internal to PPU
/// `$3F20-$3FFF`         `$00E0`   Mirrors of $3F00-$3F1F  Internal to PPU
pub const PPU = struct {
    rom: *Rom,
    /// Internal memory to keep palette tables used by a screen
    palette_table: [32]u8,
    /// 2 KiB of space to hold background information, can hold two nametables.
    vram: [2048]u8,

    /// `0x2000` - write: `PPUCTRL` - Miscellaneous settings
    ///
    /// Contains a mix of settings related to rendering, scroll position, vblank NMI, and dual-PPU configurations.
    ctrl_register: ControlRegister,

    /// `0x2001` - write:  `PPUMASK` - Rendering settings
    ///
    /// Controls the rendering of sprites and backgrounds, as well as color effects. Most commonly, PPUMASK is set to
    /// `0x00` outside of gameplay to allow transferring a large amount of data to VRAM, and `0x1E` during gameplay to
    /// enable all rendering with no color effects.
    mask_register: MaskRegister,

    /// `0x2002` - read: `PPUSTATUS` - Rendering events
    ///
    /// Reflects the state of rendering-related events and is primarily used for timing. The three flags in this
    /// register are automatically cleared on dot 1 of the prerender scanline.
    ///
    /// Reading this register has the side effect of clearing the PPU's `internal w register`. It is commonly read
    /// before writes to `PPUSCROLL` and `PPUADDR` to ensure the writes occur in the correct order.
    status_register: StatusRegister,

    /// `0x2003` - write: `OAMADDR` - Sprite RAM address
    ///
    /// Write the address of OAM you want to access here. Most games just write 0x00 here and then use OAMDMA. (DMA is
    /// implemented in the 2A03/7 chip and works by repeatedly writing to OAMDATA).
    oam_addr_register: u8,

    /// `0x2004` - read/write: `OAMDATA` - Sprite RAM data
    ///
    /// Write OAM data here. Writes will increment `OAMADDR` after the write; reads do not. Reads during vertical or
    /// forced blanking return the value from OAM at that address.
    oam_data_register: [256]u8,

    /// `0x2005` - write: `PPUSCROLL` - X and Y scroll
    ///
    /// This register is used to change the scroll position, telling the PPU which pixel of the nametable selected
    /// through `PPUCTRL` should be at the top left corner of the rendered screen. `PPUSCROLL` takes two writes: the
    /// first is the X scroll and the second is the Y scroll. Whether this is the first or second write is tracked
    /// internally by the `w register`, which is shared with `PPUADDR`.
    scroll_register: ScrollRegister,

    /// `0x2006` - write: `PPUADDR` - VRAM address (two writes: most significant byte, then least significant byte).
    ///
    /// The 16-bit address is written to PPUADDR one byte at a time. Whether this is the first or second write is
    /// tracked by the PPU's internal `w` register.
    addr_register: AddrRegister,
    tmp_addr: AddrRegister,

    /// `Internal register W`: Toggles on each write to either `PPUSCROLL` or `PPUADDR`, indicating whether this is the
    /// first or second write. Clears on reads of `PPUSTATUS`. Sometimes called the 'write latch' or 'write toggle'.
    write_toggle: bool,
    fine_x: u8,

    /// `0x4014` - write: `Sprite DMA`
    /// OAMDMA is a CPU register that suspends the CPU so it can quickly copy a page of CPU memory to PPU OAM using DMA.
    /// The value written to this register is the high byte of the source address, and the copy begins on the cycle
    /// immediately after the write.
    oam_dma_addr: u8,

    internal_data_buf: u8,

    cycle: i16,
    scanline: i16,

    bg_data: BgData,
    fg: FgData,

    nmi_interrupt: bool,

    frame_buffer: Frame,
    frame_complete: bool,

    global_cycle: u64,
    next_vblank_ppu_cycle: u64,
    next_vblank_cpu_cycle: u64,

    const Self = @This();

    pub fn init(rom: *Rom) Self {
        return .{
            .rom = rom,
            .palette_table = [_]u8{0} ** 32,
            .vram = [_]u8{0} ** 2048,
            .write_toggle = false,
            .ctrl_register = .{},
            .mask_register = .{},
            .status_register = .{},
            .oam_addr_register = 0,
            .oam_data_register = [_]u8{0} ** 256,
            .oam_dma_addr = 0,
            .scroll_register = .{},
            .addr_register = .{},
            .tmp_addr = .{},
            .internal_data_buf = 0,
            .scanline = 0,
            .cycle = 0,
            .nmi_interrupt = false,
            .bg_data = .{},
            .fg = .{},
            .fine_x = 0,
            .frame_buffer = Frame.init(),
            .frame_complete = false,
            .global_cycle = 0,
            .next_vblank_ppu_cycle = 1,
            .next_vblank_cpu_cycle = ppu_to_cpu_cycle(1),
        };
    }

    fn first_nametable(self: Self) Nametable {
        return .{ .data = self.vram[0..0x3C0], .attribute_table = self.vram[0x3C0..0x400] };
    }

    fn second_nametable(self: Self) Nametable {
        return .{ .data = self.vram[0x400..0x7C0], .attribute_table = self.vram[0x7C0..0x800] };
    }

    fn nametables(self: Self) Nametable {
        const addr = self.ctrl_register.nametable_addr();
        return switch (self.rom.get_mirroring()) {
            .VERTICAL => switch (addr) {
                0x2000, 0x2400 => self.first_nametable(),
                0x2800, 0x2C00 => self.second_nametable(),
                else => std.debug.panic("Invalid address 0x{X:04} for Vertical mirroring\n", .{addr}),
            },
            .HORIZONTAL => switch (addr) {
                0x2000, 0x2800 => self.first_nametable(),
                0x2400, 0x2C00 => self.second_nametable(),
                else => std.debug.panic("Invalid address 0x{X:04} for Horizontal mirroring\n", .{addr}),
            },
            else => std.debug.panic("Mirroring type not implemented: {any}\n", .{self.rom.get_mirroring()}),
        };
    }

    pub fn run_to(self: *Self, cpu_cycle: u64) void {
        const stop = cpu_to_ppu_cycle(cpu_cycle);
        while (self.global_cycle < stop) {
            self.tick_cycle();
            self.tick();
        }
    }

    pub fn requested_run_cycle(self: *const Self) u64 {
        return self.next_vblank_cpu_cycle;
    }

    pub fn tick(self: *Self) void {
        // All but 1 of the scanlines is visible to the user. The pre-render scanline
        // at -1, is used to configure the "shifters" for the first visible scanline, 0.
        if (self.scanline >= -1 and self.scanline < 240) {
            if (self.scanline == 0 and self.cycle == 0) {
                // "Odd frame" cycle skip
                // self.cycle = 1;
                self.tick_cycle();
            }

            // Start of new frame, clear flags
            if (self.scanline == -1 and self.cycle == 1) {
                self.status_register.vblank = false;
                self.status_register.sprite_overflow = false;
                self.status_register.sprite_zero_hit = false;

                @memset(&self.fg.shifter_pattern_lo, 0);
                @memset(&self.fg.shifter_pattern_hi, 0);
            }

            if (self.cycle >= 2 and self.cycle < 258 or self.cycle >= 321 and self.cycle < 338) {
                self.update_shifters();

                switch (@rem(self.cycle - 1, 8)) {
                    0 => {
                        self.load_bg_shifters();
                        // Fetch the next background tile ID
                        self.bg_data.next_tile_id = self.mem_read(0x2000 | (self.addr_register.addr() & 0x0FFF));
                    },
                    2 => {
                        // Fetch the next background tile attribute.
                        self.bg_data.next_tile_attr = self.mem_read(0x23C0 |
                            (@as(u16, self.addr_register.nametable_y) << 11) |
                            (@as(u16, self.addr_register.nametable_x) << 10) |
                            ((@as(u16, self.addr_register.coarse_y) >> 2) << 3) |
                            (@as(u16, self.addr_register.coarse_x) >> 2));

                        // We want the bottom two bits of our attribute word to be the palette selected. Shift as
                        // required.
                        if (self.addr_register.coarse_y & 0x02 != 0) {
                            self.bg_data.next_tile_attr >>= 4;
                        }
                        if (self.addr_register.coarse_x & 0x02 != 0) {
                            self.bg_data.next_tile_attr >>= 2;
                        }
                        self.bg_data.next_tile_attr &= 0x03;
                    },
                    4 => {
                        // Fetch the next background tile LSB bit plane from the pattern memory.
                        self.bg_data.next_tile_lsb = self.mem_read(self.ctrl_register.bg_pattern_addr() +
                            (@as(u16, self.bg_data.next_tile_id) << 4) +
                            self.addr_register.fine_y);
                    },
                    6 => {
                        // Fetch the next background tile MSB bit plane from the pattern memory.
                        self.bg_data.next_tile_msb = self.mem_read(self.ctrl_register.bg_pattern_addr() +
                            (@as(u16, self.bg_data.next_tile_id) << 4) +
                            self.addr_register.fine_y + 8);
                    },
                    7 => {
                        // Increment the background tile "pointer" to the next tile horizontally in the nametable memory.
                        self.increment_scroll_x();
                    },
                    else => {},
                }
            }
            // End of a visible scanline, so increment downwards.
            if (self.cycle == 256) {
                self.increment_scroll_y();
            }

            // reset the  x position.
            if (self.cycle == 257) {
                self.load_bg_shifters();
                self.transfer_addr_x();
            }

            if (self.cycle == 338 or self.cycle == 340) {
                self.bg_data.next_tile_id = self.mem_read(0x2000 | (self.addr_register.addr() & 0x0FFF));
            }

            // End of vertical blank period so reset the Y address ready for rendering
            if (self.scanline == -1 and self.cycle >= 280 and self.cycle < 305) {
                self.transfer_addr_y();
            }

            // Foreground rendering - NOTE: NOT CYCLE ACCURATE
            if (self.cycle == 257 and self.scanline >= 0) {
                // We've reached the end of a visible scanline. It is now time to determine
                // which sprites are visible on the next scanline, and preload this info
                // into buffers that we can work with while the scanline scans the row.

                // clear out the sprite memory. This memory is used to store the sprites to be rendered.
                @memset(&self.fg.sprite_scanline, .{});

                self.fg.sprite_count = 0;

                // clear out any residual information in sprite pattern shifters.
                @memset(&self.fg.shifter_pattern_lo, 0);
                @memset(&self.fg.shifter_pattern_hi, 0);

                // Evaluate which sprites are visible in the next scanline. We need to iterate through the OAM until we
                // have found 8 sprites that have Y-positions and heights that are within vertical range of the next
                // scanline. Once we have found 8 or exhausted the OAM we stop.
                var oam_index: usize = 0;

                self.fg.sprite_zero_hit_possible = false;

                // count to 9 sprites to set the sprite overflow flag in the event of there being > 8 sprites.
                while (oam_index < 64 and self.fg.sprite_count < 9) {
                    const diff = self.scanline - @as(
                        i16,
                        OamEntry.init(self.oam_data_register[oam_index * 4 .. oam_index * 4 + 4]).y,
                    );
                    const sprite_size: i16 = if (self.ctrl_register.sprite_size) 16 else 8;

                    if (diff >= 0 and diff < sprite_size) {
                        // Sprite is visible, so copy the attribute entry over to our scanline sprite cache.
                        if (self.fg.sprite_count < 8) {
                            // is sprite zero?
                            if (oam_index == 0) {
                                self.fg.sprite_zero_hit_possible = true;
                            }

                            self.fg.sprite_scanline[self.fg.sprite_count] = OamEntry.init(self.oam_data_register[oam_index * 4 .. oam_index * 4 + 4]);
                            self.fg.sprite_count += 1;
                        }
                    }

                    oam_index += 1;
                }

                self.status_register.sprite_overflow = self.fg.sprite_count > 8;
            }

            if (self.cycle == 340) {
                // We're at the end of the scanline, prepare the sprite shifters with the 8 or less selected sprites.
                for (0..self.fg.sprite_count) |i| {
                    var sprite_pattern_bits_lo: u8 = 0;
                    var sprite_pattern_bits_hi: u8 = 0;
                    var sprite_pattern_addr_lo: u16 = 0;
                    var sprite_pattern_addr_hi: u16 = 0;

                    const sprite_tile_id: u16 = self.fg.sprite_scanline[i].id;
                    const sprite_y_pos: i16 = self.fg.sprite_scanline[i].y;
                    const height: i16 = if (self.ctrl_register.sprite_size) 16 else 8;
                    const row_i16: i16 = self.scanline - sprite_y_pos;
                    var row: u8 = @intCast(row_i16);

                    if (self.fg.sprite_scanline[i].attr & 0x80 != 0) {
                        // Sprite is flipped vertically, i.e. upside down
                        row = @intCast(height - 1 - row_i16);
                    }

                    if (!self.ctrl_register.sprite_size) { // 8x8 sprite mode
                        const base_addr: u16 = if (self.ctrl_register.sprite_pattern_addr) 0x1000 else 0x0000;
                        sprite_pattern_addr_lo = base_addr |
                            (sprite_tile_id << 4) |
                            @as(u16, @bitCast(self.scanline -% sprite_y_pos));
                    } else { // 8x16 sprite mode
                        var effective_tile: u16 = sprite_tile_id & 0xFE; // Even tile index for top half
                        var effective_row: u16 = row;
                        if (row >= 8) {
                            effective_tile += 1; // Switch to next (bottom) tile
                            effective_row -= 8; // Offset row within bottom tile
                        }

                        const bank = (sprite_tile_id & 0x01) << 12;
                        sprite_pattern_addr_lo = bank | (effective_tile << 4) | (effective_row & 0b0000_0111);
                    }

                    sprite_pattern_addr_hi = sprite_pattern_addr_lo + 8;

                    sprite_pattern_bits_lo = self.mem_read(sprite_pattern_addr_lo);
                    sprite_pattern_bits_hi = self.mem_read(sprite_pattern_addr_hi);

                    if (self.fg.sprite_scanline[i].attr & 0x40 != 0) {
                        // flip patterns horizontally
                        sprite_pattern_bits_lo = flip_byte(sprite_pattern_bits_lo);
                        sprite_pattern_bits_hi = flip_byte(sprite_pattern_bits_hi);
                    }

                    // load the pattern into the sprite shift registers
                    self.fg.shifter_pattern_lo[i] = sprite_pattern_bits_lo;
                    self.fg.shifter_pattern_hi[i] = sprite_pattern_bits_hi;
                }
            }
        }

        if (self.scanline == 240) {
            // post render scanline - do nothing!
        }

        if (self.scanline >= 241 and self.scanline < 261) {
            if (self.scanline == 241 and self.cycle == 1) {
                self.status_register.vblank = true; // end of frame, set vblank
                self.next_vblank_ppu_cycle += CYCLES_PER_FRAME;
                self.next_vblank_cpu_cycle = ppu_to_cpu_cycle(self.next_vblank_ppu_cycle);

                if (self.ctrl_register.generate_nmi) {
                    self.nmi_interrupt = true;
                }
            }
        }

        var bg_pixel: u8 = 0; // 2-bit pixel to be rendered
        var bg_palette: u8 = 0; // 3-bit index of the palette the pixel indexes
        if (self.mask_register.render_background) {
            const bit_mux: u16 = std.math.shr(u16, 0x8000, self.fine_x);

            const p0_pixel: u8 = @intFromBool((self.bg_data.shifter_pattern_lo & bit_mux) > 0);
            const p1_pixel: u8 = @intFromBool((self.bg_data.shifter_pattern_hi & bit_mux) > 0);
            bg_pixel = (p1_pixel << 1) | p0_pixel;

            const bg_pal0: u8 = @intFromBool((self.bg_data.shifter_attr_lo & bit_mux) > 0);
            const bg_pal1: u8 = @intFromBool((self.bg_data.shifter_attr_hi & bit_mux) > 0);
            bg_palette = (bg_pal1 << 1) | bg_pal0;
        }

        var fg_pixel: u8 = 0;
        var fg_palette: u8 = 0;
        var fg_priority: bool = false;
        if (self.mask_register.render_sprite) {
            self.fg.sprite_zero_being_rendered = false;
            for (0..self.fg.sprite_count) |i| {
                if (self.fg.sprite_scanline[i].x == 0) {
                    const fg_pixel_lo: u8 = @intFromBool((self.fg.shifter_pattern_lo[i] & 0x80) > 0);
                    const fg_pixel_hi: u8 = @intFromBool((self.fg.shifter_pattern_hi[i] & 0x80) > 0);
                    fg_pixel = (fg_pixel_hi << 1) | fg_pixel_lo;

                    // Extract the palette from the bottom two bits. The foreground palettes are the latter 4 in the
                    // palette memory.
                    fg_palette = (self.fg.sprite_scanline[i].attr & 3) + 4;
                    fg_priority = (self.fg.sprite_scanline[i].attr & 0x20) == 0;

                    if (fg_pixel != 0) {
                        if (i == 0) { // is sprite zero?
                            self.fg.sprite_zero_being_rendered = true;
                        }
                        break;
                    }
                }
            }
        }

        var pixel: u8 = 0;
        var palette: u8 = 0;
        if (bg_pixel == 0 and fg_pixel == 0) {
            // The background pixel is transparent
            // The foreground pixel is transparent
            // No winner, draw "background" colour
            pixel = 0x00;
            palette = 0x00;
        } else if (bg_pixel == 0 and fg_pixel > 0) {
            // The background pixel is transparent
            // The foreground pixel is visible
            // Foreground wins!
            pixel = fg_pixel;
            palette = fg_palette;
        } else if (bg_pixel > 0 and fg_pixel == 0) {
            // The background pixel is visible
            // The foreground pixel is transparent
            // Background wins!
            pixel = bg_pixel;
            palette = bg_palette;
        } else if (bg_pixel > 0 and fg_pixel > 0) {
            // The background pixel is visible
            // The foreground pixel is visible
            if (fg_priority) {
                pixel = fg_pixel;
                palette = fg_palette;
            } else {
                pixel = bg_pixel;
                palette = bg_palette;
            }

            if (self.fg.sprite_zero_hit_possible and self.fg.sprite_zero_being_rendered) {
                // Sprite zero is a collision between foreground and background
                // so they must both be enabled
                if (self.mask_register.render_background and self.mask_register.render_sprite) {
                    // The left edge of the screen has specific switches to control
                    // its appearance. This is used to smooth inconsistencies when
                    // scrolling (since sprites x coord must be >= 0)
                    if (!(self.mask_register.render_background_left or self.mask_register.render_sprite_left)) {
                        if (self.cycle >= 9 and self.cycle < 258) {
                            self.status_register.sprite_zero_hit = true;
                        }
                    } else if (self.cycle >= 1 and self.cycle < 258) {
                        self.status_register.sprite_zero_hit = true;
                    }
                }
            }
        }

        if (self.scanline >= 0 and self.scanline < 240 and
            self.cycle >= 1 and self.cycle <= 256)
        {
            self.frame_buffer.set_pixel(
                @as(u16, @bitCast(self.cycle - 1)),
                @as(u16, @bitCast(self.scanline)),
                self.get_color(palette, pixel),
            );
        }
    }

    fn tick_cycle(self: *Self) void {
        self.global_cycle += 1;
        self.cycle += 1;
        if (self.cycle == 341) {
            self.cycle = 0;
            self.scanline += 1;
            if (self.scanline == 261) {
                self.scanline = -1;
                self.frame_complete = true;
            }
        }
    }

    fn get_color(self: *Self, palette_index: u8, pixel_index: u8) render.Color {
        // "0x3F00"       - Offset into PPU addressable range where palettes are stored
        // "palette_index * 4" - Each palette is 4 bytes in size
        // "pixel_index"        - Each pixel index is either 0, 1, 2 or 3
        // "& 0x3F"       - Stops us reading beyond the bounds of the SYSTEM_PALETTE array
        return render.SYSTEM_PALETTE[self.mem_read(0x3F00 + @as(u16, palette_index * 4) + pixel_index) & 0x3F];
    }

    fn transfer_addr_x(self: *Self) void {
        if (self.mask_register.render_background or self.mask_register.render_sprite) {
            self.addr_register.coarse_x = self.tmp_addr.coarse_x;
            self.addr_register.nametable_x = self.tmp_addr.nametable_x;
        }
    }

    fn transfer_addr_y(self: *Self) void {
        if (self.mask_register.render_background or self.mask_register.render_sprite) {
            self.addr_register.fine_y = self.tmp_addr.fine_y;
            self.addr_register.coarse_y = self.tmp_addr.coarse_y;
            self.addr_register.nametable_y = self.tmp_addr.nametable_y;
        }
    }

    // Increment the background tile "pointer" one tile/column horizontally
    fn increment_scroll_x(self: *Self) void {
        if (!(self.mask_register.render_background or self.mask_register.render_sprite)) {
            return;
        }

        if (self.addr_register.coarse_x == 31) {
            self.addr_register.coarse_x = 0;
            self.addr_register.nametable_x = ~self.addr_register.nametable_x;
        } else {
            self.addr_register.coarse_x += 1;
        }
    }

    // Increment the background tile "pointer" one scanline vertically
    fn increment_scroll_y(self: *Self) void {
        if (!(self.mask_register.render_background or self.mask_register.render_sprite)) {
            return;
        }

        if (self.addr_register.fine_y < 7) {
            self.addr_register.fine_y += 1;
        } else {
            self.addr_register.fine_y = 0;
            if (self.addr_register.coarse_y == 29) {
                self.addr_register.coarse_y = 0;
                self.addr_register.nametable_y = ~self.addr_register.nametable_y;
            } else if (self.addr_register.coarse_y == 31) {
                self.addr_register.coarse_y = 0;
            } else {
                self.addr_register.coarse_y += 1;
            }
        }
    }

    fn load_bg_shifters(self: *Self) void {
        self.bg_data.shifter_pattern_lo = (self.bg_data.shifter_pattern_lo & 0xFF00) | self.bg_data.next_tile_lsb;
        self.bg_data.shifter_pattern_hi = (self.bg_data.shifter_pattern_hi & 0xFF00) | self.bg_data.next_tile_msb;

        self.bg_data.shifter_attr_lo = (self.bg_data.shifter_attr_lo & 0xFF00) |
            if (self.bg_data.next_tile_attr & 0b01 != 0) @as(u8, 0xFF) else @as(u8, 0x00);
        self.bg_data.shifter_attr_hi = (self.bg_data.shifter_attr_hi & 0xFF00) |
            if (self.bg_data.next_tile_attr & 0b10 != 0) @as(u8, 0xFF) else @as(u8, 0x00);
    }

    fn update_shifters(self: *Self) void {
        if (self.mask_register.render_background) {
            // shift background tile pattern row
            self.bg_data.shifter_pattern_lo <<= 1;
            self.bg_data.shifter_pattern_hi <<= 1;

            // shift palette attribute by 1
            self.bg_data.shifter_attr_lo <<= 1;
            self.bg_data.shifter_attr_hi <<= 1;
        }

        if (self.mask_register.render_sprite and self.cycle >= 1 and self.cycle < 258) {
            for (0..self.fg.sprite_count) |i| {
                if (self.fg.sprite_scanline[i].x > 0) {
                    self.fg.sprite_scanline[i].x -%= 1;
                } else {
                    self.fg.shifter_pattern_lo[i] <<= 1;
                    self.fg.shifter_pattern_hi[i] <<= 1;
                }
            }
        }
    }

    /// Writes a byte to the `PPUADDR` register (`0x2006`) to set the PPU memory address.
    ///
    /// The PPU uses a 16-bit address pointer to access its memory space, but the CPU
    /// can only write 8 bits at a time through the `PPUADDR` register. This requires two
    /// sequential writes to set a complete address:
    /// 1. First write: Sets the high byte (bits 8-15) of the address
    /// 2. Second write: Sets the low byte (bits 0-7) of the address
    ///
    /// An internal write toggle tracks whether the next write should update the high
    /// or low byte. This toggle is shared with `PPUSCROLL` and is reset by reading
    /// `PPUSTATUS` (`0x2002`).
    ///
    /// ## Write Sequence
    ///
    /// ```
    /// First write:  `0xAB` -> address becomes `0xAB00`
    /// Second write: `0xCD` -> address becomes `0xABCD`
    /// Third write:  `0x12` -> address becomes `0x12CD` (high byte again)
    /// Fourth write: `0x34` -> address becomes `0x1234` (low byte again)
    /// ```
    ///
    /// ## Address Mirroring
    ///
    /// PPU address space is only 14 bits (`0x0000-0x3FFF`). Addresses above `0x3FFF`
    /// are automatically mirrored down by masking to 14 bits. For example:
    /// - `0x4000` mirrors to `0x0000`
    /// - `0x7FFF` mirrors to `0x3FFF`
    ///
    /// ## Side Effects
    /// - Toggles the write latch for the next write
    ///
    /// ## Important Notes
    /// - The write toggle is shared with `PPUSCROLL` (`0x2005`)
    /// - Reading `PPUSTATUS` (`0x2002`) resets the write toggle to "high byte" state
    /// - After setting an address, subsequent `PPUDATA` reads/writes will use this address
    pub fn addr_write(self: *Self, value: u8) void {
        // if (!self.write_toggle) {
        //     self.addr_register = self.addr_register & ~@as(u16, 0xFF00) | (@as(u16, value) << 8);
        // } else {
        //     self.addr_register = self.addr_register & ~@as(u16, 0xFF) | value;
        // }
        if (!self.write_toggle) {
            const tmp_addr = self.tmp_addr.addr() & ~@as(u16, 0xFF00) | (@as(u16, value) << 8);
            self.tmp_addr = @bitCast(tmp_addr);
        } else {
            const tmp_addr = self.tmp_addr.addr() & ~@as(u16, 0xFF) | value;
            self.tmp_addr = @bitCast(tmp_addr);
            self.addr_register = self.tmp_addr;
        }

        self.write_toggle = !self.write_toggle;
    }

    fn increment_vram_addr(self: *Self) void {
        const new_addr = self.addr_register.addr() +% self.ctrl_register.vram_addr_increment();
        self.addr_register = @bitCast(new_addr);
    }

    /// Mirrors PPU VRAM addresses according to the configured mirroring mode.
    ///
    /// The NES PPU has 4 nametable address ranges (`0x2000-0x23FF`, `0x2400-0x27FF`,
    /// `0x2800-0x2BFF`, `0x2C00-0x2FFF`), but only 2KB of actual VRAM. The mirroring
    /// mode determines how these 4 logical nametables map to the 2KB of physical memory.
    ///
    /// Address range `0x3000-0x3EFF` is mirrored down to `0x2000-0x2EFF` before processing.
    ///
    /// ## Mirroring Modes
    ///
    /// **Horizontal Mirroring** (vertical arrangement):
    /// ```
    ///   [ A ] [ B ]
    ///   [ a ] [ b ]
    /// ```
    /// - Nametables 0 and 1 map to the first 1KB (top row)
    /// - Nametables 2 and 3 map to the second 1KB (bottom row)
    ///
    /// **Vertical Mirroring** (horizontal arrangement):
    /// ```
    ///   [ A ] [ a ]
    ///   [ B ] [ b ]
    /// ```
    /// - Nametables 0 and 2 map to the first 1KB (left column)
    /// - Nametables 1 and 3 map to the second 1KB (right column)
    ///
    /// **Four-Screen**: All 4 nametables are unique (requires extra RAM on cartridge)
    fn mirror_vram_addr(self: *const Self, addr: u16) u16 {
        // mirror down 0x2000 - 0x2eff
        const mirrored_addr = addr & 0x2FFF;
        // to index (0x0000 - 0x0FFF)
        const vram_index = mirrored_addr - 0x2000;
        // to the name table index
        const name_table_idx = vram_index / 0x400;

        return switch (self.rom.get_mirroring()) {
            Mirroring.VERTICAL => switch (name_table_idx) {
                0, 2 => vram_index & 0x03FF,
                1, 3 => (vram_index & 0x03FF) + 0x0400,
                else => unreachable,
            },
            Mirroring.HORIZONTAL => switch (name_table_idx) {
                0, 1 => vram_index & 0x03FF,
                2, 3 => (vram_index & 0x03FF) + 0x0400,
                else => unreachable,
            },
            Mirroring.FOUR_SCREEN => {
                std.log.warn("PPU: FOUR_SCREEN mirroring not implemented!", .{});
                return vram_index;
            },
        };
    }

    /// Reads a byte from PPU memory through the `PPUDATA` register (`0x2007`).
    ///
    /// This function handles reads from different regions of PPU address space based on
    /// the current value in the PPU address register (set via PPUADDR writes). After
    /// each read, the VRAM address is automatically incremented based on bit 2 of
    /// `PPUCTRL` (+1 for horizontal, +32 for vertical).
    ///
    /// ## Internal Read Buffer Behavior
    ///
    /// **IMPORTANT**: Reads from PPUDATA are buffered with a one-read delay for most
    /// address ranges. When reading from `0x0000-0x3EFF`:
    /// 1. The function returns the value from the *previous* read (internal buffer)
    /// 2. The internal buffer is updated with the value at the *current* address
    /// 3. The first read after setting an address returns stale buffer data
    ///
    /// **Exception**: Palette RAM (`0x3F00-0x3FFF`) returns data immediately without
    /// buffering, allowing direct palette reads.
    ///
    /// ## Address Space Mapping
    ///
    /// - **0x0000-0x1FFF**: Pattern Tables (CHR ROM/RAM)
    ///   - Returns buffered data from CHR ROM
    ///   - Subject to one-read delay
    ///
    /// - **0x2000-0x2FFF**: Nametables (VRAM)
    ///   - Returns buffered data from nametable memory
    ///   - Mirroring is applied based on cartridge configuration
    ///   - Subject to one-read delay
    ///
    /// - **0x3000-0x3EFF**: Nametable Mirror
    ///   - Mirrors 0x2000-0x2EFF (rarely used in practice)
    ///   - Generates a warning but processes the read
    ///   - Subject to one-read delay
    ///
    /// - **0x3F00-0x3FFF**: Palette RAM
    ///   - Returns data immediately (no buffering!)
    ///   - 32 bytes with internal mirroring
    ///   - Used for background and sprite color palettes
    pub fn data_read(self: *Self) u8 {
        const addr = self.addr_register.addr();

        const result = self.internal_data_buf;
        self.internal_data_buf = self.mem_read(addr);
        self.increment_vram_addr();

        if (addr >= 0x3F00) { // palette address doesn't have buffering
            return self.internal_data_buf;
        }

        return result;
    }

    fn mem_read(self: *Self, addr: u16) u8 {
        const new_addr = addr & 0x3FFF;
        // self.rom.mapper_ppu_clock(new_addr);

        if (new_addr >= 0 and new_addr <= 0x1FFF) {
            return self.rom.chr_read(new_addr);
        } else if (new_addr >= 0x2000 and new_addr <= 0x3EFF) {
            return self.vram[self.mirror_vram_addr(new_addr)];
        } else if (new_addr >= 0x3F00 and new_addr <= 0x3FFF) {
            var palette_addr = new_addr & 0x1F;
            palette_addr = switch (palette_addr) {
                0x10, 0x14, 0x18, 0x1C => palette_addr - 0x10,
                else => palette_addr,
            };
            return self.palette_table[palette_addr] & if (self.mask_register.greyscale) @as(u8, 0x30) else 0x3F;
        } else {
            std.log.warn("PPU: unexpected access to mirrored space 0x{X:04} while reading\n", .{new_addr});
            return 0;
        }
    }

    /// Writes a byte to PPU memory through the `PPUDATA` register (`0x2007`).
    ///
    /// This function handles writes to different regions of PPU address space based on
    /// the current value in the PPU address register (set via PPUADDR writes). After
    /// each write, the VRAM address is automatically incremented based on the control
    /// register settings (`+1` or `+32`).
    ///
    /// ## Address Space Mapping
    ///
    /// - **0x0000-0x1FFF**: Pattern Tables (CHR ROM/RAM)
    ///   - Writes to this region are logged as errors if CHR ROM is present
    ///   - CHR RAM writes would be handled by the cartridge mapper
    ///
    /// - **0x2000-0x2FFF**: Nametables (VRAM)
    ///   - Writes to nametable memory with mirroring applied
    ///   - Actual storage uses 2KB VRAM with hardware mirroring
    ///
    /// - **0x3000-0x3EFF**: Nametable Mirror
    ///   - Mirrors 0x2000-0x2EFF (not commonly used)
    ///   - Generates a warning but processes the write
    ///
    /// - **0x3F00-0x3FFF**: Palette RAM
    ///   - 32 bytes of palette data with mirroring
    ///   - Stores background and sprite color palettes
    pub fn data_write(self: *Self, value: u8) void {
        const addr = self.addr_register.addr() & 0x3FFF;
        // self.rom.mapper_ppu_clock(addr);
        if (addr >= 0 and addr <= 0x1FFF) {
            self.rom.chr_write(addr, value);
        } else if (addr >= 0x2000 and addr <= 0x3EFF) {
            self.vram[self.mirror_vram_addr(addr)] = value;
        } else if (addr >= 0x3F00 and addr <= 0x3FFF) {
            var palette_addr = addr & 0x001F;
            if (palette_addr == 0x10 or palette_addr == 0x14 or palette_addr == 0x18 or palette_addr == 0x1C) {
                palette_addr -= 0x10;
            }
            self.palette_table[palette_addr] = value;
        } else {
            std.debug.panic("PPU: unexpected access to mirrored space 0x{X:04} while writing\n", .{addr});
        }
        self.increment_vram_addr();
    }

    /// Write a value to the `PPUCTRL` register.
    pub fn ctrl_write(self: *Self, value: u8) void {
        self.ctrl_register = @bitCast(value);

        self.tmp_addr.nametable_x = @as(u1, @truncate(self.ctrl_register.name_table_addr)) & 1;
        self.tmp_addr.nametable_y = @as(u1, @truncate(self.ctrl_register.name_table_addr >> 1)) & 1;
    }

    /// Writes a byte to the `PPUMASK` register (`0x2001`) to configure rendering options.
    ///
    /// The `PPUMASK` register controls various rendering features including:
    /// - Grayscale mode
    /// - Background and sprite visibility in leftmost 8 pixels
    /// - Background and sprite rendering enable/disable
    /// - Color emphasis (tint) for red, green, and blue channels
    ///
    /// ## Bit Layout
    /// ```
    /// Bit 0: Grayscale (0 = normal color, 1 = grayscale)
    /// Bit 1: Show background in leftmost 8 pixels (1 = show, 0 = hide)
    /// Bit 2: Show sprites in leftmost 8 pixels (1 = show, 0 = hide)
    /// Bit 3: Show background (1 = enabled, 0 = disabled)
    /// Bit 4: Show sprites (1 = enabled, 0 = disabled)
    /// Bit 5: Emphasize red
    /// Bit 6: Emphasize green
    /// Bit 7: Emphasize blue
    /// ```
    pub fn mask_write(self: *Self, value: u8) void {
        self.mask_register = @bitCast(value);
    }

    /// Writes a byte to the `OAMADDR` register (`0x2003`) to set the OAM write address.
    ///
    /// Object Attribute Memory (`OAM`) stores sprite data for up to 64 sprites (256 bytes
    /// total, 4 bytes per sprite). `OAMADDR` sets the starting address within OAM for
    /// subsequent `OAMDATA` writes.
    ///
    /// ## Important Notes
    /// - Most games do not use `OAMADDR`/`OAMDATA` for sprite updates
    /// - Instead, they use `OAMDMA` (register `0x4014`) for faster bulk transfers
    /// - `OAMADDR` is often set to `0x00` before VBLANK to ensure clean sprite rendering
    /// - Writing to `OAMADDR` during rendering can cause sprite corruption
    pub fn oam_addr_write(self: *Self, value: u8) void {
        self.oam_addr_register = value;
    }

    /// Performs a `DMA` transfer to Object Attribute Memory (`OAM`) via register `0x4014`.
    ///
    /// OAM DMA is the standard method for updating all sprite data in a single operation.
    /// This copies 256 bytes of sprite data directly into `OAM`, typically from CPU RAM
    /// (often from page `0x02XX` or `0x07XX`). This is much faster than writing sprites
    /// individually through `OAMDATA`.
    ///
    /// On the NES hardware, this operation:
    /// - Suspends the CPU for 513-514 cycles
    /// - Copies 256 bytes from CPU memory to OAM
    /// - Is typically performed during `VBLANK` to avoid visual glitches
    pub fn oam_dma_write(self: *Self, data: [256]u8) void { // TODO remove
        // @memmove(&self.oam_data_register, &data);
        // self.oam_addr_register +%= 0xFF;
        // self.oam_addr_register +%= 1;
        for (data) |byte| {
            self.oam_data_register[self.oam_addr_register] = byte;
            self.oam_addr_register +%= 1;
        }
    }

    /// Writes a byte to the `OAMDATA` register (`0x2004`) to update a single `OAM` byte.
    ///
    /// Writes one byte to `OAM` at the current `OAMADDR` position and automatically
    /// increments `OAMADDR` afterward. This is rarely used in practice as `OAM DMA`
    /// is much more efficient for updating sprite data.
    ///
    /// ## Side Effects
    /// - Increments OAMADDR by 1 (wraps from 0xFF to 0x00)
    ///
    /// ## Important Notes
    /// - Writing during rendering can cause sprite corruption
    /// - Most games use `OAMDMA` instead of individual `OAMDATA` writes
    /// - Each sprite occupies 4 consecutive bytes in `OAM`
    pub fn oam_data_write(self: *Self, value: u8) void {
        self.oam_data_register[self.oam_addr_register] = value;
        self.oam_addr_register +%= 1;
    }

    pub fn oam_data_read(self: Self) u8 {
        return self.oam_data_register[self.oam_addr_register];
    }

    /// Writes a byte to the `PPUSCROLL` register (`0x2005`) to set screen scroll position.
    ///
    /// The `PPUSCROLL` register controls the starting position for background rendering,
    /// enabling smooth scrolling effects. Like `PPUADDR`, it requires two sequential writes:
    /// 1. First write: Sets the X scroll offset (0-255)
    /// 2. Second write: Sets the Y scroll offset (0-239)
    ///
    /// The write toggle is shared with `PPUADDR` and is reset by reading `PPUSTATUS`.
    ///
    /// ## Side Effects
    /// - Toggles the write latch for the next write
    ///
    /// ## Important Notes
    /// - Reading `PPUSTATUS` (`0x2002`) resets the toggle to "X scroll" state
    /// - Scroll changes typically occur during `VBLANK` to avoid visual glitches
    /// - Fine scroll updates affect background rendering immediately
    pub fn scroll_write(self: *Self, data: u8) void {
        if (!self.write_toggle) {
            // self.scroll_register.fine_x = data;
            self.fine_x = data & 0x07;
            self.tmp_addr.coarse_x = @truncate(data >> 3);
        } else {
            // self.scroll_register.fine_y = data;
            self.tmp_addr.fine_y = @truncate(data & 0x07);
            self.tmp_addr.coarse_y = @truncate(data >> 3);
        }
        self.write_toggle = !self.write_toggle;
    }

    /// Reads the `PPUSTATUS` register (`0x2002`) and returns its current state.
    ///
    /// The `PPUSTATUS` register provides important timing and status information:
    /// - Bit 7: VBLANK flag (1 = in VBLANK period)
    /// - Bit 6: Sprite 0 hit flag (1 = sprite 0 overlapped background)
    /// - Bit 5: Sprite overflow flag (1 = more than 8 sprites on a scanline)
    /// - Bits 0-4: Open bus (return last value on PPU data bus)
    ///
    /// ## Side Effects
    /// - **Clears the VBLANK flag** (bit 7) after reading
    /// - **Resets the write toggle** for `PPUADDR` and `PPUSCROLL` to "first write" state
    ///
    /// ## Important Notes
    /// - Reading PPUSTATUS clears VBLANK
    /// - The VBLANK flag is automatically set at the start of VBLANK (scanline 241)
    pub fn status_read(self: *Self) StatusRegister {
        const status = self.status_register;

        self.write_toggle = false;
        self.status_register.vblank = false;

        return status;
    }

    pub fn get_pattern_table(self: *Self, i: u8, palette: u8) Frame {
        var frame = Frame.init();
        for (0..16) |tile_y| {
            for (0..16) |tile_x| {
                const offset: u16 = @intCast(tile_y * 256 + tile_x * 16);
                for (0..8) |row| {
                    var tile_lsb = self.mem_read(@intCast(@as(u16, i) * 0x1000 + offset + row));
                    var tile_msb = self.mem_read(@intCast(@as(u16, i) * 0x1000 + offset + row + 0x0008));

                    for (0..8) |col| {
                        const pixel = (tile_lsb & 1) + (tile_msb & 1);
                        tile_lsb >>= 1;
                        tile_msb >>= 1;

                        frame.set_pixel(tile_x * 8 + (7 - col), tile_y * 8 + row, self.get_color(palette, pixel));
                    }
                }
            }
        }
        return frame;
    }
};

// "flips" a byte so 0b11100000 becomes 0b00000111.
// source: https://stackoverflow.com/a/2602885
fn flip_byte(byte: u8) u8 {
    var result = (byte & 0xF0) >> 4 | (byte & 0x0F) << 4;
    result = (result & 0xCC) >> 2 | (result & 0x33) << 2;
    result = (result & 0xAA) >> 1 | (result & 0x55) << 1;
    return result;
}

// TODO: fix tests
test "PPU VRAM write" {
    var ppu = PPU.init(&[_]u8{0} ** 2048, Mirroring.HORIZONTAL);
    ppu.addr_write(0x23);
    ppu.addr_write(0x05);
    ppu.data_write(0x42);

    try std.testing.expectEqual(0x42, ppu.vram[0x0305]);
}

test "PPU VRAM read" {
    var ppu = PPU.init(&[_]u8{0} ** 2048, Mirroring.HORIZONTAL);
    ppu.ctrl_write(0);
    ppu.vram[0x0305] = 0x42;

    ppu.addr_write(0x23);
    ppu.addr_write(0x05);
    _ = ppu.data_read(); // load data to internal buffer

    try std.testing.expectEqual(0x2306, @as(u16, @bitCast(ppu.addr_register)));
    try std.testing.expectEqual(0x42, ppu.data_read());
}

test "PPU VRAM read cross page" {
    var ppu = PPU.init(&[_]u8{0} ** 2048, Mirroring.HORIZONTAL);
    ppu.ctrl_write(0);
    ppu.vram[0x01ff] = 0x42;
    ppu.vram[0x0200] = 0x69;

    ppu.addr_write(0x21);
    ppu.addr_write(0xFF);
    _ = ppu.data_read(); // load data to internal buffer

    try std.testing.expectEqual(0x42, ppu.data_read());
    try std.testing.expectEqual(0x69, ppu.data_read());
}

test "PPU VRAM read - step 32" {
    var ppu = PPU.init(&[_]u8{0} ** 2048, Mirroring.HORIZONTAL);
    ppu.ctrl_register.vram_add_increment = true;
    ppu.vram[0x01ff] = 0x12;
    ppu.vram[0x01ff + 32] = 0x13;
    ppu.vram[0x01ff + 64] = 0x22;

    ppu.addr_write(0x21);
    ppu.addr_write(0xff);
    _ = ppu.data_read(); // load data to internal buffer

    try std.testing.expectEqual(0x12, ppu.data_read());
    try std.testing.expectEqual(0x13, ppu.data_read());
    try std.testing.expectEqual(0x22, ppu.data_read());
}

// Horizontal: https://wiki.nesdev.com/w/index.php/Mirroring
//   [0x2000 A ] [0x2400 a ]
//   [0x2800 B ] [0x2C00 b ]
test "PPU VRAM horizontal mirror" {
    var ppu = PPU.init(&[_]u8{0} ** 2048, Mirroring.HORIZONTAL);
    ppu.addr_write(0x24);
    ppu.addr_write(0x05);

    ppu.data_write(0x66); // write to A

    ppu.addr_write(0x28);
    ppu.addr_write(0x05);

    ppu.data_write(0x77); // write to B

    ppu.addr_write(0x20);
    ppu.addr_write(0x05);

    _ = ppu.data_read(); // load data to internal buffer
    try std.testing.expectEqual(0x66, ppu.data_read()); // read from a

    ppu.addr_write(0x2C);
    ppu.addr_write(0x05);

    _ = ppu.data_read(); // load data to internal buffer
    try std.testing.expectEqual(0x77, ppu.data_read()); // read from b
}

// Vertical: https://wiki.nesdev.com/w/index.php/Mirroring
//   [0x2000 A ] [0x2400 B ]
//   [0x2800 a ] [0x2C00 b ]
test "PPU VRAM vertical mirror" {
    var ppu = PPU.init(&[_]u8{0} ** 2048, Mirroring.VERTICAL);
    ppu.addr_write(0x20);
    ppu.addr_write(0x05);

    ppu.data_write(0x66); // write to A

    ppu.addr_write(0x2C);
    ppu.addr_write(0x05);

    ppu.data_write(0x77); // write to B

    ppu.addr_write(0x28);
    ppu.addr_write(0x05);

    _ = ppu.data_read(); // load data to internal buffer
    try std.testing.expectEqual(0x66, ppu.data_read()); // read from a

    ppu.addr_write(0x24);
    ppu.addr_write(0x05);

    _ = ppu.data_read(); // load data to internal buffer
    try std.testing.expectEqual(0x77, ppu.data_read()); // read from b
}

test "read status resets write toggle (internal w register)" {
    var ppu = PPU.init(&[_]u8{0} ** 2048, Mirroring.HORIZONTAL);
    ppu.vram[0x0305] = 0x66;

    ppu.addr_write(0x21);
    ppu.addr_write(0x23);
    ppu.addr_write(0x05); // write toggle state is false after this

    _ = ppu.data_read(); // load data to internal buffer
    try std.testing.expect(ppu.data_read() != 0x66);

    _ = ppu.status_read();

    ppu.addr_write(0x23);
    ppu.addr_write(0x05);

    _ = ppu.data_read(); // load data to internal buffer
    try std.testing.expectEqual(0x66, ppu.data_read());
}

test "PPU VRAM mirroring" {
    var ppu = PPU.init(&[_]u8{0} ** 2048, Mirroring.HORIZONTAL);
    ppu.vram[0x0305] = 0x66;

    ppu.addr_write(0x63); // 0x6305 -> 0x2305
    ppu.addr_write(0x05);

    _ = ppu.data_read(); // load data to internal buffer
    try std.testing.expectEqual(0x66, ppu.data_read());
}

test "OAM read write" {
    var ppu = PPU.init(&[_]u8{0} ** 2048, Mirroring.HORIZONTAL);
    ppu.oam_addr_write(0x10);
    ppu.oam_data_write(0x42);
    ppu.oam_data_write(0x69);

    ppu.oam_addr_write(0x10);
    try std.testing.expectEqual(0x42, ppu.oam_data_read());

    ppu.oam_addr_write(0x11);
    try std.testing.expectEqual(0x69, ppu.oam_data_read());
}

test "read status resets vblank" {
    var ppu = PPU.init(&[_]u8{0} ** 2048, Mirroring.HORIZONTAL);
    ppu.status_register.vblank = true;

    const status = ppu.status_read();

    try std.testing.expect(status.vblank);
    try std.testing.expect(!ppu.status_register.vblank);
}

test "OAM DMA" {
    var ppu = PPU.init(&[_]u8{0} ** 2048, Mirroring.HORIZONTAL);
    var data = [_]u8{0x69} ** 256;
    data[0] = 0x42;
    data[255] = 0x13;

    ppu.oam_addr_write(0x10);
    ppu.oam_dma_write(data);

    ppu.oam_addr_write(0xF); // wrap around
    try std.testing.expectEqual(0x13, ppu.oam_data_read());

    ppu.oam_addr_write(0x10);
    try std.testing.expectEqual(0x42, ppu.oam_data_read());

    ppu.oam_data_write(0x11);
    try std.testing.expectEqual(0x69, ppu.oam_data_read());
}
