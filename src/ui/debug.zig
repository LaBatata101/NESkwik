const std = @import("std");

const clay = @import("core/clay.zig");
const c = @import("../root.zig").c;
const UI = @import("core/ui.zig").UI;
const trace = @import("../trace.zig");
const opcodes = @import("../opcodes.zig");
const UIState = @import("gui.zig").UIState;
const Shape = @import("core/widgets.zig").Shape;
const Color = @import("core/color.zig").Color;
const NES_WIDTH = @import("../root.zig").NES_WIDTH;
const NES_HEIGHT = @import("../root.zig").NES_HEIGHT;
const ProcessorStatus = @import("../cpu.zig").ProcessorStatus;
const SYSTEM_PALETTE = @import("../render.zig").SYSTEM_PALETTE;
const PatternTableFrame = @import("../render.zig").PatternTableFrame;

const theme = struct {
    const bg_base = Color.rgb(18, 20, 24);
    const bg_panel = Color.rgb(24, 27, 32);
    const bg_section = Color.rgb(30, 34, 41);
    const bg_hover = Color.rgb(38, 43, 52);
    const bg_active = Color.rgb(44, 100, 220).withAlpha(40);

    const border = Color.rgb(45, 50, 62);
    const border_dim = Color.rgb(35, 40, 50);

    const text_primary = Color.rgb(220, 225, 235); // main text
    const text_secondary = Color.rgb(140, 150, 170); // dim labels
    const text_muted = Color.rgb(80, 90, 110); // very dim
    const text_accent = Color.rgb(97, 175, 255); // blue accent (addresses, keywords)
    const text_value = Color.rgb(152, 221, 130); // green (register values)
    const text_warn = Color.rgb(255, 200, 80); // amber (PC indicator)
    const text_error = Color.rgb(255, 100, 100); // red (error/flags)

    const accent_blue = Color.rgb(60, 130, 240);
    const accent_green = Color.rgb(60, 200, 120);
    const accent_amber = Color.rgb(255, 180, 50);
    const accent_purple = Color.rgb(180, 120, 255);
    const accent_red = Color.rgb(240, 80, 80);
};

const PANEL_GAP: u16 = 1;
const SECTION_PAD: clay.Padding = .{ .left = 12, .right = 12, .top = 10, .bottom = 10 };
const HEADER_PAD: clay.Padding = .{ .left = 12, .right = 12, .top = 7, .bottom = 7 };
const LABEL_FONT: u16 = 18; // section header labels (caps)
const VALUE_FONT: u16 = 16; // register values / data
const CODE_FONT: u16 = 16; // disassembly lines
const SWATCH_SIZE: f32 = 14; // palette colour swatch
const SWATCH_GAP: u16 = 2;

pub fn drawUI(ui: *UI, ui_state: *UIState) void {
    const root = ui.row(.{ .bg_color = theme.bg_base, .sizing = .grow, .gap = 2 });
    {
        const col = ui.column(.{ .child_alignment = .{ .y = .top } });
        {
            drawGameScreen(ui, ui_state);
            drawPatternAndPalettePanel(ui, ui_state);
        }
        col.end();
        drawDebugSidebar(ui, ui_state);
    }
    root.end();
}

fn drawGameScreen(ui: *UI, ui_state: *UIState) void {
    const wrapper = ui.column(.{ .sizing = .{ .w = .grow, .h = .percent(0.677) } });
    _ = ui.canvas(.{
        .pixel_format = c.SDL_PIXELFORMAT_ABGR8888,
        .pixels = ui_state.system.frame_buffer(),
        .w = NES_WIDTH,
        .h = NES_HEIGHT,
    });
    wrapper.end();
}

fn drawDebugSidebar(ui: *UI, ui_state: *UIState) void {
    const sidebar = ui.column(.{
        .sizing = .grow,
        .bg_color = theme.bg_panel,
        .gap = PANEL_GAP,
        .child_alignment = .{ .y = .top },
    });
    {
        drawDisassemblyPanel(ui, ui_state);
        drawRegistersPanel(ui, ui_state);
        drawPPUPanel(ui, ui_state);
    }
    sidebar.end();
}

fn drawSectionHeader(ui: *UI, title: []const u8, accent: Color) void {
    const header = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .bg_color = theme.bg_section,
        .padding = HEADER_PAD,
        .child_alignment = .{ .y = .center },
        .gap = 8,
        .border_width = 1,
        .border_color = theme.border_dim,
    });
    {
        // Coloured left accent bar
        _ = ui.shape(.{
            .vertices = &Shape.SQUARE,
            .sizing = .{ .w = .fixed(3), .h = .fixed(14) },
            .color = accent,
        });

        _ = ui.label(.{
            .text = title,
            .font_size = LABEL_FONT,
            .color = theme.text_secondary,
        });
    }
    header.end();
}

fn drawDisassemblyPanel(ui: *UI, ui_state: *UIState) void {
    const system = ui_state.system;

    drawSectionHeader(ui, "DISASSEMBLY", theme.accent_amber);

    const body = ui.column(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .bg_color = theme.bg_panel,
        .padding = .{ .top = 2, .bottom = 4 },
        .gap = 1,
    });
    {
        var op_size: u8 = 0;
        for (0..12) |i| {
            const is_current = (i == 0); // first entry is always the current PC

            const code = ui_state.rom.prg_rom[(system.cpu.pc + op_size) % ui_state.rom.prg_rom.len];
            const opcode = opcodes.OP_CODES[code];

            const buf = ui.current_window.ctx.frameAlloc().alloc(u8, 23) catch @panic("alloc");
            const addr_buf = ui.current_window.ctx.frameAlloc().alloc(u8, 27) catch @panic("alloc");

            const addr_str = std.fmt.allocPrint(
                ui.current_window.ctx.frameAlloc(),
                "${X:04}",
                .{system.cpu.pc + op_size},
            ) catch @panic("fmt");
            const op_str = trace.format_opcode(
                system.cpu,
                code,
                system.cpu.pc + op_size,
                buf,
            ) catch @panic("fmt");
            const mode_str = trace.format_addressing_mode(
                system.cpu,
                opcode,
                system.cpu.pc + op_size + 1,
                addr_buf,
            ) catch @panic("fmt");

            op_size += opcode.size();

            drawDisassemblyRow(ui, addr_str, op_str, mode_str, is_current);
        }
    }
    body.end();
}

fn drawDisassemblyRow(
    ui: *UI,
    addr: []const u8,
    mnemonic: []const u8,
    operand: []const u8,
    is_current: bool,
) void {
    const bg = if (is_current) theme.bg_active else Color.rgb(0, 0, 0).withAlpha(0);
    const row = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .bg_color = bg,
        .padding = .{ .left = 10, .right = 10, .top = 3, .bottom = 3 },
        .child_alignment = .{ .y = .center },
    });
    {
        // PC arrow indicator (only for the current line)
        if (is_current) {
            _ = ui.shape(.{
                .vertices = &Shape.TRIANGLE,
                .sizing = .{ .w = .fixed(10), .h = .fixed(10) },
                .rotation = 90,
                .color = theme.accent_amber,
            });
            _ = ui.spacer(.{ .sizing = .{ .w = .fixed(5) } });
        } else {
            _ = ui.spacer(.{ .sizing = .{ .w = .fixed(12) } });
        }

        _ = ui.label(.{
            .text = addr,
            .font_size = CODE_FONT,
            .color = if (is_current) theme.text_warn else theme.text_accent,
        });

        _ = ui.spacer(.{ .sizing = .{ .w = .fixed(10) } });

        _ = ui.label(.{
            .text = mnemonic,
            .font_size = CODE_FONT,
            .color = if (is_current) theme.text_primary else theme.text_primary.withAlpha(180),
        });

        if (operand.len > 0) {
            _ = ui.spacer(.{ .sizing = .{ .w = .fixed(6) } });
            _ = ui.label(.{
                .text = operand,
                .font_size = CODE_FONT,
                .color = if (is_current) theme.text_value else theme.text_secondary,
            });
        }
    }
    row.end();
}

fn drawRegistersPanel(ui: *UI, ui_state: *UIState) void {
    const cpu = ui_state.system.cpu;

    drawSectionHeader(ui, "CPU REGISTERS", theme.accent_blue);

    const body = ui.column(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .bg_color = theme.bg_panel,
        .padding = SECTION_PAD,
        .gap = 6,
    });
    {
        const reg_row = ui.row(.{ .sizing = .{ .w = .grow, .h = .fit }, .gap = 8 });
        {
            drawRegisterBadge(
                ui,
                "A",
                std.fmt.allocPrint(ui.current_window.ctx.frameAlloc(), "${X:02}", .{cpu.register_a}) catch @panic("fmt"),
                theme.accent_blue,
            );
            drawRegisterBadge(
                ui,
                "X",
                std.fmt.allocPrint(ui.current_window.ctx.frameAlloc(), "${X:02}", .{cpu.register_x}) catch @panic("fmt"),
                theme.accent_purple,
            );
            drawRegisterBadge(
                ui,
                "Y",
                std.fmt.allocPrint(ui.current_window.ctx.frameAlloc(), "${X:02}", .{cpu.register_y}) catch @panic("fmt"),
                theme.accent_green,
            );
            drawRegisterBadge(
                ui,
                "PC",
                std.fmt.allocPrint(ui.current_window.ctx.frameAlloc(), "${X:04}", .{cpu.pc}) catch @panic("fmt"),
                theme.accent_amber,
            );
            drawRegisterBadge(
                ui,
                "SP",
                std.fmt.allocPrint(ui.current_window.ctx.frameAlloc(), "${X:02}", .{cpu.sp}) catch @panic("fmt"),
                theme.accent_red,
            );
        }
        reg_row.end();

        const flags_row = ui.row(.{
            .sizing = .{ .w = .grow, .h = .fit },
            .gap = 4,
            .child_alignment = .{ .y = .center },
        });
        {
            // _ = ui.label(.{ .text = "FLAGS:", .font_size = LABEL_FONT, .color = theme.text_muted });
            _ = ui.label(.{ .text = "FLAGS:", .font_size = LABEL_FONT, .color = theme.text_secondary });
            _ = ui.spacer(.{ .sizing = .{ .w = .fixed(6) } });
            drawFlagBadge(ui, "N", cpu.status.negative_flag);
            drawFlagBadge(ui, "V", cpu.status.overflow_flag);
            drawFlagBadge(ui, "B", cpu.status.break_command);
            drawFlagBadge(ui, "-", cpu.status.break2); // break2 placeholder
            drawFlagBadge(ui, "I", cpu.status.interrupt_disable);
            drawFlagBadge(ui, "Z", cpu.status.zero_flag);
            drawFlagBadge(ui, "C", cpu.status.carry_flag);
        }
        flags_row.end();
    }
    body.end();
}

fn drawRegisterBadge(ui: *UI, label: []const u8, value: []const u8, accent: Color) void {
    const badge = ui.row(.{
        .sizing = .fit,
        .bg_color = theme.bg_section,
        .border_width = 1,
        .border_color = theme.border,
        .corner_radius = 4,
        .padding = .{ .left = 0, .right = 8, .top = 4, .bottom = 4 },
        .child_alignment = .{ .y = .center },
        .gap = 0,
    });
    {
        _ = ui.shape(.{
            .vertices = &Shape.SQUARE,
            .sizing = .{ .w = .fixed(3), .h = .fixed(22) },
            .color = accent,
        });
        _ = ui.spacer(.{ .sizing = .{ .w = .fixed(7) } });

        // _ = ui.label(.{ .text = label, .font_size = LABEL_FONT, .color = theme.text_muted });
        _ = ui.label(.{ .text = label, .font_size = LABEL_FONT, .color = theme.text_secondary });
        _ = ui.spacer(.{ .sizing = .{ .w = .fixed(5) } });
        _ = ui.label(.{ .text = value, .font_size = VALUE_FONT, .color = theme.text_value });
    }
    badge.end();
}

fn drawFlagBadge(ui: *UI, name: []const u8, active: bool) void {
    const bg = if (active) theme.accent_blue.withAlpha(60) else theme.bg_section;
    const txt = if (active) theme.text_primary else theme.text_muted;
    const bdr = if (active) theme.accent_blue.withAlpha(120) else theme.border_dim;

    const pill = ui.row(.{
        .sizing = .{ .w = .fixed(26), .h = .fixed(22) },
        .bg_color = bg,
        .border_width = 1,
        .border_color = bdr,
        .corner_radius = 3,
        .child_alignment = .center,
    });
    {
        _ = ui.label(.{ .text = name, .font_size = LABEL_FONT, .color = txt });
    }
    pill.end();
}

fn drawPPUPanel(ui: *UI, ui_state: *UIState) void {
    const ppu = ui_state.system.ppu;

    drawSectionHeader(ui, "PPU STATE", theme.accent_purple);

    const body = ui.row(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .bg_color = theme.bg_panel,
        .padding = SECTION_PAD,
        .gap = 6,
        .child_alignment = .{ .y = .center },
    });
    {
        drawStatBadge(ui, "SCANLINE", std.fmt.allocPrint(
            ui.current_window.ctx.frameAlloc(),
            "{}",
            .{ppu.scanline},
        ) catch @panic("fmt"), theme.accent_purple);

        drawStatBadge(ui, "CYCLE", std.fmt.allocPrint(
            ui.current_window.ctx.frameAlloc(),
            "{}",
            .{ppu.cycle},
        ) catch @panic("fmt"), theme.accent_blue);

        drawStatBadge(ui, "GLOBAL", std.fmt.allocPrint(
            ui.current_window.ctx.frameAlloc(),
            "{}",
            .{ppu.global_cycle},
        ) catch @panic("fmt"), theme.accent_green);
    }
    body.end();
}

fn drawStatBadge(ui: *UI, key: []const u8, value: []const u8, accent: Color) void {
    const cell = ui.column(.{
        .sizing = .fit,
        .bg_color = theme.bg_section,
        .border_width = 1,
        .border_color = theme.border,
        .corner_radius = 4,
        .padding = .{ .left = 10, .right = 10, .top = 5, .bottom = 5 },
        .child_alignment = .{ .x = .left, .y = .top },
        .gap = 2,
    });
    {
        _ = ui.label(.{ .text = key, .font_size = LABEL_FONT, .color = accent });
        _ = ui.label(.{ .text = value, .font_size = 16, .color = theme.text_primary });
    }
    cell.end();
}

fn drawPatternAndPalettePanel(ui: *UI, ui_state: *UIState) void {
    const row = ui.row(.{
        .sizing = .grow,
        .bg_color = theme.bg_panel,
        .gap = 10,
    });
    {
        drawPatternTablesPanel(ui, ui_state);
        drawPalettePanel(ui, ui_state);
    }
    row.end();
}

fn drawPatternTablesPanel(ui: *UI, ui_state: *UIState) void {
    const system = ui_state.system;

    const col = ui.column(.{
        .sizing = .{ .w = .fit, .h = .grow },
        .bg_color = theme.bg_panel,
        .gap = PANEL_GAP,
    });
    {
        drawSectionHeader(ui, "PATTERN TABLES", theme.accent_green);

        const body = ui.row(.{
            .sizing = .fit,
            .bg_color = theme.bg_panel,
            .padding = .{ .left = 12, .right = 12, .top = 10, .bottom = 10 },
            .gap = 10,
            .child_alignment = .{ .y = .top },
        });
        {
            const pt1 = ui.current_window.ctx.frameAlloc().create(PatternTableFrame) catch @panic("alloc");
            pt1.* = system.ppu.get_pattern_table(0, 0);

            const pt1_wrapper = ui.column(.{
                .sizing = .fit,
                .bg_color = theme.bg_section,
                .border_width = 1,
                .border_color = theme.border,
                .corner_radius = 3,
                .padding = .{ .left = 4, .right = 4, .top = 4, .bottom = 2 },
                .gap = 4,
                .child_alignment = .{ .x = .center },
            });
            {
                _ = ui.canvas(.{
                    .sizing = .{ .w = .fixed(128), .h = .fixed(128) },
                    .pixel_format = c.SDL_PIXELFORMAT_ABGR8888,
                    .w = 128,
                    .h = 128,
                    .pixels = &pt1.data,
                });
                // _ = ui.label(.{ .text = "CHR $0000", .font_size = LABEL_FONT, .color = theme.text_muted });
                _ = ui.label(.{ .text = "CHR $0000", .font_size = LABEL_FONT, .color = theme.text_secondary });
            }
            pt1_wrapper.end();

            const pt2 = ui.current_window.ctx.frameAlloc().create(PatternTableFrame) catch @panic("alloc");
            pt2.* = system.ppu.get_pattern_table(1, 0);

            const pt2_wrapper = ui.column(.{
                .sizing = .fit,
                .bg_color = theme.bg_section,
                .border_width = 1,
                .border_color = theme.border,
                .corner_radius = 3,
                .padding = .{ .left = 4, .right = 4, .top = 4, .bottom = 2 },
                .gap = 4,
                .child_alignment = .{ .x = .center },
            });
            {
                _ = ui.canvas(.{
                    .sizing = .{ .w = .fixed(128), .h = .fixed(128) },
                    .pixel_format = c.SDL_PIXELFORMAT_ABGR8888,
                    .w = 128,
                    .h = 128,
                    .pixels = &pt2.data,
                });
                // _ = ui.label(.{ .text = "CHR $1000", .font_size = LABEL_FONT, .color = theme.text_muted });
                _ = ui.label(.{ .text = "CHR $1000", .font_size = LABEL_FONT, .color = theme.text_secondary });
            }
            pt2_wrapper.end();
        }
        body.end();
    }
    col.end();
}

fn drawPalettePanel(ui: *UI, ui_state: *UIState) void {
    const palette_ram = ui_state.system.ppu.palette_table;

    const col = ui.column(.{
        .sizing = .grow,
        .bg_color = theme.bg_panel,
        .gap = PANEL_GAP,
    });
    {
        drawSectionHeader(ui, "PALETTE RAM", theme.accent_red);

        const body = ui.row(.{
            .sizing = .{ .w = .grow, .h = .fit },
            .bg_color = theme.bg_panel,
            .padding = .{ .left = 12, .right = 12, .top = 10, .bottom = 10 },
            .gap = 10,
        });
        {
            drawPaletteGroup(ui, palette_ram, "BACKGROUND", false);
            drawPaletteGroup(ui, palette_ram, "SPRITE", true);
        }
        body.end();
    }
    col.end();
}

fn drawPaletteGroup(
    ui: *UI,
    palette_ram: anytype,
    label: []const u8,
    is_sprite: bool,
) void {
    const grp = ui.column(.{
        .sizing = .{ .w = .grow, .h = .fit },
        .bg_color = theme.bg_section,
        .border_width = 1,
        .border_color = theme.border,
        .corner_radius = 4,
        .padding = .{ .left = 8, .right = 8, .top = 6, .bottom = 8 },
        .gap = 6,
    });
    {
        _ = ui.label(.{ .text = label, .font_size = LABEL_FONT, .color = theme.text_secondary });

        // 4 sub-palettes, each 4 colours, separated by a subtle gap
        const palettes_row = ui.column(.{
            .sizing = .{ .w = .grow, .h = .fit },
            .gap = 6,
            .child_alignment = .{ .y = .top },
        });
        {
            for (0..4) |palette_num| {
                const sub = ui.row(.{
                    .sizing = .fit,
                    .bg_color = theme.bg_panel,
                    .border_width = 1,
                    .border_color = theme.border_dim,
                    .corner_radius = 2,
                    .padding = .{ .left = 2, .right = 2, .top = 2, .bottom = 2 },
                    .gap = SWATCH_GAP,
                });
                {
                    // Sub-palette index label
                    _ = ui.label(.{
                        .text = std.fmt.allocPrint(
                            ui.current_window.ctx.frameAlloc(),
                            "{d}",
                            .{palette_num},
                        ) catch @panic("fmt"),
                        .font_size = LABEL_FONT,
                        // .color = theme.text_muted,
                        .color = theme.text_secondary,
                    });

                    // 4 colour swatches stacked vertically per sub-palette
                    for (0..4) |color_num| {
                        const i = (palette_num * 4) + color_num;
                        const color = resolveColor(palette_ram, i, is_sprite);

                        _ = ui.shape(.{
                            .vertices = &Shape.SQUARE,
                            .sizing = .{ .w = .fixed(SWATCH_SIZE), .h = .fixed(SWATCH_SIZE) },
                            .color = color,
                        });
                    }
                }
                sub.end();
            }
        }
        palettes_row.end();
    }
    grp.end();
}

/// Resolves a palette colour from palette RAM, handling sprite mirror addresses.
inline fn resolveColor(palette_ram: anytype, i: usize, is_sprite: bool) Color {
    if (!is_sprite) {
        const idx = palette_ram[i];
        return SYSTEM_PALETTE[idx & 0x3F];
    } else {
        var addr: u8 = 0x10 + @as(u8, @truncate(i));
        addr = switch (addr) {
            0x10, 0x14, 0x18, 0x1C => addr - 0x10,
            else => addr,
        };
        const idx = palette_ram[addr];
        return SYSTEM_PALETTE[idx & 0x3F];
    }
}
