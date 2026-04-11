const clay = @import("core/clay.zig");
const Color = @import("core/color.zig").Color;

pub const theme = struct {
    pub const bg_base = Color.rgb(18, 20, 24);
    pub const bg_panel = Color.rgb(24, 27, 32);
    pub const bg_section = Color.rgb(30, 34, 41);
    pub const bg_hover = Color.rgb(38, 43, 52);
    pub const bg_active = Color.rgb(44, 100, 220).withAlpha(40);
    pub const bg_selected = Color.rgb(44, 100, 220).withAlpha(60);

    pub const border = Color.rgb(45, 50, 62);
    pub const border_dim = Color.rgb(35, 40, 50);
    pub const border_selected = Color.rgb(60, 130, 240).withAlpha(160);
    pub const border_open = Color.rgb(60, 130, 240).withAlpha(160);

    pub const text_primary = Color.rgb(220, 225, 235); // main text
    pub const text_secondary = Color.rgb(140, 150, 170); // dim labels
    pub const text_muted = Color.rgb(80, 90, 110); // very dim
    pub const text_accent = Color.rgb(97, 175, 255); // blue accent
    pub const text_value = Color.rgb(152, 221, 130); // green
    pub const text_warn = Color.rgb(255, 200, 80); // amber
    pub const text_error = Color.rgb(255, 100, 100); // red

    pub const accent_blue = Color.rgb(60, 130, 240);
    pub const accent_green = Color.rgb(60, 200, 120);
    pub const accent_amber = Color.rgb(255, 180, 50);
    pub const accent_purple = Color.rgb(180, 120, 255);
    pub const accent_red = Color.rgb(240, 80, 80);

    pub const PANEL_GAP: u16 = 1;
    pub const SECTION_PAD: clay.Padding = .{ .left = 12, .right = 12, .top = 10, .bottom = 10 };
    pub const HEADER_PAD: clay.Padding = .{ .left = 12, .right = 12, .top = 7, .bottom = 7 };
    pub const LABEL_FONT: u16 = 18;
    pub const VALUE_FONT: u16 = 16;
    pub const CODE_FONT: u16 = 16;
    pub const SWATCH_SIZE: f32 = 14;
    pub const SWATCH_GAP: u16 = 2;
};
