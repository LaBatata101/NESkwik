const std = @import("std");
const c = @import("../../root.zig").c;
const clay = @import("clay.zig");
const Color = @import("color.zig").Color;
pub const ui = @import("ui.zig");
pub const UIContext = ui.UIContext;
const utils = @import("utils.zig");

pub const CustomData = union(enum) {
    canvas: Canvas,
    shape: ShapeData,
};

pub const LayoutDirection = enum {
    row,
    column,
};

pub const Container = struct {
    id: clay.ElementId,
    params: Params,

    pub const Params = struct {
        id: ?[]const u8 = null,
        direction: LayoutDirection = .column,
        padding: clay.Padding = .{},
        gap: u16 = 0,
        sizing: clay.Sizing = .grow,
        bg_color: ?Color = null,
        corner_radius: f32 = 0,
        border_width: u16 = 0,
        border_color: ?Color = null,
        child_alignment: clay.ChildAlignment = .{ .x = .left, .y = .center },
    };
    const Self = @This();

    pub fn start(params: Params) Self {
        const element_id = if (params.id) |id| b: {
            const element_id = clay.ElementId.ID(id);
            clay.openElementWithId(element_id);
            break :b element_id;
        } else clay.openElement();

        const layout_dir: clay.LayoutDirection = if (params.direction == .row) .left_to_right else .top_to_bottom;

        clay.configureOpenElement(.{
            .layout = .{
                .sizing = params.sizing,
                .padding = params.padding,
                .child_gap = params.gap,
                .child_alignment = params.child_alignment,
                .direction = layout_dir,
            },
            .background_color = if (params.bg_color) |color| color.toClay() else clay.Color{ 0, 0, 0, 0 },
            .corner_radius = .all(params.corner_radius),
            .border = if (params.border_color) |border_color| .{
                .color = border_color.toClay(),
                .width = .outside(params.border_width),
            } else .{ .color = .{ 0, 0, 0, 0 } },
        });

        return .{ .id = element_id, .params = params };
    }

    pub fn end(_: *const Self) void {
        clay.closeElement();
    }
};

pub const Label = struct {
    params: Params,

    pub const Params = struct {
        text: []const u8,
        color: Color = Color.black,
        font_size: u16 = 16,
        line_height: ?u16 = null,
        wrap_mode: clay.TextElementConfigWrapMode = .words,
        alignment: clay.TextAlignment = .left,
    };
    const Self = @This();

    pub fn start(params: Params) Self {
        const line_height = params.line_height orelse @as(u16, @intFromFloat(@as(f32, @floatFromInt(params.font_size)) * 1.2));

        const text_config = clay.TextElementConfig{
            .color = params.color.toClay(),
            .font_size = params.font_size,
            .font_id = 0,
            .letter_spacing = 0,
            .line_height = line_height,
            .wrap_mode = params.wrap_mode,
            .alignment = params.alignment,
        };
        _ = clay.textDynamic(params.text, text_config);
        return .{ .params = params };
    }
};

pub const Button = struct {
    id: clay.ElementId,
    params: Params,

    pub const Params = struct {
        id: ?[]const u8 = null,
        text: []const u8,
        on_click: ?*const fn () void = null,
        bg_color: Color = Color.blue,
        hover_color: ?Color = null,
        text_color: Color = Color.white,
        font_size: u16 = 16,
        padding: clay.Padding = .all(8),
        corner_radius: f32 = 8,
        border_width: u16 = 0,
        elevation: u8 = 2,
        border: ?clay.BorderElementConfig = null,
        sizing: clay.Sizing = .fit,
        tooltip: ?struct {
            text: []const u8,
            text_size: u16 = 14,
            color: Color = Color.white,
            wrap_mode: clay.TextElementConfigWrapMode = .words,
        } = null,
    };
    const Self = @This();

    pub fn start(params: Params, ctx: *UIContext) Self {
        const element_id = if (params.id) |id| b: {
            const element_id = clay.ElementId.ID(id);
            clay.openElementWithId(element_id);
            break :b element_id;
        } else clay.openElement();

        const is_hovered = clay.hovered();
        const hover_col = params.hover_color orelse params.bg_color.lighten(0.15);
        const button_color = if (is_hovered) hover_col else params.bg_color;

        clay.configureOpenElement(.{
            .layout = .{ .sizing = params.sizing, .padding = params.padding, .child_alignment = .center },
            .background_color = button_color.toClay(),
            .corner_radius = .all(params.corner_radius),
            .border = if (params.border) |border|
                border
            else if (params.border_width > 0 or params.elevation > 0) .{
                .color = button_color.darken(0.3).toClay(),
                .width = .{
                    .bottom = if (is_hovered) params.elevation / 2 else params.elevation,
                    .left = params.border_width,
                    .right = params.border_width,
                    .top = params.border_width,
                },
            } else .{ .color = .{ 0, 0, 0, 0 } },
        });

        _ = Label.start(.{
            .text = params.text,
            .alignment = .center,
            .font_size = params.font_size,
            .color = params.text_color,
        });
        clay.closeElement();

        if (params.tooltip) |tooltip| {
            if (is_hovered) {
                const state = ctx.getOrCreateWidgetState(element_id, .{ .tooltip = .{
                    .hover_start_ms = c.SDL_GetTicks(),
                } });
                // Draw tooltip if 500ms has passed
                if (ctx.hasPassedSinceMS(state.tooltip.hover_start_ms, 500)) {
                    const tooltip_id = clay.ElementId.localIDI("tooltip", element_id.id);

                    const button_data = clay.getElementData(element_id);
                    const layout_dims = ctx.clay_ctx.layoutDimensions;

                    var attach_parent: clay.FloatingAttachPointType = .left_bottom;
                    var attach_element: clay.FloatingAttachPointType = .left_top;
                    var offset_y: f32 = 4;

                    if (button_data.found) {
                        const is_right_half = button_data.bounding_box.x > (layout_dims.w / 2.0);
                        const is_bottom_half = button_data.bounding_box.y > (layout_dims.h / 2.0);

                        if (is_right_half and is_bottom_half) {
                            attach_parent = .right_top;
                            attach_element = .right_bottom;
                            offset_y = -4; // Reverses the offset to avoid overlapping the button.
                        } else if (is_right_half) {
                            attach_parent = .right_bottom;
                            attach_element = .right_top;
                        } else if (is_bottom_half) {
                            attach_parent = .left_top;
                            attach_element = .left_bottom;
                            offset_y = -4;
                        }
                    }

                    const max_tooltip_width = @min(250.0, layout_dims.w - 32.0);

                    clay.openElementWithId(tooltip_id);
                    clay.configureOpenElement(.{
                        .layout = .{
                            .sizing = .{
                                .w = clay.SizingAxis.fitMinMax(.{ .max = max_tooltip_width }),
                                .h = .fit,
                            },
                            .padding = .{ .left = 8, .right = 8, .top = 5, .bottom = 5 },
                            .child_alignment = .center,
                        },
                        .background_color = Color.rgb(30, 30, 30).toClay(),
                        .corner_radius = .all(8),
                        .border = .{
                            .color = Color.rgb(80, 80, 80).toClay(),
                            .width = .outside(1),
                        },
                        .floating = .{
                            .attach_to = .to_element_with_id,
                            .parentId = element_id.id,
                            .attach_points = .{ .element = attach_element, .parent = attach_parent },
                            .offset = .{ .x = 0, .y = 4 },
                            .z_index = 200,
                            .pointer_capture_mode = .passthrough,
                        },
                    });

                    _ = Label.start(.{
                        .text = tooltip.text,
                        .font_size = tooltip.text_size,
                        .color = tooltip.color,
                        .wrap_mode = tooltip.wrap_mode,
                    });

                    clay.closeElement();
                }
            }
        }

        return .{ .id = element_id, .params = params };
    }

    pub fn clicked(self: *const Self, ctx: *UIContext) bool {
        const is_hovered = clay.pointerOver(self.id);
        return is_hovered and ctx.frame.mouse_pressed;
    }
};

pub const TextField = struct {
    id: clay.ElementId,
    params: Params,

    pub const Params = struct {
        id: ?[]const u8 = null,
        placeholder: []const u8 = "",
        width: clay.SizingAxis = .grow,
        padding_val: clay.Padding = .all(8),
        corner_radius: f32 = 8,
    };
    const Self = @This();

    pub fn start(ctx: *UIContext, params: Params) Self {
        const element_id = if (params.id) |id| b: {
            const element_id = clay.ElementId.ID(id);
            clay.openElementWithId(element_id);
            break :b element_id;
        } else clay.openElement();

        const state = ctx.getOrCreateWidgetState(element_id, .{ .text_input = .{
            .buffer = .empty,
            .cursor_pos = 0,
        } });

        const is_hovered = clay.hovered();
        const is_focused = if (ctx.frame.focused_id) |fid| fid == element_id.id else false;

        ui.setMouseCursorText(is_hovered);

        var placeholder = params.placeholder;
        if (is_hovered and ctx.frame.mouse_pressed) {
            ctx.frame.focused_id = element_id.id;
            placeholder = "";
        } else if (!is_hovered and ctx.frame.mouse_pressed and is_focused) {
            ctx.frame.focused_id = null;
        }

        if (is_focused) {
            if (ctx.frame.text_input.items.len > 0) {
                state.text_input.buffer.appendSlice(ctx.persistent_arena.allocator(), ctx.frame.text_input.items) catch @panic("panic");
            }

            if (ctx.input.isKeyPressed(.BACKSPACE, false) and state.text_input.buffer.items.len > 0) {
                _ = state.text_input.buffer.pop();
            }
        }

        clay.configureOpenElement(.{
            .layout = .{
                .sizing = .{ .w = params.width, .h = .fit },
                .padding = params.padding_val,
                .child_alignment = .{ .y = .center },
            },
            .background_color = Color.white.toClay(),
            .corner_radius = .all(params.corner_radius),
            .border = .{
                .color = if (is_focused) Color.blue.toClay() else Color.lightGray.toClay(),
                .width = .outside(if (is_focused) 2 else 1),
            },
        });

        ctx.setHotId();

        if (!is_focused and state.text_input.buffer.items.len == 0) {
            _ = Label.start(.{ .text = placeholder, .color = Color.gray });
        } else {
            _ = Label.start(.{
                .text = state.text_input.buffer.items,
                .color = Color.black,
                .wrap_mode = .none,
            });
        }

        // Start end text cursor
        const show_cursor = c.SDL_GetTicks() % c.SDL_MS_PER_SECOND < 500; // blink cursor every 500ms
        // When the text field is not focused, end the cursor transparent to avoid layout size change when focusing.
        const cursor_color = if (is_focused and show_cursor) Color.black else Color.black.withAlpha(0);

        _ = clay.openElement();
        clay.configureOpenElement(.{
            .layout = .{
                .sizing = .{ .w = .fixed(2), .h = .fixed(20) },
                .padding = .{ .left = 1 },
            },
            .background_color = cursor_color.toClay(),
        });
        clay.closeElement(); // End end text cursor

        clay.closeElement();
        return .{ .id = element_id, .params = params };
    }
};

pub const ScrollContainer = struct {
    id: clay.ElementId,
    ctx: *UIContext,
    params: Params,

    pub const Params = struct {
        element_id: ?clay.ElementId = null,
        id: ?[]const u8 = null,
        scrollbar_id: ?clay.ElementId = null,
        sizing: clay.Sizing = .grow,
        vertical: bool = true,
        horizontal: bool = false,
        padding: clay.Padding = .{},
        gap: u16 = 0,
        show_scrollbar: bool = true,
    };
    const Self = @This();

    pub fn start(ctx: *UIContext, params: Params) Self {
        const element_id = if (params.element_id) |element_id| b: {
            clay.openElementWithId(element_id);
            break :b element_id;
        } else if (params.id) |id| b: {
            const element_id = clay.ElementId.ID(id);
            clay.openElementWithId(element_id);
            break :b element_id;
        } else clay.openElement();

        ctx.pushParent(element_id);
        const state = ctx.getOrCreateWidgetState(element_id, .{ .scroll = .{
            .offset = .{ .x = 0, .y = 0 },
            .velocity = .{ .x = 0, .y = 0 },
        } });

        var padding = params.padding;
        if (params.show_scrollbar and params.vertical and state.scroll.scrollbar_visible) {
            padding.right = @max(padding.right, 15);
        }

        // Reset for this frame; set to true below if we actually render the scrollbar.
        state.scroll.scrollbar_visible = false;

        clay.configureOpenElement(.{
            .layout = .{
                .sizing = params.sizing,
                .padding = padding,
                .child_gap = params.gap,
                .direction = .top_to_bottom,
            },
            .clip = .{
                .vertical = params.vertical,
                .horizontal = params.horizontal,
                .child_offset = state.scroll.offset,
            },
        });

        if (params.show_scrollbar and params.vertical) {
            const scroll_data = clay.getScrollContainerData(element_id);
            if (scroll_data.found) {
                state.scroll.offset = scroll_data.scroll_position.*;

                if (scroll_data.content_dimensions.h > scroll_data.scroll_container_dimensions.h) {
                    state.scroll.scrollbar_visible = true;

                    const viewport_h = scroll_data.scroll_container_dimensions.h;
                    const content_h = scroll_data.content_dimensions.h;
                    var scroll_bar_height = (viewport_h / content_h) * viewport_h;
                    if (scroll_bar_height < 20.0) scroll_bar_height = 20.0;

                    const max_scroll_y = content_h - viewport_h;
                    const current_scroll_y = @abs(scroll_data.scroll_position.y);
                    const scroll_ratio = if (max_scroll_y > 0) current_scroll_y / max_scroll_y else 0;
                    const scroll_track_space = viewport_h - scroll_bar_height;
                    const scroll_bar_y = scroll_ratio * scroll_track_space;

                    const scrollbar_id = if (params.scrollbar_id) |scrollbar_id|
                        scrollbar_id
                    else
                        clay.ElementId.localIDI("scrollbar", element_id.id);
                    const is_hovered = clay.pointerOver(scrollbar_id);

                    if (is_hovered and ctx.frame.mouse_pressed) {
                        ctx.frame.active_id = scrollbar_id.id;
                    }
                    const is_dragging = (ctx.frame.active_id == scrollbar_id.id);
                    if (is_dragging) {
                        if (ctx.frame.mouse_down) {
                            // Calculate how much content moves per pixel of scrollbar movement
                            // Ratio = (Content Range) / (Scrollbar Track Range)
                            const move_ratio = max_scroll_y / scroll_track_space;

                            // Apply delta. Note:
                            // Mouse Down (+Y) -> Scrollbar Down -> Content moves UP (Negative Y)
                            // So we subtract.
                            scroll_data.scroll_position.y -= ctx.frame.mouse_delta.y * move_ratio;
                        } else {
                            // Mouse released, stop dragging
                            ctx.frame.active_id = null;
                        }
                    }

                    const scrollbar_data = ctx.frameAlloc().create(clay.ElementId) catch @panic("failed to allocate");
                    scrollbar_data.* = scrollbar_id;
                    clay.openElementWithId(scrollbar_id);
                    clay.configureOpenElement(.{
                        .layout = .{ .sizing = .{ .w = .fixed(6), .h = .fixed(scroll_bar_height) } },
                        .floating = .{
                            .attach_to = .to_parent,
                            .attach_points = .{ .element = .right_top, .parent = .right_top },
                            .offset = .{ .x = -2, .y = scroll_bar_y },
                            .z_index = 10,
                        },
                        .background_color = if (is_dragging or is_hovered) .{ 0, 0, 0, 100 } else .{ 0, 0, 0, 50 },
                        .corner_radius = .all(3),
                    });
                    clay.closeElement();
                }
            }
        }

        return .{ .id = element_id, .ctx = ctx, .params = params };
    }

    pub fn end(self: *const Self) void {
        self.ctx.popParent();
        clay.closeElement();
    }
};

pub const Spacer = struct {
    params: Params,

    pub const Params = struct {
        sizing: clay.Sizing,
    };
    const Self = @This();

    pub fn start(params: Params) @This() {
        _ = clay.openElement();
        clay.configureOpenElement(.{ .layout = .{ .sizing = params.sizing } });
        clay.closeElement();
        return .{ .params = params };
    }
};

pub const Canvas = struct {
    id: clay.ElementId,
    params: Params,

    pub const Params = struct {
        id: ?[]const u8 = null,
        sizing: clay.Sizing = .grow,
        pixel_format: c_uint,
        pixels: []const u8,
        w: u32 = 0,
        h: u32 = 0,
        fg_color: ?Color = null,
        bg_color: ?Color = null,
        aspect_ratio: utils.AspectRatio = .none,
    };
    const Self = @This();

    pub fn start(ctx: *UIContext, params: Params) Self {
        const element_id = if (params.id) |id| b: {
            const element_id = clay.ElementId.ID(id);
            clay.openElementWithId(element_id);
            break :b element_id;
        } else clay.openElement();

        const alloc = ctx.frameAlloc();
        const custom_data = alloc.create(CustomData) catch @panic("Alloc failed");
        custom_data.* = .{ .canvas = .{ .id = element_id, .params = params } };

        clay.configureOpenElement(.{
            .layout = .{ .sizing = params.sizing },
            .custom = .{ .custom_data = clay.anytypeToAnyopaquePtr(custom_data) },
        });
        clay.closeElement();

        return .{ .id = element_id, .params = params };
    }
};

pub const MenuBar = struct {
    ctx: *UIContext,
    container: Container,
    params: Params,

    pub const Params = struct {
        bg_color: Color = Color.white,
        border_color: Color = Color.lightGray,
    };
    const Self = @This();

    pub fn start(ctx: *UIContext, params: Params) Self {
        const container = Container.start(.{
            .direction = .row,
            .sizing = .{ .w = .grow, .h = .fit },
            .gap = 4,
            .border_width = 1,
            .border_color = params.border_color,
            .bg_color = params.bg_color,
            .child_alignment = .{ .y = .center },
        });
        ctx.pushParent(container.id);
        return .{ .ctx = ctx, .container = container, .params = params };
    }

    pub fn end(self: *const Self) void {
        self.ctx.popParent();
        self.container.end();
    }
};

pub const DropdownMenu = struct {
    id: clay.ElementId,
    params: Params,
    ctx: *UIContext,

    pub const Params = struct {
        id: ?[]const u8 = null,
        label: []const u8,
        width: f32 = 200,
    };
    const Self = @This();

    pub fn start(ctx: *UIContext, params: Params) Self {
        const element_id = if (params.id) |id| b: {
            const element_id = clay.ElementId.ID(id);
            clay.openElementWithId(element_id);
            break :b element_id;
        } else b: {
            const element_id = clay.ElementId.localIDI(params.label, ctx.getParent().?.id);
            clay.openElementWithId(element_id);
            break :b element_id;
        };

        ctx.pushParent(element_id);
        const state = ctx.getOrCreateWidgetState(element_id, .{ .dropdown_menu = .{ .is_open = false } });

        const is_hovered = clay.hovered();
        const bg = if (state.dropdown_menu.is_open or is_hovered) Color.lightGray else Color.white;

        const menu_list_id = clay.ElementId.localIDI("dropdown_list", element_id.id);
        const is_list_hovered = if (state.dropdown_menu.is_open) clay.pointerOver(menu_list_id) else false;

        clay.configureOpenElement(.{
            .layout = .{ .padding = .all(8) },
            .background_color = bg.toClay(),
            .corner_radius = .all(4),
        });

        _ = Label.start(.{ .text = params.label, .font_size = 14 });
        clay.closeElement();

        if (is_hovered and ctx.frame.mouse_pressed) {
            ctx.frame.active_id = element_id.id;
            state.dropdown_menu.is_open = true;
        }
        // if the menu is open and a mouse click happened outside of it
        else if (state.dropdown_menu.is_open and ctx.frame.mouse_pressed) {
            if (!is_list_hovered) { // if the mouse click didn't happened in the menu item list
                state.dropdown_menu.is_open = false;
                if (ctx.frame.active_id == element_id.id) {
                    ctx.frame.active_id = null;
                }
            }
        }

        if (state.dropdown_menu.is_open) {
            clay.openElementWithId(menu_list_id);
            clay.configureOpenElement(.{
                .layout = .{
                    .sizing = .{ .w = .fixed(params.width), .h = .fit },
                    .direction = .top_to_bottom,
                    .padding = .all(4),
                    .child_gap = 2,
                },
                .background_color = Color.white.toClay(),
                .corner_radius = .all(4),
                .border = .{ .width = .outside(1), .color = Color.gray.toClay() },
                .floating = .{
                    .attach_to = .to_element_with_id,
                    .parentId = element_id.id, // Attach to the button we just drew
                    .attach_points = .{ .element = .left_top, .parent = .left_bottom },
                    .z_index = 1,
                },
            });
        }

        return .{ .id = element_id, .params = params, .ctx = ctx };
    }

    pub fn end(self: *const Self) void {
        self.ctx.popParent();
        const state = self.ctx.getWidgetStateById(self.id).?;
        if (state.dropdown_menu.is_open) {
            clay.closeElement();

            if (self.ctx.frame.menu_item_clicked) {
                state.dropdown_menu.is_open = false;
                self.ctx.frame.active_id = null;
            }
        }
    }
};

pub const MenuItem = struct {
    id: ?clay.ElementId,
    params: Params,

    pub const Params = struct {
        id: ?[]const u8 = null,
        label: []const u8,
        shortcut: ?[]const u8 = null,
        padding: clay.Padding = .{ .left = 8, .right = 8, .top = 6, .bottom = 6 },
    };
    const Self = @This();

    pub fn start(ctx: *UIContext, params: Params) Self {
        if (ctx.getParentState()) |state| {
            // Only end the menu item if the dropdown menu is open.
            if (!state.dropdown_menu.is_open) {
                return .{ .id = null, .params = params };
            }
        }

        const id = if (params.id) |id| b: {
            const element_id = clay.ElementId.ID(id);
            clay.openElementWithId(element_id);
            break :b element_id;
        } else clay.openElement();

        const is_hovered = clay.hovered();
        clay.configureOpenElement(.{
            .layout = .{
                .sizing = .{ .w = .grow, .h = .fit },
                .padding = params.padding,
                .child_alignment = .{ .y = .center },
                .direction = .left_to_right,
            },
            .background_color = if (is_hovered) Color.blue.toClay() else Color.white.toClay(),
            .corner_radius = .all(4),
        });

        const text_col = if (is_hovered) Color.white else Color.black;

        _ = Label.start(.{ .text = params.label, .font_size = 14, .color = text_col });

        if (params.shortcut) |s| {
            _ = Spacer.start(.{ .sizing = .grow });
            _ = Label.start(.{ .text = s, .font_size = 14, .color = if (is_hovered) Color.lightGray else Color.gray });
        }

        clay.closeElement();

        if (is_hovered and ctx.frame.mouse_released) {
            ctx.frame.menu_item_clicked = true;
        }

        return .{ .id = id, .params = .{
            .label = params.label,
            .padding = params.padding,
            .shortcut = params.shortcut,
        } };
    }

    pub fn clicked(self: *const Self, ctx: *UIContext) bool {
        const id = self.id orelse return false;
        const is_hovered = clay.pointerOver(id);
        return is_hovered and ctx.frame.mouse_released;
    }
};

pub const Padding = struct {
    params: Params,

    pub const Params = struct {
        pad: clay.Padding = .{},
        sizing: clay.Sizing = .{ .h = .fit, .w = .grow },
        direction: clay.LayoutDirection = .top_to_bottom,
    };
    const Self = @This();

    pub fn start(params: Params) Self {
        _ = clay.openElement();
        clay.configureOpenElement(.{
            .layout = .{
                .direction = params.direction,
                .padding = params.pad,
                .sizing = params.sizing,
            },
        });
        return .{ .params = params };
    }

    pub fn end(_: *const Self) void {
        clay.closeElement();
    }
};

pub fn Combobox(comptime Option: type) type {
    comptime {
        if (@typeInfo(Option) != .@"enum") {
            @compileError("Combobox options_type must be an enum");
        }
        if (!std.meta.hasFn(Option, "label")) {
            @compileError("Combobox options_type must define `pub fn label(self: @This()) []const u8`");
        }
    }

    return struct {
        id: clay.ElementId,
        ctx: *UIContext,
        params: Params,

        pub const Params = struct {
            id: ?[]const u8 = null,
            options: []const Option = &.{},
            selected: ?Option = null,
            border_color: Color = .black,
            border_color_on_open: Color = .black,
            border_color_on_hover: Color = .black,
            border_width: clay.BorderWidth = .outside(1),
            bg_color: Color = .white,
            bg_color_on_hover: Color = .white,
            child_alignment: clay.ChildAlignment = .{ .y = .center },
            sizing: clay.Sizing = .{ .h = .fit, .w = .fixed(120) },
            text_color: Color = .black,
            float_panel: struct {
                bg_color: Color = .white,
                border_color: Color = .gray,
            } = .{},
            item: struct {
                bg_color: Color = .white,
                bg_color_on_hover: Color = .blue,
                text_color: Color = .black,
                text_color_on_hover: Color = .white,
            } = .{},
        };
        const Self = @This();

        fn optionKey(option: Option) u32 {
            return @intFromEnum(option);
        }

        fn findOptionByKey(options: []const Option, key: u32) Option {
            for (options) |option| {
                if (optionKey(option) == key) return option;
            }
            return options[0];
        }

        pub fn start(ctx: *UIContext, params: Params) Self {
            const element_id = if (params.id) |id| b: {
                const element_id = clay.ElementId.ID(id);
                clay.openElementWithId(element_id);
                break :b element_id;
            } else clay.openElement();

            std.debug.assert(params.options.len > 0);

            ctx.pushParent(element_id);
            const state = ctx.getOrCreateWidgetState(element_id, .{ .combobox = .{
                .is_open = false,
                .selected_key = optionKey(params.selected orelse params.options[0]),
            } });

            const menu_list_id = clay.ElementId.localIDI("combobox_list", element_id.id);
            const scroll_id = clay.ElementId.localIDI("scroll_area", element_id.id);
            const scrollbar_id = clay.ElementId.localIDI("scrollbar", scroll_id.id);

            const is_hovered = clay.hovered();
            const is_list_hovered = if (state.combobox.is_open)
                clay.pointerOver(menu_list_id) or clay.pointerOver(scrollbar_id)
            else
                false;

            {
                const trigger_bg = if (is_hovered or state.combobox.is_open)
                    params.bg_color_on_hover
                else
                    params.bg_color;
                const trigger_border = if (state.combobox.is_open)
                    params.border_color_on_open
                else if (is_hovered)
                    params.border_color_on_hover
                else
                    params.border_color;
                clay.configureOpenElement(.{
                    .layout = .{
                        .padding = .{ .left = 10, .right = 8, .top = 6, .bottom = 6 },
                        .sizing = params.sizing,
                        .direction = .left_to_right,
                        .child_alignment = params.child_alignment,
                        .child_gap = 4,
                    },
                    .background_color = trigger_bg.toClay(),
                    .corner_radius = .all(4),
                    .border = .{ .width = params.border_width, .color = trigger_border.toClay() },
                });

                _ = Label.start(.{
                    .text = findOptionByKey(params.options, state.combobox.selected_key).label(),
                    .font_size = 14,
                    .color = params.text_color,
                });
                _ = Spacer.start(.{ .sizing = .grow });

                _ = Shape.start(ctx, .{
                    .sizing = .{ .w = .fixed(10), .h = .fixed(10) },
                    .vertices = &[_]clay.Vector2{
                        .{ .x = 0.0, .y = 0.75 },
                        .{ .x = 1.0, .y = 0.75 },
                        .{ .x = 0.5, .y = 0.25 },
                    },
                    .rotation = if (state.combobox.is_open) 0 else 180,
                    .color = Color.darkGray,
                });

                clay.closeElement();
            }

            if (is_hovered and ctx.frame.mouse_pressed) {
                ctx.frame.active_id = element_id.id;
                state.combobox.is_open = !state.combobox.is_open;
            } else if (state.combobox.is_open and ctx.frame.mouse_pressed) {
                if (!is_list_hovered) {
                    state.combobox.is_open = false;
                    if (ctx.frame.active_id == element_id.id) {
                        ctx.frame.active_id = null;
                    }
                }
            }

            if (state.combobox.is_open) {
                const combobox_data = clay.getElementData(element_id);
                const layout_dims = ctx.clay_ctx.layoutDimensions;

                const max_height_cap: f32 = 200.0;
                const gap: f32 = 3.0;
                const margin: f32 = 4.0;

                var max_menu_height: f32 = max_height_cap;
                var attach_parent: clay.FloatingAttachPointType = .left_bottom;
                var attach_element: clay.FloatingAttachPointType = .left_top;
                var offset_y: f32 = gap;

                if (combobox_data.found) {
                    const bb = combobox_data.bounding_box;
                    const is_right_half = bb.x > (layout_dims.w / 2.0);
                    const space_below = layout_dims.h - (bb.y + bb.height) - gap - margin;
                    const space_above = bb.y - gap - margin;

                    const open_above = space_above > space_below and space_below < max_height_cap;

                    if (!open_above) {
                        max_menu_height = @min(space_below, max_height_cap);
                        attach_parent = if (is_right_half) .right_bottom else .left_bottom;
                        attach_element = if (is_right_half) .right_top else .left_top;
                        offset_y = gap;
                    } else {
                        max_menu_height = @min(space_above, max_height_cap);
                        attach_parent = if (is_right_half) .right_top else .left_top;
                        attach_element = if (is_right_half) .right_bottom else .left_bottom;
                        offset_y = -gap;
                    }
                }

                const panel_padding_v: f32 = 4.0;
                const scroll_max_h = max_menu_height - panel_padding_v * 2;

                clay.openElementWithId(menu_list_id);
                clay.configureOpenElement(.{
                    .layout = .{
                        .sizing = .{
                            .w = .fixed(combobox_data.bounding_box.width),
                            .h = .fitMinMax(.{ .min = 0, .max = max_menu_height }),
                        },
                        .direction = .top_to_bottom,
                        .padding = .all(@intFromFloat(panel_padding_v)),
                        .child_gap = 2,
                    },
                    .background_color = params.float_panel.bg_color.toClay(),
                    .corner_radius = .all(4),
                    .border = .{ .width = .outside(1), .color = params.float_panel.border_color.toClay() },
                    .floating = .{
                        .attach_to = .to_element_with_id,
                        .parentId = element_id.id,
                        .attach_points = .{ .element = attach_element, .parent = attach_parent },
                        .offset = .{ .x = 0, .y = offset_y },
                        .z_index = 1,
                    },
                });

                const scroll = ScrollContainer.start(ctx, .{
                    .element_id = scroll_id,
                    .scrollbar_id = scrollbar_id,
                    .sizing = .{ .w = .grow, .h = .fitMinMax(.{ .min = 0, .max = scroll_max_h }) },
                    .gap = 2,
                });
                for (params.options) |option| {
                    _ = ComboboxItem.start(ctx, .{
                        .key = optionKey(option),
                        .label = option.label(),
                        .bg_color = params.item.bg_color,
                        .bg_color_on_hover = params.item.bg_color_on_hover,
                        .text_color = params.item.text_color,
                        .text_color_on_hover = params.item.text_color_on_hover,
                    });
                }
                scroll.end();

                clay.closeElement();
            }

            return .{ .id = element_id, .ctx = ctx, .params = params };
        }

        pub fn end(self: *const Self) void {
            self.ctx.popParent();
            clay.closeElement();
        }

        pub fn selected(self: *const Self) Option {
            return findOptionByKey(self.params.options, self.ctx.getWidgetStateById(self.id).?.combobox.selected_key);
        }
    };
}

const ComboboxItem = struct {
    pub const Params = struct {
        key: u32,
        label: []const u8,
        padding: clay.Padding = .{ .left = 10, .right = 10, .top = 5, .bottom = 5 },
        bg_color: Color = Color.white,
        bg_color_on_hover: Color = Color.blue,
        text_color: Color = Color.black,
        text_color_on_hover: Color = Color.white,
    };
    const Self = @This();

    pub fn start(ctx: *UIContext, params: Params) Self {
        _ = clay.openElement();
        const is_hovered = clay.hovered();
        const bg = if (is_hovered)
            params.bg_color_on_hover
        else
            params.bg_color;
        const text_col = if (is_hovered)
            params.text_color_on_hover
        else
            params.text_color;

        clay.configureOpenElement(.{
            .layout = .{
                .sizing = .{ .w = .grow, .h = .fit },
                .padding = params.padding,
                .child_alignment = .{ .y = .center },
                .direction = .left_to_right,
                .child_gap = 6,
            },
            .background_color = bg.toClay(),
            .corner_radius = .all(3),
        });

        _ = Label.start(.{ .text = params.label, .font_size = 14, .color = text_col });

        clay.closeElement();

        if (is_hovered and ctx.frame.mouse_pressed) {
            const combobox_id = ctx.getGrandParent().?;
            const parent = ctx.getWidgetStateById(combobox_id).?;
            parent.combobox.selected_key = params.key;
            parent.combobox.is_open = false;
        }
        return .{};
    }
};

pub const Slider = struct {
    id: clay.ElementId,
    ctx: *UIContext,
    updated_value: f32,

    pub const Params = struct {
        id: []const u8,
        value: f32,
        min: f32,
        max: f32,
        step: ?f32 = null,
        /// Height of the track bar in pixels.
        height: u16 = 4,
        /// Diameter of the circular thumb; also controls the overall widget height.
        thumb_size: u16 = 14,
        fill_color: Color,
        track_color: Color,
        thumb_color: Color,
        corner_radius: f32 = 2,
    };

    const Self = @This();

    pub fn start(ctx: *UIContext, params: Params) Self {
        const element_id = clay.ElementId.ID(params.id);
        clay.openElementWithId(element_id);

        const state = ctx.getOrCreateWidgetState(element_id, .{ .slider = .{ .dragging = false } });
        const is_hovered = clay.pointerOver(element_id);

        if (is_hovered and ctx.frame.mouse_pressed) state.slider.dragging = true;
        if (ctx.frame.mouse_released) state.slider.dragging = false;

        const elem = clay.getElementData(element_id);
        const track_w = if (elem.found and elem.bounding_box.width > 1) elem.bounding_box.width else 200.0;
        const track_x = if (elem.found) elem.bounding_box.x else 0.0;

        // Compute value from the mouse's absolute X position within the track.
        var new_value = params.value;
        if (state.slider.dragging and ctx.frame.mouse_down) {
            const range = params.max - params.min;
            new_value = std.math.clamp(
                params.min + (ctx.mouse_x - track_x) / track_w * range,
                params.min,
                params.max,
            );
            if (params.step) |step| if (step > 0) {
                new_value = @round((new_value - params.min) / step) * step + params.min;
                new_value = std.math.clamp(new_value, params.min, params.max);
            };
        }

        // Full-width track with a proportional left fill and a thumb at the boundary.
        const frac = if (params.max > params.min)
            std.math.clamp((params.value - params.min) / (params.max - params.min), 0.0, 1.0)
        else
            0.0;

        const thumb_d: f32 = @floatFromInt(params.thumb_size);
        // Fill ends half a thumb-diameter before the thumb centre so the thumb sits exactly at the value boundary.
        const fill_w = std.math.clamp(track_w * frac - thumb_d / 2.0, 0.0, track_w - thumb_d);

        const track_h: f32 = @floatFromInt(params.height);

        // Outer container
        clay.configureOpenElement(.{
            .layout = .{
                .sizing = .{ .w = .grow, .h = .fixed(thumb_d) },
                .direction = .left_to_right,
                .child_alignment = .{ .y = .center },
            },
        });

        // Left fill (up to the thumb).
        if (fill_w > 0) {
            _ = clay.openElement();
            clay.configureOpenElement(.{
                .layout = .{ .sizing = .{ .w = .fixed(fill_w), .h = .fixed(track_h) } },
                .background_color = params.fill_color.toClay(),
                .corner_radius = .all(params.corner_radius),
            });
            clay.closeElement();
        }

        // Circular thumb
        _ = clay.openElement();
        clay.configureOpenElement(.{
            .layout = .{ .sizing = .{ .w = .fixed(thumb_d), .h = .fixed(thumb_d) } },
            .background_color = params.thumb_color.toClay(),
            .corner_radius = .all(thumb_d / 2.0),
        });
        clay.closeElement();

        // Right remainder track.
        _ = clay.openElement();
        clay.configureOpenElement(.{
            .layout = .{ .sizing = .{ .w = .grow, .h = .fixed(track_h) } },
            .background_color = params.track_color.toClay(),
            .corner_radius = .all(params.corner_radius),
        });
        clay.closeElement();

        clay.closeElement();
        return .{ .id = element_id, .ctx = ctx, .updated_value = new_value };
    }

    /// Returns the value after applying any drag delta this frame.
    pub fn value(self: *const Self) f32 {
        return self.updated_value;
    }
};

/// An on/off toggle switch rendered as a pill with a sliding circle thumb.
/// Click to flip.  Call `.value()` to read the (potentially updated) state.
pub const Toggle = struct {
    updated_value: bool,

    pub const Params = struct {
        id: []const u8,
        value: bool,
        on_color: Color,
        off_color: Color,
        thumb_color: Color,
        /// Height of the pill (width = height × 2).
        size: u16 = 20,
    };

    const Self = @This();

    pub fn start(ctx: *UIContext, params: Params) Self {
        const element_id = clay.ElementId.ID(params.id);
        clay.openElementWithId(element_id);

        const is_hovered = clay.pointerOver(element_id);
        var new_value = params.value;
        if (is_hovered and ctx.frame.mouse_pressed) new_value = !params.value;

        const h: f32 = @floatFromInt(params.size);
        const w: f32 = h * 2.0;
        const pad: u16 = @intFromFloat(@round(h * 0.15));
        const circle_d: f32 = h - @as(f32, @floatFromInt(pad)) * 2.0;

        const bg_color = if (params.value) params.on_color else params.off_color;

        clay.configureOpenElement(.{
            .layout = .{
                .sizing = .{ .w = .fixed(w), .h = .fixed(h) },
                .direction = .left_to_right,
                .child_alignment = .{ .y = .center },
                .padding = .all(pad),
            },
            .background_color = bg_color.toClay(),
            .corner_radius = .all(h / 2.0),
        });

        // When on, push the thumb to the right side with a spacer first.
        if (params.value) {
            _ = clay.openElement();
            clay.configureOpenElement(.{
                .layout = .{ .sizing = .{ .w = .grow, .h = .fixed(0) } },
            });
            clay.closeElement();
        }

        // Circular thumb.
        _ = clay.openElement();
        clay.configureOpenElement(.{
            .layout = .{ .sizing = .{ .w = .fixed(circle_d), .h = .fixed(circle_d) } },
            .background_color = params.thumb_color.toClay(),
            .corner_radius = .all(circle_d / 2.0),
        });
        clay.closeElement();

        // When off, push the thumb to the left with a trailing spacer.
        if (!params.value) {
            _ = clay.openElement();
            clay.configureOpenElement(.{
                .layout = .{ .sizing = .{ .w = .grow, .h = .fixed(0) } },
            });
            clay.closeElement();
        }

        clay.closeElement();
        return .{ .updated_value = new_value };
    }

    /// Returns the (potentially flipped) value after this frame's click.
    pub fn value(self: *const Self) bool {
        return self.updated_value;
    }
};

pub const ShapeData = struct {
    /// Must have values between 0.0-1.0
    vertices: []const clay.Vector2,
    rotation: f32,
    color: Color,
};

pub const Shape = struct {
    params: Params,

    pub const Params = struct {
        sizing: clay.Sizing = .{ .w = .grow, .h = .grow },
        vertices: []const clay.Vector2,
        rotation: f32 = 0,
        color: Color = Color.black,
    };
    const Self = @This();

    pub const TRIANGLE = [_]clay.Vector2{
        .{ .x = 0.0, .y = 0.75 },
        .{ .x = 1.0, .y = 0.75 },
        .{ .x = 0.5, .y = 0.25 },
    };
    pub const SQUARE = [_]clay.Vector2{
        .{ .x = 0.0, .y = 0.0 },
        .{ .x = 1.0, .y = 0.0 },
        .{ .x = 1.0, .y = 1.0 },
        .{ .x = 0.0, .y = 1.0 },
    };

    pub fn start(ctx: *UIContext, params: Params) Self {
        _ = clay.openElement();

        const custom_data = ctx.frameAlloc().create(CustomData) catch @panic("Alloc failed");
        custom_data.* = .{ .shape = .{
            .vertices = params.vertices,
            .rotation = params.rotation,
            .color = params.color,
        } };

        clay.configureOpenElement(.{
            .layout = .{ .sizing = params.sizing },
            .custom = .{ .custom_data = clay.anytypeToAnyopaquePtr(custom_data) },
        });
        clay.closeElement();

        return .{ .params = params };
    }
};

pub const Separator = struct {
    params: Params,

    pub const Direction = enum {
        horizontal,
        vertical,
    };
    pub const Params = struct {
        direction: Direction = .horizontal,
        thickness: f32 = 1.0,
        color: Color = Color.black,
    };
    const Self = @This();

    pub fn start(params: Params) Self {
        _ = clay.openElement();
        const w: clay.SizingAxis = if (params.direction == .horizontal) .grow else .fixed(params.thickness);
        const h: clay.SizingAxis = if (params.direction == .horizontal) .fixed(params.thickness) else .grow;

        clay.configureOpenElement(.{
            .layout = .{
                .sizing = .{ .w = w, .h = h },
            },
            .background_color = params.color.toClay(),
        });
        clay.closeElement();
        return .{ .params = params };
    }
};
