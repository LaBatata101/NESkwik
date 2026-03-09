const std = @import("std");
const builtin = @import("builtin");

const c = @import("../../root.zig").c;
const clay = @import("clay.zig");
pub const widgets = @import("widgets.zig");
const sdlError = @import("../../utils/sdl.zig").sdlError;
const vulkan = @import("../../root.zig").vulkan;
const FPSManager = @import("../../render.zig").FPSManager;
const shaders = @import("shaders.zig");
const utils = @import("utils.zig");

const PIXELOID_FONT = @embedFile("pixeloid_font");

fn handleClayError(error_data: clay.ErrorData) callconv(.c) void {
    std.debug.print("Clay Error: {s}\n", .{error_data.error_text.chars[0..@intCast(error_data.error_text.length)]});
}

fn measureText(
    text: []const u8,
    config: *clay.TextElementConfig,
    user_data: *c.TTF_Font,
) clay.Dimensions {
    const font = user_data;
    sdlError(c.TTF_SetFontSize(font, @floatFromInt(config.font_size)));

    var w: c_int = 0;
    var h: c_int = 0;
    sdlError(c.TTF_GetStringSize(font, text.ptr, text.len, &w, &h));

    return clay.Dimensions{
        .w = @floatFromInt(w),
        .h = @floatFromInt(h),
    };
}

pub fn setMouseCursorText(value: bool) void {
    if (value) {
        const text_cursor = sdlError(c.SDL_CreateSystemCursor(c.SDL_SYSTEM_CURSOR_TEXT));
        sdlError(c.SDL_SetCursor(text_cursor));
    } else {
        sdlError(c.SDL_SetCursor(c.SDL_GetDefaultCursor()));
    }
}

const TextCacheItem = struct {
    texture: ?*c.SDL_GPUTexture,
    width: f32,
    height: f32,
};

const CanvasCacheItem = struct {
    texture: ?*c.SDL_GPUTexture,
    width: u32,
    height: u32,
};

pub const UIContext = struct {
    /// The time between the current and previous frame.
    dt: f32,
    clay_ctx: *clay.Context,

    input: InputContext,

    // Frame-ephemeral data (reset each frame)
    frame: FrameState,
    parent_stack: std.ArrayList(clay.ElementId),

    // Persistent widget state (survives across frames)
    widgets: WidgetStateMap,

    text_cache: std.AutoArrayHashMap(u32, TextCacheItem),
    canvas_cache: std.AutoArrayHashMap(u32, CanvasCacheItem),

    mouse_x: f32 = 0,
    mouse_y: f32 = 0,
    prev_mouse_x: f32 = 0,
    prev_mouse_y: f32 = 0,

    frame_arena: std.heap.ArenaAllocator,
    persistent_arena: std.heap.ArenaAllocator,
    clay_memory: []u8,

    const Self = @This();

    pub fn init(allocator: std.mem.Allocator, layout_dimensions: clay.Dimensions, error_handler: clay.ErrorHandler) !*Self {
        var ctx = try allocator.create(UIContext);

        const clay_mem_size = clay.minMemorySize();
        const clay_memory = try allocator.alloc(u8, clay_mem_size);
        const arena = clay.createArenaWithCapacityAndMemory(clay_memory);
        ctx.clay_ctx = clay.initialize(arena, layout_dimensions, error_handler);
        ctx.clay_memory = clay_memory;

        ctx.frame_arena = std.heap.ArenaAllocator.init(allocator);
        ctx.persistent_arena = std.heap.ArenaAllocator.init(allocator);

        const persistent_arena_alloc = ctx.persistent_arena.allocator();

        ctx.parent_stack = .empty;
        ctx.frame = .{
            .text_input = .empty,
            .scroll = .{},
        };

        ctx.dt = 0;
        ctx.widgets = WidgetStateMap.init(persistent_arena_alloc);
        ctx.text_cache = std.AutoArrayHashMap(u32, TextCacheItem).init(persistent_arena_alloc);
        ctx.canvas_cache = std.AutoArrayHashMap(u32, CanvasCacheItem).init(persistent_arena_alloc);

        ctx.input = InputContext.init();

        return ctx;
    }

    pub fn deinit(self: *Self, allocator: std.mem.Allocator, device: ?*c.SDL_GPUDevice) void {
        var iterator = self.text_cache.iterator();
        while (iterator.next()) |cached_text| {
            c.SDL_ReleaseGPUTexture(device, cached_text.value_ptr.texture);
        }
        self.text_cache.deinit();

        var canvas_iter = self.canvas_cache.iterator();
        while (canvas_iter.next()) |item| {
            c.SDL_ReleaseGPUTexture(device, item.value_ptr.texture);
        }
        self.canvas_cache.deinit();

        self.frame_arena.deinit();
        self.persistent_arena.deinit();
        allocator.free(self.clay_memory);
        allocator.destroy(self);
    }

    pub fn update(self: *Self) void {
        self.input.update(self.dt);

        const scroll_smoothing: f32 = 5.0; // Higher = faster decay (try 5.0-15.0)
        const scroll_threshold: f32 = 0.1; // Stop animating below this velocity
        // Apply velocity to scroll deltas for smooth animation
        if (@abs(self.frame.scroll.velocity_x) > scroll_threshold or @abs(self.frame.scroll.velocity_y) > scroll_threshold) {
            self.frame.scroll.delta_x = self.frame.scroll.velocity_x * self.dt * 60.0;
            self.frame.scroll.delta_y = self.frame.scroll.velocity_y * self.dt * 60.0;

            const decay = @exp(-scroll_smoothing * self.dt);
            self.frame.scroll.velocity_x *= decay;
            self.frame.scroll.velocity_y *= decay;
        } else {
            // Stop scrolling when velocity is negligible
            self.frame.scroll.velocity_x = 0;
            self.frame.scroll.velocity_y = 0;
            self.frame.scroll.delta_x = 0;
            self.frame.scroll.delta_y = 0;
        }

        clay.setCurrentContext(self.clay_ctx);
        clay.setPointerState(
            .{ .x = self.mouse_x, .y = self.mouse_y },
            self.frame.mouse_down,
        );
        clay.updateScrollContainers(false, .{
            .x = self.frame.scroll.delta_x,
            .y = self.frame.scroll.delta_y,
        }, self.dt);
    }

    pub fn reset(self: *Self) void {
        _ = self.frame_arena.reset(.retain_capacity);

        self.parent_stack = .empty;
        self.frame.text_input = .empty;

        self.frame.mouse_pressed = false;
        self.frame.mouse_released = false;
        self.frame.mouse_delta = .{ .x = 0, .y = 0 };

        self.frame.menu_item_clicked = false;
        self.frame.hot_id = null;
    }

    pub fn frameAlloc(self: *Self) std.mem.Allocator {
        return self.frame_arena.allocator();
    }

    pub fn pushParent(self: *Self, id: clay.ElementId) void {
        self.parent_stack.append(self.frameAlloc(), id) catch
            @panic("Failed to allocate memory");
    }

    /// Pops the active parent (call this when the container closes/ends)
    pub fn popParent(self: *Self) void {
        _ = self.parent_stack.pop();
    }

    // TODO: find a better way of getting parents other than the immediate parent.
    /// Returns the ID of the immediate parent, or null if root
    pub fn getParent(self: *const Self) ?clay.ElementId {
        if (self.parent_stack.items.len == 0) return null;
        return self.parent_stack.items[self.parent_stack.items.len - 1];
    }

    /// Returns the ID of the grand parent, or null if root
    pub fn getGrandParent(self: *const Self) ?clay.ElementId {
        if (self.parent_stack.items.len < 2) return null;
        return self.parent_stack.items[self.parent_stack.items.len - 2];
    }

    pub fn getParentState(self: *const Self) ?*WidgetState {
        const parent_id = self.getParent() orelse return null;
        return self.getWidgetStateById(parent_id);
    }

    pub fn getWidgetStateById(self: *const Self, id: clay.ElementId) ?*WidgetState {
        return self.widgets.getPtr(id.id);
    }

    pub fn getOrCreateWidgetState(self: *Self, id: clay.ElementId, default: WidgetState) *WidgetState {
        const result = self.widgets.getOrPut(id.id) catch @panic("Failed to allocate!");
        if (!result.found_existing) {
            result.value_ptr.* = default;
        }
        return result.value_ptr;
    }

    pub fn setHotId(self: *Self) void {
        clay.onHover(*Self, self, struct {
            fn callback(element_id: clay.ElementId, _: clay.PointerData, ctx: *Self) void {
                ctx.frame.hot_id = element_id.id;
            }
        }.callback);
    }

    /// Check if the time in milliseconds (`ms`) has passed since the `start`.
    pub fn hasPassedSinceMS(_: *const Self, start: u64, ms: u64) bool {
        return c.SDL_GetTicks() - start >= ms;
    }

    fn allocWidget(self: *Self, T: type, value: T) *T {
        const w = self.frameAlloc().create(T) catch
            std.debug.panic("Failed to allocate widget: {s}", .{@typeName(T)});
        w.* = value;
        return w;
    }

    fn updateMousePos(self: *Self, motion: c.SDL_MouseMotionEvent) void {
        self.prev_mouse_x = self.mouse_x;
        self.prev_mouse_y = self.mouse_y;
        self.mouse_x = motion.x;
        self.mouse_y = motion.y;

        self.frame.mouse_delta.x += motion.xrel;
        self.frame.mouse_delta.y += motion.yrel;
    }

    fn mouseMotion(self: *const Self) bool {
        return self.mouse_x != self.prev_mouse_x and self.mouse_y != self.prev_mouse_y;
    }
};

pub const FrameState = struct {
    text_input: std.ArrayList(u8),
    /// True only on the frame the button was clicked
    mouse_pressed: bool = false,
    /// True only on the frame the button was released
    mouse_released: bool = false,
    /// True as long as the button is held
    mouse_down: bool = false,
    mouse_delta: clay.Vector2 = .{ .x = 0, .y = 0 },

    scroll: struct {
        delta_x: f32 = 0,
        delta_y: f32 = 0,
        velocity_x: f32 = 0,
        velocity_y: f32 = 0,
    },

    menu_item_clicked: bool = false,

    hot_id: ?u32 = null, // Widget under cursor
    active_id: ?u32 = null, // Widget being interacted with
    focused_id: ?u32 = null, // Widget with keyboard focus
};

pub const WidgetStateMap = std.AutoHashMap(u32, WidgetState);

pub const WidgetState = union(enum) {
    text_input: TextInputState,
    scroll: ScrollState,
    dropdown_menu: DropdownMenuState,
    combobox: ComboboxState,
    tooltip: TooltipState,

    pub const TextInputState = struct {
        buffer: std.ArrayList(u8),
        cursor_pos: usize,
    };
    pub const ScrollState = struct {
        offset: clay.Vector2,
        velocity: clay.Vector2,
    };
    pub const DropdownMenuState = struct {
        is_open: bool,
    };
    pub const ComboboxState = struct {
        is_open: bool,
        selected_option: []const u8,
    };
    pub const TooltipState = struct {
        hover_start_ms: u64,
    };
};

const DrawCall = struct {
    texture: ?*c.SDL_GPUTexture,
    index_offset: u32,
    index_count: u32,
    clip_rect: ?c.SDL_Rect,
};

pub const Key = enum(u16) {
    UNKNOWN = 0,

    A = 4,
    B = 5,
    C = 6,
    D = 7,
    E = 8,
    F = 9,
    G = 10,
    H = 11,
    I = 12,
    J = 13,
    K = 14,
    L = 15,
    M = 16,
    N = 17,
    O = 18,
    P = 19,
    Q = 20,
    R = 21,
    S = 22,
    T = 23,
    U = 24,
    V = 25,
    W = 26,
    X = 27,
    Y = 28,
    Z = 29,

    N_1 = 30,
    N_2 = 31,
    N_3 = 32,
    N_4 = 33,
    N_5 = 34,
    N_6 = 35,
    N_7 = 36,
    N_8 = 37,
    N_9 = 38,
    N_0 = 39,

    RETURN = 40,
    ESCAPE = 41,
    BACKSPACE = 42,
    TAB = 43,
    SPACE = 44,

    MINUS = 45,
    EQUALS = 46,
    LEFTBRACKET = 47,
    RIGHTBRACKET = 48,
    BACKSLASH = 49,
    NONUSHASH = 50,
    SEMICOLON = 51,
    APOSTROPHE = 52,
    GRAVE = 53,
    COMMA = 54,
    PERIOD = 55,
    SLASH = 56,

    CAPSLOCK = 57,

    F1 = 58,
    F2 = 59,
    F3 = 60,
    F4 = 61,
    F5 = 62,
    F6 = 63,
    F7 = 64,
    F8 = 65,
    F9 = 66,
    F10 = 67,
    F11 = 68,
    F12 = 69,

    PRINTSCREEN = 70,
    SCROLLLOCK = 71,
    PAUSE = 72,
    INSERT = 73,
    HOME = 74,
    PAGEUP = 75,
    DELETE = 76,
    END = 77,
    PAGEDOWN = 78,
    RIGHT = 79,
    LEFT = 80,
    DOWN = 81,
    UP = 82,

    NUMLOCKCLEAR = 83,
    KP_DIVIDE = 84,
    KP_MULTIPLY = 85,
    KP_MINUS = 86,
    KP_PLUS = 87,
    KP_ENTER = 88,
    KP_1 = 89,
    KP_2 = 90,
    KP_3 = 91,
    KP_4 = 92,
    KP_5 = 93,
    KP_6 = 94,
    KP_7 = 95,
    KP_8 = 96,
    KP_9 = 97,
    KP_0 = 98,
    KP_PERIOD = 99,

    NONUSBACKSLASH = 100,
    APPLICATION = 101,
    POWER = 102,
    KP_EQUALS = 103,
    F13 = 104,
    F14 = 105,
    F15 = 106,
    F16 = 107,
    F17 = 108,
    F18 = 109,
    F19 = 110,
    F20 = 111,
    F21 = 112,
    F22 = 113,
    F23 = 114,
    F24 = 115,
    EXECUTE = 116,
    HELP = 117,
    MENU = 118,
    SELECT = 119,
    STOP = 120,
    AGAIN = 121,
    UNDO = 122,
    CUT = 123,
    COPY = 124,
    PASTE = 125,
    FIND = 126,
    MUTE = 127,
    VOLUMEUP = 128,
    VOLUMEDOWN = 129,
    KP_COMMA = 133,
    KP_EQUALSAS400 = 134,

    INTERNATIONAL1 = 135,
    INTERNATIONAL2 = 136,
    INTERNATIONAL3 = 137,
    INTERNATIONAL4 = 138,
    INTERNATIONAL5 = 139,
    INTERNATIONAL6 = 140,
    INTERNATIONAL7 = 141,
    INTERNATIONAL8 = 142,
    INTERNATIONAL9 = 143,
    LANG1 = 144,
    LANG2 = 145,
    LANG3 = 146,
    LANG4 = 147,
    LANG5 = 148,
    LANG6 = 149,
    LANG7 = 150,
    LANG8 = 151,
    LANG9 = 152,

    ALTERASE = 153,
    SYSREQ = 154,
    CANCEL = 155,
    CLEAR = 156,
    PRIOR = 157,
    RETURN2 = 158,
    SEPARATOR = 159,
    OUT = 160,
    OPER = 161,
    CLEARAGAIN = 162,
    CRSEL = 163,
    EXSEL = 164,

    KP_00 = 176,
    KP_000 = 177,
    THOUSANDSSEPARATOR = 178,
    DECIMALSEPARATOR = 179,
    CURRENCYUNIT = 180,
    CURRENCYSUBUNIT = 181,
    KP_LEFTPAREN = 182,
    KP_RIGHTPAREN = 183,
    KP_LEFTBRACE = 184,
    KP_RIGHTBRACE = 185,
    KP_TAB = 186,
    KP_BACKSPACE = 187,
    KP_A = 188,
    KP_B = 189,
    KP_C = 190,
    KP_D = 191,
    KP_E = 192,
    KP_F = 193,
    KP_XOR = 194,
    KP_POWER = 195,
    KP_PERCENT = 196,
    KP_LESS = 197,
    KP_GREATER = 198,
    KP_AMPERSAND = 199,
    KP_DBLAMPERSAND = 200,
    KP_VERTICALBAR = 201,
    KP_DBLVERTICALBAR = 202,
    KP_COLON = 203,
    KP_HASH = 204,
    KP_SPACE = 205,
    KP_AT = 206,
    KP_EXCLAM = 207,
    KP_MEMSTORE = 208,
    KP_MEMRECALL = 209,
    KP_MEMCLEAR = 210,
    KP_MEMADD = 211,
    KP_MEMSUBTRACT = 212,
    KP_MEMMULTIPLY = 213,
    KP_MEMDIVIDE = 214,
    KP_PLUSMINUS = 215,
    KP_CLEAR = 216,
    KP_CLEARENTRY = 217,
    KP_BINARY = 218,
    KP_OCTAL = 219,
    KP_DECIMAL = 220,
    KP_HEXADECIMAL = 221,

    LCTRL = 224,
    LSHIFT = 225,
    LALT = 226,
    LGUI = 227,
    RCTRL = 228,
    RSHIFT = 229,
    RALT = 230,
    RGUI = 231,

    MODE = 257,

    SLEEP = 258,
    WAKE = 259,

    CHANNEL_INCREMENT = 260,
    CHANNEL_DECREMENT = 261,

    MEDIA_PLAY = 262,
    MEDIA_PAUSE = 263,
    MEDIA_RECORD = 264,
    MEDIA_FAST_FORWARD = 265,
    MEDIA_REWIND = 266,
    MEDIA_NEXT_TRACK = 267,
    MEDIA_PREVIOUS_TRACK = 268,
    MEDIA_STOP = 269,
    MEDIA_EJECT = 270,
    MEDIA_PLAY_PAUSE = 271,
    MEDIA_SELECT = 272,

    AC_NEW = 273,
    AC_OPEN = 274,
    AC_CLOSE = 275,
    AC_EXIT = 276,
    AC_SAVE = 277,
    AC_PRINT = 278,
    AC_PROPERTIES = 279,

    AC_SEARCH = 280,
    AC_HOME = 281,
    AC_BACK = 282,
    AC_FORWARD = 283,
    AC_STOP = 284,
    AC_REFRESH = 285,
    AC_BOOKMARKS = 286,

    SOFTLEFT = 287,
    SOFTRIGHT = 288,
    CALL = 289,
    ENDCALL = 290,

    RESERVED = 400,

    _,

    pub fn count() u16 {
        return 512;
    }
};

const KeyData = struct {
    event: KeyEventType,
    duration: f32,
    duration_prev: f32,

    const default_state: KeyData = .{
        .event = .none,
        .duration = -1.0,
        .duration_prev = -1.0,
    };
};

const KeyEventType = enum {
    down,
    released,
    none,
};

pub const InputContext = struct {
    keys: [Key.count()]KeyData,

    key_repeat_delay: f32 = 0.250, // 250ms
    key_repeat_rate: f32 = 0.050, // 50ms

    const KeyEvent = struct {
        event: KeyEventType,
        scancode: c.SDL_Scancode,
    };

    pub fn init() InputContext {
        var ctx = InputContext{
            .keys = undefined,
        };
        @memset(&ctx.keys, KeyData.default_state);
        return ctx;
    }

    pub fn addKeyEvent(self: *InputContext, event: InputContext.KeyEvent) void {
        const idx = event.scancode;
        if (idx >= 0 and idx < self.keys.len) {
            self.keys[idx].event = event.event;
        }
    }

    pub fn update(self: *InputContext, dt: f32) void {
        for (&self.keys) |*key| {
            key.duration_prev = key.duration;

            if (key.event == .down) {
                if (key.duration < 0.0) {
                    key.duration = 0.0;
                } else {
                    key.duration += dt;
                }
            } else {
                key.duration = -1.0;
            }
        }
    }

    pub fn isKeyPressed(self: *const InputContext, key: Key, repeat: bool) bool {
        const key_data = self.keys[@intFromEnum(key)];

        if (key_data.event == .released) return false;

        const t = key_data.duration;
        if (t == 0.0) return true;

        if (repeat and t > self.key_repeat_delay) {
            const t_prev = key_data.duration_prev;
            const delay = self.key_repeat_delay;
            const rate = self.key_repeat_rate;

            const count_prev = if (t_prev > delay) @divFloor((t_prev - delay), rate) else -1.0;
            const count_curr = if (t > delay) @divFloor((t - delay), rate) else -1.0;

            if (count_curr > count_prev) {
                return true;
            }
        }

        return false;
    }

    pub fn getPressedKey(self: *const InputContext) ?Key {
        for (self.keys, 0..) |key, i| { // TODO: find a way where we don't have to iterate all the keys
            if (key.event == .down and key.duration == 0.0) {
                return @enumFromInt(i);
            }
        }
        return null;
    }

    pub fn getReleasedKey(self: *const InputContext) ?Key {
        for (self.keys, 0..) |key, i| { // TODO: find a way where we don't have to iterate all the keys
            if (key.event == .released and key.duration_prev >= 0.0) {
                return @enumFromInt(i);
            }
        }
        return null;
    }
};

pub const Renderer = struct {
    allocator: std.mem.Allocator,
    device: ?*c.SDL_GPUDevice,
    pipeline: ?*c.SDL_GPUGraphicsPipeline,
    sampler: ?*c.SDL_GPUSampler,

    vertices: std.ArrayList(c.SDL_Vertex),
    indices: std.ArrayList(u32),
    draw_calls: std.ArrayList(DrawCall),

    vertex_buffer: ?*c.SDL_GPUBuffer = null,
    index_buffer: ?*c.SDL_GPUBuffer = null,
    vertex_buffer_capacity: u32 = 0,
    index_buffer_capacity: u32 = 0,

    current_texture: ?*c.SDL_GPUTexture = null,
    current_clip: ?c.SDL_Rect = null,

    // Default 1x1 white texture for solid color rendering
    white_texture: ?*c.SDL_GPUTexture,

    const Self = @This();

    fn createSDLShaderFromSpirv(
        device: ?*c.SDL_GPUDevice,
        spirv: []const u8,
        stage: c.SDL_GPUShaderStage,
        num_samplers: u32,
        num_uniform_buffers: u32,
    ) !*c.SDL_GPUShader {
        return c.SDL_CreateGPUShader(device, &.{
            .code_size = spirv.len,
            .code = @ptrCast(spirv),
            .entrypoint = "main",
            .format = c.SDL_GPU_SHADERFORMAT_SPIRV,
            .stage = stage,
            .num_samplers = num_samplers,
            .num_uniform_buffers = num_uniform_buffers,
        }) orelse error.ShaderCreateFailed;
    }

    fn init(allocator: std.mem.Allocator, device: ?*c.SDL_GPUDevice, window: ?*c.SDL_Window, vk_version: c_uint) !*Self {
        var self = try allocator.create(Self);
        self.allocator = allocator;
        self.device = device;
        self.vertices = .empty;
        self.indices = .empty;

        sdlError(c.SDL_ClaimWindowForGPUDevice(device, window));

        var shader_compiler = try shaders.ShaderCompiler.init(allocator);
        defer shader_compiler.deinit();

        try shader_compiler.addShader(shaders.VERT, .Vertex);
        try shader_compiler.addShader(shaders.FRAG, .Fragment);

        const result = try shader_compiler.compile(vk_version);
        defer {
            for (result) |spirv| {
                allocator.free(spirv.bytes);
            }
            allocator.free(result);
        }

        var vert_spirv: []const u8 = undefined;
        var frag_spirv: []const u8 = undefined;
        for (result) |spirv| {
            switch (spirv.stage) {
                .Vertex => vert_spirv = spirv.bytes,
                .Fragment => frag_spirv = spirv.bytes,
            }
        }

        const vert = try createSDLShaderFromSpirv(device, vert_spirv, c.SDL_GPU_SHADERSTAGE_VERTEX, 0, 1);
        const frag = try createSDLShaderFromSpirv(device, frag_spirv, c.SDL_GPU_SHADERSTAGE_FRAGMENT, 1, 0);
        defer {
            c.SDL_ReleaseGPUShader(device, vert);
            c.SDL_ReleaseGPUShader(device, frag);
        }

        const color_desc = c.SDL_GPUColorTargetDescription{
            .format = c.SDL_GetGPUSwapchainTextureFormat(device, window),
            .blend_state = .{
                .src_color_blendfactor = c.SDL_GPU_BLENDFACTOR_SRC_ALPHA,
                .dst_color_blendfactor = c.SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA,
                .color_blend_op = c.SDL_GPU_BLENDOP_ADD,
                .src_alpha_blendfactor = c.SDL_GPU_BLENDFACTOR_SRC_ALPHA,
                .dst_alpha_blendfactor = c.SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA,
                .alpha_blend_op = c.SDL_GPU_BLENDOP_ADD,
                .enable_blend = true,
            },
        };

        const pixels = [_]u8{ 255, 255, 255, 255 };
        const transfer_buffer = c.SDL_CreateGPUTransferBuffer(
            device,
            &.{ .usage = c.SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD, .size = 4 },
        );
        const map: [*]u8 = @ptrCast(c.SDL_MapGPUTransferBuffer(device, transfer_buffer, false));
        @memcpy(map[0..4], &pixels);
        c.SDL_UnmapGPUTransferBuffer(device, transfer_buffer);

        var vertex_attrs = [_]c.SDL_GPUVertexAttribute{
            .{ .location = 0, .buffer_slot = 0, .format = c.SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2, .offset = 0 }, // Pos
            .{ .location = 1, .buffer_slot = 0, .format = c.SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4, .offset = 8 }, // Color
            .{ .location = 2, .buffer_slot = 0, .format = c.SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2, .offset = 24 }, // UV
        };
        const vertex_bindings = [_]c.SDL_GPUVertexBufferDescription{.{
            .slot = 0,
            .pitch = @sizeOf(c.SDL_Vertex),
            .input_rate = c.SDL_GPU_VERTEXINPUTRATE_VERTEX,
            .instance_step_rate = 0,
        }};
        self.* = .{
            .allocator = allocator,
            .device = device,
            .vertices = .empty,
            .indices = .empty,
            .draw_calls = .empty,
            .current_texture = self.white_texture,
            .pipeline = sdlError(c.SDL_CreateGPUGraphicsPipeline(device, &.{
                .vertex_shader = vert,
                .fragment_shader = frag,
                .vertex_input_state = .{
                    .vertex_buffer_descriptions = &vertex_bindings,
                    .num_vertex_buffers = 1,
                    .vertex_attributes = &vertex_attrs,
                    .num_vertex_attributes = vertex_attrs.len,
                },
                .primitive_type = c.SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
                .target_info = .{ .color_target_descriptions = &color_desc, .num_color_targets = 1 },
            })),
            .sampler = sdlError(c.SDL_CreateGPUSampler(device, &.{
                .min_filter = c.SDL_GPU_FILTER_NEAREST,
                .mag_filter = c.SDL_GPU_FILTER_NEAREST,
                .mipmap_mode = c.SDL_GPU_SAMPLERMIPMAPMODE_NEAREST,
                .address_mode_u = c.SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
                .address_mode_v = c.SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
            })),
            .white_texture = sdlError(c.SDL_CreateGPUTexture(device, &.{
                .type = c.SDL_GPU_TEXTURETYPE_2D,
                .format = c.SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM,
                .width = 1,
                .height = 1,
                .layer_count_or_depth = 1,
                .num_levels = 1,
                .usage = c.SDL_GPU_TEXTUREUSAGE_SAMPLER,
            })),
            .vertex_buffer = c.SDL_CreateGPUBuffer(
                device,
                &.{ .usage = c.SDL_GPU_BUFFERUSAGE_VERTEX, .size = 4 },
            ),
            .index_buffer = c.SDL_CreateGPUBuffer(
                device,
                &.{ .usage = c.SDL_GPU_BUFFERUSAGE_INDEX, .size = 4 },
            ),
        };

        const cmd_buf = c.SDL_AcquireGPUCommandBuffer(device);
        const copy_pass = c.SDL_BeginGPUCopyPass(cmd_buf);
        var src = c.SDL_GPUTextureTransferInfo{ .transfer_buffer = transfer_buffer };
        var dst = c.SDL_GPUTextureRegion{ .texture = self.white_texture, .w = 1, .h = 1, .d = 1 };
        c.SDL_UploadToGPUTexture(copy_pass, &src, &dst, false);
        c.SDL_EndGPUCopyPass(copy_pass);
        sdlError(c.SDL_SubmitGPUCommandBuffer(cmd_buf));
        c.SDL_ReleaseGPUTransferBuffer(device, transfer_buffer);

        return self;
    }

    fn deinit(self: *Self) void {
        c.SDL_ReleaseGPUBuffer(self.device, self.vertex_buffer);
        c.SDL_ReleaseGPUBuffer(self.device, self.index_buffer);
        c.SDL_ReleaseGPUTexture(self.device, self.white_texture);
        c.SDL_ReleaseGPUSampler(self.device, self.sampler);
        c.SDL_ReleaseGPUGraphicsPipeline(self.device, self.pipeline);
        self.vertices.deinit(self.allocator);
        self.indices.deinit(self.allocator);
        self.draw_calls.deinit(self.allocator);
        self.allocator.destroy(self);
    }

    fn reset(self: *Self) void {
        self.vertices.clearRetainingCapacity();
        self.indices.clearRetainingCapacity();
        self.draw_calls.clearRetainingCapacity();
        self.current_texture = self.white_texture;
        self.current_clip = null;

        self.draw_calls.append(self.allocator, .{
            .texture = self.white_texture,
            .index_offset = 0,
            .index_count = 0,
            .clip_rect = null,
        }) catch @panic("Failed to allocate");
    }

    fn flush(self: *Self) void {
        const last = &self.draw_calls.items[self.draw_calls.items.len - 1];

        const clip_changed = blk: {
            if (last.clip_rect == null and self.current_clip == null) break :blk false;
            if (last.clip_rect != null and self.current_clip != null) {
                const a = last.clip_rect.?;
                const b = self.current_clip.?;
                break :blk (a.x != b.x or a.y != b.y or a.w != b.w or a.h != b.h);
            }
            break :blk true;
        };

        if (last.texture != self.current_texture or clip_changed) {
            if (last.index_count == 0) {
                last.texture = self.current_texture;
                last.clip_rect = self.current_clip;
            } else {
                self.draw_calls.append(self.allocator, .{
                    .texture = self.current_texture,
                    .index_offset = @intCast(self.indices.items.len),
                    .index_count = 0,
                    .clip_rect = self.current_clip,
                }) catch @panic("Failed to allocate");
            }
        }
    }

    fn setTexture(self: *Self, texture: ?*c.SDL_GPUTexture) void {
        self.current_texture = texture orelse self.white_texture;
        self.flush();
    }

    fn setClipRect(self: *Self, rect: ?c.SDL_Rect) void {
        self.current_clip = rect;
        self.flush();
    }

    fn pushRect(self: *Self, rect: c.SDL_FRect, color: c.SDL_FColor, uv: ?c.SDL_FRect) void {
        const x = rect.x;
        const y = rect.y;
        const w = rect.w;
        const h = rect.h;
        const base_idx: u32 = @intCast(self.vertices.items.len);
        const uvs = uv orelse c.SDL_FRect{ .x = 0, .y = 0, .w = 1, .h = 1 };

        // 4 vertices (Top-Left, Top-Right, Bottom-Right, Bottom-Left)
        self.vertices.appendSlice(self.allocator, &[_]c.SDL_Vertex{
            .{
                .position = .{ .x = x, .y = y },
                .color = color,
                .tex_coord = .{ .x = uvs.x, .y = uvs.y },
            },
            .{
                .position = .{ .x = x + w, .y = y },
                .color = color,
                .tex_coord = .{ .x = uvs.x + uvs.w, .y = uvs.y },
            },
            .{
                .position = .{ .x = x + w, .y = y + h },
                .color = color,
                .tex_coord = .{ .x = uvs.x + uvs.w, .y = uvs.y + uvs.h },
            },
            .{
                .position = .{ .x = x, .y = y + h },
                .color = color,
                .tex_coord = .{ .x = uvs.x, .y = uvs.y + uvs.h },
            },
        }) catch @panic("Failed to allocate");

        // 6 indices (2 triangles)
        self.indices.appendSlice(self.allocator, &[_]u32{
            base_idx, base_idx + 1, base_idx + 2, // Triangle 1
            base_idx, base_idx + 2, base_idx + 3, // Triangle 2
        }) catch @panic("Failed to allocate");

        self.draw_calls.items[self.draw_calls.items.len - 1].index_count += 6;
    }

    fn pushRoundedBorder(self: *Self, box: clay.BoundingBox, border: clay.BorderRenderData) void {
        const w = border.width;
        // If no visible border, return early
        if (w.left == 0 and w.right == 0 and w.top == 0 and w.bottom == 0) return;

        const col = c.SDL_FColor{
            .r = border.color[0] / 255.0,
            .g = border.color[1] / 255.0,
            .b = border.color[2] / 255.0,
            .a = border.color[3] / 255.0,
        };

        //  Define Outer and Inner Rects
        const outer = c.SDL_FRect{ .x = box.x, .y = box.y, .w = box.width, .h = box.height };
        const inner = c.SDL_FRect{
            .x = box.x + @as(f32, @floatFromInt(w.left)),
            .y = box.y + @as(f32, @floatFromInt(w.top)),
            .w = box.width - @as(f32, @floatFromInt(w.left + w.right)),
            .h = box.height - @as(f32, @floatFromInt(w.top + w.bottom)),
        };

        const radius_outer: f32 = @max(border.corner_radius.top_left, border.corner_radius.bottom_right);
        const border_avg = (@as(f32, @floatFromInt(w.left + w.top)) / 2.0);
        var radius_inner = radius_outer - border_avg;
        if (radius_inner < 0) radius_inner = 0;

        // Center Points for corners
        const centers_out = [_]clay.Vector2{
            .{ .x = outer.x + radius_outer, .y = outer.y + radius_outer }, // Top-Left
            .{ .x = (outer.x + outer.w) - radius_outer, .y = outer.y + radius_outer }, // Top-Right
            .{ .x = (outer.x + outer.w) - radius_outer, .y = (outer.y + outer.h) - radius_outer }, // Bot-Right
            .{ .x = outer.x + radius_outer, .y = (outer.y + outer.h) - radius_outer }, // Bot-Left
        };

        const centers_in = [_]clay.Vector2{
            .{ .x = inner.x + radius_inner, .y = inner.y + radius_inner },
            .{ .x = (inner.x + inner.w) - radius_inner, .y = inner.y + radius_inner },
            .{ .x = (inner.x + inner.w) - radius_inner, .y = (inner.y + inner.h) - radius_inner },
            .{ .x = inner.x + radius_inner, .y = (inner.y + inner.h) - radius_inner },
        };

        const segments: usize = 8;
        const step_length: f32 = 90.0 / @as(f32, @floatFromInt(segments));
        const deg2rad = std.math.pi / 180.0;
        const angles = [_]f32{ 180.0, 270.0, 0.0, 90.0 };

        // Save initial vertex index to close the loop later
        const start_v_count: u32 = @intCast(self.vertices.items.len);
        var current_v_count = start_v_count;

        // Generate geometry loop
        for (0..4) |k| {
            var angle = angles[k];
            const c_out = centers_out[k];
            const c_in = centers_in[k];

            for (0..segments + 1) |_| {
                const rad = angle * deg2rad;
                const cos_a = @cos(rad);
                const sin_a = @sin(rad);

                // Push Outer Vertex
                self.vertices.append(self.allocator, .{
                    .position = .{ .x = c_out.x + cos_a * radius_outer, .y = c_out.y + sin_a * radius_outer },
                    .color = col,
                    .tex_coord = .{ .x = 0, .y = 0 },
                }) catch @panic("Failed to allocate");

                // Push Inner Vertex
                self.vertices.append(self.allocator, .{
                    .position = .{ .x = c_in.x + cos_a * radius_inner, .y = c_in.y + sin_a * radius_inner },
                    .color = col,
                    .tex_coord = .{ .x = 0, .y = 0 },
                }) catch @panic("Failed to allocate");

                const idx_out = current_v_count;
                const idx_in = current_v_count + 1;

                // Add indices to connect to previous pair (triangle strip logic)
                // We skip the very first pair of the first corner because there is no previous pair yet
                if (current_v_count > start_v_count) {
                    self.indices.appendSlice(self.allocator, &[_]u32{
                        idx_out - 2, idx_out, idx_out - 1, // Tri 1
                        idx_out - 1, idx_out, idx_in, // Tri 2
                    }) catch @panic("Failed to allocate");
                    self.draw_calls.items[self.draw_calls.items.len - 1].index_count += 6;
                }

                current_v_count += 2;
                angle += step_length;
            }
        }

        // Close the loop (Connect end of Bot-Left to start of Top-Left)
        const last_out = current_v_count - 2;
        const last_in = current_v_count - 1;
        const first_out = start_v_count;
        const first_in = start_v_count + 1;

        self.indices.appendSlice(
            self.allocator,
            &[_]u32{ last_out, first_out, last_in, last_in, first_out, first_in },
        ) catch @panic("Failed to allocate");
        self.draw_calls.items[self.draw_calls.items.len - 1].index_count += 6;
    }

    fn pushRoundedRect(self: *Self, rect: c.SDL_FRect, color: c.SDL_FColor, corner_radius: clay.CornerRadius) void {
        const x = rect.x;
        const y = rect.y;
        const w = rect.w;
        const h = rect.h;
        // Calculate effective radius (clamped to half the shortest side)
        var radius: f32 = @max(
            corner_radius.top_left,
            @max(corner_radius.top_right, @max(corner_radius.bottom_left, corner_radius.bottom_right)),
        );
        const min_side = @min(w, h);
        if (radius * 2.0 > min_side) {
            radius = min_side / 2.0;
        }

        // If no radius, draw a standard rect
        if (radius <= 0.0) {
            self.pushRect(rect, color, null);
            return;
        }

        // Draw 5 Rectangles to fill the body (Center, Top, Bottom, Left, Right)
        // This strategy ensures no geometry overlap which is important for transparency.

        // Center (Inner)
        self.pushRect(.{ .x = x + radius, .y = y + radius, .w = w - 2 * radius, .h = h - 2 * radius }, color, null);
        // Top Strip (between top-left and top-right arcs)
        self.pushRect(.{ .x = x + radius, .y = y, .w = w - 2 * radius, .h = radius }, color, null);
        // Bottom Strip
        self.pushRect(.{ .x = x + radius, .y = y + h - radius, .w = w - 2 * radius, .h = radius }, color, null);
        // Left Strip
        self.pushRect(.{ .x = x, .y = y + radius, .w = radius, .h = h - 2 * radius }, color, null);
        // Right Strip
        self.pushRect(.{ .x = x + w - radius, .y = y + radius, .w = radius, .h = h - 2 * radius }, color, null);

        // Draw 4 Corner Fans
        // Top-Left: PI to 1.5PI
        // Top-Right: 1.5PI to 2PI (0)
        // Bottom-Right: 0 to 0.5PI
        // Bottom-Left: 0.5PI to PI
        const segments: usize = 12;
        const corners = [_]struct { cx: f32, cy: f32, start_angle: f32 }{
            .{ .cx = x + radius, .cy = y + radius, .start_angle = std.math.pi }, // TL
            .{ .cx = x + w - radius, .cy = y + radius, .start_angle = 1.5 * std.math.pi }, // TR
            .{ .cx = x + w - radius, .cy = y + h - radius, .start_angle = 0.0 }, // BR
            .{ .cx = x + radius, .cy = y + h - radius, .start_angle = 0.5 * std.math.pi }, // BL
        };

        for (corners) |c_info| {
            self.pushCornerFan(c_info.cx, c_info.cy, radius, c_info.start_angle, segments, color);
        }
    }

    fn pushCornerFan(self: *Self, cx: f32, cy: f32, radius: f32, start_angle: f32, segments: usize, color: c.SDL_FColor) void {
        const center_idx: u32 = @intCast(self.vertices.items.len);

        // Center pivot vertex of the fan
        self.vertices.append(self.allocator, .{
            .position = .{ .x = cx, .y = cy },
            .color = color,
            .tex_coord = .{ .x = 0, .y = 0 },
        }) catch @panic("Failed to allocate");

        const step = (std.math.pi / 2.0) / @as(f32, @floatFromInt(segments));

        // Generate arc vertices
        for (0..segments + 1) |i| {
            const angle = start_angle + @as(f32, @floatFromInt(i)) * step;

            self.vertices.append(self.allocator, .{
                .position = .{
                    .x = cx + @cos(angle) * radius,
                    .y = cy + @sin(angle) * radius,
                },
                .color = color,
                .tex_coord = .{ .x = 0, .y = 0 },
            }) catch @panic("Failed to allocate");

            // Generate Indices for the fan slice
            if (i > 0) {
                const current_v_idx = center_idx + @as(u32, @intCast(i));
                // Triangle: Center(0), Previous(i), Current(i+1)
                self.indices.appendSlice(
                    self.allocator,
                    &[_]u32{ center_idx, current_v_idx, current_v_idx + 1 },
                ) catch @panic("Failed to allocate");
            }
        }
        self.draw_calls.items[self.draw_calls.items.len - 1].index_count += @as(u32, @intCast(segments * 3));
    }

    fn pushTriangle(self: *Self, p1: clay.Vector2, p2: clay.Vector2, p3: clay.Vector2, color: c.SDL_FColor) void {
        const base_idx: u32 = @intCast(self.vertices.items.len);
        self.vertices.appendSlice(self.allocator, &[_]c.SDL_Vertex{
            .{ .position = .{ .x = p1.x, .y = p1.y }, .color = color, .tex_coord = .{ .x = 0, .y = 0 } },
            .{ .position = .{ .x = p2.x, .y = p2.y }, .color = color, .tex_coord = .{ .x = 0, .y = 0 } },
            .{ .position = .{ .x = p3.x, .y = p3.y }, .color = color, .tex_coord = .{ .x = 0, .y = 0 } },
        }) catch @panic("Failed to allocate!");
        self.indices.appendSlice(self.allocator, &[_]u32{
            base_idx, base_idx + 1, base_idx + 2,
        }) catch @panic("Failed to allocate!");
        self.draw_calls.items[self.draw_calls.items.len - 1].index_count += 3;
    }

    fn resizeGPUBuffers(self: *Self, vertex_count: u32, index_count: u32) void {
        const vertex_size_needed = vertex_count * @sizeOf(c.SDL_Vertex);
        const index_size_needed = index_count * @sizeOf(u32);

        if (vertex_size_needed > self.vertex_buffer_capacity) {
            if (self.vertex_buffer) |b| c.SDL_ReleaseGPUBuffer(self.device, b);
            self.vertex_buffer = c.SDL_CreateGPUBuffer(
                self.device,
                &.{ .usage = c.SDL_GPU_BUFFERUSAGE_VERTEX, .size = vertex_size_needed },
            );
            self.vertex_buffer_capacity = vertex_size_needed;
        }

        if (index_size_needed > self.index_buffer_capacity) {
            if (self.index_buffer) |b| c.SDL_ReleaseGPUBuffer(self.device, b);
            self.index_buffer = c.SDL_CreateGPUBuffer(
                self.device,
                &.{ .usage = c.SDL_GPU_BUFFERUSAGE_INDEX, .size = index_size_needed },
            );
            self.index_buffer_capacity = index_size_needed;
        }
    }
};

pub const Window = struct {
    ctx: *UIContext,
    ptr: ?*c.SDL_Window,
    renderer: *Renderer,
    width: i32,
    height: i32,
    title: []const u8,

    fn deinit(self: *@This(), alloc: std.mem.Allocator, device: ?*c.SDL_GPUDevice) void {
        c.SDL_DestroyWindow(self.ptr);
        self.ctx.deinit(alloc, device);
        self.renderer.deinit();
        alloc.destroy(self);
    }

    fn update(self: *Window) void {
        self.ctx.update();
        clay.setLayoutDimensions(.{ .w = @floatFromInt(self.width), .h = @floatFromInt(self.height) });
    }

    pub fn id(self: *const Window) c.SDL_WindowID {
        return c.SDL_GetWindowID(self.ptr);
    }
};

const SecondaryWindow = struct {
    inner: *Window,
    /// Opaque pointer forwarded to draw_fn as user data.
    user_data: ?*anyopaque = null,
    draw_fn: ?*const fn (*UI, user_data: ?*anyopaque) void = null,

    fn deinit(self: *const @This(), alloc: std.mem.Allocator, device: ?*c.SDL_GPUDevice) void {
        self.inner.deinit(alloc, device);
    }
};

pub const UI = struct {
    allocator: std.mem.Allocator,
    /// The main windows created by the program.
    main_window: *Window,
    /// Secondary windows created by the program, e.g. dialogs.
    secondary_windows: std.ArrayList(SecondaryWindow),
    /// The current window in focus.
    current_window: *Window,
    gpu_device: ?*c.SDL_GPUDevice,
    font: *c.TTF_Font,

    last_title_update: u64 = 0,

    quit: bool = false,
    fps_manager: FPSManager,

    const Self = @This();
    const CLAY_ERROR_HANDLER = clay.ErrorHandler{ .error_handler_function = handleClayError };

    pub fn init(allocator: std.mem.Allocator, title: []const u8, width: i32, height: i32) !*Self {
        if (c.glslang_initialize_process() != 1) {
            return error.GLSlangFailedToInitialize;
        }

        const vk_version = vulkan.detect_vulkan_version();
        std.log.debug("Detected Vulkan version: {}.{}.{}", .{
            c.VK_VERSION_MAJOR(vk_version),
            c.VK_VERSION_MINOR(vk_version),
            c.VK_VERSION_PATCH(vk_version),
        });

        var vkOptions: c.SDL_GPUVulkanOptions = .{
            .vulkan_api_version = c.VK_MAKE_API_VERSION(
                c.VK_API_VERSION_VARIANT(vk_version),
                c.VK_VERSION_MAJOR(vk_version),
                c.VK_VERSION_MINOR(vk_version),
                c.VK_VERSION_PATCH(vk_version),
            ),
        };

        sdlError(c.SDL_SetHint(c.SDL_HINT_QUIT_ON_LAST_WINDOW_CLOSE, "0"));

        const props = c.SDL_CreateProperties();
        defer c.SDL_DestroyProperties(props);

        sdlError(c.SDL_SetPointerProperty(
            props,
            c.SDL_PROP_GPU_DEVICE_CREATE_VULKAN_OPTIONS_POINTER,
            @ptrCast(&vkOptions),
        ));
        sdlError(c.SDL_SetBooleanProperty(props, c.SDL_PROP_GPU_DEVICE_CREATE_SHADERS_SPIRV_BOOLEAN, true));
        sdlError(c.SDL_SetBooleanProperty(
            props,
            c.SDL_PROP_GPU_DEVICE_CREATE_DEBUGMODE_BOOLEAN,
            builtin.mode == .Debug,
        ));

        const gpu_device = sdlError(c.SDL_CreateGPUDeviceWithProperties(props));

        const font_bytes = c.SDL_IOFromConstMem(PIXELOID_FONT, PIXELOID_FONT.len);
        const font = c.TTF_OpenFontIO(font_bytes, true, 16) orelse {
            std.debug.print("Could not load font: {s}\n", .{c.SDL_GetError()});
            return error.FontLoadFailed;
        };

        const ui_ctx = try UIContext.init(
            allocator,
            .{ .w = @floatFromInt(width), .h = @floatFromInt(height) },
            CLAY_ERROR_HANDLER,
        );

        clay.setMeasureTextFunction(*c.TTF_Font, font, measureText);

        if (std.process.getEnvVarOwned(allocator, "UI_DEBUG")) |value| {
            clay.setDebugModeEnabled(std.mem.eql(u8, value, "1"));
            allocator.free(value);
        } else |_| {}

        const main_window = try allocator.create(Window);
        main_window.* = .{
            .ptr = sdlError(c.SDL_CreateWindow(
                title.ptr,
                width,
                height,
                c.SDL_WINDOW_RESIZABLE,
            )),
            .height = height,
            .width = width,
            .title = title,
            .ctx = ui_ctx,
            .renderer = try Renderer.init(allocator, gpu_device, main_window.ptr, vk_version),
        };

        const gui = try allocator.create(UI);
        gui.* = .{
            .allocator = allocator,
            .main_window = main_window,
            .secondary_windows = .empty,
            .current_window = main_window,
            .gpu_device = gpu_device,
            .font = font,
            .fps_manager = FPSManager.init(),
        };

        return gui;
    }

    pub fn deinit(self: *Self) void {
        self.main_window.deinit(self.allocator, self.gpu_device);

        for (self.secondary_windows.items) |window| {
            window.deinit(self.allocator, self.gpu_device);
        }
        self.secondary_windows.deinit(self.allocator);

        c.glslang_finalize_process();
        c.SDL_DestroyGPUDevice(self.gpu_device);
        c.TTF_CloseFont(self.font);

        self.allocator.destroy(self);
    }

    pub fn setFramerate(self: *Self, fps: FPSManager.FramerateMode) void {
        self.fps_manager.setFramerate(fps);
    }

    pub fn shouldClose(self: *Self) bool {
        self.current_window.ctx.dt = @as(f32, @floatFromInt(self.fps_manager.delay())) / c.SDL_MS_PER_SECOND;
        return self.quit;
    }

    pub fn hasPassedSinceMS(self: *const Self, start: u64, ms: u64) bool {
        return self.current_window.ctx.hasPassedSinceMS(start, ms);
    }

    pub fn isWindowFullscreen(self: *const Self) bool {
        return c.SDL_GetWindowFlags(self.current_window.ptr) & c.SDL_WINDOW_FULLSCREEN != 0;
    }

    pub fn setWindowFullscreen(self: *const Self, value: bool) void {
        sdlError(c.SDL_SetWindowFullscreen(self.current_window.ptr, value));
    }

    pub fn createWindow(
        self: *Self,
        title: []const u8,
        width: i32,
        height: i32,
        params: struct {
            draw_fn_data: ?*anyopaque = null,
            draw_fn: ?*const fn (*UI, user_data: ?*anyopaque) void = null,
        },
    ) void {
        const previous_clay_ctx = clay.getCurrentContext();

        const window = self.allocator.create(Window) catch @panic("OOM");
        window.* = .{
            .ptr = sdlError(c.SDL_CreateWindow(title.ptr, width, height, c.SDL_WINDOW_RESIZABLE)),
            .width = width,
            .height = height,
            .title = title,
            .ctx = UIContext.init(
                self.allocator,
                .{ .w = @floatFromInt(width), .h = @floatFromInt(height) },
                CLAY_ERROR_HANDLER,
            ) catch @panic("OOM"),
            .renderer = Renderer.init(self.allocator, self.gpu_device, window.ptr, vulkan.detect_vulkan_version()) catch @panic("OOM"),
        };

        sdlError(c.SDL_SetWindowParent(window.ptr, self.main_window.ptr));
        sdlError(c.SDL_SetWindowModal(window.ptr, true));

        clay.setMeasureTextFunction(*c.TTF_Font, self.font, measureText);

        self.secondary_windows.append(self.allocator, .{
            .inner = window,
            .user_data = params.draw_fn_data,
            .draw_fn = params.draw_fn,
        }) catch @panic("OOM");

        clay.setCurrentContext(previous_clay_ctx);
    }

    pub fn beginFrame(self: *Self) void {
        self.main_window.ctx.reset();
        for (self.secondary_windows.items) |window| {
            window.inner.ctx.reset();
        }

        if (self.main_window.ctx.hasPassedSinceMS(self.last_title_update, 1000)) {
            self.last_title_update = c.SDL_GetTicks();
            const title = std.fmt.allocPrintSentinel(
                self.main_window.ctx.frameAlloc(),
                "{s} - FPS: {}",
                .{ self.main_window.title, self.fps_manager.getFPS() },
                0,
            ) catch "0";
            sdlError(c.SDL_SetWindowTitle(self.main_window.ptr, title.ptr));
        }

        var event: c.SDL_Event = undefined;
        while (c.SDL_PollEvent(&event)) {
            self.handleEvent(&event);
        }

        self.main_window.update();

        clay.beginLayout();
    }

    pub fn endFrame(self: *Self) void {
        const render_commands = clay.endLayout();
        self.renderCommands(self.main_window, render_commands);

        // Render secondary windows
        for (self.secondary_windows.items) |window| {
            window.inner.update();

            clay.beginLayout();

            if (window.draw_fn) |draw_fn| {
                draw_fn(self, window.user_data);
            }

            self.renderCommands(window.inner, clay.endLayout());
        }

        clay.setCurrentContext(self.main_window.ctx.clay_ctx);
    }

    pub fn isKeyPressed(self: *const Self, key: Key) bool {
        return self.current_window.ctx.input.isKeyPressed(key, false);
    }

    pub fn getPressedKey(self: *const Self) ?Key {
        return self.current_window.ctx.input.getPressedKey();
    }

    pub fn getReleasedKey(self: *const Self) ?Key {
        return self.current_window.ctx.input.getReleasedKey();
    }

    pub fn mouseMotion(self: *const Self) bool {
        return self.current_window.ctx.mouseMotion();
    }

    fn handleEvent(self: *Self, event: *c.SDL_Event) void {
        switch (event.type) {
            c.SDL_EVENT_QUIT, c.SDL_EVENT_TERMINATING => {
                self.quit = true;
                return;
            },
            c.SDL_EVENT_WINDOW_FOCUS_GAINED => {
                const focused_win = event.window.windowID;
                if (focused_win == self.main_window.id()) {
                    self.current_window = self.main_window;
                } else {
                    for (self.secondary_windows.items) |window| {
                        if (window.inner.id() == focused_win) {
                            self.current_window = window.inner;
                        }
                    }
                }

                return;
            },
            else => {},
        }

        // Only process the events of the focused window
        if (event.window.windowID == self.current_window.id()) {
            self.handleWindowEvent(event);
        }
    }

    fn handleWindowEvent(self: *Self, event: *c.SDL_Event) void {
        switch (event.type) {
            c.SDL_EVENT_WINDOW_CLOSE_REQUESTED => {
                if (self.current_window.id() == self.main_window.id()) {
                    self.quit = true;
                } else {
                    const window = self.secondary_windows.pop() orelse unreachable;
                    sdlError(c.SDL_HideWindow(window.inner.ptr));
                    window.deinit(self.allocator, self.gpu_device);

                    self.current_window = self.main_window;
                }
            },
            c.SDL_EVENT_WINDOW_RESIZED => {
                sdlError(c.SDL_GetWindowSize(
                    self.current_window.ptr,
                    &self.current_window.width,
                    &self.current_window.height,
                ));
            },
            c.SDL_EVENT_MOUSE_MOTION => self.current_window.ctx.updateMousePos(event.motion),
            c.SDL_EVENT_MOUSE_BUTTON_DOWN => {
                if (event.button.button == c.SDL_BUTTON_LEFT) {
                    self.current_window.ctx.frame.mouse_pressed = true;
                    self.current_window.ctx.frame.mouse_down = true;
                }
            },
            c.SDL_EVENT_MOUSE_BUTTON_UP => {
                if (event.button.button == c.SDL_BUTTON_LEFT) {
                    self.current_window.ctx.frame.mouse_released = true;
                    self.current_window.ctx.frame.mouse_down = false;
                }
            },
            c.SDL_EVENT_MOUSE_WHEEL => {
                const scroll_multiplier = 2.0;
                self.current_window.ctx.frame.scroll.velocity_x += event.wheel.x * scroll_multiplier;
                self.current_window.ctx.frame.scroll.velocity_y += event.wheel.y * scroll_multiplier;
            },
            c.SDL_EVENT_KEY_DOWN => self.current_window.ctx.input.addKeyEvent(.{ .event = .down, .scancode = event.key.scancode }),
            c.SDL_EVENT_KEY_UP => self.current_window.ctx.input.addKeyEvent(.{ .event = .released, .scancode = event.key.scancode }),
            c.SDL_EVENT_TEXT_INPUT => {
                const text: []const u8 = std.mem.span(event.text.text);
                self.current_window.ctx.frame.text_input.appendSlice(self.current_window.ctx.frameAlloc(), text) catch
                    @panic("Fafiled to allocate memory!");
            },
            else => {},
        }

        if (self.current_window.ctx.frame.focused_id != null) {
            sdlError(c.SDL_StartTextInput(self.current_window.ptr));
        } else {
            sdlError(c.SDL_StopTextInput(self.current_window.ptr));
        }
    }

    fn renderCommands(self: *Self, window: *Window, commands: []clay.RenderCommand) void {
        window.renderer.reset();

        for (commands) |cmd| {
            switch (cmd.command_type) {
                clay.RenderCommandType.rectangle => self.renderRectangle(&cmd, window),
                clay.RenderCommandType.text => self.renderText(&cmd, window),
                clay.RenderCommandType.border => self.renderBorder(&cmd, window),
                clay.RenderCommandType.scissor_start => {
                    const clip_rect = c.SDL_Rect{
                        .x = @intFromFloat(cmd.bounding_box.x),
                        .y = @intFromFloat(cmd.bounding_box.y),
                        .w = @intFromFloat(cmd.bounding_box.width),
                        .h = @intFromFloat(cmd.bounding_box.height),
                    };
                    window.renderer.setClipRect(clip_rect);
                },
                clay.RenderCommandType.scissor_end => window.renderer.setClipRect(null),
                clay.RenderCommandType.custom => self.renderCustom(&cmd, window),
                else => {
                    std.log.warn("Render command not impl: {any}", .{cmd.command_type});
                },
            }
        }
        const cmd = c.SDL_AcquireGPUCommandBuffer(self.gpu_device);
        var swapchain_tex: ?*c.SDL_GPUTexture = null;
        var win_w: u32 = 0;
        var win_h: u32 = 0;
        sdlError(c.SDL_WaitAndAcquireGPUSwapchainTexture(
            cmd,
            window.ptr,
            &swapchain_tex,
            &win_w,
            &win_h,
        ));
        if (swapchain_tex == null) return;

        const vertices_len: u32 = @intCast(window.renderer.vertices.items.len);
        const indices_len: u32 = @intCast(window.renderer.indices.items.len);
        const vertices_size = vertices_len * @sizeOf(c.SDL_Vertex);
        const indices_size = indices_len * @sizeOf(u32);
        window.renderer.resizeGPUBuffers(vertices_len, indices_len);

        if (vertices_size > 0) {
            const total_size = vertices_size + indices_size;
            const transfer_buffer = c.SDL_CreateGPUTransferBuffer(self.gpu_device, &.{
                .usage = c.SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD,
                .size = total_size,
            });

            const ptr = @as([*]u8, @ptrCast(@alignCast(c.SDL_MapGPUTransferBuffer(
                self.gpu_device,
                transfer_buffer,
                false,
            ))));
            @memcpy(ptr[0..vertices_size], std.mem.sliceAsBytes(window.renderer.vertices.items));
            @memcpy(ptr[vertices_size..total_size], std.mem.sliceAsBytes(window.renderer.indices.items));
            c.SDL_UnmapGPUTransferBuffer(self.gpu_device, transfer_buffer);

            const copy_pass = c.SDL_BeginGPUCopyPass(cmd);

            c.SDL_UploadToGPUBuffer(
                copy_pass,
                &.{ .transfer_buffer = transfer_buffer, .offset = 0 },
                &.{ .buffer = window.renderer.vertex_buffer.?, .offset = 0, .size = vertices_size },
                false,
            );

            c.SDL_UploadToGPUBuffer(
                copy_pass,
                &.{ .transfer_buffer = transfer_buffer, .offset = vertices_size },
                &.{ .buffer = window.renderer.index_buffer.?, .offset = 0, .size = indices_size },
                false,
            );
            c.SDL_EndGPUCopyPass(copy_pass);
            c.SDL_ReleaseGPUTransferBuffer(self.gpu_device, transfer_buffer);
        }

        var color_target = c.SDL_GPUColorTargetInfo{
            .texture = swapchain_tex,
            .clear_color = .{ .r = 0.94, .g = 0.94, .b = 0.98, .a = 1.0 },
            .load_op = c.SDL_GPU_LOADOP_CLEAR,
            .store_op = c.SDL_GPU_STOREOP_STORE,
        };

        const render_pass = c.SDL_BeginGPURenderPass(cmd, &color_target, 1, null);
        c.SDL_BindGPUGraphicsPipeline(render_pass, window.renderer.pipeline);

        const MVP = [16]f32{
            2.0 / @as(f32, @floatFromInt(window.width)), 0,                                             0, 0,
            0,                                           -2.0 / @as(f32, @floatFromInt(window.height)), 0, 0,
            0,                                           0,                                             1, 0,
            -1,                                          1,                                             0, 1,
        };
        c.SDL_PushGPUVertexUniformData(cmd, 0, &MVP, @sizeOf(@TypeOf(MVP)));

        c.SDL_BindGPUVertexBuffers(
            render_pass,
            0,
            &.{ .buffer = window.renderer.vertex_buffer, .offset = 0 },
            1,
        );
        c.SDL_BindGPUIndexBuffer(
            render_pass,
            &.{ .buffer = window.renderer.index_buffer, .offset = 0 },
            c.SDL_GPU_INDEXELEMENTSIZE_32BIT,
        );

        for (window.renderer.draw_calls.items) |call| {
            if (call.index_count == 0) continue;

            c.SDL_BindGPUFragmentSamplers(
                render_pass,
                0,
                &.{ .texture = call.texture, .sampler = window.renderer.sampler },
                1,
            );

            if (call.clip_rect) |rect| {
                c.SDL_SetGPUScissor(render_pass, &rect);
            } else {
                c.SDL_SetGPUScissor(render_pass, &.{ .x = 0, .y = 0, .w = @intCast(win_w), .h = @intCast(win_h) });
            }

            c.SDL_DrawGPUIndexedPrimitives(render_pass, call.index_count, 1, call.index_offset, 0, 0);
        }
        c.SDL_EndGPURenderPass(render_pass);

        sdlError(c.SDL_SubmitGPUCommandBuffer(cmd));
    }

    fn renderCustom(self: *Self, cmd: *const clay.RenderCommand, window: *Window) void {
        const data = clay.anyopaquePtrToType(*widgets.CustomData, cmd.render_data.custom.custom_data.?);
        switch (data.*) {
            .canvas => |canvas_| {
                var texture: ?*c.SDL_GPUTexture = undefined;
                var should_create = true;

                if (window.ctx.canvas_cache.getPtr(cmd.id)) |item| {
                    if (item.width != canvas_.params.w or item.height != canvas_.params.h) {
                        c.SDL_ReleaseGPUTexture(self.gpu_device, item.texture);
                    } else {
                        texture = item.texture;
                        should_create = false;
                    }
                }

                if (should_create) {
                    texture = sdlError(c.SDL_CreateGPUTexture(self.gpu_device, &.{
                        .type = c.SDL_GPU_TEXTURETYPE_2D,
                        .format = c.SDL_GetGPUTextureFormatFromPixelFormat(canvas_.params.pixel_format),
                        .width = @intCast(canvas_.params.w),
                        .height = @intCast(canvas_.params.h),
                        .layer_count_or_depth = 1,
                        .num_levels = 1,
                        .usage = c.SDL_GPU_TEXTUREUSAGE_SAMPLER,
                    }));

                    window.ctx.canvas_cache.put(cmd.id, .{
                        .texture = texture,
                        .width = canvas_.params.w,
                        .height = canvas_.params.h,
                    }) catch @panic("Failed to cache canvas texture");
                }

                const buffer_size: u32 = canvas_.params.h * canvas_.params.w *
                    c.SDL_GetPixelFormatDetails(canvas_.params.pixel_format).*.bytes_per_pixel;
                const tb = c.SDL_CreateGPUTransferBuffer(
                    self.gpu_device,
                    &.{ .usage = c.SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD, .size = buffer_size },
                );

                const map = c.SDL_MapGPUTransferBuffer(self.gpu_device, tb, false);
                @memcpy(@as([*]u8, @ptrCast(map))[0..buffer_size], canvas_.params.pixels);
                c.SDL_UnmapGPUTransferBuffer(self.gpu_device, tb);

                const cmd_buf = c.SDL_AcquireGPUCommandBuffer(self.gpu_device);
                const copy_pass = c.SDL_BeginGPUCopyPass(cmd_buf);

                c.SDL_UploadToGPUTexture(
                    copy_pass,
                    &.{ .transfer_buffer = tb },
                    &.{
                        .texture = texture,
                        .w = canvas_.params.w,
                        .h = canvas_.params.h,
                        .d = 1,
                    },
                    false,
                );

                c.SDL_EndGPUCopyPass(copy_pass);
                _ = c.SDL_SubmitGPUCommandBuffer(cmd_buf);
                c.SDL_ReleaseGPUTransferBuffer(self.gpu_device, tb);

                const dest: c.SDL_FRect = if (canvas_.params.aspect_ratio) |aspect_ratio| blk: {
                    const viewport = utils.calculateViewport(
                        cmd.bounding_box.x,
                        cmd.bounding_box.y,
                        cmd.bounding_box.width,
                        cmd.bounding_box.height,
                        aspect_ratio,
                    );
                    break :blk .{
                        .x = viewport.x,
                        .y = viewport.y,
                        .w = viewport.w,
                        .h = viewport.h,
                    };
                } else .{
                    .x = cmd.bounding_box.x,
                    .y = cmd.bounding_box.y,
                    .w = cmd.bounding_box.width,
                    .h = cmd.bounding_box.height,
                };

                const bg_color_sdl: c.SDL_FColor = if (canvas_.params.bg_color) |bg|
                    bg.toSDL()
                else
                    .{ .r = 0.0, .g = 0.0, .b = 0.0, .a = 1.0 };
                window.renderer.setTexture(null);
                window.renderer.pushRect(.{
                    .x = cmd.bounding_box.x,
                    .y = cmd.bounding_box.y,
                    .w = cmd.bounding_box.width,
                    .h = cmd.bounding_box.height,
                }, bg_color_sdl, null);

                const color: c.SDL_FColor = if (canvas_.params.fg_color) |fg_color| blk: {
                    break :blk fg_color.toSDL();
                } else .{ .r = 1.0, .g = 1.0, .b = 1.0, .a = 1.0 };

                window.renderer.setTexture(texture);
                window.renderer.pushRect(dest, color, null);
                window.renderer.flush();
            },
            .shape => |shape_data| {
                const rect = cmd.bounding_box;
                const x = rect.x;
                const y = rect.y;
                const w = rect.width;
                const h = rect.height;

                // Center of the bounding box
                const cx = x + (w / 2.0);
                const cy = y + (h / 2.0);

                const rads: f32 = std.math.degreesToRadians(shape_data.rotation);
                const cos_theta = std.math.cos(rads);
                const sin_theta = std.math.sin(rads);

                const color = shape_data.color.toSDL();

                window.renderer.setTexture(null);

                // We need at least 3 vertices to draw a visible shape
                if (shape_data.vertices.len >= 3) {
                    const transform = struct {
                        fn apply(
                            v: clay.Vector2,
                            bx: f32,
                            by: f32,
                            bw: f32,
                            bh: f32,
                            bcx: f32,
                            bcy: f32,
                            cos_a: f32,
                            sin_a: f32,
                        ) clay.Vector2 {
                            // Map normalized coordinates (0.0-1.0) to actual bounding box
                            const px = bx + v.x * bw;
                            const py = by + v.y * bh;

                            // Apply rotation around the center (bcx, bcy)
                            return .{
                                .x = cos_a * (px - bcx) - sin_a * (py - bcy) + bcx,
                                .y = sin_a * (px - bcx) + cos_a * (py - bcy) + bcy,
                            };
                        }
                    }.apply;

                    // Map and rotate the anchor vertex (index 0)
                    const p0 = transform(shape_data.vertices[0], x, y, w, h, cx, cy, cos_theta, sin_theta);

                    var i: usize = 1;
                    while (i + 1 < shape_data.vertices.len) : (i += 1) {
                        const p1 = transform(shape_data.vertices[i], x, y, w, h, cx, cy, cos_theta, sin_theta);
                        const p2 = transform(shape_data.vertices[i + 1], x, y, w, h, cx, cy, cos_theta, sin_theta);

                        window.renderer.pushTriangle(p0, p1, p2, color);
                    }
                }
            },
        }
    }

    fn renderRectangle(_: *const Self, cmd: *const clay.RenderCommand, window: *Window) void {
        const config = cmd.render_data.rectangle;
        const color = c.SDL_FColor{
            .r = config.background_color[0] / 255.0,
            .g = config.background_color[1] / 255.0,
            .b = config.background_color[2] / 255.0,
            .a = config.background_color[3] / 255.0,
        };
        const radius = cmd.render_data.rectangle.corner_radius;
        const rect = c.SDL_FRect{
            .x = cmd.bounding_box.x,
            .y = cmd.bounding_box.y,
            .w = cmd.bounding_box.width,
            .h = cmd.bounding_box.height,
        };

        window.renderer.setTexture(null);

        if (radius.top_left > 0.0 or radius.top_right > 0.0 or radius.bottom_left > 0.0 or radius.bottom_right > 0.0) {
            window.renderer.pushRoundedRect(rect, color, radius);
        } else {
            window.renderer.pushRect(rect, color, null);
        }
    }

    fn renderBorder(_: *const Self, cmd: *const clay.RenderCommand, window: *Window) void {
        window.renderer.setTexture(null);
        window.renderer.pushRoundedBorder(cmd.bounding_box, cmd.render_data.border);
    }

    fn renderText(self: *Self, cmd: *const clay.RenderCommand, window: *Window) void {
        const text_data = cmd.render_data.text;
        const color = text_data.text_color;
        const text_slice = text_data.string_contents;
        if (text_slice.length == 0) {
            return;
        }

        sdlError(c.TTF_SetFontSize(self.font, @floatFromInt(text_data.font_size)));

        var hasher = std.hash.XxHash32.init(0);
        hasher.update(text_slice.chars[0..@intCast(text_slice.length)]);
        std.hash.autoHash(&hasher, [_]u32{
            @bitCast(color[0]),
            @bitCast(color[1]),
            @bitCast(color[2]),
            @bitCast(color[3]),
            text_data.font_size,
        });
        const key = hasher.final();
        const font_item = blk: {
            if (window.ctx.text_cache.get(key)) |item| {
                break :blk item;
            } else {
                const sdl_color = c.SDL_Color{
                    .r = @intFromFloat(color[0]),
                    .g = @intFromFloat(color[1]),
                    .b = @intFromFloat(color[2]),
                    .a = @intFromFloat(color[3]),
                };
                const surface = sdlError(c.TTF_RenderText_Blended(self.font, text_slice.chars, @intCast(text_slice.length), sdl_color));
                defer c.SDL_DestroySurface(surface);
                const format_details = c.SDL_GetPixelFormatDetails(surface.*.format);

                const texture = sdlError(c.SDL_CreateGPUTexture(self.gpu_device, &.{
                    .type = c.SDL_GPU_TEXTURETYPE_2D,
                    .format = c.SDL_GetGPUTextureFormatFromPixelFormat(surface.*.format),
                    .width = @intCast(surface.*.w),
                    .height = @intCast(surface.*.h),
                    .layer_count_or_depth = 1,
                    .num_levels = 1,
                    .usage = c.SDL_GPU_TEXTUREUSAGE_SAMPLER,
                }));

                const buffer_size: u32 = @intCast(surface.*.pitch * surface.*.h);
                var tb_info = c.SDL_GPUTransferBufferCreateInfo{ .usage = c.SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD, .size = buffer_size };
                const tb = c.SDL_CreateGPUTransferBuffer(self.gpu_device, &tb_info);

                const map = c.SDL_MapGPUTransferBuffer(self.gpu_device, tb, false);
                const surface_pixels = @as([*]u8, @ptrCast(surface.*.pixels));
                @memcpy(@as([*]u8, @ptrCast(map))[0..buffer_size], surface_pixels[0..buffer_size]);
                c.SDL_UnmapGPUTransferBuffer(self.gpu_device, tb);

                const cmd_buf = c.SDL_AcquireGPUCommandBuffer(self.gpu_device);
                const copy_pass = c.SDL_BeginGPUCopyPass(cmd_buf);

                var src = c.SDL_GPUTextureTransferInfo{
                    .transfer_buffer = tb,
                    .offset = 0,
                    .pixels_per_row = @intCast(@divExact(surface.*.pitch, format_details.*.bytes_per_pixel)),
                    .rows_per_layer = @intCast(surface.*.h),
                };
                var dst = c.SDL_GPUTextureRegion{
                    .texture = texture,
                    .w = @intCast(surface.*.w),
                    .h = @intCast(surface.*.h),
                    .d = 1,
                };
                c.SDL_UploadToGPUTexture(copy_pass, &src, &dst, false);

                c.SDL_EndGPUCopyPass(copy_pass);
                _ = c.SDL_SubmitGPUCommandBuffer(cmd_buf);
                c.SDL_ReleaseGPUTransferBuffer(self.gpu_device, tb);

                const item: TextCacheItem = .{
                    .texture = texture,
                    .width = @floatFromInt(surface.*.w),
                    .height = @floatFromInt(surface.*.h),
                };

                window.ctx.text_cache.put(key, item) catch @panic("Failed to allocate memory!");
                break :blk item;
            }
        };

        const dest = c.SDL_FRect{
            .x = cmd.bounding_box.x,
            .y = cmd.bounding_box.y,
            .w = font_item.width,
            .h = font_item.height,
        };
        window.renderer.setTexture(font_item.texture);
        window.renderer.pushRect(
            dest,
            .{ .r = color[0] / 255.0, .g = color[1] / 255.0, .b = color[2] / 255.0, .a = color[3] / 255.0 },
            null,
        );
    }

    pub fn scrollArea(self: *Self, params: widgets.ScrollContainer.Params) *widgets.ScrollContainer {
        return self.current_window.ctx.allocWidget(widgets.ScrollContainer, .start(self.current_window.ctx, params));
    }

    pub fn row(self: *Self, params: widgets.Container.Params) *widgets.Container {
        var p = params;
        p.direction = .row;
        return self.current_window.ctx.allocWidget(widgets.Container, .start(p));
    }

    pub fn column(self: *Self, params: widgets.Container.Params) *widgets.Container {
        var p = params;
        p.direction = .column;
        return self.current_window.ctx.allocWidget(widgets.Container, .start(p));
    }

    pub fn label(self: *Self, params: widgets.Label.Params) *widgets.Label {
        return self.current_window.ctx.allocWidget(widgets.Label, .start(params));
    }

    pub fn button(self: *Self, params: widgets.Button.Params) *widgets.Button {
        return self.current_window.ctx.allocWidget(widgets.Button, .start(params, self.current_window.ctx));
    }

    pub fn spacer(self: *Self, params: widgets.Spacer.Params) *widgets.Spacer {
        return self.current_window.ctx.allocWidget(widgets.Spacer, .start(params));
    }

    pub fn canvas(self: *Self, params: widgets.Canvas.Params) *widgets.Canvas {
        return self.current_window.ctx.allocWidget(widgets.Canvas, .start(self.current_window.ctx, params));
    }

    pub fn menuBar(self: *Self, params: widgets.MenuBar.Params) *widgets.MenuBar {
        return self.current_window.ctx.allocWidget(widgets.MenuBar, .start(self.current_window.ctx, params));
    }

    pub fn dropdownMenu(self: *Self, params: widgets.DropdownMenu.Params) *widgets.DropdownMenu {
        return self.current_window.ctx.allocWidget(widgets.DropdownMenu, .start(self.current_window.ctx, params));
    }

    pub fn menuItem(self: *Self, params: widgets.MenuItem.Params) *widgets.MenuItem {
        return self.current_window.ctx.allocWidget(widgets.MenuItem, .start(self.current_window.ctx, params));
    }

    pub fn textField(self: *Self, params: widgets.TextField.Params) *widgets.TextField {
        return self.current_window.ctx.allocWidget(widgets.TextField, .start(self.current_window.ctx, params));
    }

    pub fn padding(self: *Self, params: widgets.Padding.Params) *widgets.Padding {
        return self.current_window.ctx.allocWidget(widgets.Padding, .start(params));
    }

    pub fn combobox(self: *Self, params: widgets.Combobox.Params) *widgets.Combobox {
        return self.current_window.ctx.allocWidget(widgets.Combobox, .start(self.current_window.ctx, params));
    }

    pub fn comboboxItem(self: *Self, params: widgets.ComboboxItem.Params) *widgets.ComboboxItem {
        return self.current_window.ctx.allocWidget(widgets.ComboboxItem, .start(self.current_window.ctx, params));
    }

    pub fn shape(self: *Self, params: widgets.Shape.Params) *widgets.Shape {
        return self.current_window.ctx.allocWidget(widgets.Shape, .start(self.current_window.ctx, params));
    }

    pub fn separator(self: *Self, params: widgets.Separator.Params) *widgets.Separator {
        return self.current_window.ctx.allocWidget(widgets.Separator, .start(params));
    }
};
