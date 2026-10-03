const std = @import("std");

const TokType = enum {
    Id,
    Bool,
    Eq,
    String,
    Number,
    Newline,
    Reference,
    Eof,
};

const Token = struct {
    type: TokType,
    value: []const u8,

    pub fn format(self: @This(), writer: *std.Io.Writer) !void {
        const value = if (std.mem.eql(u8, self.value, "\n")) "\\n" else self.value;
        try writer.print(".{{ type: {any}, value: \"{s}\" }}", .{ self.type, value });
    }
};

fn lex(alloc: std.mem.Allocator, source: []const u8) ![]Token {
    var tokens: std.ArrayList(Token) = .empty;

    var is_after_eq = false;
    var pos: u32 = 0;
    while (pos < source.len) {
        const start = pos;
        switch (source[pos]) {
            '/' => {
                pos += 1;
                // `//` line comment
                if (pos < source.len and source[pos] == '/') {
                    while (pos < source.len and source[pos] != '\n' and source[pos] != '\r') pos += 1;
                }
            },
            '#' => {
                pos += 1;
                while (pos < source.len and std.ascii.isAlphabetic(source[pos])) pos += 1;
                if (std.mem.eql(u8, source[start..pos], "#reference")) {
                    while (pos < source.len and std.ascii.isWhitespace(source[pos])) pos += 1;
                    const value = if (pos < source.len and source[pos] == '"') blk: {
                        pos += 1; // opening quote
                        const start_path = pos;
                        while (pos < source.len and source[pos] != '"') pos += 1;
                        const end_path = pos;
                        if (pos < source.len) pos += 1; // closing quote
                        break :blk source[start_path..end_path];
                    } else blk: {
                        const start_path = pos;
                        while (pos < source.len and source[pos] != '\n') pos += 1;
                        break :blk source[start_path..pos];
                    };

                    try tokens.append(alloc, .{
                        .type = .Reference,
                        .value = value,
                    });
                } else {
                    while (pos < source.len and source[pos] != '\n' and source[pos] != '\r') pos += 1;
                }
            },
            '\n', '\r' => {
                if (source[pos] == '\r') {
                    pos += 2; // \r\n
                } else {
                    pos += 1;
                }
                try tokens.append(alloc, .{
                    .type = .Newline,
                    .value = source[start..pos],
                });
                is_after_eq = false;
            },
            '=' => {
                pos += 1;
                try tokens.append(alloc, .{
                    .type = .Eq,
                    .value = source[start..pos],
                });
                is_after_eq = true;
            },
            '"' => {
                pos += 1; // opening quote
                while (pos < source.len and source[pos] != '"') pos += 1;
                if (pos < source.len) pos += 1; // closing quote
                const value = source[start + 1 .. pos - 1];
                if (std.mem.eql(u8, value, "true") or std.mem.eql(u8, value, "false")) {
                    try tokens.append(alloc, .{
                        .type = .Bool,
                        .value = value,
                    });
                } else if (value.len != 0 and (std.ascii.isDigit(value[0]) or value[0] == '-')) {
                    try tokens.append(alloc, .{
                        .type = .Number,
                        .value = value,
                    });
                } else {
                    try tokens.append(alloc, .{
                        .type = .String,
                        .value = value,
                    });
                }
            },
            '0'...'9', '-' => {
                if (source[pos] == '-') pos += 1;
                while (pos < source.len and std.ascii.isDigit(source[pos])) pos += 1;
                if (pos < source.len and source[pos] == '.') {
                    pos += 1; // dot
                    while (pos < source.len and std.ascii.isDigit(source[pos])) pos += 1;
                }
                try tokens.append(alloc, .{
                    .type = .Number,
                    .value = source[start..pos],
                });
            },
            'a'...'z', 'A'...'Z', '_', '.' => {
                // Handle the case where we're lexing a relative path after '='
                if (is_after_eq) {
                    while (pos < source.len and source[pos] != '\n' and source[pos] != '\r') pos += 1;
                    const value = source[start..pos];
                    try tokens.append(alloc, .{
                        .type = if (std.mem.eql(u8, value, "true") or std.mem.eql(u8, value, "false"))
                            .Bool
                        else
                            .String,
                        .value = value,
                    });
                    continue;
                }

                while (pos < source.len and (std.ascii.isAlphanumeric(source[pos]) or source[pos] == '_')) pos += 1;
                if (pos == start) {
                    pos += 1; // stray '.' outside a value
                    continue;
                }
                const value = source[start..pos];
                try tokens.append(alloc, .{
                    .type = .Id,
                    .value = value,
                });
            },
            else => pos += 1,
        }
    }

    try tokens.append(alloc, .{ .type = .Eof, .value = "" });

    return tokens.toOwnedSlice(alloc);
}

pub const ScaleType = enum {
    source,
    viewport,
    absolute,

    fn fromSlice(slice: []const u8) !@This() {
        const trimmed = std.mem.trim(u8, slice, &std.ascii.whitespace);
        return std.meta.stringToEnum(@This(), trimmed) orelse error.InvalidScaleType;
    }
};

pub const WrapMode = enum {
    clamp_to_border,
    clamp_to_edge,
    repeat,
    mirrored_repeat,

    fn fromSlice(slice: []const u8) !@This() {
        const trimmed = std.mem.trim(u8, slice, &std.ascii.whitespace);
        return std.meta.stringToEnum(@This(), trimmed) orelse error.InvalidWrapMode;
    }
};

pub const ShaderPassParams = struct {
    /// How the texture of the input to this pass will be filtered.
    filter_linear: ?bool = null,
    /// Output scaling type in X direction.
    scale_type: ?ScaleType = null,
    scale_type_x: ?ScaleType = null,
    scale_type_y: ?ScaleType = null,
    /// Output scale factors.
    scale: ?f32 = null,
    scale_x: ?f32 = null,
    scale_y: ?f32 = null,
    /// Texture wrap / address mode.
    wrap_mode: ?WrapMode = null,
    /// Named alias for this pass (used for feedback loops or cross-pass references).
    alias: ?[]const u8 = null,
    /// Whether to use mipmaps on the input texture.
    mipmap_input: ?bool = null,
    /// Whether this pass outputs to a floating-point framebuffer.
    float_framebuffer: ?bool = null,
    /// Whether this pass uses sRGB color space output.
    srgb_framebuffer: ?bool = null,
    /// Modulo applied to `frame_count` in this pass.
    frame_count_mod: ?u32 = null,
};

pub const ShaderPass = struct {
    id: usize,
    path: []const u8,
    params: ShaderPassParams,

    fn deinit(self: *@This(), alloc: std.mem.Allocator) void {
        if (self.params.alias) |alias| {
            alloc.free(alias);
        }
    }

    pub fn format(self: @This(), writer: *std.Io.Writer) !void {
        try writer.print(".{{ path: {s}, params: {any} }}", .{ self.path, self.params });
    }
};

pub const LutEntry = struct {
    path: []const u8 = "",
    linear: bool = false,
    wrap_mode: WrapMode = .clamp_to_edge,
    mipmap: bool = false,
};
pub const Luts = std.StringHashMap(LutEntry);

pub const ShaderConfig = struct {
    total_passes: usize,
    passes: []ShaderPass,
    textures: Luts,
    shader_params_initial_values: std.StringHashMap(TypeUnion),

    pub fn deinit(self: *@This(), alloc: std.mem.Allocator) void {
        for (self.passes) |*pass| {
            pass.deinit(alloc);
        }
        alloc.free(self.passes);

        var iter = self.shader_params_initial_values.iterator();
        while (iter.next()) |entry| {
            alloc.free(entry.key_ptr.*);
            if (entry.value_ptr.* == .String) {
                alloc.free(entry.value_ptr.*.String);
            }
        }
        self.shader_params_initial_values.deinit();

        var texture_iter = self.textures.iterator();
        while (texture_iter.next()) |entry| {
            alloc.free(entry.key_ptr.*);
            alloc.free(entry.value_ptr.*.path);
        }
        self.textures.deinit();
    }
};

pub const TypeUnion = union(enum) {
    Float: f32,
    Bool: bool,
    String: []const u8,

    pub fn format(self: @This(), writer: *std.Io.Writer) !void {
        switch (self) {
            .String => |value| try writer.print(".{{ .{s} = \"{s}\" }}", .{ @tagName(self), value }),
            else => try writer.print("{any}", .{self}),
        }
    }

    pub inline fn bytes(self: @This()) []const u8 {
        return switch (self) {
            inline else => |value| std.mem.asBytes(&value),
        };
    }
};

pub fn parse_slangp(alloc: std.mem.Allocator, source: []const u8) !ShaderConfig {
    const tokens = try lex(alloc, source);
    defer alloc.free(tokens);

    var pos: usize = 0;
    while (tokens[pos].type == .Newline) pos += 1;

    if (tokens[pos].type != .Reference and
        !(tokens[pos].type == .Id and std.mem.eql(u8, tokens[pos].value, "shaders")))
    {
        return error.ShouldDefineTotalShadersOrReference;
    }

    pos += 1; // tok: 'shaders' id or reference
    pos += 1; // tok: '='

    const total_passes = try std.fmt.parseInt(usize, tokens[pos].value, 10);
    var passes = try alloc.alloc(ShaderPass, total_passes);
    @memset(passes, .{ .id = 0, .path = "", .params = .{} });

    var textures: Luts = .init(alloc);
    var shader_params: std.StringHashMap(TypeUnion) = .init(alloc);

    while (pos < tokens.len and tokens[pos].type != .Eof) : (pos += 1) {
        const token = tokens[pos];
        switch (token.type) {
            .Id => {
                if (tokens[pos + 1].type != .Eq) continue;
                const field = token.value;
                pos += 2; // Id + Eq
                const value_tok = tokens[pos];
                pos += 1;

                if (std.mem.startsWith(u8, field, "textures")) {
                    var values = std.mem.splitScalar(u8, value_tok.value, ';');
                    while (values.next()) |texture_name| {
                        try textures.put(try alloc.dupe(u8, texture_name), .{});
                    }
                    continue;
                }

                if (textures.getEntry(field)) |entry| {
                    entry.value_ptr.*.path = try alloc.dupe(u8, value_tok.value);
                    continue;
                }

                if (std.mem.endsWith(u8, field, "_linear")) {
                    const base = field[0 .. field.len - "_linear".len];
                    if (textures.getEntry(base)) |entry| {
                        entry.value_ptr.*.linear = std.mem.eql(u8, value_tok.value, "true");
                        continue;
                    }
                } else if (std.mem.endsWith(u8, field, "_wrap_mode")) {
                    const base = field[0 .. field.len - "_wrap_mode".len];
                    if (textures.getEntry(base)) |entry| {
                        entry.value_ptr.*.wrap_mode = try WrapMode.fromSlice(value_tok.value);
                        continue;
                    }
                } else if (std.mem.endsWith(u8, field, "_mipmap")) {
                    const base = field[0 .. field.len - "_mipmap".len];
                    if (textures.getEntry(base)) |entry| {
                        entry.value_ptr.*.mipmap = std.mem.eql(u8, value_tok.value, "true");
                        continue;
                    }
                }

                const pass_field = parsePassField(field) orelse {
                    const value: TypeUnion = blk: {
                        if (is_float(value_tok.value)) {
                            break :blk .{ .Float = try std.fmt.parseFloat(f32, value_tok.value) };
                        } else if (std.mem.eql(u8, value_tok.value, "true")) {
                            break :blk .{ .Bool = true };
                        } else if (std.mem.eql(u8, value_tok.value, "false")) {
                            break :blk .{ .Bool = false };
                        } else {
                            break :blk .{ .String = try alloc.dupe(u8, value_tok.value) };
                        }
                    };
                    try shader_params.put(try alloc.dupe(u8, field), value);
                    continue;
                };

                const pass_id = pass_field.id;
                if (pass_id >= total_passes) continue;
                passes[pass_id].id = pass_id;
                const params = &passes[pass_id].params;
                const value = value_tok.value;
                switch (pass_field.key) {
                    .shader => passes[pass_id].path = value,
                    .filter_linear => params.filter_linear = std.mem.eql(u8, value, "true"),
                    .scale_type_x => params.scale_type_x = try ScaleType.fromSlice(value),
                    .scale_type_y => params.scale_type_y = try ScaleType.fromSlice(value),
                    .scale_type => params.scale_type = try ScaleType.fromSlice(value),
                    .scale_x => params.scale_x = try std.fmt.parseFloat(f32, value),
                    .scale_y => params.scale_y = try std.fmt.parseFloat(f32, value),
                    .scale => params.scale = try std.fmt.parseFloat(f32, value),
                    .wrap_mode => params.wrap_mode = try WrapMode.fromSlice(value),
                    .alias => {
                        if (params.alias) |old| alloc.free(old);
                        params.alias = try alloc.dupe(u8, value);
                    },
                    .mipmap_input => params.mipmap_input = std.mem.eql(u8, value, "true"),
                    .float_framebuffer => params.float_framebuffer = std.mem.eql(u8, value, "true"),
                    .srgb_framebuffer => params.srgb_framebuffer = std.mem.eql(u8, value, "true"),
                    .frame_count_mod => params.frame_count_mod = try std.fmt.parseInt(u32, value, 10),
                }
            },
            else => {},
        }
    }

    return .{
        .passes = passes,
        .total_passes = total_passes,
        .textures = textures,
        .shader_params_initial_values = shader_params,
    };
}

fn is_float(value: []const u8) bool {
    if (value.len == 0) return false;
    for (value) |ch| {
        if (!std.ascii.isDigit(ch) and ch != '.' and ch != '-') return false;
    }
    return true;
}

const PassKey = enum {
    shader,
    filter_linear,
    scale_type_x,
    scale_type_y,
    scale_type,
    scale_x,
    scale_y,
    scale,
    wrap_mode,
    alias,
    mipmap_input,
    float_framebuffer,
    srgb_framebuffer,
    frame_count_mod,
};

/// Splits a per-pass field like `scale_type_x3` into its key and pass index.
/// Returns null for anything else (shader parameters such as `JINC2_SINC`).
fn parsePassField(field: []const u8) ?struct { key: PassKey, id: usize } {
    var digits_start = field.len;
    while (digits_start > 0 and std.ascii.isDigit(field[digits_start - 1])) digits_start -= 1;
    if (digits_start == field.len) return null;
    const key = std.meta.stringToEnum(PassKey, field[0..digits_start]) orelse return null;
    const id = std.fmt.parseInt(usize, field[digits_start..], 10) catch return null;
    return .{ .key = key, .id = id };
}

pub fn get_field_pass_id(field: []const u8) !usize {
    var pos: usize = 0;
    while (pos < field.len and !std.ascii.isDigit(field[pos])) {
        pos += 1;
    }
    if (pos >= field.len) return error.NoPassIdInField;
    return try std.fmt.parseInt(usize, field[pos..], 10);
}

test "lex_slangp" {
    const alloc = std.testing.allocator;
    const source =
        \\#reference "shaders/crt-sony-megatron-sdr.slangp"
        \\shaders = "3"
        \\
        \\shader0 = "../stock.slang"
        \\filter_linear0 = "false"
        \\scale_type_x0 = "absolute"
        \\scale_x0 = "640.000000"
        \\scale_type_y0 = "source"
        \\scale_y0 = "1.000000"
        \\
        \\shader1 = "blargg/blargg-0.slang"
        \\filter_linear1 = false
        \\
        \\shader2 = "blargg/blargg-1.slang"
        \\filter_linear2 = false
    ;
    const tokens = try lex(alloc, source);
    defer alloc.free(tokens);

    const expected = [_][]const u8{
        ".{ type: .Reference, value: \"shaders/crt-sony-megatron-sdr.slangp\" }",
        ".{ type: .Newline, value: \"\\n\" }",
        ".{ type: .Id, value: \"shaders\" }",
        ".{ type: .Eq, value: \"=\" }",
        ".{ type: .Number, value: \"3\" }",
        ".{ type: .Newline, value: \"\\n\" }",
        ".{ type: .Newline, value: \"\\n\" }",
        ".{ type: .Id, value: \"shader0\" }",
        ".{ type: .Eq, value: \"=\" }",
        ".{ type: .String, value: \"../stock.slang\" }",
        ".{ type: .Newline, value: \"\\n\" }",
        ".{ type: .Id, value: \"filter_linear0\" }",
        ".{ type: .Eq, value: \"=\" }",
        ".{ type: .Bool, value: \"false\" }",
        ".{ type: .Newline, value: \"\\n\" }",
        ".{ type: .Id, value: \"scale_type_x0\" }",
        ".{ type: .Eq, value: \"=\" }",
        ".{ type: .String, value: \"absolute\" }",
        ".{ type: .Newline, value: \"\\n\" }",
        ".{ type: .Id, value: \"scale_x0\" }",
        ".{ type: .Eq, value: \"=\" }",
        ".{ type: .Number, value: \"640.000000\" }",
        ".{ type: .Newline, value: \"\\n\" }",
        ".{ type: .Id, value: \"scale_type_y0\" }",
        ".{ type: .Eq, value: \"=\" }",
        ".{ type: .String, value: \"source\" }",
        ".{ type: .Newline, value: \"\\n\" }",
        ".{ type: .Id, value: \"scale_y0\" }",
        ".{ type: .Eq, value: \"=\" }",
        ".{ type: .Number, value: \"1.000000\" }",
        ".{ type: .Newline, value: \"\\n\" }",
        ".{ type: .Newline, value: \"\\n\" }",
        ".{ type: .Id, value: \"shader1\" }",
        ".{ type: .Eq, value: \"=\" }",
        ".{ type: .String, value: \"blargg/blargg-0.slang\" }",
        ".{ type: .Newline, value: \"\\n\" }",
        ".{ type: .Id, value: \"filter_linear1\" }",
        ".{ type: .Eq, value: \"=\" }",
        ".{ type: .Bool, value: \"false\" }",
        ".{ type: .Newline, value: \"\\n\" }",
        ".{ type: .Newline, value: \"\\n\" }",
        ".{ type: .Id, value: \"shader2\" }",
        ".{ type: .Eq, value: \"=\" }",
        ".{ type: .String, value: \"blargg/blargg-1.slang\" }",
        ".{ type: .Newline, value: \"\\n\" }",
        ".{ type: .Id, value: \"filter_linear2\" }",
        ".{ type: .Eq, value: \"=\" }",
        ".{ type: .Bool, value: \"false\" }",
        ".{ type: .Eof, value: \"\" }",
    };

    for (tokens, 0..) |token, i| {
        const got = try std.fmt.allocPrint(alloc, "{f}", .{token});
        defer alloc.free(got);
        try std.testing.expect(std.mem.eql(u8, got, expected[i]));
    }
}

test "parse_slangp" {
    const alloc = std.testing.allocator;
    const source =
        \\shaders = "3"
        \\
        \\shader0 = "../stock.slang"
        \\filter_linear0 = "false"
        \\scale_type_x0 = "absolute"
        \\scale_x0 = "640.000000"
        \\scale_type_y0 = "source"
        \\scale_y0 = "1.000000"
        \\
        \\shader1 = "blargg/blargg-0.slang"
        \\filter_linear1 = false
        \\
        \\shader2 = "blargg/blargg-1.slang"
        \\filter_linear2 = false
    ;
    var shader_config = try parse_slangp(alloc, source);
    defer shader_config.deinit(alloc);

    try std.testing.expectEqual(3, shader_config.total_passes);
    try std.testing.expectEqualStrings("../stock.slang", shader_config.passes[0].path);
    try std.testing.expectEqual(false, shader_config.passes[0].params.filter_linear.?);
    try std.testing.expectEqual(.absolute, shader_config.passes[0].params.scale_type_x.?);
    try std.testing.expectEqual(.source, shader_config.passes[0].params.scale_type_y.?);
    try std.testing.expectApproxEqAbs(640.0, shader_config.passes[0].params.scale_x.?, 0.001);
    try std.testing.expectApproxEqAbs(1.0, shader_config.passes[0].params.scale_y.?, 0.001);
}

test "parse_slangp_with_params" {
    const alloc = std.testing.allocator;
    const source =
        \\shaders = "3"
        \\
        \\shader0 = "../stock.slang"
        \\filter_linear0 = "false"
        \\scale_type_x0 = "absolute"
        \\ntscsignal = "1.000000"
        \\avalue = "0.000000"
        \\bvalue = "0.000000"
        \\scantime = "47.900070"
    ;
    var shader_config = try parse_slangp(alloc, source);
    defer shader_config.deinit(alloc);

    const expected = [_]struct { []const u8, f32 }{
        .{ "ntscsignal", 1.0 },
        .{ "avalue", 0.0 },
        .{ "bvalue", 0.0 },
        .{ "scantime", 47.90007 },
    };

    try std.testing.expectEqual(4, shader_config.shader_params_initial_values.count());
    for (expected) |entry| {
        const value = shader_config.shader_params_initial_values.get(entry[0]);
        try std.testing.expect(value != null);
        try std.testing.expectApproxEqAbs(entry[1], value.?.Float, 0.0001);
    }
}

test "parse_slangp_with_textures" {
    const alloc = std.testing.allocator;
    const source =
        \\shaders = "3"
        \\
        \\shader0 = "../stock.slang"
        \\filter_linear0 = "false"
        \\scale_type_x0 = "absolute"
        \\scantime = "47.900070"
        \\
        \\textures = COLOR_PALETTE;BACKGROUND;SamplerLUT1
        \\COLOR_PALETTE = gameboy/resources/palette.png
        \\COLOR_PALETTE_linear = false
        \\BACKGROUND = gameboy/resources/background.png
        \\BACKGROUND_linear = true
        \\SamplerLUT1 = shaders/guest/advanced/lut/trinitron-lut.png
        \\SamplerLUT1_linear = true
        \\SamplerLUT1_wrap_mode = repeat
        \\SamplerLUT1_mipmap = true
    ;
    var shader_config = try parse_slangp(alloc, source);
    defer shader_config.deinit(alloc);

    const expected = [_]struct { []const u8, []const u8, bool }{
        .{ "COLOR_PALETTE", "gameboy/resources/palette.png", false },
        .{ "BACKGROUND", "gameboy/resources/background.png", true },
        .{ "SamplerLUT1", "shaders/guest/advanced/lut/trinitron-lut.png", true },
    };

    try std.testing.expectEqual(3, shader_config.textures.count());
    for (expected) |entry| {
        const value = shader_config.textures.get(entry[0]);
        try std.testing.expect(value != null);
        try std.testing.expect(std.mem.eql(u8, value.?.path, entry[1]));
        try std.testing.expectEqual(entry[2], value.?.linear);
    }
    const sampler_lut = shader_config.textures.get("SamplerLUT1").?;
    try std.testing.expectEqual(WrapMode.repeat, sampler_lut.wrap_mode);
    try std.testing.expect(sampler_lut.mipmap);
}

test "parse_slangp_with_line_comments" {
    const alloc = std.testing.allocator;
    const source =
        \\shaders = 2
        \\
        \\// Monitor-like spot size (less variation than TVs)
        \\MaxSpotSize = 0.95
        \\// From https://github.com/Calinou/free-blue-noise-textures. Using
        \\textures = BlueNoiseTex
        \\BlueNoiseTex = shaders/crt-beans/blue_noise.png
        \\
        \\//======== Linearize gamma.
        \\shader0 = shaders/crt-beans/linearize.slang
        \\alias0 = Linearized
        \\shader1 = shaders/crt-beans/output.slang
    ;
    var shader_config = try parse_slangp(alloc, source);
    defer shader_config.deinit(alloc);

    try std.testing.expectEqual(2, shader_config.total_passes);
    try std.testing.expectEqualStrings("shaders/crt-beans/linearize.slang", shader_config.passes[0].path);
    try std.testing.expectEqualStrings("Linearized", shader_config.passes[0].params.alias.?);
    try std.testing.expectEqualStrings("shaders/crt-beans/output.slang", shader_config.passes[1].path);
    try std.testing.expectEqualStrings("shaders/crt-beans/blue_noise.png", shader_config.textures.get("BlueNoiseTex").?.path);
    try std.testing.expectEqual(1, shader_config.shader_params_initial_values.count());
    try std.testing.expectEqual(0.95, shader_config.shader_params_initial_values.get("MaxSpotSize").?.Float);
}
