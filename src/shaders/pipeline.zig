/// RetroArch-compatible shader pipeline for NES emulation.
///
/// Supports multi-pass `.slangp` preset files with full SPIR-V compilation,
/// shader reflection, feedback loops, LUT textures, and frame history.
///
/// Everything here is independent of the graphics API: `Pipeline` is
/// instantiated with a backend that creates and binds the GPU objects
/// (`sdl_gpu_backend.zig` natively, `wasm/gles_backend.zig` for WebGL 2).
const std = @import("std");
const builtin = @import("builtin");
const features = @import("features");

const c = @import("../root.zig").c;
const builtin_shaders = @import("builtin.zig");
const slangp = @import("slangp.zig");
const parser = @import("parser.zig");
const ShaderCache = @import("cache.zig").ShaderCache;
pub const SpirvPair = @import("cache.zig").SpirvPair;
const ThreadPool = @import("../utils/pool.zig");
const vulkan = @import("../utils/vulkan.zig");

pub const ShaderPipeline = Pipeline(if (features.wasm)
    @import("../wasm/gles_backend.zig").Backend
else
    @import("sdl_gpu_backend.zig").Backend);

pub const Viewport = struct { x: i32, y: i32, w: u32, h: u32 };

const BuiltinUniforms = struct {
    MVP: [16]f32 = .{
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
    },
    OriginalSize: [4]f32 = undefined,
    OutputSize: [4]f32 = undefined,
    SourceSize: [4]f32 = undefined,
    FinalViewportSize: [4]f32 = undefined,
    FrameCount: u32 = 0,
};

const FieldType = union(enum) {
    MVP,
    SourceSize,
    OutputSize,
    OriginalSize,
    FinalViewportSize,
    FrameCount,
    OriginalFPS,
    OriginalAspect,
    OriginalAspectRotated,
    FrameTimeDelta,
    Rotation,
    /// A size uniform for a named alias (e.g. `MyPassSize`).
    SizeVariant: []const u8, // owned
    /// A size uniform for a numbered alias (e.g. `MyPass0Size`).
    SizeVariantWithId: struct { name: []const u8, id: usize }, // name owned
    /// Any other custom parameter.
    Other: []const u8, // owned

    fn deinit(self: FieldType, alloc: std.mem.Allocator) void {
        switch (self) {
            .SizeVariant => |name| alloc.free(name),
            .SizeVariantWithId => |it| alloc.free(it.name),
            .Other => |name| alloc.free(name),
            else => {},
        }
    }
};

const MemberInfo = struct {
    field_type: FieldType,
    offset: u32,
    size: u32,
    param_data: ?[]u8 = null,
};

const UniformBufferLayout = struct {
    size: u32,
    members: std.ArrayList(MemberInfo),

    fn deinit(self: *UniformBufferLayout, alloc: std.mem.Allocator) void {
        for (self.members.items) |member| {
            member.field_type.deinit(alloc);
        }
        self.members.deinit(alloc);
    }
};

const SamplerType = union(enum) {
    Original,
    Source,
    OriginalHistory: usize,
    PassOutput: usize,
    PassFeedback: usize,
    Alias: []const u8, // not owned (points into reflection name)
};

const BindingType = union(enum) {
    push_params: UniformBufferLayout,
    UBO: UniformBufferLayout,
    sampler2D: SamplerType,
};

const BindingInfo = struct {
    name: []const u8, // owned
    binding: u32,
    binding_type: BindingType,
    uniform_payload: []u8,
};

const DescriptorSetInfo = struct {
    set_number: u32,
    bindings: std.ArrayList(BindingInfo),

    fn deinit(self: *DescriptorSetInfo, alloc: std.mem.Allocator) void {
        for (self.bindings.items) |*binding| {
            alloc.free(binding.name);
            if (binding.uniform_payload.len > 0) alloc.free(binding.uniform_payload);
            switch (binding.binding_type) {
                .push_params, .UBO => |*layout| layout.deinit(alloc),
                .sampler2D => {},
            }
        }
        self.bindings.deinit(alloc);
    }
};

pub const ShaderReflection = struct {
    descriptor_sets: std.ArrayList(DescriptorSetInfo) = .empty,

    pub fn deinit(self: *ShaderReflection, alloc: std.mem.Allocator) void {
        for (self.descriptor_sets.items) |*set| {
            set.deinit(alloc);
        }
        self.descriptor_sets.deinit(alloc);
    }
};

pub const ParamInfo = struct {
    /// Key used to read/write the value (points into pass reflection data).
    name: []const u8,
    /// Human-readable label from `#pragma parameter` (owned by LoadedPreset).
    display_name: []const u8,
    min: f32,
    max: f32,
    step: ?f32,
};

fn spvcError(result: c.spvc_result) void {
    switch (result) {
        c.SPVC_ERROR_INVALID_SPIRV => std.debug.panic("SPVC: Invalid SPIRV", .{}),
        c.SPVC_ERROR_UNSUPPORTED_SPIRV => std.debug.panic("SPVC: Unsupported SPIRV", .{}),
        c.SPVC_ERROR_OUT_OF_MEMORY => std.debug.panic("SPVC: Out of memory", .{}),
        c.SPVC_ERROR_INVALID_ARGUMENT => std.debug.panic("SPVC: Invalid argument", .{}),
        else => {},
    }
}

fn getSpirvVersion(vk_version: c_uint) c_uint {
    return switch (vk_version) {
        c.GLSLANG_TARGET_VULKAN_1_0 => c.GLSLANG_TARGET_SPV_1_0,
        c.GLSLANG_TARGET_VULKAN_1_1 => c.GLSLANG_TARGET_SPV_1_3,
        c.GLSLANG_TARGET_VULKAN_1_2 => c.GLSLANG_TARGET_SPV_1_5,
        c.GLSLANG_TARGET_VULKAN_1_3, c.GLSLANG_TARGET_VULKAN_1_4 => c.GLSLANG_TARGET_SPV_1_6,
        else => c.GLSLANG_TARGET_SPV_1_0,
    };
}

fn compileShader(
    alloc: std.mem.Allocator,
    vk_version: c_uint,
    glsl_version: u32,
    source: []const u8,
    stage: parser.ShaderStage,
) ![]u8 {
    const source_z = try alloc.dupeZ(u8, source);
    defer alloc.free(source_z);

    const target_version = vulkan.vk_to_glslang_version(vk_version);
    const glslang_input: c.glslang_input_t = .{
        .client = c.GLSLANG_CLIENT_VULKAN,
        .language = c.GLSLANG_SOURCE_GLSL,
        .stage = if (stage == .Vertex) c.GLSLANG_STAGE_VERTEX else c.GLSLANG_STAGE_FRAGMENT,
        .client_version = target_version,
        .target_language = c.GLSLANG_TARGET_SPV,
        .target_language_version = getSpirvVersion(target_version),
        .code = source_z.ptr,
        .default_version = @intCast(glsl_version),
        .default_profile = c.GLSLANG_NO_PROFILE,
        .messages = c.GLSLANG_MSG_DEFAULT_BIT,
        .resource = c.glslang_default_resource(),
    };

    const glslang_shader = c.glslang_shader_create(&glslang_input);
    defer c.glslang_shader_delete(glslang_shader);

    if (c.glslang_shader_preprocess(glslang_shader, &glslang_input) != 1) {
        std.log.err("GLSL preprocess failed:\n{s}\n{s}", .{
            c.glslang_shader_get_info_log(glslang_shader),
            c.glslang_shader_get_info_debug_log(glslang_shader),
        });
        return error.GLSL_PreprocessFailed;
    }

    const preprocessed_code = c.glslang_shader_get_preprocessed_code(glslang_shader);
    const patched = try parser.patchShaderSource(alloc, std.mem.span(preprocessed_code), stage);
    defer alloc.free(patched);
    const patched_z = try alloc.dupeZ(u8, patched);
    defer alloc.free(patched_z);

    c.glslang_shader_set_preprocessed_code(glslang_shader, patched_z.ptr);

    if (c.glslang_shader_parse(glslang_shader, &glslang_input) != 1) {
        std.log.err("GLSL parse failed:\n{s}", .{c.glslang_shader_get_info_log(glslang_shader)});
        return error.GLSL_ParsingFailed;
    }

    const program = c.glslang_program_create();
    defer c.glslang_program_delete(program);

    c.glslang_program_add_shader(program, glslang_shader);

    if (c.glslang_program_link(program, c.GLSLANG_MSG_SPV_RULES_BIT | c.GLSLANG_MSG_VULKAN_RULES_BIT) != 1) {
        std.log.err("GLSL link failed:\n{s}", .{c.glslang_program_get_info_log(program)});
        return error.GLSL_LinkingFailed;
    }

    if (c.glslang_program_map_io(program) != 1) {
        std.log.err("GLSL IO mapping failed:\n{s}", .{c.glslang_program_get_info_log(program)});
        return error.GLSL_IOMappingFailed;
    }

    var options = [_]c.struct_glslang_spv_options_s{.{
        .generate_debug_info = builtin.mode == .Debug,
        .strip_debug_info = false,
        .disable_optimizer = builtin.mode == .Debug,
        .optimize_size = false,
        .disassemble = false,
        .validate = true,
        .emit_nonsemantic_shader_debug_info = false,
        .emit_nonsemantic_shader_debug_source = false,
        .compile_only = false,
        .optimize_allow_expanded_id_bound = false,
    }};
    c.glslang_program_SPIRV_generate_with_options(program, glslang_input.stage, &options);

    if (c.glslang_program_SPIRV_get_messages(program)) |msg| {
        std.log.debug("SPIRV messages: {s}", .{msg});
    }

    const size = c.glslang_program_SPIRV_get_size(program);
    const ptr = c.glslang_program_SPIRV_get_ptr(program);
    return try alloc.dupe(u8, std.mem.sliceAsBytes(ptr[0..size]));
}

fn reflectResources(
    alloc: std.mem.Allocator,
    resources: c.spvc_resources,
    compiler: c.spvc_compiler,
    resource_type: c.spvc_resource_type,
    reflection: *ShaderReflection,
) !void {
    var resource_list: [*c]const c.spvc_reflected_resource = undefined;
    var count: usize = 0;
    spvcError(c.spvc_resources_get_resource_list_for_type(resources, resource_type, &resource_list, &count));

    for (resource_list[0..count]) |res| {
        const name = try alloc.dupe(u8, std.mem.span(res.name));
        const set = c.spvc_compiler_get_decoration(compiler, res.id, c.SpvDecorationDescriptorSet);
        const binding = c.spvc_compiler_get_decoration(compiler, res.id, c.SpvDecorationBinding);

        var set_info = DescriptorSetInfo{
            .set_number = @intCast(set),
            .bindings = .empty,
        };

        switch (resource_type) {
            c.SPVC_RESOURCE_TYPE_UNIFORM_BUFFER => {
                const struct_type_id = res.base_type_id;
                const struct_type = c.spvc_compiler_get_type_handle(compiler, struct_type_id);

                var block_size: usize = 0;
                spvcError(c.spvc_compiler_get_declared_struct_size(compiler, struct_type, &block_size));

                var layout = UniformBufferLayout{
                    .size = @intCast(block_size),
                    .members = .empty,
                };

                const member_count = c.spvc_type_get_num_member_types(struct_type);
                for (0..member_count) |i| {
                    var offset: u32 = 0;
                    spvcError(c.spvc_compiler_type_struct_member_offset(compiler, struct_type, @intCast(i), &offset));

                    var msize: usize = 0;
                    spvcError(c.spvc_compiler_get_declared_struct_member_size(compiler, struct_type, @intCast(i), &msize));

                    const member_name_raw = c.spvc_compiler_get_member_name(compiler, struct_type_id, @intCast(i));
                    const member_name = try alloc.dupe(u8, std.mem.span(member_name_raw));
                    defer alloc.free(member_name);

                    var field_type: FieldType = classifyFieldName(member_name) orelse
                        .{ .Other = try alloc.dupe(u8, member_name) };

                    // Detect size variants for pass aliases (e.g. `MyPassSize`, `MyPass0Size`)
                    if (field_type == .Other) {
                        if (std.mem.indexOf(u8, member_name, "Size")) |idx| {
                            const var_name = member_name[0..idx];
                            const is_builtin = std.mem.eql(u8, var_name, "Original") or
                                std.mem.eql(u8, var_name, "Source") or
                                std.mem.eql(u8, var_name, "Output") or
                                std.mem.eql(u8, var_name, "Final");
                            if (!is_builtin) {
                                // Free what classifyFieldName set (nothing, we set Other above)
                                alloc.free(field_type.Other);
                                const suffix = member_name[idx + 4 ..];
                                if (suffix.len > 0 and std.ascii.isDigit(suffix[0])) {
                                    field_type = .{ .SizeVariantWithId = .{
                                        .name = try alloc.dupe(u8, var_name),
                                        .id = try std.fmt.parseInt(usize, suffix[0..1], 10),
                                    } };
                                } else {
                                    field_type = .{ .SizeVariant = try alloc.dupe(u8, var_name) };
                                }
                            }
                        }
                    }

                    try layout.members.append(alloc, .{
                        .field_type = field_type,
                        .offset = offset,
                        .size = @intCast(msize),
                    });
                }

                const uniform_payload = try alloc.alloc(u8, block_size);
                var payload_transferred = false;
                defer if (!payload_transferred) alloc.free(uniform_payload);

                try set_info.bindings.append(alloc, .{
                    .name = name,
                    .binding = @intCast(binding),
                    .binding_type = if (std.mem.eql(u8, name, "Push"))
                        .{ .push_params = layout }
                    else
                        .{ .UBO = layout },
                    .uniform_payload = uniform_payload,
                });
                payload_transferred = true;
            },

            c.SPVC_RESOURCE_TYPE_SAMPLED_IMAGE => {
                const sampler_type: SamplerType = classifySamplerName(name);
                try set_info.bindings.append(alloc, .{
                    .name = name,
                    .binding = @intCast(binding),
                    .binding_type = .{ .sampler2D = sampler_type },
                    .uniform_payload = &.{},
                });
            },
            else => {
                alloc.free(name);
                continue;
            },
        }

        try reflection.descriptor_sets.append(alloc, set_info);
    }
}

fn classifyFieldName(name: []const u8) ?FieldType {
    if (std.mem.eql(u8, name, "MVP")) return .MVP;
    if (std.mem.eql(u8, name, "Rotation")) return .Rotation;
    if (std.mem.eql(u8, name, "SourceSize")) return .SourceSize;
    if (std.mem.eql(u8, name, "OutputSize")) return .OutputSize;
    if (std.mem.eql(u8, name, "OriginalSize")) return .OriginalSize;
    if (std.mem.eql(u8, name, "FinalViewportSize")) return .FinalViewportSize;
    if (std.mem.eql(u8, name, "FrameCount")) return .FrameCount;
    if (std.mem.eql(u8, name, "FrameTimeDelta")) return .FrameTimeDelta;
    if (std.mem.eql(u8, name, "OriginalFPS")) return .OriginalFPS;
    if (std.mem.eql(u8, name, "OriginalAspect")) return .OriginalAspect;
    if (std.mem.eql(u8, name, "OriginalAspectRotated")) return .OriginalAspectRotated;
    return null;
}

fn classifySamplerName(name: []const u8) SamplerType {
    if (std.mem.eql(u8, name, "Original")) return .Original;
    if (std.mem.eql(u8, name, "Source")) return .Source;
    if (std.mem.startsWith(u8, name, "PassFeedback")) {
        const id = slangp.get_field_pass_id(name) catch return .{ .Alias = name };
        return .{ .PassFeedback = id };
    }
    if (std.mem.startsWith(u8, name, "PassOutput")) {
        const id = slangp.get_field_pass_id(name) catch return .{ .Alias = name };
        return .{ .PassOutput = id };
    }
    if (std.mem.startsWith(u8, name, "OriginalHistory")) {
        const id = slangp.get_field_pass_id(name) catch return .{ .Alias = name };
        return .{ .OriginalHistory = id };
    }
    return .{ .Alias = name };
}

fn reflectShaderInfo(alloc: std.mem.Allocator, spirv: []const u8) !ShaderReflection {
    var reflection: ShaderReflection = .{};
    errdefer reflection.deinit(alloc);

    var context: c.spvc_context = undefined;
    spvcError(c.spvc_context_create(&context));
    defer c.spvc_context_destroy(context);

    var parsed_ir: c.spvc_parsed_ir = undefined;
    spvcError(c.spvc_context_parse_spirv(
        context,
        @ptrCast(@alignCast(spirv.ptr)),
        spirv.len / @sizeOf(u32),
        &parsed_ir,
    ));

    var compiler: c.spvc_compiler = undefined;
    spvcError(c.spvc_context_create_compiler(
        context,
        c.SPVC_BACKEND_NONE,
        parsed_ir,
        c.SPVC_CAPTURE_MODE_TAKE_OWNERSHIP,
        &compiler,
    ));

    var resources: c.spvc_resources = undefined;
    spvcError(c.spvc_compiler_create_shader_resources(compiler, &resources));

    try reflectResources(alloc, resources, compiler, c.SPVC_RESOURCE_TYPE_UNIFORM_BUFFER, &reflection);
    try reflectResources(alloc, resources, compiler, c.SPVC_RESOURCE_TYPE_SAMPLED_IMAGE, &reflection);

    return reflection;
}

const Param = struct {
    name: []const u8, // borrowed from reflection (not owned here)
    display_name: []const u8, // owned (duped from ShaderParam.option_name)
    value: f32,
    min: f32,
    max: f32,
    step: ?f32,
};

/// Returns SPIR-V for both stages, consulting the cache when available.
fn getOrCompileSpirv(
    alloc: std.mem.Allocator,
    vk_version: c_uint,
    shader: *const parser.ParsedShader,
    shader_cache: *ShaderCache,
    pass_path: []const u8,
    pass_id: usize,
) !SpirvPair {
    const key = ShaderCache.computeKey(
        @intCast(vk_version),
        shader.version,
        shader.vertex,
        shader.fragment,
    );

    if (shader_cache.lookup(alloc, key)) |pair| {
        std.log.info("Loading shader pass {} from cache", .{pass_id});
        return pair;
    }

    std.log.info("Compiling shader pass {}: {s}", .{ pass_id, pass_path });
    const vert = try compileShader(alloc, vk_version, shader.version, shader.vertex, .Vertex);
    const frag = try compileShader(alloc, vk_version, shader.version, shader.fragment, .Fragment);

    try shader_cache.store(key, vert, frag);

    return .{ .vert = vert, .frag = frag };
}

const ParsedPreset = struct {
    content: []u8,
    shader_config: slangp.ShaderConfig,
    dir: []u8,

    fn deinit(self: *ParsedPreset, alloc: std.mem.Allocator) void {
        self.shader_config.deinit(alloc);
        alloc.free(self.dir);
        alloc.free(self.content);
    }
};

/// Read and parse a `.slangp` preset file.
fn parsePresetFile(alloc: std.mem.Allocator, io: std.Io, path: []const u8) !ParsedPreset {
    if (builtin_shaders.sourceForPath(path)) |source| {
        const content = try alloc.dupe(u8, source);
        errdefer alloc.free(content);

        const dir = try alloc.dupe(u8, std.fs.path.dirname(path) orelse builtin_shaders.border_shader_dir);
        errdefer alloc.free(dir);

        const shader_config = try slangp.parse_slangp(alloc, content);
        return .{ .content = content, .shader_config = shader_config, .dir = dir };
    }

    const full_path = std.Io.Dir.cwd().realPathFileAlloc(io, path, alloc) catch blk: {
        if (std.fs.path.isAbsolute(path)) break :blk try alloc.dupeZ(u8, path);

        const cwd = try std.process.currentPathAlloc(io, alloc);
        defer alloc.free(cwd);
        const resolved = try std.fs.path.resolve(alloc, &.{ cwd, path });
        defer alloc.free(resolved);

        break :blk try alloc.dupeZ(u8, resolved);
    };
    defer alloc.free(full_path);
    const dir = try alloc.dupe(u8, std.fs.path.dirname(full_path).?);
    errdefer alloc.free(dir);

    const content = try std.Io.Dir.cwd().readFileAlloc(io, full_path, alloc, .limited(1024 * 1024));
    errdefer alloc.free(content);

    const shader_config = try slangp.parse_slangp(alloc, content);
    return .{ .content = content, .shader_config = shader_config, .dir = dir };
}


/// Format of a render target or texture. `default` is the backend's
/// presentation format (the swapchain format on SDL GPU, RGBA8 on GLES).
pub const TextureFormat = enum {
    default,
    r8_unorm,
    rgba8_unorm,
    rgba8_srgb,
    r16_float,
    rg16_float,
    rgba16_float,
    r32_float,
    rg32_float,
    rgba32_float,
    rgb10a2_unorm,

    fn parse(name: []const u8) TextureFormat {
        const names = [_]struct { []const u8, TextureFormat }{
            .{ "R8_UNORM", .r8_unorm },
            .{ "R8G8B8A8_UNORM", .rgba8_unorm },
            .{ "R8G8B8A8_SRGB", .rgba8_srgb },
            .{ "R16_SFLOAT", .r16_float },
            .{ "R16G16_SFLOAT", .rg16_float },
            .{ "R16G16B16A16_SFLOAT", .rgba16_float },
            .{ "R32_SFLOAT", .r32_float },
            .{ "R32G32_SFLOAT", .rg32_float },
            .{ "R32G32B32A32_SFLOAT", .rgba32_float },
            .{ "A2B10G10R10_UNORM_PACK32", .rgb10a2_unorm },
        };
        for (names) |entry| {
            if (std.mem.eql(u8, name, entry[0])) return entry[1];
        }
        std.log.warn("Unknown texture format '{s}', defaulting to R8G8B8A8_UNORM", .{name});
        return .rgba8_unorm;
    }
};

/// How a pass samples its inputs.
pub const SamplerDesc = struct {
    linear: bool = false,
    wrap_mode: slangp.WrapMode = .clamp_to_edge,
    /// The pass reads its input with mipmapping (`mipmap_input`).
    mipmaps: bool = false,
};

const ScaleParams = struct {
    scale_type: ?slangp.ScaleType = null,
    scale_type_x: ?slangp.ScaleType = null,
    scale_type_y: ?slangp.ScaleType = null,
    scale: ?f32 = null,
    scale_x: ?f32 = null,
    scale_y: ?f32 = null,

    fn outputSize(self: ScaleParams, input_w: u32, input_h: u32, viewport_w: u32, viewport_h: u32) struct { w: u32, h: u32 } {
        const out_w = scaleAxis(self.scale_type_x orelse self.scale_type, self.scale_x orelse self.scale, input_w, viewport_w);
        const out_h = scaleAxis(self.scale_type_y orelse self.scale_type, self.scale_y orelse self.scale, input_h, viewport_h);
        // Zero-sized targets cannot be created, e.g. while a window is collapsed.
        return .{ .w = @max(1, out_w), .h = @max(1, out_h) };
    }

    fn scaleAxis(scale_type: ?slangp.ScaleType, scale: ?f32, input: u32, viewport: u32) u32 {
        const factor = scale orelse 1.0;
        return switch (scale_type orelse .source) {
            .source => @intFromFloat(@as(f32, @floatFromInt(input)) * factor),
            .viewport => @intFromFloat(@as(f32, @floatFromInt(viewport)) * factor),
            .absolute => @intFromFloat(factor),
        };
    }
};

/// RGBA8 pixels of a decoded look-up texture.
const LutImage = struct {
    pixels: []u8,
    width: u32,
    height: u32,
};

fn decodeLut(alloc: std.mem.Allocator, name: []const u8, path: []const u8) !LutImage {
    const path_z = try alloc.dupeZ(u8, path);
    defer alloc.free(path_z);

    var surface = c.SDL_LoadPNG(path_z.ptr) orelse {
        std.log.err("Failed to decode LUT PNG '{s}' ({s}): {s}", .{ name, path, c.SDL_GetError() });
        return error.FailedToLoadLutTexture;
    };
    defer c.SDL_DestroySurface(surface);

    if (surface.*.format != c.SDL_PIXELFORMAT_RGBA32) {
        const converted = c.SDL_ConvertSurface(surface, c.SDL_PIXELFORMAT_RGBA32) orelse {
            std.log.err("Failed to convert LUT '{s}' to RGBA: {s}", .{ name, c.SDL_GetError() });
            return error.FailedToLoadLutTexture;
        };
        c.SDL_DestroySurface(surface);
        surface = converted;
    }

    const w: u32 = @intCast(surface.*.w);
    const h: u32 = @intCast(surface.*.h);
    const row_size = w * 4;
    const pixels = try alloc.alloc(u8, row_size * h);
    const src: [*]const u8 = @ptrCast(surface.*.pixels);
    const pitch: usize = @intCast(surface.*.pitch);
    for (0..h) |row| {
        @memcpy(pixels[row * row_size ..][0..row_size], src[row * pitch ..][0..row_size]);
    }
    return .{ .pixels = pixels, .width = w, .height = h };
}

/// Collect the parameters defined via `#pragma parameter` that the shader's
/// uniform blocks use. The display names are owned by the caller.
fn collectParams(alloc: std.mem.Allocator, shader: *const parser.ParsedShader, reflection: *const ShaderReflection) ![]Param {
    var params = std.ArrayList(Param).empty;
    errdefer {
        for (params.items) |param| alloc.free(param.display_name);
        params.deinit(alloc);
    }
    const config = shader.config orelse return params.toOwnedSlice(alloc);
    for (reflection.descriptor_sets.items) |set_info| {
        for (set_info.bindings.items) |binding| {
            switch (binding.binding_type) {
                .push_params, .UBO => |layout| {
                    for (layout.members.items) |member| {
                        if (member.field_type != .Other) continue;
                        const pname = member.field_type.Other;
                        const field = config.getPtr(pname) orelse blk: {
                            std.log.warn("UBO field '{s}' has no #pragma parameter, defaulting to 0", .{pname});
                            break :blk &parser.ShaderParam{ .option_name = pname };
                        };
                        try params.append(alloc, .{
                            .name = pname,
                            .display_name = try alloc.dupe(u8, field.option_name),
                            .value = field.initial,
                            .min = field.min,
                            .max = field.max,
                            .step = field.step,
                        });
                    }
                },
                .sampler2D => {},
            }
        }
    }
    return params.toOwnedSlice(alloc);
}

fn calcMipLevels(w: u32, h: u32) u32 {
    var size = @max(w, h);
    var levels: u32 = 0;
    while (size != 0) : (levels += 1) size >>= 1;
    return levels;
}

/// The shader pipeline, parameterized by the graphics API `Backend`. All of
/// the RetroArch semantics live here; the backend only creates and binds GPU
/// objects. Backends must provide:
///
/// - `TextureHandle`, `Sampler`, `PassObjects` (default-initializable),
///   `Frame` (per-frame context), `RenderPass` and `InitArgs` types;
/// - a `vk_version` field (the Vulkan version shaders are compiled for);
/// - `init`, `deinit`, `compileJobs`;
/// - `compilePass` (runs on worker threads), `activatePass` (main thread),
///   `destroyPass`;
/// - `createTexture`, `destroyTexture`, `uploadTexture`, `copyTexture`,
///   `generateMipmaps`, `createSampler`, `destroySampler`;
/// - `beginPasses`, `endPasses`, `beginPass`, `pushUniforms`, `bindTexture`,
///   `endPass`.
pub fn Pipeline(comptime BackendType: type) type {
    return struct {
        alloc: std.mem.Allocator,
        io: std.Io,
        backend: Backend,

        preset: ?LoadedPreset = null,
        preset_path: ?[]u8 = null,
        cache: ShaderCache,

        // Async compile state
        compile_progress: std.atomic.Value(u32) = .init(0),
        compile_total: u32 = 0,
        compile_thread: ?std.Thread = null,
        compile_mutex: std.Io.Mutex = .init,
        compile_failed: bool = false,
        compile_error_msg: ?[]u8 = null,
        compile_done: std.atomic.Value(bool) = .init(false),
        /// Preset finished by the compile thread; `pollLoadResult` activates it
        /// on the rendering thread.
        compiled_preset: ?LoadedPreset = null,
        compiled_path: ?[]u8 = null,

        // Usage flags derived from preset reflection at load time
        uses_pass_output: bool = false,
        uses_pass_feedback: bool = false,

        // Per-frame accumulation state
        frame_count: u32 = 0,
        pass_output: []?*Texture = &.{},
        pass_feedback: []?*Texture = &.{},
        texture_aliases: std.StringHashMap(*Texture),
        /// Copies of previous frames; slot N holds the frame from N + 1 frames ago.
        frame_history: std.ArrayList(?*Texture) = .empty,

        const Self = @This();
        pub const Backend = BackendType;

        pub const Texture = struct {
            alloc: std.mem.Allocator,
            ptr: ?Backend.TextureHandle = null,
            width: u32 = 0,
            height: u32 = 0,
            format: TextureFormat = .default,
            num_levels: u32 = 1,
            refcount: u32 = 0,

            fn init(
                alloc: std.mem.Allocator,
                ptr: ?Backend.TextureHandle,
                w: u32,
                h: u32,
                format: TextureFormat,
                num_levels: u32,
            ) !*Texture {
                const obj = try alloc.create(Texture);
                obj.* = .{
                    .alloc = alloc,
                    .ptr = ptr,
                    .width = w,
                    .height = h,
                    .format = format,
                    .num_levels = num_levels,
                    .refcount = 1,
                };
                return obj;
            }

            fn matches(self: *const Texture, w: u32, h: u32, format: TextureFormat, num_levels: u32) bool {
                return self.ptr != null and
                    self.width == w and
                    self.height == h and
                    self.format == format and
                    self.num_levels == num_levels;
            }

            fn release(self: *Texture, backend: *Backend) void {
                if (self.refcount > 0) self.refcount -= 1;
                if (self.refcount == 0) {
                    if (self.ptr) |ptr| backend.destroyTexture(ptr);
                    self.alloc.destroy(self);
                }
            }

            fn ref(self: *Texture) *Texture {
                self.refcount += 1;
                return self;
            }

            /// Returns `[width, height, 1/width, 1/height]` as expected by RetroArch uniforms.
            fn sizeVec4(self: Texture) [4]f32 {
                return sizeVec4From(self.width, self.height);
            }
        };

        /// Look-up texture
        const Lut = struct {
            /// Created on the main thread by `activatePreset`.
            texture: ?*Texture = null,
            sampler: ?Backend.Sampler = null,
            /// Decoded by the compile thread and freed once uploaded.
            image: ?LutImage = null,
            sampler_desc: SamplerDesc,

            fn deinit(self: *Lut, alloc: std.mem.Allocator, backend: *Backend) void {
                if (self.texture) |t| t.release(backend);
                if (self.sampler) |s| backend.destroySampler(s);
                if (self.image) |image| alloc.free(image.pixels);
            }
        };

        const ShaderPass = struct {
            id: usize = 0,
            gpu: Backend.PassObjects = .{},
            output_texture: ?*Texture = null,
            fb_feedback: ?*Texture = null,
            scale_params: ScaleParams = .{},
            sampler: SamplerDesc = .{},
            frame_count_mod: ?u32 = null,
            alias: ?[]const u8 = null, // owned
            feedback_alias: ?[]const u8 = null, // owned
            format: TextureFormat = .default,
            vertex_reflection: ShaderReflection = .{},
            fragment_reflection: ShaderReflection = .{},

            fn deinit(self: *ShaderPass, alloc: std.mem.Allocator, backend: *Backend) void {
                backend.destroyPass(alloc, &self.gpu);
                if (self.output_texture) |t| t.release(backend);
                if (self.fb_feedback) |t| t.release(backend);
                if (self.alias) |a| alloc.free(a);
                if (self.feedback_alias) |a| alloc.free(a);
                self.vertex_reflection.deinit(alloc);
                self.fragment_reflection.deinit(alloc);
            }
        };

        const LoadedPreset = struct {
            passes: []ShaderPass,
            /// Parameter name -> bytes of the f32 value.
            param_data: std.StringHashMap([]u8),
            /// Ordered list of parameter metadata for UI display.
            param_meta: std.ArrayList(ParamInfo) = .empty,
            luts: std.StringHashMap(Lut),

            fn deinit(self: *LoadedPreset, alloc: std.mem.Allocator, backend: *Backend) void {
                for (self.passes) |*pass| pass.deinit(alloc, backend);
                alloc.free(self.passes);

                var pd_it = self.param_data.valueIterator();
                while (pd_it.next()) |bytes| alloc.free(bytes.*);
                self.param_data.deinit();

                for (self.param_meta.items) |info| alloc.free(info.display_name);
                self.param_meta.deinit(alloc);

                var lut_it = self.luts.iterator();
                while (lut_it.next()) |entry| {
                    alloc.free(entry.key_ptr.*);
                    entry.value_ptr.deinit(alloc, backend);
                }
                self.luts.deinit();
            }

            /// Add the parameters of a pass, taking ownership of their display
            /// names. Parameters shared by several passes are registered once.
            fn registerParams(
                self: *LoadedPreset,
                alloc: std.mem.Allocator,
                params: []const Param,
                initial_values: *const std.StringHashMap(slangp.TypeUnion),
            ) !void {
                for (params) |param| {
                    const bytes = if (initial_values.get(param.name)) |initial|
                        try alloc.dupe(u8, initial.bytes())
                    else
                        try alloc.dupe(u8, std.mem.asBytes(&param.value));

                    if (try self.param_data.fetchPut(param.name, bytes)) |old| {
                        alloc.free(old.value);
                        alloc.free(param.display_name);
                    } else {
                        try self.param_meta.append(alloc, .{
                            .name = param.name,
                            .display_name = param.display_name,
                            .min = param.min,
                            .max = param.max,
                            .step = param.step,
                        });
                    }
                }
            }

            /// Point uniform members at their parameter value so rendering
            /// does not look them up by name every frame.
            fn resolveUniformParamRefs(self: *LoadedPreset) void {
                for (self.passes) |*pass| {
                    for ([_]*ShaderReflection{ &pass.vertex_reflection, &pass.fragment_reflection }) |reflection| {
                        for (reflection.descriptor_sets.items) |*set_info| {
                            for (set_info.bindings.items) |*binding| switch (binding.binding_type) {
                                .push_params, .UBO => |*layout| for (layout.members.items) |*member| {
                                    member.param_data = switch (member.field_type) {
                                        .Other => |name| self.param_data.get(name),
                                        else => null,
                                    };
                                },
                                .sampler2D => {},
                            };
                        }
                    }
                }
            }
        };

        const WorkerArgs = struct {
            alloc: std.mem.Allocator,
            io: std.Io,
            backend: *Backend,
            preset: *LoadedPreset,
            shader_dir: []const u8,
            pass: *slangp.ShaderPass,
            initial_values: *const std.StringHashMap(slangp.TypeUnion),
            mutex: *std.Io.Mutex,
            had_error: *bool,
            compile_progress: *std.atomic.Value(u32),
            shader_cache: *ShaderCache,
        };

        fn compilePassWorker(args: WorkerArgs) void {
            compilePassWorkerInner(args) catch |err| {
                std.log.err("Failed to compile shader pass {}: {s}", .{ args.pass.id, @errorName(err) });
                args.mutex.lockUncancelable(args.io);
                args.had_error.* = true;
                args.mutex.unlock(args.io);
            };
        }

        fn compilePassWorkerInner(args: WorkerArgs) !void {
            const alloc = args.alloc;
            const pass_params = &args.pass.params;

            const path = if (std.mem.eql(u8, args.shader_dir, builtin_shaders.border_shader_dir))
                try builtin_shaders.joinBorderShaderPath(alloc, args.pass.path)
            else
                try std.fs.path.resolve(alloc, &.{ args.shader_dir, args.pass.path });
            defer alloc.free(path);

            const embedded_source = builtin_shaders.sourceForPath(path);
            const raw_source = embedded_source orelse
                try std.Io.Dir.cwd().readFileAlloc(args.io, path, alloc, .limited(1024 * 1024));
            defer if (embedded_source == null) alloc.free(raw_source);

            var shader = try parser.parseShader(alloc, args.io, std.fs.path.dirname(path).?, raw_source);
            defer shader.deinit(alloc);

            const spirv = try getOrCompileSpirv(alloc, args.backend.vk_version, &shader, args.shader_cache, path, args.pass.id);
            defer spirv.deinit(alloc);

            var vertex_reflection = try reflectShaderInfo(alloc, spirv.vert);
            errdefer vertex_reflection.deinit(alloc);
            var fragment_reflection = try reflectShaderInfo(alloc, spirv.frag);
            errdefer fragment_reflection.deinit(alloc);

            const params = try collectParams(alloc, &shader, &vertex_reflection);
            defer alloc.free(params);
            var params_owned = true;
            errdefer if (params_owned) for (params) |param| alloc.free(param.display_name);

            const format: TextureFormat = if (shader.texture_format) |name|
                TextureFormat.parse(name)
            else if (pass_params.float_framebuffer orelse false)
                .rgba16_float
            else if (pass_params.srgb_framebuffer orelse false)
                .rgba8_srgb
            else
                .default;
            const sampler: SamplerDesc = .{
                .linear = pass_params.filter_linear orelse false,
                .wrap_mode = pass_params.wrap_mode orelse .clamp_to_edge,
                .mipmaps = pass_params.mipmap_input orelse false,
            };

            var gpu = try args.backend.compilePass(
                alloc,
                spirv,
                &vertex_reflection,
                &fragment_reflection,
                format,
                sampler,
                args.pass.id,
            );
            errdefer args.backend.destroyPass(alloc, &gpu);

            const alias: ?[]const u8 = if (shader.alias orelse pass_params.alias) |a| try alloc.dupe(u8, a) else null;
            errdefer if (alias) |a| alloc.free(a);
            const feedback_alias = if (alias) |a| try std.fmt.allocPrint(alloc, "{s}Feedback", .{a}) else null;
            errdefer if (feedback_alias) |a| alloc.free(a);

            args.mutex.lockUncancelable(args.io);
            defer args.mutex.unlock(args.io);

            params_owned = false;
            try args.preset.registerParams(alloc, params, args.initial_values);

            args.preset.passes[args.pass.id] = .{
                .id = args.pass.id,
                .gpu = gpu,
                .scale_params = .{
                    .scale_type = pass_params.scale_type,
                    .scale_type_x = pass_params.scale_type_x,
                    .scale_type_y = pass_params.scale_type_y,
                    .scale = pass_params.scale,
                    .scale_x = pass_params.scale_x,
                    .scale_y = pass_params.scale_y,
                },
                .sampler = sampler,
                .frame_count_mod = pass_params.frame_count_mod,
                .alias = alias,
                .feedback_alias = feedback_alias,
                .format = format,
                .vertex_reflection = vertex_reflection,
                .fragment_reflection = fragment_reflection,
            };
            _ = args.compile_progress.fetchAdd(1, .monotonic);
        }

        /// Decode the LUTs and compile all shader passes of a parsed preset.
        /// Runs on the compile thread; GPU objects that must be created on the
        /// rendering thread are created later by `activatePreset`.
        fn compilePreset(
            alloc: std.mem.Allocator,
            io: std.Io,
            backend: *Backend,
            parsed: *ParsedPreset,
            progress: *std.atomic.Value(u32),
            shader_cache: *ShaderCache,
        ) !LoadedPreset {
            const shader_config = &parsed.shader_config;

            var preset = LoadedPreset{
                .passes = try alloc.alloc(ShaderPass, shader_config.total_passes),
                .param_data = .init(alloc),
                .luts = .init(alloc),
            };
            @memset(preset.passes, .{});
            errdefer preset.deinit(alloc, backend);

            var tex_it = shader_config.textures.iterator();
            while (tex_it.next()) |entry| {
                const tex_path = try std.fs.path.resolve(alloc, &.{ parsed.dir, entry.value_ptr.path });
                defer alloc.free(tex_path);
                std.log.info("Loading LUT '{s}': {s}", .{ entry.key_ptr.*, tex_path });

                const image = try decodeLut(alloc, entry.key_ptr.*, tex_path);
                errdefer alloc.free(image.pixels);
                const name = try alloc.dupe(u8, entry.key_ptr.*);
                errdefer alloc.free(name);
                try preset.luts.put(name, .{ .image = image, .sampler_desc = .{
                    .linear = entry.value_ptr.linear,
                    .wrap_mode = entry.value_ptr.wrap_mode,
                } });
            }

            var mutex: std.Io.Mutex = .init;
            var had_error: bool = false;
            var args = WorkerArgs{
                .alloc = alloc,
                .io = io,
                .backend = backend,
                .preset = &preset,
                .shader_dir = parsed.dir,
                .pass = undefined,
                .initial_values = &shader_config.shader_params_initial_values,
                .mutex = &mutex,
                .had_error = &had_error,
                .compile_progress = progress,
                .shader_cache = shader_cache,
            };

            const jobs = @min(Backend.compileJobs(), shader_config.passes.len);
            if (jobs <= 1) {
                // Not worth extra threads (which are a limited resource on WASM).
                for (shader_config.passes) |*pass| {
                    args.pass = pass;
                    compilePassWorker(args);
                }
            } else {
                var pool: ThreadPool = undefined;
                try pool.init(.{ .allocator = alloc, .io = io, .n_jobs = jobs });
                defer pool.deinit();

                var wg = ThreadPool.WaitGroup.init(io);
                for (shader_config.passes) |*pass| {
                    args.pass = pass;
                    pool.spawnWg(&wg, compilePassWorker, .{args});
                }
                wg.wait();
            }

            if (had_error) return error.ShaderCompilationFailed;
            preset.resolveUniformParamRefs();
            return preset;
        }

        fn prepareUniformPayload(
            payload: []u8,
            preset: *const LoadedPreset,
            layout: *const UniformBufferLayout,
            source_tex: *Texture,
            original_tex: *Texture,
            uniforms: *const BuiltinUniforms,
            aliases: *const std.StringHashMap(*Texture),
            history: []const ?*Texture,
            pass_feedback: []const ?*Texture,
            pass_output: []const ?*Texture,
        ) []u8 {
            const payload_size: usize = @intCast(layout.size);
            std.debug.assert(payload.len >= payload_size);
            const out = payload[0..payload_size];

            for (layout.members.items) |member| {
                const dest = out[member.offset .. member.offset + member.size];
                const src: []const u8 = switch (member.field_type) {
                    .MVP => std.mem.asBytes(&uniforms.MVP),
                    .OriginalSize => std.mem.asBytes(&uniforms.OriginalSize),
                    .SourceSize => std.mem.asBytes(&uniforms.SourceSize),
                    .OutputSize => std.mem.asBytes(&uniforms.OutputSize),
                    .FinalViewportSize => std.mem.asBytes(&uniforms.FinalViewportSize),
                    .FrameCount => std.mem.asBytes(&uniforms.FrameCount),
                    .Other => |name| member.param_data orelse preset.param_data.get(name) orelse &[_]u8{0} ** 4,
                    .SizeVariant => |name| blk: {
                        const tex = aliases.get(name) orelse source_tex;
                        break :blk std.mem.asBytes(&tex.sizeVec4());
                    },
                    .SizeVariantWithId => |field| blk: {
                        const tex = if (std.mem.eql(u8, field.name, "PassFeedback"))
                            passTextureAt(pass_feedback, field.id) orelse source_tex
                        else if (std.mem.eql(u8, field.name, "OriginalHistory"))
                            historyTextureAt(history, original_tex, field.id)
                        else if (std.mem.eql(u8, field.name, "PassOutput"))
                            passTextureAt(pass_output, field.id) orelse source_tex
                        else
                            break :blk &[_]u8{0} ** 16;
                        break :blk std.mem.asBytes(&tex.sizeVec4());
                    },
                    else => &[_]u8{0} ** 16,
                };
                @memcpy(dest, src[0..@min(src.len, dest.len)]);
            }
            return out;
        }

        fn bindShaderResources(
            backend: *Backend,
            frame: Backend.Frame,
            render_pass: *Backend.RenderPass,
            pass: *ShaderPass,
            original_tex: *Texture,
            source_tex: *Texture,
            uniforms: *const BuiltinUniforms,
            preset: *const LoadedPreset,
            aliases: *const std.StringHashMap(*Texture),
            history: []const ?*Texture,
            pass_feedback: []const ?*Texture,
            pass_output: []const ?*Texture,
        ) !void {
            const stages = [_]struct { parser.ShaderStage, *ShaderReflection }{
                .{ .Vertex, &pass.vertex_reflection },
                .{ .Fragment, &pass.fragment_reflection },
            };
            for (stages) |entry| {
                const stage, const reflection = entry;
                for (reflection.descriptor_sets.items) |*set_info| {
                    for (set_info.bindings.items) |*binding| switch (binding.binding_type) {
                        .push_params, .UBO => |*layout| {
                            const payload = prepareUniformPayload(
                                binding.uniform_payload,
                                preset,
                                layout,
                                source_tex,
                                original_tex,
                                uniforms,
                                aliases,
                                history,
                                pass_feedback,
                                pass_output,
                            );
                            backend.pushUniforms(frame, render_pass, &pass.gpu, stage, binding.binding, payload);
                        },
                        .sampler2D => |sampler_type| {
                            // Only fragment shaders sample textures.
                            if (stage != .Fragment) continue;

                            var lut_sampler: ?Backend.Sampler = null; // null: the pass sampler
                            const tex: *Texture = switch (sampler_type) {
                                .Original => original_tex,
                                .Source => source_tex,
                                .OriginalHistory => |id| historyTextureAt(history, original_tex, id),
                                .PassFeedback => |id| passTextureAt(pass_feedback, id) orelse source_tex,
                                .PassOutput => |id| passTextureAt(pass_output, id) orelse source_tex,
                                .Alias => |alias| blk: {
                                    if (aliases.get(alias)) |t| break :blk t;
                                    if (preset.luts.get(alias)) |lut| {
                                        lut_sampler = lut.sampler;
                                        break :blk lut.texture.?;
                                    }
                                    std.log.warn("No texture for alias '{s}', using source", .{alias});
                                    break :blk source_tex;
                                },
                            };
                            try backend.bindTexture(render_pass, &pass.gpu, binding.binding, tex.ptr, lut_sampler);
                        },
                    };
                }
            }
        }

        fn passTextureAt(slots: []const ?*Texture, id: usize) ?*Texture {
            if (id >= slots.len) return null;
            return slots[id];
        }

        /// `OriginalHistory0` is the current frame, `OriginalHistoryN` the frame
        /// from N frames ago. Frames that have not been rendered yet fall back to
        /// the current one.
        fn historyTextureAt(history: []const ?*Texture, original_tex: *Texture, id: usize) *Texture {
            if (id == 0 or id > history.len) return original_tex;
            return history[id - 1] orelse original_tex;
        }

        fn setRetainedPassTextureSlot(backend: *Backend, slot: *?*Texture, tex: ?*Texture) void {
            if (tex) |new_tex| {
                if (slot.*) |old_tex| {
                    if (old_tex == new_tex) return;
                    old_tex.release(backend);
                }
                slot.* = new_tex.ref();
            } else {
                if (slot.*) |old_tex| old_tex.release(backend);
                slot.* = null;
            }
        }

        fn clearPassTextureSlots(backend: *Backend, slots: []?*Texture) void {
            for (slots) |*slot| {
                if (slot.*) |tex| tex.release(backend);
                slot.* = null;
            }
        }

        fn createRenderTarget(
            alloc: std.mem.Allocator,
            backend: *Backend,
            slot: *?*Texture,
            w: u32,
            h: u32,
            format: TextureFormat,
            num_levels: u32,
        ) !*Texture {
            if (slot.*) |tex| {
                if (tex.matches(w, h, format, num_levels)) return tex;
                tex.release(backend);
                slot.* = null;
            }

            const handle = try backend.createTexture(w, h, format, num_levels);
            errdefer backend.destroyTexture(handle);
            slot.* = try Texture.init(alloc, handle, w, h, format, num_levels);
            return slot.*.?;
        }

        // Main render loop for one preset
        fn renderPasses(
            self: *Self,
            frame: Backend.Frame,
            preset: *LoadedPreset,
            uniforms: *BuiltinUniforms,
            original_tex: *Texture,
            viewport: Viewport,
        ) !void {
            const backend = &self.backend;
            var source_tex = original_tex;
            var current_w = original_tex.width;
            var current_h = original_tex.height;

            for (preset.passes, 0..) |*pass, i| {
                const last_pass = (i == preset.passes.len - 1);
                const output_size = pass.scale_params.outputSize(current_w, current_h, viewport.w, viewport.h);
                // `mipmap_input` asks for the input of a pass to be mipmapped, so
                // the levels live on the output of the pass that feeds it.
                const next_mipmap_input = !last_pass and preset.passes[i + 1].sampler.mipmaps;
                const num_levels = if (next_mipmap_input) calcMipLevels(output_size.w, output_size.h) else 1;

                const needs_feedback = self.uses_pass_feedback or (pass.alias != null);
                if (needs_feedback) {
                    const previous_output = pass.output_texture;
                    pass.output_texture = pass.fb_feedback;
                    pass.fb_feedback = previous_output;

                    if (pass.fb_feedback) |feedback| {
                        if (!feedback.matches(output_size.w, output_size.h, pass.format, num_levels)) {
                            feedback.release(backend);
                            pass.fb_feedback = null;
                        }
                    }
                }

                const target_tex = try createRenderTarget(
                    self.alloc,
                    backend,
                    &pass.output_texture,
                    output_size.w,
                    output_size.h,
                    pass.format,
                    num_levels,
                );

                if (self.uses_pass_feedback and i < self.pass_feedback.len) {
                    setRetainedPassTextureSlot(backend, &self.pass_feedback[i], pass.fb_feedback);
                }
                if (self.uses_pass_output and i < self.pass_output.len) {
                    setRetainedPassTextureSlot(backend, &self.pass_output[i], target_tex);
                }

                // Handle named aliases for feedback loops
                if (pass.alias) |alias| {
                    if (try self.texture_aliases.fetchPut(alias, target_tex.ref())) |old| {
                        old.value.release(backend);
                    }
                    if (pass.feedback_alias) |fb_alias| {
                        const fb_tex = pass.fb_feedback orelse original_tex;
                        if (try self.texture_aliases.fetchPut(fb_alias, fb_tex.ref())) |old| {
                            old.value.release(backend);
                        }
                    }
                }

                if (pass.sampler.mipmaps and source_tex.num_levels > 1) {
                    backend.generateMipmaps(frame, source_tex.ptr.?);
                }

                var render_pass = try backend.beginPass(frame, &pass.gpu, target_tex.ptr.?, output_size.w, output_size.h);

                uniforms.FrameCount = if (pass.frame_count_mod) |m| self.frame_count % m else self.frame_count;
                uniforms.SourceSize = sizeVec4From(current_w, current_h);
                uniforms.OutputSize = sizeVec4From(output_size.w, output_size.h);

                bindShaderResources(
                    backend,
                    frame,
                    &render_pass,
                    pass,
                    original_tex,
                    source_tex,
                    uniforms,
                    preset,
                    &self.texture_aliases,
                    self.frame_history.items,
                    self.pass_feedback,
                    self.pass_output,
                ) catch |err| {
                    backend.endPass(frame, &render_pass, &pass.gpu, false);
                    return err;
                };
                backend.endPass(frame, &render_pass, &pass.gpu, true);

                if (!last_pass) {
                    source_tex = pass.output_texture.?;
                    current_w = output_size.w;
                    current_h = output_size.h;
                }
            }
        }

        pub fn init(alloc: std.mem.Allocator, io: std.Io, backend_args: Backend.InitArgs) !*Self {
            const self = try alloc.create(Self);
            errdefer alloc.destroy(self);

            var cache = try ShaderCache.init(alloc, io);
            errdefer cache.deinit();

            self.* = .{
                .alloc = alloc,
                .io = io,
                .backend = try Backend.init(alloc, backend_args),
                .cache = cache,
                .texture_aliases = .init(alloc),
            };
            return self;
        }

        pub fn deinit(self: *Self) void {
            self.unloadPreset();
            self.texture_aliases.deinit();
            self.frame_history.deinit(self.alloc);
            self.backend.deinit();
            self.cache.deinit();
            self.alloc.destroy(self);
        }

        /// Unload the current preset and reset all GPU state.
        /// If an async load is in progress it is joined first.
        pub fn unloadPreset(self: *Self) void {
            if (self.compile_thread) |t| {
                t.join();
                self.compile_thread = null;
            }
            {
                self.compile_mutex.lockUncancelable(self.io);
                defer self.compile_mutex.unlock(self.io);
                if (self.compile_error_msg) |m| {
                    self.alloc.free(m);
                    self.compile_error_msg = null;
                }
                self.compile_failed = false;
                if (self.compiled_preset) |*p| {
                    p.deinit(self.alloc, &self.backend);
                    self.compiled_preset = null;
                }
                if (self.compiled_path) |path| {
                    self.alloc.free(path);
                    self.compiled_path = null;
                }
            }
            self.compile_progress.store(0, .monotonic);
            self.compile_total = 0;
            self.compile_done.store(false, .monotonic);

            clearPassTextureSlots(&self.backend, self.pass_output);
            clearPassTextureSlots(&self.backend, self.pass_feedback);
            self.alloc.free(self.pass_output);
            self.alloc.free(self.pass_feedback);
            self.pass_output = &.{};
            self.pass_feedback = &.{};

            var alias_it = self.texture_aliases.valueIterator();
            while (alias_it.next()) |t| t.*.release(&self.backend);
            self.texture_aliases.clearRetainingCapacity();

            for (self.frame_history.items) |tex| {
                if (tex) |t| t.release(&self.backend);
            }
            self.frame_history.clearRetainingCapacity();

            if (self.preset) |*p| {
                p.deinit(self.alloc, &self.backend);
                self.preset = null;
            }
            if (self.preset_path) |path| {
                self.alloc.free(path);
                self.preset_path = null;
            }
            self.frame_count = 0;
            self.uses_pass_output = false;
            self.uses_pass_feedback = false;
        }

        pub fn isActive(self: *const Self) bool {
            return self.preset != null;
        }

        pub fn getPresetPath(self: *const Self) ?[]const u8 {
            return self.preset_path;
        }

        /// Returns the ordered list of tunable parameters for the active preset.
        pub fn getParamInfos(self: *const Self) []const ParamInfo {
            const p = self.preset orelse return &.{};
            return p.param_meta.items;
        }

        /// Read the current f32 value of a parameter by name.
        pub fn getParam(self: *const Self, name: []const u8) f32 {
            const p = self.preset orelse return 0;
            const bytes = p.param_data.get(name) orelse return 0;
            return std.mem.bytesAsValue(f32, bytes[0..4]).*;
        }

        /// Write a new f32 value for a parameter by name.  No-op if the preset
        /// is not loaded or the parameter does not exist.
        pub fn setParam(self: *Self, name: []const u8, value: f32) void {
            const p = &(self.preset orelse return);
            if (p.param_data.getPtr(name)) |bytes_ptr| {
                @memcpy(bytes_ptr.*[0..4], std.mem.asBytes(&value));
            }
        }

        pub fn isCompiling(self: *const Self) bool {
            return self.compile_thread != null and !self.compile_done.load(.acquire);
        }

        /// Start an asynchronous shader preset load.  Progress can be tracked
        /// via `compile_progress` / `compile_total`, and the result retrieved
        /// by calling `pollLoadResult` each frame.
        pub fn loadPreset(self: *Self, path: []const u8) !void {
            // unloadPreset joins any in-progress thread and resets all state.
            self.unloadPreset();

            var parsed = try parsePresetFile(self.alloc, self.io, path);
            errdefer parsed.deinit(self.alloc);
            self.compile_total = @intCast(parsed.shader_config.total_passes);

            const ctx = try self.alloc.create(AsyncLoadCtx);
            errdefer self.alloc.destroy(ctx);
            ctx.* = .{
                .pipeline = self,
                .path = try self.alloc.dupe(u8, path),
                .parsed = parsed, // ownership transferred; freed by asyncLoadFn
            };
            errdefer self.alloc.free(ctx.path);

            self.compile_thread = try std.Thread.spawn(.{}, asyncLoadFn, .{ctx});
        }

        /// The result of a `pollLoadResult` call.
        pub const ShaderLoadPoll = union(enum) {
            /// No async load has been started.
            idle,
            /// Compilation is in progress; `completed`/`total` indicate how many
            /// passes have finished.
            compiling: struct { completed: u32, total: u32 },
            /// Compilation succeeded and the preset is now active.
            done,
            /// Compilation failed; the slice is the error message (owned by the
            /// pipeline — do not free it).
            failed: []const u8,
        };

        /// Poll the status of an async `loadPreset` call. Must be called from
        /// the rendering thread, which activates the compiled preset.
        /// When `.done` or `.failed` is returned the compile thread has been
        /// joined and the pipeline has been updated.
        pub fn pollLoadResult(self: *Self) ShaderLoadPoll {
            if (self.compile_thread == null) return .idle;

            if (!self.compile_done.load(.acquire)) {
                return .{ .compiling = .{
                    .completed = self.compile_progress.load(.monotonic),
                    .total = self.compile_total,
                } };
            }

            // Compilation finished — join the thread.
            if (self.compile_thread) |t| {
                t.join();
                self.compile_thread = null;
            }

            self.compile_mutex.lockUncancelable(self.io);
            defer self.compile_mutex.unlock(self.io);

            if (!self.compile_failed) {
                var preset = self.compiled_preset.?;
                const path = self.compiled_path.?;
                self.compiled_preset = null;
                self.compiled_path = null;

                self.activatePreset(&preset, path) catch |err| {
                    preset.deinit(self.alloc, &self.backend);
                    self.alloc.free(path);
                    self.setCompileError(err);
                };
            }

            if (self.compile_failed) {
                return .{ .failed = self.compile_error_msg orelse "Unknown error" };
            }
            return .done;
        }

        /// Record a failed load. Must be called with `compile_mutex` held.
        fn setCompileError(self: *Self, err: anyerror) void {
            self.compile_failed = true;
            self.compile_error_msg = std.fmt.allocPrint(self.alloc, "Load failed: {s}", .{@errorName(err)}) catch null;
        }

        /// Create the remaining GPU objects of a compiled preset and make it
        /// the active one.
        fn activatePreset(self: *Self, preset: *LoadedPreset, path: []u8) !void {
            var lut_it = preset.luts.valueIterator();
            while (lut_it.next()) |lut| {
                const image = lut.image.?;
                const handle = try self.backend.createTexture(image.width, image.height, .rgba8_unorm, 1);
                lut.texture = Texture.init(self.alloc, handle, image.width, image.height, .rgba8_unorm, 1) catch |err| {
                    self.backend.destroyTexture(handle);
                    return err;
                };
                try self.backend.uploadTexture(handle, image.pixels, image.width, image.height);
                self.alloc.free(image.pixels);
                lut.image = null;

                lut.sampler = try self.backend.createSampler(lut.sampler_desc);
            }

            for (preset.passes) |*pass| {
                try self.backend.activatePass(self.alloc, &pass.gpu, &pass.vertex_reflection, &pass.fragment_reflection, pass.id);
            }

            var uses_pass_output = false;
            var uses_pass_feedback = false;
            var max_frame_history: usize = 0;
            for (preset.passes) |pass| {
                for (pass.fragment_reflection.descriptor_sets.items) |set_info| {
                    for (set_info.bindings.items) |binding| switch (binding.binding_type) {
                        .sampler2D => |st| switch (st) {
                            .PassOutput => uses_pass_output = true,
                            .PassFeedback => uses_pass_feedback = true,
                            .OriginalHistory => |id| max_frame_history = @max(id, max_frame_history),
                            else => {},
                        },
                        else => {},
                    };
                }
            }

            const pass_output = try self.alloc.alloc(?*Texture, preset.passes.len);
            errdefer self.alloc.free(pass_output);
            const pass_feedback = try self.alloc.alloc(?*Texture, preset.passes.len);
            errdefer self.alloc.free(pass_feedback);
            try self.frame_history.appendNTimes(self.alloc, null, max_frame_history);

            @memset(pass_output, null);
            @memset(pass_feedback, null);
            self.pass_output = pass_output;
            self.pass_feedback = pass_feedback;
            self.preset = preset.*;
            self.preset_path = path;
            self.uses_pass_output = uses_pass_output;
            self.uses_pass_feedback = uses_pass_feedback;

            std.log.info("Loaded shader preset: {s} ({} pass(es))", .{ path, preset.passes.len });
        }

        /// Context passed to the async compilation thread.
        const AsyncLoadCtx = struct {
            pipeline: *Self,
            /// Owned copy of the preset path (stored as `compiled_path` on success).
            path: []u8,
            /// Already-parsed preset data; ownership transferred from `loadPreset`.
            parsed: ParsedPreset,
        };

        fn asyncLoadFn(ctx: *AsyncLoadCtx) void {
            const self = ctx.pipeline;
            defer {
                ctx.parsed.deinit(self.alloc);
                self.alloc.free(ctx.path);
                self.alloc.destroy(ctx);
            }

            const result = compilePreset(
                self.alloc,
                self.io,
                &self.backend,
                &ctx.parsed,
                &self.compile_progress,
                &self.cache,
            );

            self.compile_mutex.lockUncancelable(self.io);
            defer self.compile_mutex.unlock(self.io);
            if (result) |preset| {
                self.compiled_preset = preset;
                self.compiled_path = ctx.path;
                // Ownership moved to `compiled_path`; keep the defer above from freeing it.
                ctx.path = &.{};
            } else |err| {
                self.setCompileError(err);
            }
            self.compile_done.store(true, .release);
        }

        pub fn renderFrame(
            self: *Self,
            input_texture: ?Backend.TextureHandle,
            src_w: u32,
            src_h: u32,
            viewport: Viewport,
            frame: Backend.Frame,
            win_w: u32,
            win_h: u32,
        ) !?*Texture {
            const preset = &self.preset.?;
            const handle = input_texture orelse return null;

            const input = try Texture.init(self.alloc, handle, src_w, src_h, .default, 1);
            defer {
                input.ptr = null; // the caller owns input_texture
                input.release(&self.backend);
            }

            var uniforms = BuiltinUniforms{};
            uniforms.OriginalSize = input.sizeVec4();
            uniforms.FinalViewportSize = sizeVec4From(win_w, win_h);

            self.backend.beginPasses(frame, handle);
            defer self.backend.endPasses(frame);
            try self.renderPasses(frame, preset, &uniforms, input, viewport);

            // Update frame history for OriginalHistory# uniforms
            if (self.frame_history.items.len > 0) try self.pushFrameHistory(frame, input);

            // Release pass_output textures if no feedback is needed
            if (self.uses_pass_output and !self.uses_pass_feedback) {
                clearPassTextureSlots(&self.backend, self.pass_output);
            }

            self.frame_count += 1;
            return preset.passes[preset.passes.len - 1].output_texture;
        }

        /// Store a copy of the current frame as the most recent history entry,
        /// reusing the texture of the oldest one. The input texture is
        /// overwritten by the UI every frame, so it cannot be referenced directly.
        fn pushFrameHistory(self: *Self, frame: Backend.Frame, input: *Texture) !void {
            const history = self.frame_history.items;
            var slot = history[history.len - 1];
            std.mem.copyBackwards(?*Texture, history[1..], history[0 .. history.len - 1]);
            history[0] = null;

            const target = try createRenderTarget(self.alloc, &self.backend, &slot, input.width, input.height, .rgba8_unorm, 1);
            history[0] = target;
            self.backend.copyTexture(frame, input.ptr.?, target.ptr.?, input.width, input.height);
        }
    };
}

fn sizeVec4From(w: u32, h: u32) [4]f32 {
    return .{
        @floatFromInt(w),
        @floatFromInt(h),
        1.0 / @as(f32, @floatFromInt(w)),
        1.0 / @as(f32, @floatFromInt(h)),
    };
}
