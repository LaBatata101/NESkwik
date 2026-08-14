//! OpenGL ES 3.0 (WebGL 2) backend of the shader pipeline (`shaders/pipeline.zig`).
//!
//! WebGL cannot consume SPIR-V, so each compiled stage is translated to
//! GLSL ES 3.00 with SPIRV-Cross. The WebGL context belongs to the browser
//! main thread: the compile workers only translate, and the GL objects are
//! created when the preset is activated.
const std = @import("std");

const c = @import("../root.zig").c;
const pipeline = @import("../shaders/pipeline.zig");
const parser = @import("../shaders/parser.zig");
const RendererTexture = @import("renderer.zig").Texture;
const spirv_arrays = @import("spirv_arrays.zig");

const ShaderReflection = pipeline.ShaderReflection;
const TextureFormat = pipeline.TextureFormat;
const SamplerDesc = pipeline.SamplerDesc;

const Vertex = extern struct {
    x: f32,
    y: f32,
    z: f32,
    u: f32,
    v: f32,
};

// Full-screen quad vertices. Unlike the Vulkan quad, V is not flipped: GL puts
// row 0 of a render target at the bottom (NDC y = -1), so mapping V = 0 there
// keeps every pass output in the same row order as its input (row 0 = top of
// the image). Texture coordinates and gl_FragCoord then match what the shaders
// see on Vulkan, and the final texture is displayed by the UI renderer as-is.
const QUAD_VERTICES = [_]Vertex{
    .{ .x = -1, .y = -1, .z = 0, .u = 0, .v = 0 }, .{ .x = 1, .y = -1, .z = 0, .u = 1, .v = 0 },
    .{ .x = 1, .y = 1, .z = 0, .u = 1, .v = 1 },   .{ .x = -1, .y = -1, .z = 0, .u = 0, .v = 0 },
    .{ .x = 1, .y = 1, .z = 0, .u = 1, .v = 1 },   .{ .x = -1, .y = 1, .z = 0, .u = 0, .v = 1 },
};

pub const Backend = struct {
    alloc: std.mem.Allocator,
    /// Shaders are compiled for Vulkan 1.0 (`VK_MAKE_API_VERSION(0, 1, 0, 0)`),
    /// whose SPIR-V 1.0 output is what SPIRV-Cross translates most reliably.
    vk_version: c_uint = 1 << 22,
    vertex_buffer: c.GLuint,
    vertex_array: c.GLuint,
    /// Read framebuffer used to copy frames into the history.
    copy_framebuffer: c.GLuint,
    max_texture_units: u32,

    pub const TextureHandle = *RendererTexture;
    pub const Sampler = c.GLuint;
    pub const Frame = void;
    pub const InitArgs = struct {};

    pub const RenderPass = struct {
        next_texture_unit: u32 = 0,
    };

    pub const PassObjects = struct {
        program: c.GLuint = 0,
        sampler: c.GLuint = 0,
        framebuffer: c.GLuint = 0,
        vertex_resources: GLStageResources = .{},
        fragment_resources: GLStageResources = .{},
        sampler_desc: SamplerDesc = .{},
        /// GLSL ES produced by the compile workers; consumed by `activatePass`.
        sources: ?GlslSources = null,
    };

    pub fn init(alloc: std.mem.Allocator, _: InitArgs) !Backend {
        var vertex_array: c.GLuint = 0;
        c.glGenVertexArrays(1, &vertex_array);
        if (vertex_array == 0) return error.GLVertexArrayCreateFailed;
        errdefer c.glDeleteVertexArrays(1, &vertex_array);

        var vertex_buffer: c.GLuint = 0;
        c.glGenBuffers(1, &vertex_buffer);
        if (vertex_buffer == 0) return error.GLBufferCreateFailed;
        errdefer c.glDeleteBuffers(1, &vertex_buffer);

        var copy_framebuffer: c.GLuint = 0;
        c.glGenFramebuffers(1, &copy_framebuffer);
        if (copy_framebuffer == 0) return error.GLFramebufferCreateFailed;

        // Upload the quad vertices
        c.glBindVertexArray(vertex_array);
        c.glBindBuffer(c.GL_ARRAY_BUFFER, vertex_buffer);
        c.glBufferData(c.GL_ARRAY_BUFFER, @sizeOf(@TypeOf(QUAD_VERTICES)), &QUAD_VERTICES, c.GL_STATIC_DRAW);
        c.glEnableVertexAttribArray(0);
        c.glVertexAttribPointer(0, 3, c.GL_FLOAT, c.GL_FALSE, @sizeOf(Vertex), @ptrFromInt(@offsetOf(Vertex, "x")));
        c.glEnableVertexAttribArray(1);
        c.glVertexAttribPointer(1, 2, c.GL_FLOAT, c.GL_FALSE, @sizeOf(Vertex), @ptrFromInt(@offsetOf(Vertex, "u")));
        c.glBindVertexArray(0);
        c.glBindBuffer(c.GL_ARRAY_BUFFER, 0);

        var max_texture_units: c.GLint = 0;
        c.glGetIntegerv(c.GL_MAX_TEXTURE_IMAGE_UNITS, &max_texture_units);

        return .{
            .alloc = alloc,
            .vertex_buffer = vertex_buffer,
            .vertex_array = vertex_array,
            .copy_framebuffer = copy_framebuffer,
            .max_texture_units = @intCast(max_texture_units),
        };
    }

    pub fn deinit(self: *Backend) void {
        c.glDeleteFramebuffers(1, &self.copy_framebuffer);
        c.glDeleteBuffers(1, &self.vertex_buffer);
        c.glDeleteVertexArrays(1, &self.vertex_array);
    }

    /// Worker threads used to compile the passes of a preset. Emscripten only
    /// starts new Web Workers while the main thread is idle, so all threads
    /// that can run at once must fit in the prestarted pool
    /// (`-sPTHREAD_POOL_SIZE` in build.zig), or `unloadPreset` joining a
    /// compile thread could deadlock. That is the emulation thread, plus a
    /// compile thread and its jobs for the main and border presets (both can
    /// be multi-pass) and a compile thread for the single-pass snow shader:
    /// 1 + 2 x (1 + 2) + 1 = 8.
    pub fn compileJobs() usize {
        return 2;
    }

    fn glFormat(format: TextureFormat) c.GLenum {
        return switch (format) {
            .default, .rgba8_unorm => c.GL_RGBA8,
            .r8_unorm => c.GL_R8,
            .rgba8_srgb => c.GL_SRGB8_ALPHA8,
            .r16_float => c.GL_R16F,
            .rg16_float => c.GL_RG16F,
            .rgba16_float => c.GL_RGBA16F,
            .r32_float => c.GL_R32F,
            .rg32_float => c.GL_RG32F,
            .rgba32_float => c.GL_RGBA32F,
            .rgb10a2_unorm => c.GL_RGB10_A2,
        };
    }

    /// Translate a pass to GLSL ES. The WebGL context belongs to the browser
    /// main thread, so the GL objects are only created by `activatePass`.
    pub fn compilePass(
        _: *Backend,
        alloc: std.mem.Allocator,
        spirv: pipeline.SpirvPair,
        _: *const ShaderReflection,
        _: *const ShaderReflection,
        _: TextureFormat,
        sampler: SamplerDesc,
        pass_id: usize,
    ) !PassObjects {
        var flat_varyings: u64 = 0;
        const fragment = translateSpirv(alloc, spirv.frag, .Fragment, &flat_varyings) catch |err| {
            std.log.err("Failed to translate the fragment stage of shader pass {}", .{pass_id});
            return err;
        };
        errdefer alloc.free(fragment);
        const vertex = translateSpirv(alloc, spirv.vert, .Vertex, &flat_varyings) catch |err| {
            std.log.err("Failed to translate the vertex stage of shader pass {}", .{pass_id});
            return err;
        };
        return .{ .sampler_desc = sampler, .sources = .{ .vertex = vertex, .fragment = fragment } };
    }

    /// Create the program, resources, sampler and framebuffer of a pass.
    pub fn activatePass(
        self: *Backend,
        alloc: std.mem.Allocator,
        objects: *PassObjects,
        vertex_reflection: *const ShaderReflection,
        fragment_reflection: *const ShaderReflection,
        pass_id: usize,
    ) !void {
        const sources = objects.sources.?;
        defer {
            sources.deinit(alloc);
            objects.sources = null;
        }

        const vert_shader = try compileGLShader(c.GL_VERTEX_SHADER, sources.vertex, pass_id);
        defer c.glDeleteShader(vert_shader);
        const frag_shader = try compileGLShader(c.GL_FRAGMENT_SHADER, sources.fragment, pass_id);
        defer c.glDeleteShader(frag_shader);

        objects.program = c.glCreateProgram();
        if (objects.program == 0) return error.GLProgramCreateFailed;
        c.glAttachShader(objects.program, vert_shader);
        c.glAttachShader(objects.program, frag_shader);
        c.glLinkProgram(objects.program);

        var linked: c.GLint = 0;
        c.glGetProgramiv(objects.program, c.GL_LINK_STATUS, &linked);
        if (linked == 0) {
            var log: [4096]u8 = undefined;
            var len: c.GLsizei = 0;
            c.glGetProgramInfoLog(objects.program, log.len, &len, &log);
            std.log.err("GLSL ES program of pass {} failed to link:\n{s}", .{ pass_id, log[0..@intCast(@max(0, len))] });
            return error.GLProgramLinkFailed;
        }

        try queryStageResources(alloc, objects.program, vertex_reflection, .Vertex, 0, &objects.vertex_resources);
        try queryStageResources(
            alloc,
            objects.program,
            fragment_reflection,
            .Fragment,
            @intCast(objects.vertex_resources.uniform_blocks.items.len),
            &objects.fragment_resources,
        );

        objects.sampler = try self.createSampler(objects.sampler_desc);

        c.glGenFramebuffers(1, &objects.framebuffer);
        if (objects.framebuffer == 0) return error.GLFramebufferCreateFailed;
    }

    /// May run on a compile worker for a preset that failed to compile, where
    /// no GL object exists yet and the GL context must not be touched.
    pub fn destroyPass(_: *Backend, alloc: std.mem.Allocator, objects: *PassObjects) void {
        if (objects.program != 0) c.glDeleteProgram(objects.program);
        if (objects.sampler != 0) c.glDeleteSamplers(1, &objects.sampler);
        if (objects.framebuffer != 0) c.glDeleteFramebuffers(1, &objects.framebuffer);
        objects.vertex_resources.deinit(alloc);
        objects.fragment_resources.deinit(alloc);
        if (objects.sources) |sources| sources.deinit(alloc);
        objects.* = .{};
    }

    pub fn createTexture(self: *Backend, w: u32, h: u32, format: TextureFormat, num_levels: u32) !TextureHandle {
        var id: c.GLuint = 0;
        c.glGenTextures(1, &id);
        if (id == 0) return error.GLTextureCreateFailed;
        errdefer c.glDeleteTextures(1, &id);

        c.glBindTexture(c.GL_TEXTURE_2D, id);
        // Immutable storage keeps the texture complete for any sampler state.
        c.glTexStorage2D(c.GL_TEXTURE_2D, @intCast(num_levels), glFormat(format), @intCast(w), @intCast(h));
        return RendererTexture.init(self.alloc, id, w, h);
    }

    pub fn destroyTexture(self: *Backend, texture: TextureHandle) void {
        texture.deinit(self.alloc);
    }

    /// Upload tightly packed RGBA8 pixels.
    pub fn uploadTexture(_: *Backend, texture: TextureHandle, pixels: []const u8, w: u32, h: u32) !void {
        c.glBindTexture(c.GL_TEXTURE_2D, texture.id);
        c.glPixelStorei(c.GL_UNPACK_ALIGNMENT, 1);
        c.glTexSubImage2D(c.GL_TEXTURE_2D, 0, 0, 0, @intCast(w), @intCast(h), c.GL_RGBA, c.GL_UNSIGNED_BYTE, pixels.ptr);
    }

    pub fn copyTexture(self: *Backend, _: Frame, src: TextureHandle, dst: TextureHandle, w: u32, h: u32) void {
        c.glBindFramebuffer(c.GL_READ_FRAMEBUFFER, self.copy_framebuffer);
        defer c.glBindFramebuffer(c.GL_READ_FRAMEBUFFER, 0);
        c.glFramebufferTexture2D(c.GL_READ_FRAMEBUFFER, c.GL_COLOR_ATTACHMENT0, c.GL_TEXTURE_2D, src.id, 0);
        c.glBindTexture(c.GL_TEXTURE_2D, dst.id);
        c.glCopyTexSubImage2D(c.GL_TEXTURE_2D, 0, 0, 0, 0, 0, @intCast(w), @intCast(h));
    }

    pub fn generateMipmaps(_: *Backend, _: Frame, texture: TextureHandle) void {
        c.glBindTexture(c.GL_TEXTURE_2D, texture.id);
        c.glGenerateMipmap(c.GL_TEXTURE_2D);
    }

    pub fn createSampler(_: *Backend, desc: SamplerDesc) !Sampler {
        var sampler: c.GLuint = 0;
        c.glGenSamplers(1, &sampler);
        if (sampler == 0) return error.GLSamplerCreateFailed;

        const filter: c.GLenum = if (desc.linear) c.GL_LINEAR else c.GL_NEAREST;
        const min_filter: c.GLenum = if (!desc.mipmaps)
            filter
        else if (desc.linear)
            c.GL_LINEAR_MIPMAP_LINEAR
        else
            c.GL_NEAREST_MIPMAP_LINEAR;
        // GLES has no clamp-to-border either.
        const wrap_mode: c.GLenum = switch (desc.wrap_mode) {
            .clamp_to_border, .clamp_to_edge => c.GL_CLAMP_TO_EDGE,
            .repeat => c.GL_REPEAT,
            .mirrored_repeat => c.GL_MIRRORED_REPEAT,
        };
        c.glSamplerParameteri(sampler, c.GL_TEXTURE_MIN_FILTER, @intCast(min_filter));
        c.glSamplerParameteri(sampler, c.GL_TEXTURE_MAG_FILTER, @intCast(filter));
        c.glSamplerParameteri(sampler, c.GL_TEXTURE_WRAP_S, @intCast(wrap_mode));
        c.glSamplerParameteri(sampler, c.GL_TEXTURE_WRAP_T, @intCast(wrap_mode));
        return sampler;
    }

    pub fn destroySampler(_: *Backend, sampler: Sampler) void {
        c.glDeleteSamplers(1, &sampler);
    }

    /// Set the fixed state of the SDL GPU pipelines; the UI renderer leaves
    /// blending and scissoring enabled and restores them in `resumeRendering`.
    pub fn beginPasses(self: *Backend, _: Frame, input: TextureHandle) void {
        c.glDisable(c.GL_BLEND);
        c.glDisable(c.GL_SCISSOR_TEST);
        c.glDisable(c.GL_DEPTH_TEST);
        c.glDisable(c.GL_CULL_FACE);
        c.glColorMask(c.GL_TRUE, c.GL_TRUE, c.GL_TRUE, c.GL_TRUE);
        c.glBindVertexArray(self.vertex_array);

        // The UI texture is created without mipmaps; limit it to its base
        // level so it stays complete under a mipmapped pass sampler.
        c.glBindTexture(c.GL_TEXTURE_2D, input.id);
        c.glTexParameteri(c.GL_TEXTURE_2D, c.GL_TEXTURE_MAX_LEVEL, 0);
    }

    pub fn endPasses(_: *Backend, _: Frame) void {
        c.glBindVertexArray(0);
        c.glBindFramebuffer(c.GL_FRAMEBUFFER, 0);
        c.glBindBuffer(c.GL_UNIFORM_BUFFER, 0);
        c.glActiveTexture(c.GL_TEXTURE0);
    }

    pub fn beginPass(_: *Backend, _: Frame, objects: *const PassObjects, target: TextureHandle, w: u32, h: u32) !RenderPass {
        c.glBindFramebuffer(c.GL_FRAMEBUFFER, objects.framebuffer);
        c.glFramebufferTexture2D(c.GL_FRAMEBUFFER, c.GL_COLOR_ATTACHMENT0, c.GL_TEXTURE_2D, target.id, 0);
        const status = c.glCheckFramebufferStatus(c.GL_FRAMEBUFFER);
        if (status != c.GL_FRAMEBUFFER_COMPLETE) {
            std.log.err("Shader pass framebuffer is incomplete: 0x{x}", .{status});
            return error.GLFramebufferIncomplete;
        }
        c.glViewport(0, 0, @intCast(w), @intCast(h));
        c.glClearColor(0, 0, 0, 1);
        c.glClear(c.GL_COLOR_BUFFER_BIT);
        c.glUseProgram(objects.program);
        return .{};
    }

    pub fn pushUniforms(
        _: *Backend,
        _: Frame,
        _: *RenderPass,
        objects: *const PassObjects,
        stage: parser.ShaderStage,
        binding: u32,
        data: []const u8,
    ) void {
        const resources = if (stage == .Vertex) &objects.vertex_resources else &objects.fragment_resources;
        const block = resources.uniformBlock(binding) orelse return;
        c.glBindBuffer(c.GL_UNIFORM_BUFFER, block.buffer);
        c.glBufferSubData(c.GL_UNIFORM_BUFFER, 0, @intCast(@min(data.len, block.size)), data.ptr);
        c.glBindBufferBase(c.GL_UNIFORM_BUFFER, block.binding_point, block.buffer);
    }

    pub fn bindTexture(
        self: *Backend,
        render_pass: *RenderPass,
        objects: *const PassObjects,
        binding: u32,
        texture: ?TextureHandle,
        sampler: ?Sampler,
    ) !void {
        const location = objects.fragment_resources.samplerLocation(binding) orelse return;
        const unit = render_pass.next_texture_unit;
        if (unit >= self.max_texture_units) return error.GLTooManyTextureUnits;
        render_pass.next_texture_unit += 1;

        c.glActiveTexture(@as(c.GLenum, c.GL_TEXTURE0) + unit);
        c.glBindTexture(c.GL_TEXTURE_2D, if (texture) |t| t.id else 0);
        c.glBindSampler(unit, sampler orelse objects.sampler);
        c.glUniform1i(location, @intCast(unit));
    }

    pub fn endPass(_: *Backend, _: Frame, render_pass: *RenderPass, _: *const PassObjects, draw: bool) void {
        if (draw) c.glDrawArrays(c.GL_TRIANGLES, 0, QUAD_VERTICES.len);
        // Sampler objects override texture state, so unbind them before the
        // UI renderer reuses the texture units.
        for (0..render_pass.next_texture_unit) |unit| c.glBindSampler(@intCast(unit), 0);
    }
};

/// A uniform block of a linked program. SDL GPU takes uniform data per stage
/// and binding slot; the GLES equivalent is one buffer per block, so blocks
/// are renamed per stage and binding before translation (`uniformBlockName`).
const GLUniformBlock = struct {
    binding: u32,
    buffer: c.GLuint,
    binding_point: c.GLuint,
    size: u32,
};

const GLSampler = struct {
    binding: u32,
    location: c.GLint,
};

/// GL handles of the reflected resources of one shader stage.
const GLStageResources = struct {
    uniform_blocks: std.ArrayList(GLUniformBlock) = .empty,
    samplers: std.ArrayList(GLSampler) = .empty,

    fn deinit(self: *GLStageResources, alloc: std.mem.Allocator) void {
        for (self.uniform_blocks.items) |*block| c.glDeleteBuffers(1, &block.buffer);
        self.uniform_blocks.deinit(alloc);
        self.samplers.deinit(alloc);
    }

    fn uniformBlock(self: *const GLStageResources, binding: u32) ?GLUniformBlock {
        for (self.uniform_blocks.items) |block| {
            if (block.binding == binding) return block;
        }
        return null;
    }

    fn samplerLocation(self: *const GLStageResources, binding: u32) ?c.GLint {
        for (self.samplers.items) |sampler| {
            if (sampler.binding == binding) return sampler.location;
        }
        return null;
    }
};

/// GLSL ES sources of a pass, produced by the compile workers and consumed by
/// `Backend.activatePass` on the main thread.
const GlslSources = struct {
    vertex: [:0]u8,
    fragment: [:0]u8,

    fn deinit(self: GlslSources, alloc: std.mem.Allocator) void {
        alloc.free(self.vertex);
        alloc.free(self.fragment);
    }
};

fn stagePrefix(stage: parser.ShaderStage) []const u8 {
    return if (stage == .Vertex) "vs" else "fs";
}

/// Name given to a uniform block during translation. GLES has no binding
/// qualifiers below GLSL ES 3.10, so blocks and samplers are found by name.
/// The vertex and fragment blocks get distinct names, which keeps their data
/// separate exactly like the per-stage uniform slots of SDL GPU.
fn uniformBlockName(buf: []u8, stage: parser.ShaderStage, binding: u32) ![:0]const u8 {
    return std.fmt.bufPrintZ(buf, "neskwik_{s}_ubo{d}", .{ stagePrefix(stage), binding });
}

fn samplerName(buf: []u8, stage: parser.ShaderStage, binding: u32) ![:0]const u8 {
    return std.fmt.bufPrintZ(buf, "neskwik_{s}_sampler{d}", .{ stagePrefix(stage), binding });
}

/// GLSL ES 3.00 links varyings by name, Vulkan GLSL by location. Both stages
/// name their interface variables after the location so they still match.
fn varyingName(buf: []u8, location: u32) ![:0]const u8 {
    return std.fmt.bufPrintZ(buf, "neskwik_varying{d}", .{location});
}

fn spvcCheck(context: c.spvc_context, result: c.spvc_result) !void {
    if (result == c.SPVC_SUCCESS) return;
    std.log.err("SPIRV-Cross: {s}", .{c.spvc_context_get_last_error_string(context)});
    return error.SPIRVCrossFailed;
}

fn renameResources(
    compiler: c.spvc_compiler,
    resources: c.spvc_resources,
    resource_type: c.spvc_resource_type,
    stage: parser.ShaderStage,
    flat_varyings: *u64,
) !void {
    var list: [*c]const c.spvc_reflected_resource = undefined;
    var count: usize = 0;
    if (c.spvc_resources_get_resource_list_for_type(resources, resource_type, &list, &count) != c.SPVC_SUCCESS) {
        return error.SPIRVCrossFailed;
    }

    var name_buf: [64]u8 = undefined;
    for (list[0..count]) |res| {
        switch (resource_type) {
            c.SPVC_RESOURCE_TYPE_UNIFORM_BUFFER => {
                const binding = c.spvc_compiler_get_decoration(compiler, res.id, c.SpvDecorationBinding);
                // The block name is the name of its struct type.
                c.spvc_compiler_set_name(compiler, res.base_type_id, try uniformBlockName(&name_buf, stage, binding));
            },
            c.SPVC_RESOURCE_TYPE_SAMPLED_IMAGE => {
                const binding = c.spvc_compiler_get_decoration(compiler, res.id, c.SpvDecorationBinding);
                c.spvc_compiler_set_name(compiler, res.id, try samplerName(&name_buf, stage, binding));
            },
            c.SPVC_RESOURCE_TYPE_STAGE_INPUT, c.SPVC_RESOURCE_TYPE_STAGE_OUTPUT => {
                if (c.spvc_compiler_has_decoration(compiler, res.id, c.SpvDecorationLocation) == c.SPVC_FALSE) continue;
                const location = c.spvc_compiler_get_decoration(compiler, res.id, c.SpvDecorationLocation);
                c.spvc_compiler_set_name(compiler, res.id, try varyingName(&name_buf, location));

                // Vulkan only requires `flat` on the fragment input, GLSL ES
                // 3.00 on both sides of the interface.
                if (location >= 64) continue;
                const bit = @as(u64, 1) << @intCast(location);
                if (resource_type == c.SPVC_RESOURCE_TYPE_STAGE_INPUT) {
                    if (c.spvc_compiler_has_decoration(compiler, res.id, c.SpvDecorationFlat) != c.SPVC_FALSE) {
                        flat_varyings.* |= bit;
                    }
                } else if (flat_varyings.* & bit != 0) {
                    c.spvc_compiler_set_decoration(compiler, res.id, c.SpvDecorationFlat, 0);
                }
            },
            else => unreachable,
        }
    }
}

/// Translate Vulkan SPIR-V produced by `getOrCompileSpirv` to GLSL ES 3.00.
/// `flat_varyings` is a mask of varying locations: translating the fragment
/// stage records its `flat` inputs, translating the vertex stage afterwards
/// applies them to the matching outputs.
fn translateSpirv(
    alloc: std.mem.Allocator,
    spirv: []const u8,
    stage: parser.ShaderStage,
    flat_varyings: *u64,
) ![:0]u8 {
    var context: c.spvc_context = undefined;
    if (c.spvc_context_create(&context) != c.SPVC_SUCCESS) return error.SPIRVCrossFailed;
    defer c.spvc_context_destroy(context);

    const words: []const u32 = @alignCast(std.mem.bytesAsSlice(u32, spirv));
    const expanded = try spirv_arrays.expandArrayOfArrayStores(alloc, words);
    defer if (expanded) |module| alloc.free(module);
    const module = expanded orelse words;

    var parsed_ir: c.spvc_parsed_ir = undefined;
    try spvcCheck(context, c.spvc_context_parse_spirv(context, module.ptr, module.len, &parsed_ir));

    var compiler: c.spvc_compiler = undefined;
    try spvcCheck(context, c.spvc_context_create_compiler(
        context,
        c.SPVC_BACKEND_GLSL,
        parsed_ir,
        c.SPVC_CAPTURE_MODE_TAKE_OWNERSHIP,
        &compiler,
    ));

    var resources: c.spvc_resources = undefined;
    try spvcCheck(context, c.spvc_compiler_create_shader_resources(compiler, &resources));
    try renameResources(compiler, resources, c.SPVC_RESOURCE_TYPE_UNIFORM_BUFFER, stage, flat_varyings);
    try renameResources(compiler, resources, c.SPVC_RESOURCE_TYPE_SAMPLED_IMAGE, stage, flat_varyings);
    // Vertex inputs keep their locations (GLSL ES 3.00 supports them there);
    // only the varyings between the two stages need matching names.
    try renameResources(
        compiler,
        resources,
        if (stage == .Vertex) c.SPVC_RESOURCE_TYPE_STAGE_OUTPUT else c.SPVC_RESOURCE_TYPE_STAGE_INPUT,
        stage,
        flat_varyings,
    );

    // Vulkan places the fragment origin at the upper left, GLSL ES only allows
    // the lower left. With the quad layout above, GL's lower-left origin lands
    // on row 0 of the target, which is the row Vulkan's upper-left origin
    // refers to, so the execution mode can be dropped without changing results.
    if (stage == .Fragment) c.spvc_compiler_unset_execution_mode(compiler, c.SpvExecutionModeOriginUpperLeft);

    var options: c.spvc_compiler_options = undefined;
    try spvcCheck(context, c.spvc_compiler_create_compiler_options(compiler, &options));
    try spvcCheck(context, c.spvc_compiler_options_set_uint(options, c.SPVC_COMPILER_OPTION_GLSL_VERSION, 300));
    try spvcCheck(context, c.spvc_compiler_options_set_bool(options, c.SPVC_COMPILER_OPTION_GLSL_ES, c.SPVC_TRUE));
    // GLSL ES 3.00 has no arrays of arrays.
    try spvcCheck(context, c.spvc_compiler_options_set_bool(
        options,
        c.SPVC_COMPILER_OPTION_FLATTEN_MULTIDIMENSIONAL_ARRAYS,
        c.SPVC_TRUE,
    ));
    try spvcCheck(context, c.spvc_compiler_options_set_bool(
        options,
        c.SPVC_COMPILER_OPTION_GLSL_ES_DEFAULT_FLOAT_PRECISION_HIGHP,
        c.SPVC_TRUE,
    ));
    try spvcCheck(context, c.spvc_compiler_options_set_bool(
        options,
        c.SPVC_COMPILER_OPTION_GLSL_ES_DEFAULT_INT_PRECISION_HIGHP,
        c.SPVC_TRUE,
    ));
    try spvcCheck(context, c.spvc_compiler_install_compiler_options(compiler, options));

    var source: [*c]const u8 = undefined;
    try spvcCheck(context, c.spvc_compiler_compile(compiler, &source));
    return alloc.dupeZ(u8, std.mem.span(source));
}


fn compileGLShader(kind: c.GLenum, source: [:0]const u8, pass_id: usize) !c.GLuint {
    const shader = c.glCreateShader(kind);
    if (shader == 0) return error.GLShaderCreateFailed;
    errdefer c.glDeleteShader(shader);

    const sources = [_][*c]const c.GLchar{source.ptr};
    c.glShaderSource(shader, 1, &sources, null);
    c.glCompileShader(shader);

    var compiled: c.GLint = 0;
    c.glGetShaderiv(shader, c.GL_COMPILE_STATUS, &compiled);
    if (compiled == 0) {
        var log: [4096]u8 = undefined;
        var len: c.GLsizei = 0;
        c.glGetShaderInfoLog(shader, log.len, &len, &log);
        std.log.err("GLSL ES {s} shader of pass {} failed to compile:\n{s}\n{s}", .{
            if (kind == c.GL_VERTEX_SHADER) "vertex" else "fragment",
            pass_id,
            log[0..@intCast(@max(0, len))],
            source,
        });
        return error.GLShaderCompileFailed;
    }
    return shader;
}

/// Find the GL objects behind the reflected uniform blocks and samplers of
/// one stage. Resources the GLSL compiler optimized away are skipped.
fn queryStageResources(
    alloc: std.mem.Allocator,
    program: c.GLuint,
    reflection: *const ShaderReflection,
    stage: parser.ShaderStage,
    first_binding_point: c.GLuint,
    resources: *GLStageResources,
) !void {
    var max_bindings: c.GLint = 0;
    c.glGetIntegerv(c.GL_MAX_UNIFORM_BUFFER_BINDINGS, &max_bindings);

    var binding_point = first_binding_point;
    var name_buf: [64]u8 = undefined;
    for (reflection.descriptor_sets.items) |set_info| {
        for (set_info.bindings.items) |binding| {
            switch (binding.binding_type) {
                .push_params, .UBO => {
                    const name = try uniformBlockName(&name_buf, stage, binding.binding);
                    const index = c.glGetUniformBlockIndex(program, name.ptr);
                    if (index == c.GL_INVALID_INDEX) continue;
                    if (binding_point >= max_bindings) return error.GLTooManyUniformBlocks;

                    var size: c.GLint = 0;
                    c.glGetActiveUniformBlockiv(program, index, c.GL_UNIFORM_BLOCK_DATA_SIZE, &size);
                    c.glUniformBlockBinding(program, index, binding_point);

                    var buffer: c.GLuint = 0;
                    c.glGenBuffers(1, &buffer);
                    if (buffer == 0) return error.GLBufferCreateFailed;
                    c.glBindBuffer(c.GL_UNIFORM_BUFFER, buffer);
                    c.glBufferData(c.GL_UNIFORM_BUFFER, size, null, c.GL_DYNAMIC_DRAW);
                    c.glBindBuffer(c.GL_UNIFORM_BUFFER, 0);

                    resources.uniform_blocks.append(alloc, .{
                        .binding = binding.binding,
                        .buffer = buffer,
                        .binding_point = binding_point,
                        .size = @intCast(size),
                    }) catch |err| {
                        c.glDeleteBuffers(1, &buffer);
                        return err;
                    };
                    binding_point += 1;
                },
                .sampler2D => {
                    const name = try samplerName(&name_buf, stage, binding.binding);
                    const location = c.glGetUniformLocation(program, name.ptr);
                    if (location < 0) continue;
                    try resources.samplers.append(alloc, .{ .binding = binding.binding, .location = location });
                },
            }
        }
    }
}
