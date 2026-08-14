//! SDL GPU backend of the shader pipeline (`pipeline.zig`).
const std = @import("std");

const c = @import("../root.zig").c;
const pipeline = @import("pipeline.zig");
const parser = @import("parser.zig");
const sdlError = @import("../utils/sdl.zig").sdlError;

const ShaderReflection = pipeline.ShaderReflection;
const TextureFormat = pipeline.TextureFormat;
const SamplerDesc = pipeline.SamplerDesc;

const Vertex = struct {
    x: f32,
    y: f32,
    z: f32,
    u: f32,
    v: f32,
};

// Full-screen quad vertices (NDC space, UV flipped vertically for Vulkan).
const QUAD_VERTICES = [_]Vertex{
    .{ .x = -1, .y = -1, .z = 0, .u = 0, .v = 1 }, .{ .x = 1, .y = -1, .z = 0, .u = 1, .v = 1 },
    .{ .x = 1, .y = 1, .z = 0, .u = 1, .v = 0 },   .{ .x = -1, .y = -1, .z = 0, .u = 0, .v = 1 },
    .{ .x = 1, .y = 1, .z = 0, .u = 1, .v = 0 },   .{ .x = -1, .y = 1, .z = 0, .u = 0, .v = 0 },
};

pub const Backend = struct {
    device: ?*c.SDL_GPUDevice,
    vk_version: c_uint,
    /// Format of render targets without an explicit format.
    swapchain_format: c.SDL_GPUTextureFormat,
    vertex_buffer: ?*c.SDL_GPUBuffer,

    pub const TextureHandle = *c.SDL_GPUTexture;
    pub const Sampler = *c.SDL_GPUSampler;
    pub const Frame = ?*c.SDL_GPUCommandBuffer;
    pub const RenderPass = ?*c.SDL_GPURenderPass;

    pub const PassObjects = struct {
        pipeline: ?*c.SDL_GPUGraphicsPipeline = null,
        sampler: ?*c.SDL_GPUSampler = null,
    };

    pub const InitArgs = struct {
        device: ?*c.SDL_GPUDevice,
        vk_version: c_uint,
        swapchain_format: c.SDL_GPUTextureFormat,
    };

    pub fn init(_: std.mem.Allocator, args: InitArgs) !Backend {
        const vb_size = @sizeOf(Vertex) * QUAD_VERTICES.len;
        const vertex_buffer = sdlError(c.SDL_CreateGPUBuffer(args.device, &.{
            .usage = c.SDL_GPU_BUFFERUSAGE_VERTEX,
            .size = vb_size,
        }));

        // Upload the quad vertices
        const tb = sdlError(c.SDL_CreateGPUTransferBuffer(
            args.device,
            &.{ .usage = c.SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD, .size = vb_size },
        ));
        defer c.SDL_ReleaseGPUTransferBuffer(args.device, tb);
        const map = c.SDL_MapGPUTransferBuffer(args.device, tb, false);
        @memcpy(@as([*]Vertex, @ptrCast(@alignCast(map)))[0..QUAD_VERTICES.len], &QUAD_VERTICES);
        c.SDL_UnmapGPUTransferBuffer(args.device, tb);

        const cmd = sdlError(c.SDL_AcquireGPUCommandBuffer(args.device));
        const copy = sdlError(c.SDL_BeginGPUCopyPass(cmd));
        c.SDL_UploadToGPUBuffer(copy, &.{ .transfer_buffer = tb }, &.{ .buffer = vertex_buffer, .size = vb_size }, false);
        c.SDL_EndGPUCopyPass(copy);
        sdlError(c.SDL_SubmitGPUCommandBuffer(cmd));

        return .{
            .device = args.device,
            .vk_version = args.vk_version,
            .swapchain_format = args.swapchain_format,
            .vertex_buffer = vertex_buffer,
        };
    }

    pub fn deinit(self: *Backend) void {
        c.SDL_ReleaseGPUBuffer(self.device, self.vertex_buffer);
    }

    pub fn compileJobs() usize {
        return std.Thread.getCpuCount() catch 4;
    }

    fn sdlFormat(self: *const Backend, format: TextureFormat) c.SDL_GPUTextureFormat {
        return switch (format) {
            .default => self.swapchain_format,
            .r8_unorm => c.SDL_GPU_TEXTUREFORMAT_R8_UNORM,
            .rgba8_unorm => c.SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM,
            .rgba8_srgb => c.SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM_SRGB,
            .r16_float => c.SDL_GPU_TEXTUREFORMAT_R16_FLOAT,
            .rg16_float => c.SDL_GPU_TEXTUREFORMAT_R16G16_FLOAT,
            .rgba16_float => c.SDL_GPU_TEXTUREFORMAT_R16G16B16A16_FLOAT,
            .r32_float => c.SDL_GPU_TEXTUREFORMAT_R32_FLOAT,
            .rg32_float => c.SDL_GPU_TEXTUREFORMAT_R32G32_FLOAT,
            .rgba32_float => c.SDL_GPU_TEXTUREFORMAT_R32G32B32A32_FLOAT,
            .rgb10a2_unorm => c.SDL_GPU_TEXTUREFORMAT_R10G10B10A2_UNORM,
        };
    }

    fn countBindings(reflection: *const ShaderReflection) struct { samplers: u32, ubos: u32 } {
        var samplers: u32 = 0;
        var ubos: u32 = 0;
        for (reflection.descriptor_sets.items) |set| {
            for (set.bindings.items) |binding| {
                switch (binding.binding_type) {
                    .UBO, .push_params => ubos += 1,
                    .sampler2D => samplers += 1,
                }
            }
        }
        return .{ .samplers = samplers, .ubos = ubos };
    }

    fn createShader(
        self: *Backend,
        spirv: []const u8,
        stage: c.SDL_GPUShaderStage,
        reflection: *const ShaderReflection,
    ) ?*c.SDL_GPUShader {
        const counts = countBindings(reflection);
        return sdlError(c.SDL_CreateGPUShader(self.device, &.{
            .code_size = spirv.len,
            .code = spirv.ptr,
            .entrypoint = "main",
            .format = c.SDL_GPU_SHADERFORMAT_SPIRV,
            .stage = stage,
            .num_samplers = counts.samplers,
            .num_uniform_buffers = counts.ubos,
            .num_storage_buffers = 0,
            .num_storage_textures = 0,
        }));
    }

    /// Create the graphics pipeline and sampler of a pass. SDL GPU is
    /// thread-safe, so this all happens on the compile workers.
    pub fn compilePass(
        self: *Backend,
        _: std.mem.Allocator,
        spirv: pipeline.SpirvPair,
        vertex_reflection: *const ShaderReflection,
        fragment_reflection: *const ShaderReflection,
        format: TextureFormat,
        sampler: SamplerDesc,
        _: usize,
    ) !PassObjects {
        const vert_shader = self.createShader(spirv.vert, c.SDL_GPU_SHADERSTAGE_VERTEX, vertex_reflection);
        defer c.SDL_ReleaseGPUShader(self.device, vert_shader);
        const frag_shader = self.createShader(spirv.frag, c.SDL_GPU_SHADERSTAGE_FRAGMENT, fragment_reflection);
        defer c.SDL_ReleaseGPUShader(self.device, frag_shader);

        const color_desc = c.SDL_GPUColorTargetDescription{
            .format = self.sdlFormat(format),
            .blend_state = .{ .enable_blend = false },
        };
        const vertex_attrs = [_]c.SDL_GPUVertexAttribute{
            .{ .location = 0, .format = c.SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3, .offset = @offsetOf(Vertex, "x") },
            .{ .location = 1, .format = c.SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2, .offset = @offsetOf(Vertex, "u") },
        };
        const vb_desc = c.SDL_GPUVertexBufferDescription{
            .slot = 0,
            .pitch = @sizeOf(Vertex),
            .input_rate = c.SDL_GPU_VERTEXINPUTRATE_VERTEX,
        };
        const graphics_pipeline = sdlError(c.SDL_CreateGPUGraphicsPipeline(self.device, &.{
            .vertex_shader = vert_shader,
            .fragment_shader = frag_shader,
            .vertex_input_state = .{
                .vertex_buffer_descriptions = &vb_desc,
                .num_vertex_buffers = 1,
                .vertex_attributes = &vertex_attrs,
                .num_vertex_attributes = vertex_attrs.len,
            },
            .primitive_type = c.SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
            .target_info = .{ .color_target_descriptions = &color_desc, .num_color_targets = 1 },
        }));
        errdefer c.SDL_ReleaseGPUGraphicsPipeline(self.device, graphics_pipeline);

        return .{ .pipeline = graphics_pipeline, .sampler = try self.createSampler(sampler) };
    }

    pub fn activatePass(_: *Backend, _: std.mem.Allocator, _: *PassObjects, _: *const ShaderReflection, _: *const ShaderReflection, _: usize) !void {}

    pub fn destroyPass(self: *Backend, _: std.mem.Allocator, objects: *PassObjects) void {
        if (objects.pipeline) |p| c.SDL_ReleaseGPUGraphicsPipeline(self.device, p);
        if (objects.sampler) |s| c.SDL_ReleaseGPUSampler(self.device, s);
        objects.* = .{};
    }

    pub fn createTexture(self: *Backend, w: u32, h: u32, format: TextureFormat, num_levels: u32) !TextureHandle {
        return c.SDL_CreateGPUTexture(self.device, &.{
            .type = c.SDL_GPU_TEXTURETYPE_2D,
            .format = self.sdlFormat(format),
            .usage = c.SDL_GPU_TEXTUREUSAGE_SAMPLER | c.SDL_GPU_TEXTUREUSAGE_COLOR_TARGET,
            .width = w,
            .height = h,
            .layer_count_or_depth = 1,
            .num_levels = num_levels,
            .sample_count = c.SDL_GPU_SAMPLECOUNT_1,
        }) orelse {
            std.log.err("Failed to create {}x{} {s} texture: {s}", .{ w, h, @tagName(format), c.SDL_GetError() });
            return error.TextureCreateFailed;
        };
    }

    pub fn destroyTexture(self: *Backend, texture: TextureHandle) void {
        c.SDL_ReleaseGPUTexture(self.device, texture);
    }

    /// Upload tightly packed RGBA8 pixels.
    pub fn uploadTexture(self: *Backend, texture: TextureHandle, pixels: []const u8, w: u32, h: u32) !void {
        const tb = sdlError(c.SDL_CreateGPUTransferBuffer(
            self.device,
            &.{ .size = @intCast(pixels.len), .usage = c.SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD },
        ));
        defer c.SDL_ReleaseGPUTransferBuffer(self.device, tb);
        const map: [*]u8 = @ptrCast(c.SDL_MapGPUTransferBuffer(self.device, tb, false) orelse return error.TransferBufferMapFailed);
        @memcpy(map[0..pixels.len], pixels);
        c.SDL_UnmapGPUTransferBuffer(self.device, tb);

        const cmd = sdlError(c.SDL_AcquireGPUCommandBuffer(self.device));
        const copy = sdlError(c.SDL_BeginGPUCopyPass(cmd));
        c.SDL_UploadToGPUTexture(
            copy,
            &.{ .transfer_buffer = tb, .pixels_per_row = w, .rows_per_layer = h },
            &.{ .texture = texture, .w = w, .h = h, .d = 1 },
            false,
        );
        c.SDL_EndGPUCopyPass(copy);
        sdlError(c.SDL_SubmitGPUCommandBuffer(cmd));
    }

    pub fn copyTexture(_: *Backend, cmd: Frame, src: TextureHandle, dst: TextureHandle, w: u32, h: u32) void {
        // A blit converts from whatever format the UI uploaded the frame in.
        c.SDL_BlitGPUTexture(cmd, &.{
            .source = .{ .texture = src, .w = w, .h = h },
            .destination = .{ .texture = dst, .w = w, .h = h },
            .load_op = c.SDL_GPU_LOADOP_DONT_CARE,
            .filter = c.SDL_GPU_FILTER_NEAREST,
        });
    }

    pub fn generateMipmaps(_: *Backend, cmd: Frame, texture: TextureHandle) void {
        c.SDL_GenerateMipmapsForGPUTexture(cmd, texture);
    }

    pub fn createSampler(self: *Backend, desc: SamplerDesc) !Sampler {
        const filter: c.SDL_GPUFilter = if (desc.linear) c.SDL_GPU_FILTER_LINEAR else c.SDL_GPU_FILTER_NEAREST;
        // SDL GPU has no clamp-to-border.
        const address_mode: c.SDL_GPUSamplerAddressMode = switch (desc.wrap_mode) {
            .clamp_to_border, .clamp_to_edge => c.SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE,
            .repeat => c.SDL_GPU_SAMPLERADDRESSMODE_REPEAT,
            .mirrored_repeat => c.SDL_GPU_SAMPLERADDRESSMODE_MIRRORED_REPEAT,
        };
        return c.SDL_CreateGPUSampler(self.device, &.{
            .min_filter = filter,
            .mag_filter = filter,
            .mipmap_mode = if (desc.mipmaps) c.SDL_GPU_SAMPLERMIPMAPMODE_LINEAR else c.SDL_GPU_SAMPLERMIPMAPMODE_NEAREST,
            // A zero `max_lod` would clamp sampling to the base level.
            .max_lod = if (desc.mipmaps) 1000 else 0,
            .address_mode_u = address_mode,
            .address_mode_v = address_mode,
            .address_mode_w = address_mode,
        }) orelse {
            std.log.err("Failed to create sampler: {s}", .{c.SDL_GetError()});
            return error.SamplerCreateFailed;
        };
    }

    pub fn destroySampler(self: *Backend, sampler: Sampler) void {
        c.SDL_ReleaseGPUSampler(self.device, sampler);
    }

    pub fn beginPasses(_: *Backend, _: Frame, _: TextureHandle) void {}

    pub fn endPasses(_: *Backend, _: Frame) void {}

    pub fn beginPass(self: *Backend, cmd: Frame, objects: *const PassObjects, target: TextureHandle, w: u32, h: u32) !RenderPass {
        const color_target = c.SDL_GPUColorTargetInfo{
            .texture = target,
            .load_op = c.SDL_GPU_LOADOP_CLEAR,
            .store_op = c.SDL_GPU_STOREOP_STORE,
            .clear_color = .{ .r = 0, .g = 0, .b = 0, .a = 1 },
        };
        const rp = c.SDL_BeginGPURenderPass(cmd, &color_target, 1, null) orelse {
            std.log.err("Failed to begin shader render pass: {s}", .{c.SDL_GetError()});
            return error.RenderPassBeginFailed;
        };
        c.SDL_BindGPUGraphicsPipeline(rp, objects.pipeline);
        c.SDL_BindGPUVertexBuffers(rp, 0, &.{ .buffer = self.vertex_buffer }, 1);
        c.SDL_SetGPUViewport(rp, &.{ .x = 0, .y = 0, .w = @floatFromInt(w), .h = @floatFromInt(h), .min_depth = 0, .max_depth = 1 });
        c.SDL_SetGPUScissor(rp, &.{ .x = 0, .y = 0, .w = @intCast(w), .h = @intCast(h) });
        return rp;
    }

    pub fn pushUniforms(
        _: *Backend,
        cmd: Frame,
        _: *RenderPass,
        _: *const PassObjects,
        stage: parser.ShaderStage,
        binding: u32,
        data: []const u8,
    ) void {
        switch (stage) {
            .Vertex => c.SDL_PushGPUVertexUniformData(cmd, binding, data.ptr, @intCast(data.len)),
            .Fragment => c.SDL_PushGPUFragmentUniformData(cmd, binding, data.ptr, @intCast(data.len)),
        }
    }

    pub fn bindTexture(
        _: *Backend,
        rp: *RenderPass,
        objects: *const PassObjects,
        binding: u32,
        texture: ?TextureHandle,
        sampler: ?Sampler,
    ) !void {
        c.SDL_BindGPUFragmentSamplers(rp.*, binding, &.{ .texture = texture, .sampler = sampler orelse objects.sampler }, 1);
    }

    pub fn endPass(_: *Backend, _: Frame, rp: *RenderPass, _: *const PassObjects, draw: bool) void {
        if (draw) c.SDL_DrawGPUPrimitives(rp.*, QUAD_VERTICES.len, 1, 0, 0);
        c.SDL_EndGPURenderPass(rp.*);
    }
};
